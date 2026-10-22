"""Exercise mode requests, task authorization and existing lights/voices over DDS."""
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32


def main():
    namespace = '/mode_switch_test_' + uuid.uuid4().hex
    rclpy.init(args=[])
    node = rclpy.create_node('probe', namespace=namespace)
    samples = {name: [] for name in ('ad_mode_req', 'ads_light_req', 'voice_broadcast_id')}
    subscriptions = [node.create_subscription(
        Bool if name == 'ad_mode_req' else Int32, name,
        lambda msg, key=name: samples[key].append((time.monotonic(), msg.data)),
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)) for name in samples]
    types = {
        'veh_mode_st': Int32, 'mode_sw_cmd': Bool, 'adc_self_check_st': Bool,
        'veh_running_st': Bool, 'obs_detect_st': Bool, 'alarm_diag_st': Int32,
        'emergency_stop_st': Bool, 'ads_abnormal_st': Bool, 'veh_gear_st': Int32,
    }
    publishers = {name: node.create_publisher(kind, name, 10) for name, kind in types.items()}
    process = subprocess.Popen(
        [sys.argv[1], '--ros-args', '-r', '__ns:=' + namespace],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    expected_requests = []
    latencies = []

    def wait_for(predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert process.poll() is None, 'body_control_node exited unexpectedly'
            assert time.monotonic() < deadline, 'Timed out waiting for ROS condition'
            rclpy.spin_once(node, timeout_sec=0.005)

    def spin_for(duration=0.08):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert process.poll() is None, 'body_control_node exited unexpectedly'
            rclpy.spin_once(node, timeout_sec=0.005)

    def send(name, value):
        publishers[name].publish(types[name](data=value))
        spin_for()

    def assert_requests():
        spin_for()
        actual = [data for _, data in samples['ad_mode_req']]
        assert actual == expected_requests, (actual, expected_requests)

    def change(name, value, request):
        sent = time.monotonic()
        publishers[name].publish(types[name](data=value))
        expected_requests.append(request)
        wait_for(lambda: len(samples['ad_mode_req']) >= len(expected_requests))
        latency = samples['ad_mode_req'][-1][0] - sent
        assert latency <= 0.2, f'ADC request response exceeded 200ms: {latency}'
        latencies.append(latency)
        assert_requests()

    def latest_light(expected):
        wait_for(lambda: samples['ads_light_req'] and samples['ads_light_req'][-1][1] == expected)

    def voice_after(start, expected):
        wait_for(lambda: any(data == expected for _, data in samples['voice_broadcast_id'][start:]))

    try:
        wait_for(lambda: all(pub.get_subscription_count() for pub in publishers.values())
                 and all(node.count_publishers(name) == 1 for name in samples), timeout=10)
        latest_light(0)
        endpoint = node.get_publishers_info_by_topic('ad_mode_req')[0]
        assert endpoint.topic_type == 'std_msgs/msg/Bool'
        inputs = node.get_subscriptions_info_by_topic('adc_self_check_st')
        assert len(inputs) == 1 and inputs[0].topic_type == 'std_msgs/msg/Bool'
        assert inputs[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        send('mode_sw_cmd', True)
        for name, value in (
                ('veh_mode_st', 0), ('veh_running_st', False), ('obs_detect_st', False),
                ('alarm_diag_st', 0), ('emergency_stop_st', False), ('ads_abnormal_st', False)):
            send(name, value)
        assert_requests()  # Missing self-check must not default to ready.
        send('adc_self_check_st', False)
        assert_requests()
        change('adc_self_check_st', True, True)
        send('veh_gear_st', 1)  # Reverse voice requires feedback AND a new start command.
        send('mode_sw_cmd', True)
        for _ in range(2):
            send('veh_mode_st', 0)
            send('adc_self_check_st', True)
        assert_requests()
        assert not samples['voice_broadcast_id']
        send('veh_mode_st', 1)
        latest_light(1)
        assert not samples['voice_broadcast_id']
        start = len(samples['voice_broadcast_id'])
        send('mode_sw_cmd', True)
        voice_after(start, 12)
        assert_requests()
        send('veh_running_st', True)
        send('obs_detect_st', True)
        assert_requests()  # Entry checks do not revoke running tasks.

        change('emergency_stop_st', True, False)
        start = len(samples['voice_broadcast_id'])
        send('emergency_stop_st', False)
        send('veh_mode_st', 1)
        assert_requests()
        assert len(samples['voice_broadcast_id']) == start
        change('mode_sw_cmd', True, True)
        voice_after(start, 12)
        change('adc_self_check_st', False, False)
        send('adc_self_check_st', True)
        assert_requests()  # Readiness recovery does not restart tasks.
        change('mode_sw_cmd', True, True)
        change('ads_abnormal_st', True, False)
        latest_light(2)
        send('ads_abnormal_st', False)
        latest_light(1)
        change('mode_sw_cmd', True, True)
        change('alarm_diag_st', -1, False)
        send('alarm_diag_st', 0)
        change('mode_sw_cmd', True, True)

        start = len(samples['voice_broadcast_id'])
        send('veh_mode_st', 2)
        latest_light(0)
        voice_after(start, 14)
        assert_requests()  # No exit request after takeover feedback.
        start = len(samples['voice_broadcast_id'])
        send('veh_mode_st', 1)  # Direct remote -> auto cannot restore ADC permission.
        latest_light(1)  # ADS follows VCU feedback independently of permission.
        send('mode_sw_cmd', True)
        send('mode_sw_cmd', False)
        send('adc_self_check_st', False)
        send('adc_self_check_st', True)
        send('mode_sw_cmd', True)
        assert_requests()
        assert len(samples['voice_broadcast_id']) == start
        send('veh_running_st', False)
        send('obs_detect_st', False)
        change('veh_mode_st', 0, True)
        send('veh_mode_st', 1)
        start = len(samples['voice_broadcast_id'])
        send('mode_sw_cmd', True)
        voice_after(start, 12)
        change('mode_sw_cmd', False, False)
        send('veh_mode_st', 0)
        send('adc_self_check_st', True)
        send('mode_sw_cmd', False)
        assert_requests()
        change('mode_sw_cmd', True, True)
        change('mode_sw_cmd', False, False)
        send('adc_self_check_st', False)
        change('adc_self_check_st', True, True)
        start = len(samples['voice_broadcast_id'])
        spin_for(0.3)
        assert_requests()
        assert len(samples['voice_broadcast_id']) == start
        print(f'Mode switch ROS checks passed; max observed ADC response {max(latencies):.4f}s')
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        error_output = process.stderr.read().decode(errors='replace')
        process.stderr.close()
        if sys.exc_info()[0]:
            print(error_output, file=sys.stderr)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
