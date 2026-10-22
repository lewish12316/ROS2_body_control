"""Verify ADS enum requests through the actual ROS node."""

import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32


def main():
    namespace = '/ads_light_test_' + uuid.uuid4().hex
    rclpy.init(args=[])
    node = rclpy.create_node('probe', namespace=namespace)
    samples = []
    subscription = node.create_subscription(
        Int32, 'ads_light_req',
        lambda msg: samples.append((time.monotonic(), msg.data)),
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
    types = {
        'ads_abnormal_st': Bool, 'veh_mode_st': Int32, 'mode_sw_cmd': Bool,
        'veh_running_st': Bool, 'alarm_diag_st': Int32,
        'mode_transition_abnormal_st': Bool, 'emergency_stop_st': Bool,
    }
    publishers = {name: node.create_publisher(kind, name, 10)
                  for name, kind in types.items()}
    process = subprocess.Popen(
        [sys.argv[1], '--ros-args', '-r', '__ns:=' + namespace],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    current = 0
    latencies = []

    def wait_for(predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert process.poll() is None, 'body_control_node exited unexpectedly'
            assert time.monotonic() < deadline, 'Timed out waiting for ROS condition'
            rclpy.spin_once(node, timeout_sec=0.005)

    def spin_for(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.005)

    def check(name, value, expected):
        nonlocal current
        spin_for(0.06)
        samples.clear()
        sent = time.monotonic()
        publishers[name].publish(types[name](data=value))
        wait_for(lambda: any(data == expected for _, data in samples))
        received = next(at for at, data in samples if data == expected)
        # Only output changes can establish that a new input was processed.
        if current != expected:
            latency = received - sent
            latencies.append(latency)
            assert latency <= 0.1, f'Response exceeded 100ms: {latency}'
        spin_for(0.22)
        after = [data for at, data in samples if at >= received]
        assert len(after) >= 4, 'Missing 50ms periodic refresh'
        assert all(data == expected for data in after), f'Unstable ADS request: {after}'
        current = expected

    try:
        wait_for(lambda: all(pub.get_subscription_count() for pub in publishers.values())
                 and node.count_publishers('ads_light_req') == 1, timeout=10.0)
        wait_for(lambda: len(samples) >= 3)
        assert all(data == 0 for _, data in samples), 'Initial output must be zero'
        endpoint = node.get_publishers_info_by_topic('ads_light_req')[0]
        assert endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert endpoint.qos_profile.durability == DurabilityPolicy.VOLATILE
        inputs = node.get_subscriptions_info_by_topic('ads_abnormal_st')
        assert len(inputs) == 1
        assert inputs[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        # Local queue depths are not exposed by DDS endpoint discovery.

        # Mode first: unknown abnormal feedback defaults to normal.
        check('veh_mode_st', 1, 1)
        check('veh_mode_st', 1, 1)
        check('ads_abnormal_st', True, 2)
        check('ads_abnormal_st', True, 2)
        check('ads_abnormal_st', False, 1)
        check('veh_mode_st', 0, 0)
        # Abnormal first: mode takes precedence; re-entry uses cached abnormal.
        check('ads_abnormal_st', True, 0)
        check('veh_mode_st', 1, 2)
        for mode in (0, 2, -1, 99):
            check('veh_mode_st', mode, 0)
            check('ads_abnormal_st', False, 0)
            check('ads_abnormal_st', True, 0)
            check('veh_mode_st', 1, 2)

        # Other inputs must not replace either normal or abnormal ADS state.
        for abnormal, expected in ((False, 1), (True, 2)):
            check('ads_abnormal_st', abnormal, expected)
            for name, value in (
                ('mode_sw_cmd', True), ('mode_sw_cmd', False),
                ('veh_running_st', True), ('veh_running_st', False),
                ('alarm_diag_st', 1), ('alarm_diag_st', -1), ('alarm_diag_st', 0),
                ('mode_transition_abnormal_st', True),
                ('mode_transition_abnormal_st', False),
                ('emergency_stop_st', True), ('emergency_stop_st', False)):
                check(name, value, expected)
        # No new feedback: the cached abnormal state continues as enum 2.
        samples.clear()
        spin_for(1.1)
        assert len(samples) >= 15 and all(data == 2 for _, data in samples)
        check('ads_abnormal_st', False, 1)
        check('veh_mode_st', 2, 0)
        print(f'ADS light ROS checks passed; {len(latencies)} output transitions, '
              f'max send-to-output latency {max(latencies) * 1000:.2f}ms')
    finally:
        process.terminate()
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
