"""Exercise the actual node over isolated ROS topics, including refresh and latency."""

import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32


def main():
    namespace = '/reverse_light_test_' + uuid.uuid4().hex
    rclpy.init(args=[])
    node = rclpy.create_node('probe', namespace=namespace)
    samples = []
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    subscription = node.create_subscription(
        Bool, 'reverse_light_req',
        lambda msg: samples.append((time.monotonic(), msg.data)), qos)
    publishers = {
        name: node.create_publisher(kind, name, 10)
        for name, kind in [
            ('veh_gear_st', Int32), ('veh_mode_st', Int32),
            ('mode_sw_cmd', Bool), ('veh_running_st', Bool),
            ('alarm_diag_st', Int32)]
    }
    process = subprocess.Popen(
        [sys.argv[1], '--ros-args', '-r', '__ns:=' + namespace],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
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

    def send(name, value):
        kind = Bool if name in ('mode_sw_cmd', 'veh_running_st') else Int32
        publishers[name].publish(kind(data=value))

    def gear(value):
        # Flush prior samples before measuring a new transition. Send-to-receive
        # includes input transport and is a conservative bound on node response.
        spin_for(0.06)
        samples.clear()
        sent = time.monotonic()
        send('veh_gear_st', value)
        expected = value == 1
        wait_for(lambda: any(data == expected for _, data in samples))
        received = next(at for at, data in samples if data == expected)
        latencies.append(received - sent)
        assert received - sent <= 0.1, f'Light response exceeded 100ms: {received - sent}'
        samples.clear()
        spin_for(0.22)
        assert len(samples) >= 3, 'Missing 50ms periodic refresh'
        assert all(data == expected for _, data in samples), 'Unstable light output'

    try:
        wait_for(lambda: all(pub.get_subscription_count() for pub in publishers.values())
                 and node.count_publishers('reverse_light_req') == 1, timeout=10.0)
        wait_for(lambda: len(samples) >= 3)
        assert all(not data for _, data in samples), 'Initial output must be false'
        endpoint = node.get_publishers_info_by_topic('reverse_light_req')[0]
        # DDS discovery does not expose the writer's local history depth.
        # Depth 1 is configured explicitly at publisher construction.
        assert endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE

        # No startup authorization or running feedback is required.
        gear(1)
        gear(1)
        for value in (0, 2, 3, -1, 99):
            gear(1)
            gear(value)

        for mode in (0, 1, 2, -1):
            send('veh_mode_st', mode)
            for running in (False, True):
                send('veh_running_st', running)
                send('mode_sw_cmd', False)
                send('alarm_diag_st', 1)
                gear(1)
                # Change unrelated inputs while R is held: lamp must stay on.
                send('mode_sw_cmd', True)
                send('alarm_diag_st', 0)
                samples.clear()
                spin_for(0.16)
                assert samples and all(data for _, data in samples)
                gear(3)

        print(f'Reverse light ROS checks passed; {len(latencies)} gear updates, '
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
