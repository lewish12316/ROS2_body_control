"""Verify turn requests and latency through the actual ROS node."""

import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32


def main():
    namespace = '/turn_light_test_' + uuid.uuid4().hex
    rclpy.init(args=[])
    node = rclpy.create_node('probe', namespace=namespace)
    samples = []
    subscription = node.create_subscription(
        Int32, 'turn_light_req',
        lambda msg: samples.append((time.monotonic(), msg.data)),
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
    publishers = {
        name: node.create_publisher(kind, name, 10)
        for name, kind in [
            ('veh_mode_st', Int32), ('veh_turn_st', Int32),
            ('mode_sw_cmd', Bool), ('veh_running_st', Bool),
            ('alarm_diag_st', Int32)]
    }
    process = subprocess.Popen(
        [sys.argv[1], '--ros-args', '-r', '__ns:=' + namespace],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    latencies = []
    current = 0

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

    def check(name, value, expected):
        nonlocal current
        spin_for(0.06)
        samples.clear()
        sent = time.monotonic()
        send(name, value)
        wait_for(lambda: any(data == expected for _, data in samples))
        received = next(at for at, data in samples if data == expected)
        # Measure only output changes: unchanged periodic output cannot prove
        # that the triggering input has been processed.
        if expected != current:
            latency = received - sent
            latencies.append(latency)
            assert latency <= 0.1, f'Response exceeded 100ms: {latency}'
        spin_for(0.22)
        after = [data for at, data in samples if at >= received]
        assert len(after) >= 4, 'Missing 50ms refresh'
        assert all(data == expected for data in after), f'Unstable output: {after}'
        current = expected

    try:
        wait_for(lambda: all(pub.get_subscription_count() for pub in publishers.values())
                 and node.count_publishers('turn_light_req') == 1, timeout=10.0)
        wait_for(lambda: len(samples) >= 3)
        assert all(data == 0 for _, data in samples), 'Initial output must be zero'
        endpoint = node.get_publishers_info_by_topic('turn_light_req')[0]
        assert endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert endpoint.qos_profile.durability == DurabilityPolicy.VOLATILE
        # DDS discovery does not expose the publisher's local history depth.

        # Direction first: held inactive until mode becomes valid.
        check('veh_turn_st', 1, 0)
        check('veh_mode_st', 1, 1)
        check('veh_turn_st', 2, 2)  # Direct switch, no intermediate zero.
        check('veh_turn_st', 2, 2)
        check('veh_turn_st', 0, 0)
        for invalid in (-1, 3, 99):
            check('veh_turn_st', 1, 1)
            check('veh_turn_st', invalid, 0)

        # Mode first, then direction; leaving/re-entering restores cached turn.
        check('veh_mode_st', 2, 0)
        check('veh_turn_st', 2, 2)
        for mode in (0, -1, 99):
            check('veh_mode_st', mode, 0)
            check('veh_turn_st', 1, 0)
            check('veh_mode_st', 2, 1)
        check('veh_mode_st', 1, 1)
        check('veh_mode_st', 2, 1)  # Valid mode switch must not insert zero.

        for mode in (1, 2):
            check('veh_mode_st', mode, 1)
            for running in (False, True):
                check('veh_running_st', running, 1)
                check('mode_sw_cmd', False, 1)
                check('alarm_diag_st', 1, 1)
                check('mode_sw_cmd', True, 1)
                check('alarm_diag_st', 0, 1)
        check('veh_turn_st', 0, 0)
        print(f'Turn light ROS checks passed; {len(latencies)} output transitions, '
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
