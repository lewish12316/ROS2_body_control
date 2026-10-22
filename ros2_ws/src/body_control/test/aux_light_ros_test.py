"""Verify aux-light topic wiring, source union and real turn-off delay."""

import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, Int32


def main():
    namespace = '/aux_light_test_' + uuid.uuid4().hex
    rclpy.init(args=[])
    node = rclpy.create_node('probe', namespace=namespace)
    samples = []
    subscription = node.create_subscription(
        Int32, 'aux_light_req',
        lambda msg: samples.append((time.monotonic(), msg.data)),
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
    types = {
        'low_speed_parking_st': Bool, 'veh_turn_st': Int32,
        'ambient_light_level_st': Float64, 'veh_running_st': Bool,
        'veh_mode_st': Int32, 'mode_sw_cmd': Bool,
        'emergency_stop_st': Bool, 'alarm_diag_st': Int32,
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

    def send(name, value):
        publishers[name].publish(types[name](data=value))

    def stable(expected, duration=0.22):
        samples.clear()
        spin_for(duration)
        assert len(samples) >= 3, 'Missing 50ms refresh'
        assert all(data == expected for _, data in samples), samples

    def check(name, value, expected):
        nonlocal current
        spin_for(0.06)
        samples.clear()
        sent = time.monotonic()
        send(name, value)
        wait_for(lambda: any(data == expected for _, data in samples))
        received = next(at for at, data in samples if data == expected)
        if expected != current:
            latency = received - sent
            latencies.append(latency)
            assert latency <= 0.1, f'Response exceeded 100ms: {latency}'
        current = expected
        stable(expected)

    try:
        wait_for(lambda: all(pub.get_subscription_count() for pub in publishers.values())
                 and node.count_publishers('aux_light_req') == 1, timeout=10.0)
        wait_for(lambda: len(samples) >= 3)
        assert all(data == 0 for _, data in samples)
        endpoint = node.get_publishers_info_by_topic('aux_light_req')[0]
        assert endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert endpoint.qos_profile.durability == DurabilityPolicy.VOLATILE
        for topic in ('low_speed_parking_st', 'veh_turn_st',
                      'veh_running_st', 'ambient_light_level_st'):
            endpoints = node.get_subscriptions_info_by_topic(topic)
            assert len(endpoints) == 1, 'Existing subscriptions must be reused'
            assert endpoints[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT

        check('low_speed_parking_st', True, 3)
        check('low_speed_parking_st', False, 0)
        check('ambient_light_level_st', float('nan'), 0)
        check('ambient_light_level_st', 5.0, 0)  # Unknown running state.
        check('veh_running_st', True, 3)
        for light in (-1.0, float('nan'), float('inf'), -float('inf')):
            check('ambient_light_level_st', light, 3)
        check('veh_running_st', False, 0)
        check('veh_running_st', True, 3)
        check('ambient_light_level_st', 5.001, 0)
        check('ambient_light_level_st', -1.0, 0)  # Preserve last bright judgment.

        check('veh_turn_st', 1, 1)  # Mode unknown/standby does not block turn fill.
        check('veh_turn_st', 2, 2)  # Direct switch has no old-side hold.
        check('veh_turn_st', 1, 1)
        for mode in (0, 1, 2, -1, 99):
            check('veh_mode_st', mode, 1)
        check('mode_sw_cmd', True, 1)
        check('mode_sw_cmd', False, 1)
        check('emergency_stop_st', True, 1)
        check('alarm_diag_st', 1, 1)

        # Repeated straight feedback must not restart the real 1s deadline.
        samples.clear()
        sent = time.monotonic()
        send('veh_turn_st', 0)
        spin_for(0.35)
        assert samples and all(data == 1 for _, data in samples)
        send('veh_turn_st', 0)
        spin_for(0.4)
        assert all(data == 1 for _, data in samples), 'Turn hold ended early'
        wait_for(lambda: any(data == 0 for _, data in samples), timeout=0.5)
        first_off = next(at for at, data in samples if data == 0)
        assert 1.0 <= first_off - sent < 1.15, 'Turn delay was reset or wrong'
        current = 0
        stable(0)

        check('veh_turn_st', 1, 1)
        send('veh_turn_st', 0)
        spin_for(0.15)
        check('veh_turn_st', 2, 2)  # Replace the pending old-side delay.
        check('low_speed_parking_st', True, 3)
        check('low_speed_parking_st', False, 2)  # Turn survives parking exit.
        check('ambient_light_level_st', 5.0, 3)
        check('low_speed_parking_st', True, 3)
        check('low_speed_parking_st', False, 3)  # Dark running survives parking exit.
        check('veh_running_st', False, 2)  # Current turn survives ambient exit.
        check('low_speed_parking_st', True, 3)
        send('veh_turn_st', 0)
        stable(3, duration=1.2)  # Turn expiry while parking remains active.
        check('low_speed_parking_st', False, 0)
        print(f'Aux light ROS checks passed; {len(latencies)} immediate transitions, '
              f'max send-to-output latency {max(latencies) * 1000:.2f}ms; '
              f'turn hold {first_off - sent:.3f}s')
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
