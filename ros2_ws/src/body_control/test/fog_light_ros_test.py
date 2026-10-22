"""Verify rear fog lamp mode, command and power behavior over actual ROS topics."""

import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, Int32


def main():
    namespace = '/fog_light_test_' + uuid.uuid4().hex
    rclpy.init(args=[])
    node = rclpy.create_node('probe', namespace=namespace)
    samples, brakes = [], []
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    subscription = node.create_subscription(
        Bool, 'fog_light_req',
        lambda msg: samples.append((time.monotonic(), msg.data)), qos)
    brake_sub = node.create_subscription(
        Bool, 'brake_light_req', lambda msg: brakes.append(msg.data), qos)
    types = {
        'veh_power_st': Bool, 'low_visibility_st': Bool, 'fog_light_sw_cmd': Bool,
        'veh_mode_st': Int32, 'ambient_light_level_st': Float64,
        'mode_sw_cmd': Bool, 'veh_running_st': Bool, 'veh_brake_st': Bool,
    }
    publishers = {name: node.create_publisher(kind, name, 10)
                  for name, kind in types.items()}
    process = subprocess.Popen(
        [sys.argv[1], '--ros-args', '-r', '__ns:=' + namespace],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    latencies = []
    current = False

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
        if current != expected:
            latency = received - sent
            latencies.append(latency)
            assert latency <= 0.1, f'Response exceeded 100ms: {latency}'
        spin_for(0.22)
        after = [data for at, data in samples if at >= received]
        assert len(after) >= 4, 'Missing 50ms refresh'
        assert all(data == expected for data in after), after
        current = expected

    try:
        wait_for(lambda: all(pub.get_subscription_count() for pub in publishers.values())
                 and node.count_publishers('fog_light_req') == 1, timeout=10.0)
        wait_for(lambda: len(samples) >= 3)
        assert all(not data for _, data in samples)
        endpoint = node.get_publishers_info_by_topic('fog_light_req')[0]
        assert endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert endpoint.qos_profile.durability == DurabilityPolicy.VOLATILE
        for topic in ('veh_power_st', 'low_visibility_st', 'fog_light_sw_cmd',
                      'veh_mode_st', 'ambient_light_level_st'):
            endpoints = node.get_subscriptions_info_by_topic(topic)
            assert len(endpoints) == 1
            assert endpoints[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT

        check('fog_light_sw_cmd', True, False)  # Ignore command before mode/power.
        check('veh_mode_st', 1, False)
        check('low_visibility_st', True, False)
        check('veh_power_st', True, True)  # Unknown light doesn't block visibility.
        check('low_visibility_st', False, False)
        check('ambient_light_level_st', float('nan'), False)
        check('ambient_light_level_st', 5.0, True)
        for value in (-1.0, float('nan'), float('inf'), -float('inf')):
            check('ambient_light_level_st', value, True)
        check('low_visibility_st', True, True)
        check('ambient_light_level_st', 5.001, True)
        check('low_visibility_st', False, False)
        check('ambient_light_level_st', -1.0, False)
        check('low_visibility_st', True, True)
        check('fog_light_sw_cmd', False, True)  # Auto ignores manual close.
        for name, value in (('mode_sw_cmd', True), ('mode_sw_cmd', False),
                            ('veh_running_st', True), ('veh_running_st', False)):
            check(name, value, True)
        check('fog_light_sw_cmd', True, True)  # Auto ignores manual open too.
        check('veh_mode_st', 2, False)  # Does not restore automatic-mode command.
        check('fog_light_sw_cmd', True, True)
        check('veh_power_st', True, True)
        check('veh_mode_st', 2, True)  # Repeated valid feedback must not reset.
        check('low_visibility_st', False, True)  # Recovery doesn't close remote lamp.
        check('fog_light_sw_cmd', False, False)
        check('ambient_light_level_st', 0.0, False)  # Dark alone can't enable remote.
        check('fog_light_sw_cmd', True, True)
        check('veh_brake_st', True, True)
        check('veh_power_st', False, False)
        brakes.clear()
        spin_for(0.22)
        assert brakes and all(brakes), 'Power input must not change existing brake lamp'
        check('fog_light_sw_cmd', True, False)  # Down-state command is dropped.
        check('veh_power_st', True, False)  # Fresh command required.
        check('fog_light_sw_cmd', True, True)
        for mode in (0, -1, 99):
            check('veh_mode_st', mode, False)
            check('fog_light_sw_cmd', True, False)
            check('veh_mode_st', 2, False)
            check('fog_light_sw_cmd', True, True)
        check('veh_mode_st', 1, True)  # Auto uses retained dark state.
        check('veh_power_st', False, False)
        check('ambient_light_level_st', -1.0, False)
        check('veh_power_st', True, True)  # Valid light judgment survives power cycle.
        check('ambient_light_level_st', 6.0, False)
        print(f'Fog light ROS checks passed; {len(latencies)} output transitions, '
              f'max send-to-output latency {max(latencies) * 1000:.2f}ms')
    finally:
        process.terminate()
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()
        node.destroy_subscription(subscription)
        node.destroy_subscription(brake_sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
