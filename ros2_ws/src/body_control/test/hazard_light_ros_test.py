"""Exercise all hazard causes and real 30-second timer wiring over ROS topics."""

import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32


def main():
    namespace = '/hazard_light_test_' + uuid.uuid4().hex
    rclpy.init(args=[])
    node = rclpy.create_node('probe', namespace=namespace)
    samples, turns = [], []
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    hazard_sub = node.create_subscription(
        Bool, 'hazard_light_req',
        lambda msg: samples.append((time.monotonic(), msg.data)), qos)
    turn_sub = node.create_subscription(
        Int32, 'turn_light_req', lambda msg: turns.append(msg.data), qos)
    types = {
        'emergency_stop_st': Bool, 'mode_transition_abnormal_st': Bool,
        'veh_mode_st': Int32, 'alarm_diag_st': Int32, 'veh_running_st': Bool,
        'obs_detect_st': Bool, 'battery_low_power_st': Bool,
        'veh_turn_st': Int32, 'mode_sw_cmd': Bool, 'veh_brake_st': Bool,
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
        if current != expected:
            latency = received - sent
            latencies.append(latency)
            assert latency <= 0.1, f'Response exceeded 100ms: {latency}'
        current = expected
        stable(expected)

    try:
        wait_for(lambda: all(pub.get_subscription_count() for pub in publishers.values())
                 and node.count_publishers('hazard_light_req') == 1, timeout=10.0)
        wait_for(lambda: len(samples) >= 3)
        assert all(not data for _, data in samples)
        endpoint = node.get_publishers_info_by_topic('hazard_light_req')[0]
        assert endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert endpoint.qos_profile.durability == DurabilityPolicy.VOLATILE
        for topic in ('emergency_stop_st', 'mode_transition_abnormal_st'):
            endpoints = node.get_subscriptions_info_by_topic(topic)
            assert len(endpoints) == 1
            assert endpoints[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT

        # Unknown running feedback must not imply stationary parking.
        check('alarm_diag_st', 1, False)
        check('battery_low_power_st', True, False)
        check('emergency_stop_st', True, True)
        check('mode_transition_abnormal_st', True, True)
        check('emergency_stop_st', False, True)
        check('mode_transition_abnormal_st', False, False)
        check('battery_low_power_st', False, False)
        check('veh_running_st', False, True)  # Fault + known stopped.
        check('alarm_diag_st', 2, True)
        check('alarm_diag_st', -1, False)
        check('battery_low_power_st', True, True)
        check('veh_running_st', True, False)
        check('veh_running_st', False, True)
        check('battery_low_power_st', False, False)

        check('veh_mode_st', 2, True)  # Remote even stopped, no startup.
        check('mode_sw_cmd', False, True)
        check('emergency_stop_st', True, True)
        check('veh_mode_st', 99, True)  # Other cause keeps request active.
        check('emergency_stop_st', False, False)
        check('veh_mode_st', -1, False)  # Unknown mode alone is not abnormal.
        check('veh_brake_st', True, False)  # Ordinary braking is not emergency.

        # Hazard does not suppress or alternate the current turn request.
        check('veh_mode_st', 1, False)
        send('veh_turn_st', 1)
        wait_for(lambda: turns and turns[-1] == 1)
        check('mode_transition_abnormal_st', True, True)
        turns.clear()
        spin_for(0.22)
        assert len(turns) >= 3 and all(value == 1 for value in turns)
        send('veh_turn_st', 2)
        wait_for(lambda: turns[-1] == 2)
        check('mode_transition_abnormal_st', False, False)
        turns.clear()
        spin_for(0.22)
        assert turns and all(value == 2 for value in turns)

        # Real timer wiring: no repeated input is needed to trigger after 30s.
        samples.clear()
        sent = time.monotonic()
        send('obs_detect_st', True)
        spin_for(29.8)
        assert samples and all(not data for _, data in samples), 'Triggered early'
        wait_for(lambda: any(data for _, data in samples), timeout=0.7)
        first_on = next(at for at, data in samples if data)
        assert first_on - sent >= 30.0, 'Obstacle threshold must be 30 seconds'
        current = True
        stable(True)
        check('obs_detect_st', False, False)
        check('obs_detect_st', True, False)  # New wait must start from zero.
        check('veh_running_st', True, False)
        check('veh_running_st', False, False)
        check('obs_detect_st', False, False)
        print(f'Hazard ROS checks passed; {len(latencies)} immediate transitions, '
              f'max send-to-output latency {max(latencies) * 1000:.2f}ms; '
              f'obstacle trigger after {first_on - sent:.3f}s')
    finally:
        process.terminate()
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()
        node.destroy_subscription(hazard_sub)
        node.destroy_subscription(turn_sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
