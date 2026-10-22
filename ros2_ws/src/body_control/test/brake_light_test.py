"""Exercise brake light requests through the actual ROS node."""

import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32


def main():
    namespace = '/brake_light_test_' + uuid.uuid4().hex
    rclpy.init(args=[])
    node = rclpy.create_node('probe', namespace=namespace)
    samples = []
    subscription = node.create_subscription(
        Bool, 'brake_light_req',
        lambda msg: samples.append((time.monotonic(), msg.data)),
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
    types = {
        'veh_brake_st': Bool, 'veh_mode_st': Int32, 'mode_sw_cmd': Bool,
        'veh_running_st': Bool, 'veh_gear_st': Int32, 'veh_epb_st': Bool,
        'alarm_diag_st': Int32,
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
        assert len(samples) >= 3, 'Missing 50ms periodic refresh'
        assert all(data == expected for _, data in samples), samples

    def brake(value):
        nonlocal current
        spin_for(0.06)
        samples.clear()
        sent = time.monotonic()
        send('veh_brake_st', value)
        wait_for(lambda: any(data == value for _, data in samples))
        received = next(at for at, data in samples if data == value)
        # Only actual output transitions prove a new input was processed.
        if value != current:
            latency = received - sent
            latencies.append(latency)
            assert latency <= 0.1, f'Response exceeded 100ms: {latency}'
        current = value
        stable(value)

    try:
        wait_for(lambda: all(pub.get_subscription_count() for pub in publishers.values())
                 and node.count_publishers('brake_light_req') == 1, timeout=10.0)
        wait_for(lambda: len(samples) >= 3)
        assert all(not data for _, data in samples), 'Initial output must be false'
        endpoint = node.get_publishers_info_by_topic('brake_light_req')[0]
        assert endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert endpoint.qos_profile.durability == DurabilityPolicy.VOLATILE
        inputs = node.get_subscriptions_info_by_topic('veh_brake_st')
        assert len(inputs) == 1
        assert inputs[0].qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        # DDS discovery does not expose local history depths.

        # Braking works before any mode, startup or running feedback.
        brake(True)
        brake(True)
        stable(True, duration=1.1)  # No feedback refresh: retain cached braking.
        brake(False)
        brake(False)
        for mode in (0, 1, 2, -1, 99):
            send('veh_mode_st', mode)
            for running in (False, True):
                send('veh_running_st', running)
                brake(True)
                # Each unrelated input must leave the lit request unchanged.
                for name, value in (
                    ('mode_sw_cmd', False), ('mode_sw_cmd', True),
                    ('veh_gear_st', 1), ('veh_gear_st', 0),
                    ('veh_epb_st', True), ('alarm_diag_st', 1)):
                    send(name, value)
                    stable(True)
                # Release also turns off while running, EPB held and fault active.
                brake(False)
                for name, value in (
                    ('veh_epb_st', False), ('veh_epb_st', True),
                    ('alarm_diag_st', 0), ('mode_sw_cmd', False)):
                    send(name, value)
                    stable(False)
        print(f'Brake light ROS checks passed; {len(latencies)} output transitions, '
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
