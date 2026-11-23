"""
Testes da zona morta no nó `follower` (R1, R2 e R4), com `rclpy`, sem o Gazebo.

O teste publica o `ftl/state` deste veículo, parado na origem, e o do veículo da frente, 10 m à
frente, com a posição estimada oscilando 0,15 m por eixo. Com `uncertainty:=1.0`, o `gap` fica
estável e o seguidor não anda; com a frente andando, ele segue. Sem `uncertainty`, vale a B0, e a
trilha infla com o vaivém.
"""
import math
import os
import random
import time

from ftl_follower_pure_pursuit.follower import Follower
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64


def state(x, y, v=0.0):
    msg = Odometry()
    msg.header.frame_id = 'ftl_map'
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = v
    return msg


def run(extra, ahead_x, ahead_v=0.0, seconds=6.0, sigma=0.15):
    """Roda o seguidor; devolve as velocidades comandadas e os `gap` publicados."""
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(30, 99))
    args = ['--ros-args', '-p', 'ahead:=/lead'] + [a for p in extra for a in ('-p', p)]
    rclpy.init(args=args)
    try:
        follower = Follower()
        probe = rclpy.create_node('probe')
        cmds, gaps = [], []
        probe.create_subscription(Twist, '/cmd_vel', lambda m: cmds.append(m.linear.x), 10)
        probe.create_subscription(Float64, '/ftl/gap', lambda m: gaps.append(m.data), 10)
        me = probe.create_publisher(Odometry, '/ftl/state', 10)
        lead = probe.create_publisher(Odometry, '/lead/ftl/state', 10)
        executor = SingleThreadedExecutor()
        executor.add_node(follower)
        executor.add_node(probe)
        rng = random.Random(5)
        start = time.monotonic()
        while (t := time.monotonic() - start) < seconds:
            me.publish(state(0.0, 0.0))
            x = ahead_x(t) + rng.gauss(0.0, sigma)
            lead.publish(state(x, rng.gauss(0.0, sigma), ahead_v))
            end = time.monotonic() + 0.05
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)
        executor.shutdown()
        follower.destroy_node()
        probe.destroy_node()
        return cmds, gaps
    finally:
        rclpy.shutdown()


def test_parked_leader_with_dead_band_keeps_the_follower_still():
    cmds, gaps = run(['uncertainty:=1.0'], lambda t: 10.0)
    assert len(gaps) > 50
    assert max(abs(v) for v in cmds) == 0.0
    assert max(gaps) - min(gaps) < 1.0


def test_parked_leader_without_dead_band_inflates_the_gap():
    # A B0 sem zona morta: o comportamento observado na simulação.
    _, gaps = run([], lambda t: 10.0)
    assert gaps[-1] - gaps[0] > 3.0


def test_moving_leader_is_still_followed():
    cmds, gaps = run(['uncertainty:=1.0'], lambda t: 10.0 + 2.0 * t, ahead_v=2.0)
    assert gaps[-1] > gaps[0] + 3.0
    assert max(cmds) > 1.0
    assert math.isfinite(gaps[-1])


def test_uncertainty_parameter_defaults_to_the_baseline():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(30, 99))
    rclpy.init(args=['--ros-args', '-p', 'ahead:=/lead'])
    try:
        follower = Follower()
        assert follower.get_parameter('uncertainty').value == pytest.approx(0.0)
        follower.destroy_node()
    finally:
        rclpy.shutdown()
