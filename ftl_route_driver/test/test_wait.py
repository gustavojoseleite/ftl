"""
O `route_driver` espera os estimadores antes de arrancar (F2.5, R6), com `rclpy`.

Com `wait_for` igual a `['/tractor_1/ftl/state']`, o nó manda `cmd_vel` com velocidade zero
enquanto esse tópico não publicou, mesmo com a pose real do líder chegando. Depois da primeira
mensagem nele, o líder arranca: a velocidade comandada passa de zero.
"""
import os
import random
import time

from ftl_route_driver.node import RouteDriverNode
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor


def test_leader_waits_for_the_estimators():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(30, 99))
    rclpy.init(args=['--ros-args', '-r', '__ns:=/pickup', '-p', 'route:=citrus_farm_90',
                     '-p', "wait_for:=['/tractor_1/ftl/state']"])
    try:
        driver = RouteDriverNode()
        probe = rclpy.create_node('probe')
        speeds = []
        probe.create_subscription(Twist, '/pickup/cmd_vel',
                                  lambda m: speeds.append(m.linear.x), 10)
        leader = probe.create_publisher(PoseStamped, '/ground_truth/pickup/pose', 10)
        state = probe.create_publisher(Odometry, '/tractor_1/ftl/state', 10)
        executor = SingleThreadedExecutor()
        executor.add_node(driver)
        executor.add_node(probe)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                pose = PoseStamped()
                pose.pose.position.x, pose.pose.position.y = -205.0, 0.0
                pose.pose.orientation.z, pose.pose.orientation.w = 0.7071068, 0.7071068
                leader.publish(pose)
                executor.spin_once(timeout_sec=0.02)

        spin(2.0)
        assert speeds, 'o nó não publicou cmd_vel'
        assert max(speeds) == 0.0
        state.publish(Odometry())
        spin(2.0)
        assert max(speeds) > 0.0
        executor.shutdown()
        driver.destroy_node()
        probe.destroy_node()
    finally:
        rclpy.shutdown()
