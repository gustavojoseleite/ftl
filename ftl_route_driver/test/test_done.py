"""
O `route_driver` avisa o fim do protocolo em `route/done` (F2.3, R5), com `rclpy`.

O teste publica a pose real do líder parado na largada e a do último trator andando pela rota,
de trás da largada até depois do fim da última curva. O nó publica `route/done` (`std_msgs/Bool`,
QoS retida) com `True` quando para pelo protocolo, e não antes.
"""
import os
import random
import time

from ftl_route_driver.node import RouteDriverNode
from ftl_route_driver.routes import route
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


def pose(x, y):
    msg = PoseStamped()
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.w = 1.0
    return msg


def test_done_after_the_last_tractor_passes_the_last_curve():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(30, 99))
    rclpy.init(args=['--ros-args', '-r', '__ns:=/pickup', '-p', 'route:=citrus_farm_90'])
    try:
        driver = RouteDriverNode()
        probe = rclpy.create_node('probe')
        done = []
        probe.create_subscription(Bool, '/pickup/route/done', lambda m: done.append(m.data),
                                  QoSProfile(depth=1,
                                             durability=DurabilityPolicy.TRANSIENT_LOCAL))
        leader = probe.create_publisher(PoseStamped, '/ground_truth/pickup/pose', 10)
        last = probe.create_publisher(PoseStamped, '/ground_truth/tractor_1/pose', 10)
        executor = SingleThreadedExecutor()
        executor.add_node(driver)
        executor.add_node(probe)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)

        r = route('citrus_farm_90')
        end_of_last_curve = r.length - ((2 * 205 - 2 * 24) / 2 + 2)
        leader.publish(pose(-205.0, 0.0))
        last.publish(pose(-205.0, -8.0))
        spin(1.0)
        s = 0.0
        while s < end_of_last_curve - 5.0:
            x, y, _ = r.at(s)
            leader.publish(pose(-205.0, 0.0))
            last.publish(pose(x, y))
            spin(0.02)
            s += 5.0
        spin(0.5)
        assert True not in done
        for extra in (5.0, 10.0, 15.0):
            x, y, _ = r.at(end_of_last_curve + extra)
            last.publish(pose(x, y))
            spin(0.3)
        assert done and done[-1] is True
        executor.shutdown()
        driver.destroy_node()
        probe.destroy_node()
    finally:
        rclpy.shutdown()
