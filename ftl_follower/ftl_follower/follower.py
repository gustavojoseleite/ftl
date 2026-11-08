#!/usr/bin/env python3
"""Drive along the path of the vehicle ahead, a set distance behind it.

    ros2 run ftl_follower follower --ros-args -r __ns:=/tractor_1 -p ahead:=/pickup

Reads this vehicle's `ftl/state` and the `ftl/state` of the vehicle ahead
(both from pose_estimator, in the shared ftl_map frame) and publishes
`cmd_vel`. In a convoy every follower follows the one in front of it, so a
disturbance passes down the line rather than every machine chasing the leader
across the field.

Path, not position. The vehicle ahead's positions are kept as a trail, and the
follower steers by pure pursuit on a point of that trail one lookahead ahead
of itself. Aiming at the vehicle itself would cut every corner of a carreador.

Gap, along the path. How far behind is the arc length of the trail between
the two machines, so a bend does not make the follower think it is too close.
Speed is the vehicle ahead's speed plus a pull on the gap error, ramped, and
zero below min_gap or when either state goes stale.

Published besides cmd_vel, for experiments: ftl/gap (metres along the path)
and ftl/cross_track (metres off the path, positive = left of it).
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from ftl_follower.control import (clamp_curvature, gap_speed, lookahead,
                                  pure_pursuit_curvature, ramp)
from ftl_follower.geo import yaw_from_quaternion
from ftl_follower.trail import Trail


def _state(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w), msg.twist.twist.linear.x


class Follower(Node):
    def __init__(self):
        super().__init__("follower")

        def param(name, default):
            return self.declare_parameter(name, default).value

        ahead = param("ahead", "")
        if not ahead:
            raise SystemExit("follower: o parâmetro 'ahead' (namespace do veículo da frente) é obrigatório")
        self.gap_desired = param("gap", 10.0)
        self.min_gap = param("min_gap", 5.0)
        self.k_gap = param("k_gap", 0.4)
        self.v_max = param("v_max", 4.0)
        self.accel = param("accel", 1.0)
        self.ld_min = param("lookahead_min", 4.0)
        self.ld_gain = param("lookahead_gain", 1.0)
        self.ld_max = param("lookahead_max", 10.0)
        self.min_radius = param("min_radius", 3.2)
        self.stale_after = param("stale_after", 1.0)
        rate = param("rate", 20.0)
        self.dt = 1.0 / rate

        self.trail = Trail(param("trail_spacing", 0.3))
        self.hint = 0
        self.me = self.ahead = None
        self.me_t = self.ahead_t = None
        self.v = 0.0
        self.last_log = None

        self.cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.gap_pub = self.create_publisher(Float64, "ftl/gap", 10)
        self.xte_pub = self.create_publisher(Float64, "ftl/cross_track", 10)
        self.create_subscription(Odometry, "ftl/state", self._on_me, 10)
        self.create_subscription(Odometry, f"{ahead.rstrip('/')}/ftl/state", self._on_ahead, 10)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"seguindo {ahead} a {self.gap_desired:.1f} m")

    def _on_me(self, msg):
        self.me, self.me_t = _state(msg), self.get_clock().now()

    def _on_ahead(self, msg):
        self.ahead, self.ahead_t = _state(msg), self.get_clock().now()

    def _fresh(self, t, now):
        return t is not None and (now - t).nanoseconds * 1e-9 < self.stale_after

    def _stop(self):
        self.v = 0.0
        self.cmd.publish(Twist())

    def _tick(self):
        now = self.get_clock().now()
        if not (self._fresh(self.me_t, now) and self._fresh(self.ahead_t, now)):
            if self.v != 0.0 or self.me is not None:
                self._stop()
            return
        x, y, yaw, _ = self.me
        ax, ay, _, v_ahead = self.ahead

        if len(self.trail) == 0:
            self.trail.seed_line((x, y), (ax, ay))
        self.trail.append(ax, ay)

        s, xte, self.hint = self.trail.project(x, y, self.hint)
        gap = self.trail.end_s - s

        v_target = gap_speed(v_ahead, gap, self.gap_desired, self.k_gap, self.v_max, self.min_gap)
        self.v = 0.0 if v_target == 0.0 else ramp(self.v, v_target, self.accel, self.dt)

        ld = lookahead(self.v, self.ld_min, self.ld_gain, self.ld_max)
        tx, ty = self.trail.point_at(s + ld)
        kappa = clamp_curvature(pure_pursuit_curvature(x, y, yaw, tx, ty), self.min_radius)

        out = Twist()
        out.linear.x = self.v
        out.angular.z = self.v * kappa
        self.cmd.publish(out)
        self.gap_pub.publish(Float64(data=gap))
        self.xte_pub.publish(Float64(data=xte))

        self.hint -= self.trail.prune(s - 5.0)
        self.hint = max(0, self.hint)

        if self.last_log is None or (now - self.last_log).nanoseconds > 5e9:
            self.last_log = now
            self.get_logger().info(f"gap {gap:5.1f} m  v {self.v:4.2f} m/s  fora da trilha {xte:+.2f} m")


def main():
    rclpy.init()
    node = Follower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
