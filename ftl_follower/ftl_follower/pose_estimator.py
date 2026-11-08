#!/usr/bin/env python3
"""Where this vehicle is, in the frame the whole convoy shares.

    ros2 run ftl_follower pose_estimator --ros-args -r __ns:=/tractor_1

Runs on every vehicle of the convoy, leader included, and publishes
`ftl/state` (nav_msgs/Odometry, frame `ftl_map`): position east/north of the
datum, heading, and speed. Followers read the state of the vehicle ahead from
there, which on the real machines is what would travel over the radio.

How. Heading comes from the IMU, which in these worlds reports orientation in
the world's ENU frame, so it needs no alignment with anything. Position is
dead-reckoned from the odometry's distance travelled along that heading, and
pulled slowly towards the GNSS fix: odometry for smoothness between fixes, the
fix for the absence of drift. It is the fusion road_follow uses, with the same
four-second time constant at 5 Hz and fix_alpha 0.05. The fix is reported at
the antenna, so the antenna's offset from base_footprint (from this vehicle's
TF) is taken off before it is used.

Odometry alone is not enough: each vehicle's odometry starts where that
vehicle was spawned, and drifts metres per lap. GNSS alone is not enough
either: a metre of noise at 5 Hz is a wheel that never stops twitching.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import Buffer, TransformListener

from ftl_follower.geo import LocalFrame, yaw_from_quaternion


class PoseEstimator(Node):
    def __init__(self):
        super().__init__("pose_estimator")

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.frame = LocalFrame(param("datum_lat", -20.9494), param("datum_lon", -48.4794))
        self.fix_samples = param("fix_samples", 25)
        self.fix_alpha = param("fix_alpha", 0.05)
        self.base_frame = param("base_frame", "base_footprint")
        self.gnss_frame = param("gnss_frame", "gnss_link")
        self.map_frame = param("map_frame", "ftl_map")

        self.yaw = None
        self.speed = 0.0
        self.last_odom = None
        self.antenna = None          # (forward, left) of the antenna from base, metres
        self.seen = []
        self.xy = None               # base position in the shared frame, once known

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Odometry, "ftl/state", 10)
        self.create_subscription(Imu, "imu/data", self._on_imu, 50)
        self.create_subscription(Odometry, "odom", self._on_odom, 20)
        self.create_subscription(NavSatFix, "gnss/fix", self._on_fix, 10)
        self.create_timer(1.0 / param("rate", 20.0), self._publish)
        self.create_timer(0.5, self._lookup_antenna)

    def _lookup_antenna(self):
        if self.antenna is not None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.gnss_frame, rclpy.time.Time())
        except Exception:                                  # noqa: BLE001 — not there yet
            return
        t = tf.transform.translation
        self.antenna = (t.x, t.y)
        self.get_logger().info(f"antena GNSS a {t.x:.3f} m à frente, {t.y:.3f} m à esquerda da base")

    def _on_imu(self, msg):
        q = msg.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.speed = msg.twist.twist.linear.x
        if self.last_odom is not None and self.xy is not None and self.yaw is not None:
            d = math.hypot(p.x - self.last_odom[0], p.y - self.last_odom[1])
            d = math.copysign(d, self.speed) if abs(self.speed) > 1e-3 else d
            self.xy = (self.xy[0] + d * math.cos(self.yaw), self.xy[1] + d * math.sin(self.yaw))
        self.last_odom = (p.x, p.y)

    def _base_from_fix(self, msg):
        e, n = self.frame.to_enu(msg.latitude, msg.longitude)
        fwd, left = self.antenna
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return e - (c * fwd - s * left), n - (s * fwd + c * left)

    def _on_fix(self, msg):
        if math.isnan(msg.latitude) or self.yaw is None or self.antenna is None:
            return
        meas = self._base_from_fix(msg)
        if self.xy is None:
            # The convoy starts standing still: average the first fixes for a
            # starting point with a fraction of one fix's noise.
            self.seen.append(meas)
            if len(self.seen) >= self.fix_samples:
                self.xy = (sum(m[0] for m in self.seen) / len(self.seen),
                           sum(m[1] for m in self.seen) / len(self.seen))
                self.get_logger().info(
                    f"posição inicial ({self.xy[0]:.2f}, {self.xy[1]:.2f}) m do datum, "
                    f"rumo {math.degrees(self.yaw):.1f} graus")
            return
        a = self.fix_alpha
        self.xy = (self.xy[0] + (meas[0] - self.xy[0]) * a,
                   self.xy[1] + (meas[1] - self.xy[1]) * a)

    def _publish(self):
        if self.xy is None or self.yaw is None:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.xy
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        msg.twist.twist.linear.x = self.speed
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PoseEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
