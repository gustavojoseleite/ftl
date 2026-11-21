#!/usr/bin/env python3
"""
Onde este veículo está, no referencial que o comboio inteiro compartilha.

    ros2 run ftl_follower pose_estimator --ros-args -r __ns:=/tractor_1

Roda em todos os veículos do comboio, líder incluído, e publica `ftl/state`
(`nav_msgs/Odometry`, referencial `ftl_map`): posição leste/norte do `datum`,
rumo e velocidade. Os seguidores leem daí o estado do veículo da frente, o que
nas máquinas reais seria o que viaja pelo rádio.

Como. O rumo vem da IMU, que nestes mundos informa a orientação no referencial
ENU do mundo, então não precisa de alinhamento com nada. A posição é estimada
por `dead reckoning` a partir da distância percorrida pela odometria nesse rumo,
e puxada devagar para o fix GNSS: a odometria dá suavidade entre os fixes, e o
fix dá a ausência de deriva. É a mesma fusão do `road_follow`, com a mesma
constante de tempo de quatro segundos a 5 Hz e `fix_alpha` 0,05. O fix é
informado na antena, então o deslocamento da antena em relação ao
`base_footprint` (do TF deste veículo) é descontado antes de usá-lo.

Só a odometria não basta: a de cada veículo começa onde ele foi posicionado e
deriva metros por volta. Só o GNSS também não basta: um metro de ruído a 5 Hz é
um volante que nunca para de tremer.
"""

import math

from ftl_follower.geo import LocalFrame, yaw_from_quaternion
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import Buffer, TransformListener


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.frame = LocalFrame(param('datum_lat', -20.9494), param('datum_lon', -48.4794))
        self.fix_samples = param('fix_samples', 25)
        self.fix_alpha = param('fix_alpha', 0.05)
        self.base_frame = param('base_frame', 'base_footprint')
        self.gnss_frame = param('gnss_frame', 'gnss_link')
        self.map_frame = param('map_frame', 'ftl_map')

        self.yaw = None
        self.speed = 0.0
        self.last_odom = None
        self.antenna = None          # (frente, esquerda) da antena em relação à base, em metros
        self.seen = []
        self.xy = None               # posição da base no referencial comum, depois de conhecida

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Odometry, 'ftl/state', 10)
        self.create_subscription(Imu, 'imu/data', self._on_imu, 50)
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)
        self.create_subscription(NavSatFix, 'gnss/fix', self._on_fix, 10)
        self.create_timer(1.0 / param('rate', 20.0), self._publish)
        self.create_timer(0.5, self._lookup_antenna)

    def _lookup_antenna(self):
        if self.antenna is not None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.gnss_frame, rclpy.time.Time())
        except Exception:                                  # noqa: BLE001 — ainda não existe
            return
        t = tf.transform.translation
        self.antenna = (t.x, t.y)
        self.get_logger().info(
            f'antena GNSS a {t.x:.3f} m à frente, '
            f'{t.y:.3f} m à esquerda da base')

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
            # O comboio começa parado: a média dos primeiros fixes dá um ponto
            # inicial com uma fração do ruído de um fix.
            self.seen.append(meas)
            if len(self.seen) >= self.fix_samples:
                self.xy = (sum(m[0] for m in self.seen) / len(self.seen),
                           sum(m[1] for m in self.seen) / len(self.seen))
                self.get_logger().info(
                    f'posição inicial ({self.xy[0]:.2f}, {self.xy[1]:.2f}) m do datum, '
                    f'rumo {math.degrees(self.yaw):.1f} graus')
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


if __name__ == '__main__':
    main()
