"""
O UKF de um veículo com entradas sintéticas e o relógio real (F3.1, R1 a R3).

`ros2 launch ftl_localization localization.launch.py namespace:=veh` sobe o `gnss_odometry` e o
`ukf` em `/veh`. O teste faz o papel do veículo: um TF estático `base_footprint -> gnss_link`
com a antena 1,075 m à frente, a IMU a 20 Hz com rumo leste, a odometria a 20 Hz a 1 m/s e o fix
GNSS a 1 Hz com σ de 2 cm, todos coerentes com o veículo andando para leste a partir do `datum`.

Fases:
1. 25 s com GNSS: `ftl/state` a pelo menos 15 Hz, em `ftl_map`, filho `base_footprint`; no fim, a
   posição a menos de 0,3 m da verdade.
2. 15 s sem GNSS: a variância de x não cai e termina pelo menos 4 vezes maior que no fim da fase 1.
3. 10 s com GNSS de volta: a variância de x termina menor que um quarto da do fim da fase 2.
"""
import math
import os
import random
import signal
import subprocess
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage

DATUM = (-20.9494, -48.4794)
EARTH = 6378137.0
ANTENNA = 1.075
SPEED = 1.0
RATE = 20.0


def _fix_at(east, north):
    lat = DATUM[0] + math.degrees(north / EARTH)
    lon = DATUM[1] + math.degrees(east / (EARTH * math.cos(math.radians(DATUM[0]))))
    return lat, lon


class Vehicle:
    """O veículo sintético: publica os sensores e guarda o que o UKF publica."""

    def __init__(self, node):
        self.node = node
        self.start = time.monotonic()
        self.states = []
        static = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static = node.create_publisher(TFMessage, '/veh/tf_static', static)
        self.imu = node.create_publisher(Imu, '/veh/imu/data', 10)
        self.odom = node.create_publisher(Odometry, '/veh/odom', 10)
        self.fix = node.create_publisher(NavSatFix, '/veh/gnss/fix', 10)
        node.create_subscription(Odometry, '/veh/ftl/state', self._on_state, 50)
        self.last_fix = -1.0
        self._antenna()

    def _antenna(self):
        tf = TransformStamped()
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id = 'base_footprint'
        tf.child_frame_id = 'gnss_link'
        tf.transform.translation.x = ANTENNA
        tf.transform.translation.z = 2.685
        tf.transform.rotation.w = 1.0
        self.tf_static.publish(TFMessage(transforms=[tf]))

    def _on_state(self, msg):
        self.states.append((time.monotonic(), msg))

    def elapsed(self):
        return time.monotonic() - self.start

    def step(self, gnss):
        t = self.elapsed()
        stamp = self.node.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'base_footprint'
        imu.orientation.w = 1.0
        imu.orientation_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        imu.angular_velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        imu.linear_acceleration_covariance = [1e-2, 0.0, 0.0, 0.0, 1e-2, 0.0, 0.0, 0.0, 1e-2]
        self.imu.publish(imu)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = SPEED * t
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = SPEED
        odom.twist.covariance = [0.0] * 36
        for i in range(6):
            odom.twist.covariance[i * 7] = 1e-3
        self.odom.publish(odom)
        if gnss and t - self.last_fix >= 1.0:
            self.last_fix = t
            fix = NavSatFix()
            fix.header.stamp, fix.header.frame_id = stamp, 'gnss_link'
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.status.service = NavSatStatus.SERVICE_GPS
            fix.latitude, fix.longitude = _fix_at(SPEED * t + ANTENNA, 0.0)
            fix.altitude = 0.0
            fix.position_covariance = [4e-4, 0.0, 0.0, 0.0, 4e-4, 0.0, 0.0, 0.0, 1.6e-3]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.fix.publish(fix)

    def run(self, seconds, gnss):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.step(gnss)
            deadline = time.monotonic() + 1.0 / RATE
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.005)

    def variance_x(self):
        return self.states[-1][1].pose.covariance[0]


def test_ukf_state_and_covariance():
    domain = str(random.randint(30, 99))
    os.environ['ROS_DOMAIN_ID'] = domain
    launch = subprocess.Popen(
        ['ros2', 'launch', 'ftl_localization', 'localization.launch.py', 'namespace:=veh'],
        env=dict(os.environ, ROS_DOMAIN_ID=domain), start_new_session=True)
    rclpy.init()
    try:
        node = rclpy.create_node('synthetic_vehicle')
        vehicle = Vehicle(node)
        vehicle.run(25.0, gnss=True)
        assert vehicle.states, 'o UKF não publicou ftl/state'
        recent = [m for t, m in vehicle.states if t > time.monotonic() - 10.0]
        assert len(recent) >= 150, f'{len(recent)} mensagens em 10 s'
        last = vehicle.states[-1][1]
        assert last.header.frame_id == 'ftl_map'
        assert last.child_frame_id == 'base_footprint'
        assert last.pose.covariance[0] > 0.0 and last.pose.covariance[7] > 0.0
        truth = SPEED * vehicle.elapsed()
        p = last.pose.pose.position
        assert math.hypot(p.x - truth, p.y) < 0.3, (p.x, p.y, truth)

        with_gnss = vehicle.variance_x()
        mark = len(vehicle.states)
        vehicle.run(15.0, gnss=False)
        variances = [m.pose.covariance[0] for _, m in vehicle.states[mark:]]
        assert all(b >= a * 0.999 for a, b in zip(variances, variances[1:])), \
            'a variância caiu sem GNSS'
        without = vehicle.variance_x()
        assert without >= 4.0 * with_gnss, (with_gnss, without)

        vehicle.run(10.0, gnss=True)
        assert vehicle.variance_x() < without / 4.0, (without, vehicle.variance_x())
        node.destroy_node()
    finally:
        rclpy.shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
