"""
Converte o fix GNSS em posição da base no referencial comum, para o UKF.

    gnss/fix + imu/data + TF base_footprint -> gnss_link  ->  odometry/gps  (+ set_pose)
    ftl/state_filter (saída do UKF)                       ->  ftl/state, depois da partida

Faz o papel do ``navsat_transform`` do ``robot_localization`` com a mesma conversão do
``pose_estimator`` da B0 (``ftl_follower_pure_pursuit.geo.LocalFrame``): metros leste/norte a
partir do ``datum`` do comboio, num plano tangente, como o Gazebo gera o fix. O
``navsat_transform`` passa pela UTM, cuja escala deslocava o fix cerca de 0,18 m a 205 m do
``datum``, e no plano tangente dele (``use_local_cartesian``) a origem não ficava no ``datum``.

- **Antena.** A posição da base é a da antena menos o braço de alavanca ``base_footprint ->
  gnss_link`` (do TF do veículo), girado pela atitude completa da IMU (rolagem, arfagem e rumo).
  A antena do trator fica a 2,685 m do chão: num terreno com 6° de inclinação, ela sai 0,3 m do
  lugar na horizontal, e girar o braço só pelo rumo deixava esse erro na estimativa.
- **Covariância.** A do fix, quando ele a traz; senão, ``gnss_sigma_h`` (2 cm, o RTK fixo da D13)
  na horizontal e ``gnss_sigma_v`` na vertical. O GNSS do Gazebo não preenche a covariância, e
  uma medida de covariância zero leva o filtro a NaN.
- **Fixes inválidos** (sem solução, NaN ou latitude e longitude nulas, como o primeiro do
  simulador) são descartados.
- **Partida.** O UKF nasce na origem do ``ftl_map``, longe do veículo. Enquanto a saída dele
  (``ftl/state_filter``) estiver a mais de ``reset_distance`` da posição do GNSS, cada fix válido
  também vai para ``set_pose``, com o rumo da IMU, e o filtro recomeça ali.
- **Estado publicado.** O nó repassa a saída do UKF para ``ftl/state`` só depois que ela chegou
  perto do GNSS pela primeira vez. Antes disso, o estado na origem do mapa fazia o trilho começar
  lá e os seguidores andarem para lugar nenhum.
- **Saltos.** Uma amostra do UKF que exige mais de ``max_speed`` (20 m/s) desde a última repassada
  é descartada (``StepGate``). No arranque do líder, de 0 a 3 m/s num passo, o UKF soltou uma
  amostra única 19 m atrás, e o trilho recomeçava por ela. Se o salto persistir por ``max_drops``
  (10) amostras seguidas, ele é aceito; um ``set_pose`` também libera o salto seguinte.
"""

import math

from ftl_follower_pure_pursuit.geo import LocalFrame, yaw_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
import tf2_ros


def valid_fix(msg):
    """Diz se o fix tem solução e coordenadas utilizáveis."""
    if msg.status.status < NavSatStatus.STATUS_FIX:
        return False
    if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
        return False
    return not (msg.latitude == 0.0 and msg.longitude == 0.0)


def rotate(q, v):
    """Gira o vetor ``v`` pelo quaternion ``q = (x, y, z, w)``."""
    x, y, z, w = q
    vx, vy, vz = v
    # v' = v + 2w (q × v) + 2 q × (q × v)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2.0 * (w * cx + ccx), vy + 2.0 * (w * cy + ccy), vz + 2.0 * (w * cz + ccz))


def base_position(antenna_xy, attitude, lever):
    """
    Dá a posição da base a partir da posição da antena.

    Args:
        antenna_xy: ``(x, y)`` da antena no referencial comum, em m.
        attitude: Quaternion ``(x, y, z, w)`` da atitude do veículo (rolagem, arfagem e rumo).
        lever: ``(frente, esquerda, cima)`` da antena em relação à base, em m.

    Returns:
        ``(x, y)`` da base, em m.
    """
    dx, dy, _ = rotate(attitude, lever)
    return antenna_xy[0] - dx, antenna_xy[1] - dy


def horizontal_variances(msg, sigma_h, sigma_v):
    """
    Dá as variâncias ``(leste, norte, vertical)`` do fix, em m².

    Usa a covariância do fix quando ela é conhecida e não nula, e as de ``sigma_h`` e ``sigma_v``
    quando não.
    """
    cov = msg.position_covariance
    known = msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
    if known and cov[0] > 0.0 and cov[4] > 0.0:
        return cov[0], cov[4], max(cov[8], 1e-6)
    return sigma_h ** 2, sigma_h ** 2, sigma_v ** 2


class StepGate:
    """Descarta amostras isoladas do estado que pedem uma velocidade impossível."""

    def __init__(self, max_speed=20.0, max_drops=10):
        """
        Guarda os limites.

        Args:
            max_speed: Velocidade, em m/s, acima da qual um passo é um salto.
            max_drops: Quantas amostras seguidas podem ser descartadas antes de aceitar o salto.
        """
        self.max_speed, self.max_drops = max_speed, max_drops
        self.clear()

    def clear(self):
        """Esquece a última amostra: a próxima é aceita como vier."""
        self.last = None
        self.drops = 0

    def accept(self, t, x, y):
        """
        Diz se a amostra ``(x, y)`` no instante ``t``, em s, deve ser repassada.

        Returns:
            ``False`` para um salto isolado; ``True`` nos outros casos.
        """
        if not (math.isfinite(x) and math.isfinite(y)):
            return False
        if self.last is not None and self.drops < self.max_drops:
            lt, lx, ly = self.last
            step = math.hypot(x - lx, y - ly)
            if step > self.max_speed * max(t - lt, 0.05):
                self.drops += 1
                return False
        self.last, self.drops = (t, x, y), 0
        return True


class GnssOdometry(Node):
    """Publica a posição da base vinda do GNSS e reinicia o UKF na partida."""

    def __init__(self):
        """Lê os parâmetros e liga os tópicos."""
        super().__init__('gnss_odometry')

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.local = LocalFrame(param('datum_lat', -20.9494), param('datum_lon', -48.4794))
        self.sigma_h = param('gnss_sigma_h', 0.02)
        self.sigma_v = param('gnss_sigma_v', 0.04)
        self.reset_distance = param('reset_distance', 3.0)
        self.gate = StepGate(param('max_speed', 20.0), param('max_drops', 10))
        self.world = param('world_frame', 'ftl_map')
        self.base = param('base_frame', 'base_footprint')
        self.gnss = param('gnss_frame', 'gnss_link')
        self.yaw = None
        self.attitude = None
        self.state = None
        self.ready = False
        self.lever = None
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.odometry = self.create_publisher(Odometry, 'odometry/gps', 10)
        self.reset = self.create_publisher(PoseWithCovarianceStamped, 'set_pose', 10)
        self.output = self.create_publisher(Odometry, 'ftl/state', 20)
        self.create_subscription(Imu, 'imu/data', self._on_imu, 20)
        self.create_subscription(Odometry, 'ftl/state_filter', self._on_state, 20)
        self.create_subscription(NavSatFix, 'gnss/fix', self._on_fix, 10)
        self.create_timer(0.5, self._lookup_lever)

    def _lookup_lever(self):
        if self.lever is not None:
            return
        try:
            t = self.buffer.lookup_transform(self.base, self.gnss, Time()).transform
        except tf2_ros.TransformException:
            return
        self.lever = (t.translation.x, t.translation.y, t.translation.z)
        self.get_logger().info('GNSS antenna at ({:.3f}, {:.3f}, {:.3f}) m'.format(*self.lever))

    def _on_imu(self, msg):
        q = msg.orientation
        self.attitude = (q.x, q.y, q.z, q.w)
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _on_state(self, msg):
        p = msg.pose.pose.position
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if not self.ready:
            self.state = (p.x, p.y)
            return
        if self.gate.accept(t, p.x, p.y):
            self.state = (p.x, p.y)
            self.output.publish(msg)

    def _on_fix(self, msg):
        if not valid_fix(msg) or self.yaw is None or self.lever is None:
            return
        x, y = base_position(self.local.to_enu(msg.latitude, msg.longitude), self.attitude,
                             self.lever)
        var_e, var_n, var_u = horizontal_variances(msg, self.sigma_h, self.sigma_v)
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.world
        out.child_frame_id = self.base
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        out.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0], cov[7], cov[14] = var_e, var_n, var_u
        for i in (21, 28, 35):
            cov[i] = 1e6  # a orientação não vem do GNSS
        out.pose.covariance = cov
        self.odometry.publish(out)
        if self._far(x, y):
            self._reset(msg, x, y, var_e, var_n)
        elif not self.ready:
            self.ready = True
            self.get_logger().info('filter on the GNSS, publishing ftl/state')

    def _far(self, x, y):
        if self.state is None or not all(math.isfinite(v) for v in self.state):
            return True
        return math.hypot(self.state[0] - x, self.state[1] - y) > self.reset_distance

    def _reset(self, msg, x, y, var_e, var_n):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.world
        pose.pose.pose.position.x, pose.pose.pose.position.y = x, y
        pose.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        cov = [0.0] * 36
        cov[0], cov[7], cov[14] = var_e, var_n, 1e-4
        cov[21], cov[28], cov[35] = 1e-4, 1e-4, 1e-3
        pose.pose.covariance = cov
        self.reset.publish(pose)
        self.gate.clear()
        self.get_logger().info(f'filter reset to ({x:.2f}, {y:.2f}) m')


def main():
    """Roda o nó."""
    rclpy.init()
    node = GnssOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
