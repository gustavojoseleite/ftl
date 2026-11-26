"""
Constrói o trilho único do comboio a partir do líder e o publica para todos os tratores.

    ros2 run ftl_trajectory trail_publisher --ros-args -r __ns:=/tractor_1 -p leader:=/pickup

Roda no primeiro trator, que é quem percebe o líder (seção 4.3.1 da monografia). Lê o
``ftl/state`` do líder, amostra as posições dele no espaço (``AdaptiveSampler``), suaviza o vetor
uma vez só com o CCMA (``Ccma``) e publica ``ftl/trail`` (``nav_msgs/Path``, no referencial do
estado do líder) a ``rate`` Hz. O trilho publicado são os pontos corrigidos finais seguidos dos
pontos amostrados que ainda não têm correção final, na ponta junto ao líder. Os pontos finais
nunca mudam, então o comprimento de arco de um ponto do trilho é o mesmo em toda mensagem.

Um salto do estado do líder maior que ``max_jump`` (10 m) entre duas mensagens não é movimento,
é a localização que acabou de se acertar: o trilho recomeça a partir dali.

O líder não anda de ré. Uma posição mais de ``max_back`` (0,5 m) atrás da última posição aceita,
no rumo do líder, é a estimativa recuando (o UKF faz isso na arrancada) e fica fora do trilho.
Sem isso o trilho dobra sobre si mesmo e quem está na dobra não sabe mais para que lado seguir.

Parâmetros: ``leader`` (namespace do líder, obrigatório), ``d_base`` (1,0 m), ``d_min``
(0,25 m), ``dpsi_max_deg`` (3,0°), ``w_ma`` (5 pontos), ``w_cc`` (3 pontos), ``max_jump``
(10 m), ``max_back`` (0,5 m) e ``rate`` (5 Hz).
"""

import math

from ftl_trajectory.ccma import Ccma
from ftl_trajectory.sampling import AdaptiveSampler, behind
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TrailPublisher(Node):
    """Amostra o líder, aplica o CCMA e publica o trilho."""

    def __init__(self):
        """Lê os parâmetros e liga os tópicos."""
        super().__init__('trail_publisher')

        def param(name, default):
            return self.declare_parameter(name, default).value

        leader = param('leader', '')
        if not leader:
            raise SystemExit("trail_publisher: o parâmetro 'leader' é obrigatório")
        sampling = (param('d_base', 1.0), param('d_min', 0.25),
                    math.radians(param('dpsi_max_deg', 3.0)))
        windows = (param('w_ma', 5), param('w_cc', 3))
        self.make_sampler = lambda: AdaptiveSampler(*sampling)  # noqa: E731
        self.make_filter = lambda: Ccma(*windows)  # noqa: E731
        self.sampler, self.filter = self.make_sampler(), self.make_filter()
        self.max_jump = param('max_jump', 10.0)
        self.max_back = param('max_back', 0.5)
        self.last = self.anchor = None
        self.frame = None
        self.publisher = self.create_publisher(Path, 'ftl/trail', 10)
        self.create_subscription(Odometry, f"{leader.rstrip('/')}/ftl/state", self._on_leader,
                                 10)
        self.create_timer(1.0 / param('rate', 5.0), self._publish)
        self.get_logger().info(f'building the trail from {leader}')

    def _on_leader(self, msg):
        pose = msg.pose.pose
        if not all(math.isfinite(v) for v in (pose.position.x, pose.position.y)):
            return
        self.frame = msg.header.frame_id
        here = (pose.position.x, pose.position.y)
        if self.last is not None and math.dist(self.last, here) > self.max_jump:
            self.get_logger().warn(f'leader jumped {math.dist(self.last, here):.1f} m, '
                                   'restarting the trail')
            self.sampler, self.filter = self.make_sampler(), self.make_filter()
            self.anchor = None
        self.last = here
        yaw = _yaw(pose.orientation)
        if self.anchor is not None and behind(self.anchor, *here, yaw, self.max_back):
            self.get_logger().warn('leader moved backwards, point left out of the trail',
                                   throttle_duration_sec=5.0)
            return
        self.anchor = here
        if self.sampler.offer(pose.position.x, pose.position.y, yaw):
            self.filter.add(pose.position.x, pose.position.y)

    def _publish(self):
        if self.frame is None:
            return
        self.filter.update()
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame
        for x, y in self.filter.smoothed + self.filter.pending():
            pose = PoseStamped()
            pose.header = out.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            out.poses.append(pose)
        self.publisher.publish(out)


def main():
    """Roda o nó."""
    rclpy.init()
    node = TrailPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
