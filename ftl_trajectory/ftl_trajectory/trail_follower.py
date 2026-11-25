"""
Segue o trilho único do comboio, a uma distância fixa atrás do veículo da frente.

    ros2 run ftl_trajectory trail_follower --ros-args -r __ns:=/tractor_2
        -p ahead:=/tractor_1 -p trail:=/tractor_1/ftl/trail

É o seguidor da proposta inicial do artigo (plano, seção 8.7): o mesmo *pure pursuit* e a mesma
lei de velocidade da B0 (``ftl_follower_pure_pursuit.control``), com os mesmos parâmetros, mas
com outra referência.

- **Rumo, pelo trilho do líder.** Todos os tratores esterçam para um ponto do mesmo trilho, o
  ``ftl/trail`` que o primeiro trator publica, a uma distância de visada adiante. Nenhum trator
  segue o caminho de outro, então o erro de um não vira a referência do seguinte.
- **Distância, pelo veículo da frente.** O espaçamento é a diferença entre os comprimentos de arco
  das projeções do veículo da frente e deste no trilho. Antes do começo do trilho e depois do fim
  dele, a projeção segue o trecho da ponta, o que mede o trator que ainda não chegou ao trilho e
  o líder à frente do último ponto.

Para sem trilho (menos de dois pontos) ou com um dos estados obsoleto. Publica, como a B0,
``cmd_vel``, ``ftl/gap`` (m ao longo do trilho) e ``ftl/cross_track`` (m fora do trilho, positivo
à esquerda).
"""

import math

from ftl_follower_pure_pursuit.control import (clamp_curvature, gap_speed, lookahead,
                                               pure_pursuit_curvature, ramp)
from ftl_trajectory.path import SharedPath
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64


def _state(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return p.x, p.y, yaw, msg.twist.twist.linear.x


def command(path, me, ahead, hints, params, v_now, dt):
    """
    Calcula o comando de um ciclo do seguidor.

    Args:
        path: O trilho (``SharedPath``), com pelo menos dois pontos.
        me: ``(x, y, rumo, v)`` deste veículo.
        ahead: ``(x, y, rumo, v)`` do veículo da frente.
        hints: ``{'me': s, 'ahead': s}`` das projeções anteriores; atualizado aqui.
        params: Dicionário com ``gap``, ``min_gap``, ``k_gap``, ``v_max``, ``accel``,
            ``lookahead_min``, ``lookahead_gain``, ``lookahead_max`` e ``min_radius``.
        v_now: Velocidade comandada no ciclo anterior, em m/s.
        dt: Período do ciclo, em s.

    Returns:
        ``(v, ômega, gap, lateral)``: velocidade e velocidade angular comandadas, o espaçamento
        ao longo do trilho e a distância deste veículo ao trilho.
    """
    x, y, yaw, _ = me
    s, lateral = path.project(x, y, hints.get('me'))
    s_ahead, _ = path.project(ahead[0], ahead[1], hints.get('ahead'))
    hints['me'], hints['ahead'] = s, s_ahead
    gap = s_ahead - s
    target = gap_speed(ahead[3], gap, params['gap'], params['k_gap'], params['v_max'],
                       params['min_gap'])
    v = 0.0 if target == 0.0 else ramp(v_now, target, params['accel'], dt)
    ld = lookahead(v, params['lookahead_min'], params['lookahead_gain'],
                   params['lookahead_max'])
    tx, ty = path.point_at(s + ld)
    kappa = clamp_curvature(pure_pursuit_curvature(x, y, yaw, tx, ty), params['min_radius'])
    return v, v * kappa, gap, lateral


class TrailFollower(Node):
    """Segue o trilho único e guarda a distância do veículo da frente."""

    DEFAULTS = {'gap': 10.0, 'min_gap': 5.0, 'k_gap': 0.4, 'v_max': 4.0, 'accel': 1.0,
                'lookahead_min': 4.0, 'lookahead_gain': 1.0, 'lookahead_max': 10.0,
                'min_radius': 3.2}

    def __init__(self):
        """Lê os parâmetros e liga os tópicos."""
        super().__init__('follower')
        ahead = self.declare_parameter('ahead', '').value
        trail = self.declare_parameter('trail', '').value
        if not ahead or not trail:
            raise SystemExit("trail_follower: os parâmetros 'ahead' e 'trail' são obrigatórios")
        self.params = {k: self.declare_parameter(k, v).value for k, v in self.DEFAULTS.items()}
        self.stale_after = self.declare_parameter('stale_after', 1.0).value
        self.dt = 1.0 / self.declare_parameter('rate', 20.0).value
        self.path = None
        self.me = self.ahead = None
        self.me_t = self.ahead_t = None
        self.hints = {}
        self.v = 0.0
        self.last_log = None
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gap_pub = self.create_publisher(Float64, 'ftl/gap', 10)
        self.xte_pub = self.create_publisher(Float64, 'ftl/cross_track', 10)
        self.create_subscription(Odometry, 'ftl/state', self._on_me, 10)
        self.create_subscription(Odometry, f"{ahead.rstrip('/')}/ftl/state", self._on_ahead, 10)
        self.create_subscription(Path, trail, self._on_trail, 10)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f'following the trail {trail}, {self.params["gap"]:.1f} m '
                               f'behind {ahead}')

    def _on_me(self, msg):
        self.me, self.me_t = _state(msg), self.get_clock().now()

    def _on_ahead(self, msg):
        self.ahead, self.ahead_t = _state(msg), self.get_clock().now()

    def _on_trail(self, msg):
        path = SharedPath((p.pose.position.x, p.pose.position.y) for p in msg.poses)
        if len(path) >= 2:
            self.path = path

    def _fresh(self, t, now):
        return t is not None and (now - t).nanoseconds * 1e-9 < self.stale_after

    def _tick(self):
        now = self.get_clock().now()
        ready = (self.path is not None and self._fresh(self.me_t, now)
                 and self._fresh(self.ahead_t, now)
                 and all(math.isfinite(v) for v in self.me[:3] + self.ahead[:3]))
        if not ready:
            if self.v != 0.0 or self.me is not None:
                self.v = 0.0
                self.cmd.publish(Twist())
            return
        self.v, omega, gap, lateral = command(self.path, self.me, self.ahead, self.hints,
                                              self.params, self.v, self.dt)
        out = Twist()
        out.linear.x, out.angular.z = self.v, omega
        self.cmd.publish(out)
        self.gap_pub.publish(Float64(data=gap))
        self.xte_pub.publish(Float64(data=lateral))
        if self.last_log is None or (now - self.last_log).nanoseconds > 5e9:
            self.last_log = now
            self.get_logger().info(f'gap {gap:5.1f} m  v {self.v:4.2f} m/s  '
                                   f'off the trail {lateral:+.2f} m')


def main():
    """Roda o nó."""
    rclpy.init()
    node = TrailFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
