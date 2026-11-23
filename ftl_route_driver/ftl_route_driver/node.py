"""O nó ``route_driver``: um invólucro fino de ROS em torno das rotas e do perfil."""

import math

from ftl_route_driver.profile import speed_at, speed_profile
from ftl_route_driver.protocol import finished, Progress
from ftl_route_driver.routes import route as load_route
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64

RATE = 20.0  # frequência do controle, em Hz
MAX_CURVATURE = 0.6  # limite da curvatura comandada, em 1/m
RAMP = 1.0  # rampa da velocidade, em m/s²


def yaw_of(pose):
    """Devolve o rumo, em rad, de uma ``geometry_msgs/Pose``."""
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _descriptor(text, low, high):
    return ParameterDescriptor(
        description=text,
        floating_point_range=[FloatingPointRange(from_value=low, to_value=high, step=0.0)])


class RouteDriverNode(Node):
    """Dirige o líder pela rota do mundo e publica o erro lateral e o progresso."""

    def __init__(self):
        super().__init__('route_driver')
        self.declare_parameter('route', 'citrus_farm_90', ParameterDescriptor(
            description='Rota a percorrer, igual ao nome do mundo sem .sdf. Unidade: texto. '
                        'Efeito: escolhe a geometria, o relevo e o fim da rota.'))
        self.declare_parameter('cruise_kmh', 10.8, _descriptor(
            'Velocidade de cruzeiro. Unidade: km/h. Faixa: de 1 a 40. '
            'Efeito: quanto maior, mais rápido nas retas.', 1.0, 40.0))
        self.declare_parameter('a_lat', 1.0, _descriptor(
            'Aceleração lateral máxima nas curvas. Unidade: m/s². Faixa: de 0,2 a 3. '
            'Efeito: quanto maior, mais rápido nas curvas.', 0.2, 3.0))
        self.declare_parameter('leader', 'pickup', ParameterDescriptor(
            description='Nome do líder, em /ground_truth/<nome>/pose. Unidade: texto.'))
        self.declare_parameter('last', 'tractor_1', ParameterDescriptor(
            description='Nome do último trator, em /ground_truth/<nome>/pose. Unidade: texto. '
                        'Efeito: a rota termina quando ele passa a última curva.'))
        self.route = load_route(self.get_parameter('route').value)
        cruise = self.get_parameter('cruise_kmh').value / 3.6
        self.profile = speed_profile(self.route, cruise, self.get_parameter('a_lat').value)
        self.leader_progress = Progress(self.route)
        self.last_progress = Progress(self.route)
        self.leader_s = None
        self.last_s = None
        self.pose = None
        self.speed = 0.0
        self.time = None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.error_pub = self.create_publisher(Float64, 'route/error', 10)
        self.progress_pub = self.create_publisher(Float64, 'route/progress', 10)
        for name, callback in ((self.get_parameter('leader').value, self.on_leader),
                               (self.get_parameter('last').value, self.on_last)):
            self.create_subscription(PoseStamped, f'/ground_truth/{name}/pose', callback, 10)
        self.create_timer(1.0 / RATE, self.step)

    def on_leader(self, msg):
        """Guarda a pose real do líder e o progresso dele."""
        p = msg.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose))
        self.leader_s = self.leader_progress.update(p.x, p.y)

    def on_last(self, msg):
        """Guarda o progresso do último trator."""
        p = msg.pose.position
        self.last_s = self.last_progress.update(p.x, p.y)

    def step(self):
        """Calcula e publica o comando do líder."""
        if self.pose is None:
            return
        x, y, yaw = self.pose
        _, lateral = self.route.project(x, y)
        self.error_pub.publish(Float64(data=lateral))
        self.progress_pub.publish(Float64(data=self.leader_s))
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.time is None else min(now - self.time, 0.5)
        self.time = now
        done = self.last_s is not None and finished(self.route, self.last_s)
        target = 0.0 if done else speed_at(self.profile, self.leader_s)
        delta = RAMP * dt
        self.speed += max(-delta, min(delta, target - self.speed))
        lookahead = max(4.0, self.speed)
        tx, ty, _ = self.route.at(self.leader_s + lookahead)
        alpha = math.atan2(ty - y, tx - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        curvature = 2.0 * math.sin(alpha) / lookahead
        curvature = max(-MAX_CURVATURE, min(MAX_CURVATURE, curvature))
        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = self.speed * curvature
        self.pub.publish(cmd)


def main():
    """Executa o nó até o desligamento."""
    rclpy.init()
    node = RouteDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
