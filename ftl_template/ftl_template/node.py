"""O nó ``limiter``: um invólucro fino de ROS em torno de ``logic.limit_rate``."""

from ftl_template.logic import limit_rate
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64


class LimiterNode(Node):
    """Restringe a velocidade de variação do sinal de ``input`` e o publica em ``output``."""

    def __init__(self):
        super().__init__('limiter')
        self.declare_parameter(
            'max_rate', 1.0,
            ParameterDescriptor(
                description=(
                    'Variação máxima da saída por segundo. '
                    'Unidade: u/s (unidade do sinal por segundo). '
                    'Efeito: quanto menor, mais suave e lenta a saída; '
                    '0 mantém a saída parada.'),
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, to_value=100.0, step=0.0)]))
        self.last_output = None
        self.last_time = None
        self.pub = self.create_publisher(Float64, 'output', 10)
        self.create_subscription(Float64, 'input', self.on_input, 10)

    def on_input(self, msg):
        """Aplica o limite à nova amostra e a publica."""
        now = self.get_clock().now()
        if self.last_output is None:
            out = msg.data
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt <= 0.0:
                out = self.last_output
            else:
                max_rate = self.get_parameter('max_rate').value
                out = limit_rate(self.last_output, msg.data, max_rate, dt)
        self.last_output, self.last_time = out, now
        self.pub.publish(Float64(data=out))


def main():
    """Executa o nó até o desligamento."""
    rclpy.init()
    node = LimiterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
