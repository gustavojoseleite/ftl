"""
Limitador de taxa: o exemplo do pacote modelo.

Este módulo é Python puro, sem ``rclpy``, e se testa sem o ROS (ver
``test/test_logic.py``). Um pacote novo troca esta lógica pela sua e mantém a
mesma divisão: a lógica aqui e um nó fino em ``node.py``.

As linhas de fórmula levam uma citação no formato ``# Eq. X.Y``. Num pacote
real, ali vai o número da equação na dissertação.
"""


def limit_rate(previous, target, max_rate, dt):
    """
    Aproxima ``previous`` de ``target`` em no máximo ``max_rate * dt``.

    Args:
        previous: Última saída, na unidade do sinal (u).
        target: Valor desejado, na unidade do sinal (u).
        max_rate: Maior variação permitida por segundo, em u/s; zero ou mais.
        dt: Tempo desde a última saída, em segundos; positivo.

    Returns:
        A nova saída, em u.

    Raises:
        ValueError: Se ``dt`` não for positivo ou ``max_rate`` for negativo.
    """
    if dt <= 0.0:
        raise ValueError(f'dt deve ser positivo, recebido {dt}')
    if max_rate < 0.0:
        raise ValueError(f'max_rate não pode ser negativo, recebido {max_rate}')
    step = max_rate * dt
    return previous + max(-step, min(step, target - previous))  # Eq. X.Y
