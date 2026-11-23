"""
Protocolo do experimento: o espaçamento da fila, o fim da rota e o progresso contínuo.

Este módulo é Python puro, sem ``rclpy``, e roda nos testes (``test/test_profile.py``); o nó
``route_driver`` em ``node.py`` o usa.
"""

import math

# Balanços do URDF em relação ao base_link, (traseiro, dianteiro) em m.
OVERHANGS = {'autonomous_tractor': (1.15, 2.81), 'pickup': (1.12, 4.10)}
# Tolerância no fim da última curva, em m, para a rota que termina nela.
END_MARGIN = 0.5


def spacing(gap=4.0, leader='pickup', follower='autonomous_tractor'):
    """
    Dá a distância entre os ``base_link`` que deixa ``gap`` metros entre os para-choques.

    Args:
        gap: Folga entre o para-choque traseiro do líder e o dianteiro do seguidor, em m.
        leader: O veículo da frente, uma chave de ``OVERHANGS``.
        follower: O veículo de trás, uma chave de ``OVERHANGS``.

    Returns:
        A distância entre os ``base_link``, em m.
    """
    return gap + OVERHANGS[leader][0] + OVERHANGS[follower][1]


def finished(route, last_s):
    """
    Diz se o último trator já passou o fim da última curva.

    Args:
        route: A ``Route``.
        last_s: Progresso do último trator ao longo da rota, em m.

    Returns:
        Verdadeiro se ``last_s`` passou do fim da última curva.
    """
    return last_s >= route.turns_end - END_MARGIN


class Progress:
    """Progresso contínuo de um veículo ao longo de uma rota fechada, em metros."""

    def __init__(self, route):
        """
        Cria o acompanhamento.

        Args:
            route: A ``Route``.
        """
        self.route = route
        self.value = None

    def update(self, x, y):
        """
        Atualiza o progresso com a posição do veículo.

        A volta é fechada e a última reta passa do início; por isso, na primeira leitura, um
        ponto projetado na segunda metade da volta (ou atrás do início) é medido pelo rumo
        inicial a partir de ``start`` e fica negativo atrás da largada. Depois disso, o
        progresso é desembrulhado: cresce sem saltos de uma volta.

        Args:
            x: Posição em X, em m.
            y: Posição em Y, em m.

        Returns:
            O progresso, em m.
        """
        route = self.route
        s, _ = route.project(x, y)
        if self.value is None:
            sx, sy, heading = route.start
            along = (x - sx) * math.cos(heading) + (y - sy) * math.sin(heading)
            self.value = along if s > route.length / 2 or along < 0.0 else s
        else:
            self.value = s + round((self.value - s) / route.length) * route.length
        return self.value
