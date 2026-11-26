"""
Amostragem espacial adaptativa das posições do veículo-alvo (F6.1, R1 e R3).

Um ponto novo entra no vetor de *waypoints* quando o veículo andou ``d_base`` metros desde o
último ponto, ou quando o rumo mudou pelo menos ``dpsi_max`` desde o último ponto e ele já andou
``d_min`` metros. Numa curva de raio ``R``, o rumo muda ``Δs / R`` a cada ``Δs`` metros, então o
espaçamento é::

    Δs = max(d_min, min(d_base, R · dpsi_max))

e a densidade de pontos cresce com a curvatura de forma explícita: ``1 / Δs`` pontos por metro,
de ``1 / d_base`` nas retas até ``1 / d_min`` nas curvas mais fechadas. Parado, o veículo não
anda e o vetor não cresce.
"""

import math


def wrap(angle):
    """Leva um ângulo, em rad, para o intervalo ``(-π, π]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def spacing(radius, d_base, d_min, dpsi_max):
    """
    Dá o espaçamento entre pontos numa curva de raio constante.

    Args:
        radius: Raio da curva, em m; ``math.inf`` numa reta.
        d_base: Espaçamento nas retas, em m.
        d_min: Espaçamento mínimo, em m.
        dpsi_max: Mudança de rumo que antecipa um ponto, em rad.

    Returns:
        O espaçamento, em m.
    """
    return max(d_min, min(d_base, radius * dpsi_max))


def behind(anchor, x, y, yaw, tolerance):
    """
    Diz se uma posição ficou para trás de outra, no rumo do veículo.

    Args:
        anchor: ``(x, y)`` da última posição aceita, em m.
        x: Posição nova, em m.
        y: Posição nova, em m.
        yaw: Rumo do veículo na posição nova, em rad.
        tolerance: Quanto a posição pode recuar sem contar como recuo, em m.

    Returns:
        ``True`` se a posição nova está mais de ``tolerance`` atrás de ``anchor``.
    """
    ahead = (x - anchor[0]) * math.cos(yaw) + (y - anchor[1]) * math.sin(yaw)
    return ahead < -tolerance


class AdaptiveSampler:
    """Decide quais posições do veículo-alvo entram no vetor de *waypoints*."""

    def __init__(self, d_base=1.0, d_min=0.25, dpsi_max=math.radians(3.0)):
        """
        Guarda os parâmetros da amostragem.

        Args:
            d_base: Espaçamento nas retas, em m.
            d_min: Distância mínima entre dois pontos, em m.
            dpsi_max: Mudança de rumo, em rad, que antecipa um ponto.
        """
        if not 0.0 < d_min <= d_base:
            raise ValueError('é preciso 0 < d_min <= d_base')
        if dpsi_max <= 0.0:
            raise ValueError('dpsi_max deve ser positivo')
        self.d_base, self.d_min, self.dpsi_max = d_base, d_min, dpsi_max
        self.last = None

    def offer(self, x, y, yaw):
        """
        Oferece uma posição do veículo-alvo.

        Args:
            x: Posição em X, em m.
            y: Posição em Y, em m.
            yaw: Rumo do veículo, em rad.

        Returns:
            ``True`` se a posição entrou no vetor.
        """
        if self.last is None:
            self.last = (x, y, yaw)
            return True
        lx, ly, lyaw = self.last
        ds = math.hypot(x - lx, y - ly)
        turned = abs(wrap(yaw - lyaw)) >= self.dpsi_max
        if ds >= self.d_base or (turned and ds >= self.d_min):
            self.last = (x, y, yaw)
            return True
        return False
