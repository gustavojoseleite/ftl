"""
Os trechos de uma rota: cada reta e cada curva, na ordem em que o líder as percorre.

Python puro, sem ``rclpy``. Um trecho é nomeado ``straight_<n>`` ou ``curve_<n>``, com ``n``
contado por tipo desde 0.
"""

import math
from typing import NamedTuple

import numpy as np


class Segment(NamedTuple):
    """Um trecho da rota, de ``s0`` a ``s1`` metros do início."""

    kind: str
    index: int
    s0: float
    s1: float
    turn: int

    @property
    def name(self):
        """O nome do trecho, como ``straight_0`` ou ``curve_3``."""
        return f'{self.kind}_{self.index}'


def segments(route):
    """
    Separa uma rota em retas e curvas.

    Args:
        route: Uma ``Route`` do ``ftl_route_driver``, com as peças em ``pieces``.

    Returns:
        A lista de ``Segment``; ``turn`` é +1 nas curvas à esquerda, -1 à direita e 0 nas retas.
    """
    parts = []
    counts = {'straight': 0, 'curve': 0}
    s = 0.0
    for piece in route.pieces:
        if piece[0] == 'straight':
            kind, length, turn = 'straight', piece[1], 0
        else:
            kind, length, turn = ('curve', abs(math.radians(piece[1])) * piece[2],
                                  int(math.copysign(1, piece[1])))
        parts.append(Segment(kind, counts[kind], s, s + length, turn))
        counts[kind] += 1
        s += length
    return parts


def locate(parts, s):
    """
    Acha o trecho de cada posição ao longo da rota.

    Args:
        parts: A lista devolvida por ``segments``.
        s: Posições ao longo da rota, em m; fora da rota valem as pontas.

    Returns:
        Os índices em ``parts``, um por posição.
    """
    edges = np.array([part.s0 for part in parts[1:]])
    return np.searchsorted(edges, np.asarray(s, dtype=float), side='right')
