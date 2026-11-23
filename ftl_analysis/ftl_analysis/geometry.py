"""
Geometria da análise: o ponto de referência de cada veículo e a projeção num caminho.

Python puro, sem ``rclpy``. O ponto de referência é a antena projetada no chão (seção 5.6 do
plano); o caminho do líder é uma poligonal, e o erro lateral é a distância com sinal a ela.
"""

import math

import numpy as np
from scipy.spatial import cKDTree

# Distância da antena à frente da base, em m, por tipo de veículo.
ANTENNA = {'autonomous_tractor': 1.075, 'pickup': 1.545}
MIN_STEP = 0.02  # pontos mais próximos que isto, em m, do anterior são descartados


def reference_point(x, y, yaw, robot):
    """
    Dá o ponto de referência de um veículo: a antena projetada no chão.

    Args:
        x: Posição da base em X, em m.
        y: Posição da base em Y, em m.
        yaw: Rumo da base, em rad.
        robot: Tipo do veículo, uma chave de ``ANTENNA``.

    Returns:
        ``(x, y)`` da antena, em m.
    """
    distance = ANTENNA[robot]
    return x + distance * math.cos(yaw), y + distance * math.sin(yaw)


class Polyline:
    """Uma poligonal com comprimento de arco e projeção de pontos."""

    def __init__(self, xy):
        """
        Monta a poligonal, sem os pontos repetidos ou quase parados.

        Args:
            xy: Matriz ``(n, 2)`` de pontos, em m.

        Raises:
            ValueError: Se sobrarem menos de dois pontos distintos.
        """
        points = np.asarray(xy, dtype=float).reshape(-1, 2)
        kept = [points[0]]
        for point in points[1:]:
            if math.hypot(point[0] - kept[-1][0], point[1] - kept[-1][1]) >= MIN_STEP:
                kept.append(point)
        if len(kept) < 2:
            raise ValueError('a polyline needs at least two distinct points')
        self._xy = np.array(kept)
        self._d = np.diff(self._xy, axis=0)
        self._len = np.hypot(self._d[:, 0], self._d[:, 1])
        self._s = np.concatenate([[0.0], np.cumsum(self._len)])
        self._tree = cKDTree(self._xy)
        self.length = float(self._s[-1])

    def project(self, x, y):
        """
        Projeta um ponto na poligonal.

        Um ponto além das pontas cai na ponta, e então ``s`` vale 0 ou ``length``.

        Args:
            x: Posição em X, em m.
            y: Posição em Y, em m.

        Returns:
            ``(s, lateral)``: o arco até a projeção e a distância com sinal, ambos em m, positiva
            à esquerda do sentido da poligonal.
        """
        _, nearest = self._tree.query((x, y))
        best = None
        for first in (int(nearest) - 1, int(nearest)):
            if first < 0 or first >= len(self._len):
                continue
            ax, ay = self._xy[first]
            dx, dy = self._d[first]
            t = min(max(((x - ax) * dx + (y - ay) * dy) / self._len[first] ** 2, 0.0), 1.0)
            distance = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            if best is None or distance < best[0]:
                best = (distance, first, t)
        _, first, t = best
        dx, dy = self._d[first]
        ax, ay = self._xy[first]
        lateral = (dx * (y - ay) - dy * (x - ax)) / self._len[first]
        return float(self._s[first] + t * self._len[first]), float(lateral)
