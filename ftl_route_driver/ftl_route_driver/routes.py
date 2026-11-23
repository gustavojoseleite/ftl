"""
Rotas do líder: a geometria que o motorista percorre e o relevo do mundo.

Este módulo é Python puro, sem ``rclpy``, e roda nos testes (``test/test_routes.py``); o nó
``route_driver`` em ``node.py`` o usa. As rotas ``citrus_farm_30``, ``_60`` e ``_90`` são os
perímetros dos mundos de mesmo nome, pela mesma fórmula do gerador dos mundos
(``polygon_perimeter``), e ``citrus_farm`` é a volta ``lap`` do ``road_follow``.
"""

import math

import numpy as np

STEP = 0.1  # distância entre as amostras da rota, em m
INRADIUS = 205.0  # raio inscrito dos perímetros, em m
# Perfil do relevo dos mundos da fazenda, (x, altura) em m, copiado do gerador dos mundos.
TERRAIN_PROFILE = [(-260, 14.0), (-195, 7.0), (-140, 1.0), (-90, -2.5), (-46, -1.2),
                   (-25, 0.0), (25, 0.0), (46, 1.4), (110, 8.0), (160, 10.5), (195, 7.0),
                   (260, 2.0)]
# Perímetros: nome da rota, (ângulo de cada curva em graus, raio em m).
PERIMETERS = {'citrus_farm_30': (30.0, 30.0), 'citrus_farm_60': (60.0, 26.0),
              'citrus_farm_90': (90.0, 24.0)}
# A volta do road_follow: retas e curvas à esquerda, (comprimento da reta em m, raio em m).
LAP = [(268.0, 12.0), (244.0, 24.0), (244.0, 12.0), (262.0, 6.0)]
ROUTES = ['citrus_farm', 'citrus_farm_30', 'citrus_farm_60', 'citrus_farm_90']


def terrain_height(x):
    """
    Dá a altura do chão em ``x``, pelo perfil do relevo dos mundos da fazenda.

    Args:
        x: Posição ao longo de X, em m.

    Returns:
        A altura, em m, interpolada linearmente e extrapolada pelo segmento da ponta.
    """
    nodes = TERRAIN_PROFILE
    index = 1
    while index < len(nodes) - 1 and x > nodes[index][0]:
        index += 1
    (x0, z0), (x1, z1) = nodes[index - 1], nodes[index]
    return z0 + (z1 - z0) * (x - x0) / (x1 - x0)


class Route:
    """Uma rota amostrada: posição, rumo e curvatura a cada ``STEP`` metros."""

    def __init__(self, pieces, start):
        """
        Monta a rota a partir de retas e curvas.

        Args:
            pieces: Lista de ``('straight', comprimento_m)`` e
                ``('turn', graus, raio_m)``; graus positivos viram à esquerda.
            start: ``(x, y, rumo)`` do início, com o rumo em rad.
        """
        x, y, heading = start
        columns = [[], [], [], [], []]  # s, x, y, rumo, curvatura
        s = 0.0
        self._turns = []
        self.turns_end = 0.0
        for piece in pieces:
            if piece[0] == 'straight':
                length, curvature = piece[1], 0.0
            else:
                degrees, radius = piece[1], piece[2]
                dheading = math.radians(degrees)
                length = abs(dheading) * radius
                curvature = math.copysign(1.0 / radius, dheading)
                self._turns.append((degrees, radius))
            count = max(1, math.ceil(length / STEP))
            for k in range(count + 1):
                u = length * k / count
                ph = heading + curvature * u
                if curvature == 0.0:
                    px, py = x + u * math.cos(heading), y + u * math.sin(heading)
                else:
                    px = x + (math.sin(ph) - math.sin(heading)) / curvature
                    py = y - (math.cos(ph) - math.cos(heading)) / curvature
                if k < count:
                    for column, value in zip(columns, (s + u, px, py, ph, curvature)):
                        column.append(value)
            x, y, heading = px, py, ph
            s += length
            if piece[0] == 'turn':
                self.turns_end = s
        for column, value in zip(columns, (s, x, y, heading, 0.0)):
            column.append(value)
        self._s, self._x, self._y, self._h, self._k = (np.array(c) for c in columns)
        self.length = s
        self.start = tuple(float(v) for v in start)
        self.start_z = terrain_height(self.start[0])

    def turns(self):
        """
        Lista as curvas da rota.

        Returns:
            Uma lista de ``(graus, raio)``; graus negativos viram à direita (graus e m).
        """
        return [(float(degrees), float(radius)) for degrees, radius in self._turns]

    def at(self, s):
        """
        Dá a pose na rota a ``s`` metros do início.

        Fora da rota, prolonga em linha reta pelo rumo da ponta.

        Args:
            s: Distância ao longo da rota, em m.

        Returns:
            ``(x, y, rumo)``, em m, m e rad.
        """
        clamped = min(max(s, 0.0), self.length)
        x = float(np.interp(clamped, self._s, self._x))
        y = float(np.interp(clamped, self._s, self._y))
        heading = float(np.interp(clamped, self._s, self._h))
        extra = s - clamped
        return x + extra * math.cos(heading), y + extra * math.sin(heading), heading

    def curvature(self, s):
        """
        Dá a curvatura com sinal em ``s``.

        Args:
            s: Distância ao longo da rota, em m.

        Returns:
            A curvatura, em 1/m; positiva para a esquerda.
        """
        index = int(np.searchsorted(self._s, s, side='right')) - 1
        return float(self._k[min(max(index, 0), len(self._k) - 1)])

    def project(self, x, y):
        """
        Projeta um ponto na rota.

        Args:
            x: Posição em X, em m.
            y: Posição em Y, em m.

        Returns:
            ``(s, lateral)``: a distância ao longo da rota e o erro lateral com sinal, em m,
            positivo à esquerda da rota.
        """
        nearest = int(np.argmin((self._x - x) ** 2 + (self._y - y) ** 2))
        best = None
        for first in (nearest - 1, nearest):
            if first < 0 or first + 1 >= len(self._s):
                continue
            ax, ay = self._x[first], self._y[first]
            dx, dy = self._x[first + 1] - ax, self._y[first + 1] - ay
            t = ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)
            t = min(max(t, 0.0), 1.0)
            distance = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            if best is None or distance < best[0]:
                best = (distance, first, t)
        _, first, t = best
        s = self._s[first] + t * (self._s[first + 1] - self._s[first])
        px, py, heading = self.at(float(s))
        lateral = -math.sin(heading) * (x - px) + math.cos(heading) * (y - py)
        return float(s), float(lateral)


def _perimeter(angle, radius):
    """Monta o perímetro de ângulo e raio dados, como o ``polygon_perimeter`` do gerador."""
    count = round(360 / angle)
    side = 2 * INRADIUS * math.tan(math.pi / count)
    leg = side - 2 * radius * math.tan(math.radians(angle) / 2)
    pieces = [('straight', leg / 2)]
    for i in range(count):
        pieces.append(('turn', -angle, radius))
        pieces.append(('straight', leg if i < count - 1 else leg / 2 + 2))
    return Route(pieces, (-INRADIUS, 0.0, math.pi / 2))


def _lap():
    """Monta a volta ``lap`` do ``road_follow``, de retas e curvas de 90 graus à esquerda."""
    pieces = []
    for length, radius in LAP:
        pieces += [('straight', length), ('turn', 90.0, radius)]
    return Route(pieces, (0.0, 0.0, 0.0))


def route(name):
    """
    Devolve a rota de um mundo.

    Args:
        name: Um nome de ``ROUTES``, igual ao do mundo sem ``.sdf``.

    Returns:
        A ``Route``.

    Raises:
        KeyError: Se não houver rota com esse nome.
    """
    if name == 'citrus_farm':
        return _lap()
    if name in PERIMETERS:
        return _perimeter(*PERIMETERS[name])
    raise KeyError(f'rota desconhecida: {name!r}; as rotas são {ROUTES}')
