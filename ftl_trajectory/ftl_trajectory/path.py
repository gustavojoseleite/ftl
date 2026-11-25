"""
O trilho compartilhado como uma polilinha com comprimento de arco, para quem o segue.

O seguidor projeta nele a própria posição e a do veículo da frente, mede a distância entre os
dois ao longo do trilho e escolhe o alvo do *pure pursuit* a uma distância de visada adiante. A
projeção procura só numa janela em volta da última, para não saltar para outro trecho do trilho
que passe perto (o começo e o fim de uma volta, por exemplo).
"""

import bisect
import math


class SharedPath:
    """Polilinha ``[(x, y), ...]`` com o comprimento de arco acumulado de cada ponto."""

    def __init__(self, points):
        """Monta o trilho a partir dos pontos, descartando os repetidos."""
        self.points = []
        self.s = []
        for x, y in points:
            if self.points and math.dist(self.points[-1], (x, y)) < 1e-6:
                continue
            self.s.append(self.s[-1] + math.dist(self.points[-1], (x, y)) if self.s else 0.0)
            self.points.append((float(x), float(y)))

    def __len__(self):
        """Quantos pontos o trilho tem."""
        return len(self.points)

    @property
    def length(self):
        """O comprimento do trilho, em m."""
        return self.s[-1] if self.s else 0.0

    def project(self, x, y, hint=None, window=40.0):
        """
        Projeta um ponto no trilho.

        Args:
            x: Posição em X, em m.
            y: Posição em Y, em m.
            hint: Comprimento de arco da projeção anterior, em m; ``None`` procura no trilho todo.
            window: Meia janela da busca em volta de ``hint``, em m.

        Returns:
            ``(s, lateral)``: o comprimento de arco da projeção, em m, e a distância ao trilho, em
            m, positiva à esquerda. Antes do começo, ``s`` é negativo, e depois do fim passa do
            comprimento do trilho, pela distância ao longo do trecho da ponta: é o que mede um
            trator que ainda não chegou ao trilho ou o líder à frente dele.
        """
        if len(self.points) < 2:
            raise ValueError('o trilho precisa de pelo menos dois pontos')
        lo, hi = 0, len(self.points) - 1
        if hint is not None:
            lo = max(0, bisect.bisect_left(self.s, hint - window) - 1)
            hi = min(len(self.points) - 1, bisect.bisect_right(self.s, hint + window) + 1)
            hi = max(hi, lo + 1)
        best = None
        for k in range(lo, hi):
            (ax, ay), (bx, by) = self.points[k], self.points[k + 1]
            dx, dy = bx - ax, by - ay
            seg = math.hypot(dx, dy)
            t = ((x - ax) * dx + (y - ay) * dy) / (seg * seg)
            # Antes do começo e depois do fim, a projeção segue a direção do trecho da ponta.
            if k > 0:
                t = max(0.0, t)
            if k < len(self.points) - 2:
                t = min(1.0, t)
            px, py = ax + t * dx, ay + t * dy
            distance = math.hypot(x - px, y - py)
            if best is None or distance < best[0]:
                lateral = (dx * (y - ay) - dy * (x - ax)) / seg
                best = (distance, self.s[k] + t * seg, lateral)
        return best[1], best[2]

    def point_at(self, s):
        """Dá o ponto do trilho no comprimento de arco ``s``, limitado às pontas."""
        if s <= 0.0:
            return self.points[0]
        if s >= self.length:
            return self.points[-1]
        k = bisect.bisect_right(self.s, s) - 1
        (ax, ay), (bx, by) = self.points[k], self.points[k + 1]
        t = (s - self.s[k]) / (self.s[k + 1] - self.s[k])
        return ax + t * (bx - ax), ay + t * (by - ay)
