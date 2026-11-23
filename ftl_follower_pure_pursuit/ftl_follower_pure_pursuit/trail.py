"""
O caminho que o veículo da frente percorreu, como uma polilinha com comprimento de arco.

Um seguidor não mira no veículo da frente: ele passa por onde aquele veículo passou.
Mirar direto nele corta todas as curvas, e em um carreador isso sai da estrada;
repetir o caminho mantém a estrada que o líder manteve. O espaçamento é medido
ao longo do mesmo caminho, então continua certo em uma curva, onde a distância em
linha reta diminuiria.
"""

import math


class Trail:
    def __init__(self, spacing=0.3):
        self.spacing = spacing
        self.xs, self.ys, self.ss = [], [], []

    def __len__(self):
        return len(self.xs)

    @property
    def end_s(self):
        return self.ss[-1] if self.ss else 0.0

    def append(self, x, y):
        """Acrescenta um ponto se ele estiver ao menos a `spacing` do último."""
        if not self.xs:
            self.xs, self.ys, self.ss = [x], [y], [0.0]
            return
        d = math.hypot(x - self.xs[-1], y - self.ys[-1])
        if d >= self.spacing:
            self.xs.append(x)
            self.ys.append(y)
            self.ss.append(self.ss[-1] + d)

    def seed_line(self, a, b):
        """
        Inicia a trilha como a reta de a até b.

        No começo o veículo da frente ainda não percorreu nada. O comboio é
        posicionado em fila na estrada, então a reta do seguidor até ele é a
        estrada, e dá ao seguidor um caminho onde se apoiar.
        """
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / self.spacing))
        for i in range(n + 1):
            t = i / n
            self.append(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

    def project(self, x, y, hint=0, window=200):
        """
        Ponto da trilha mais próximo de (x, y), buscado em torno do índice `hint`.

        Devolve (s, lateral, índice): o comprimento de arco desse ponto, o desvio
        com sinal (positivo = (x, y) está à esquerda da trilha) e o índice do
        segmento, para passar de volta como o próximo `hint`. A busca é local
        porque uma trilha pode se cruzar (uma volta) e o ponto mais próximo
        global seria a passada errada.
        """
        n = len(self.xs)
        if n == 0:
            return 0.0, 0.0, 0
        if n == 1:
            return self.ss[0], 0.0, 0
        lo = max(0, hint - 20)
        hi = min(n - 1, hint + window)
        best = (float('inf'), 0.0, 0.0, lo)
        for i in range(lo, hi):
            ax, ay, bx, by = self.xs[i], self.ys[i], self.xs[i + 1], self.ys[i + 1]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            t = 0.0 if seg2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
            px, py = ax + t * dx, ay + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best[0]:
                seg = math.sqrt(seg2)
                side = (dx * (y - ay) - dy * (x - ax))
                lateral = math.copysign(math.sqrt(d2), side) if d2 > 0 else 0.0
                best = (d2, self.ss[i] + t * seg, lateral, i)
        return best[1], best[2], best[3]

    def point_at(self, s):
        """Devolve o ponto da trilha no comprimento de arco s, limitado às pontas."""
        if not self.xs:
            raise ValueError('empty trail')
        if s <= self.ss[0]:
            return self.xs[0], self.ys[0]
        if s >= self.ss[-1]:
            return self.xs[-1], self.ys[-1]
        lo, hi = 0, len(self.ss) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.ss[mid] <= s:
                lo = mid
            else:
                hi = mid
        t = (s - self.ss[lo]) / (self.ss[hi] - self.ss[lo])
        return (self.xs[lo] + (self.xs[hi] - self.xs[lo]) * t,
                self.ys[lo] + (self.ys[hi] - self.ys[lo]) * t)

    def prune(self, s_before):
        """Esquece a trilha atrás de s_before (guarda dois pontos) e devolve quantos saíram."""
        k = 0
        while k < len(self.ss) - 2 and self.ss[k + 1] < s_before:
            k += 1
        if k:
            del self.xs[:k], self.ys[:k], self.ss[:k]
        return k
