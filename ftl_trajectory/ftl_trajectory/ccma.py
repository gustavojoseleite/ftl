"""
Média móvel com curvatura corrigida (CCMA), de Steinecker e Wuensche (2023) (F6.1, R2).

A Eq. 2.12 da monografia::

    p_ccma,i = Σ_j w_ma,j p_(i+j)
               + u⊥,i · Σ_j w_cc,j (1 / c_(i+j)) (1 / (w_ma,0 + 2 Σ_k w_ma,k cos(α_(i+j) k)) - 1)

- a primeira soma é a média móvel (MA), com janela ``±w_ma`` e pesos gaussianos;
- ``c_m`` é a curvatura dos pontos da MA em ``m`` (a do círculo pelos pontos ``m - 1``, ``m`` e
  ``m + 1``);
- ``α_m`` é o ângulo entre dois pontos sucessivos vistos do centro do círculo de raio
  ``1 / |c_m|``, dado pelo espaçamento médio ``d`` dos pontos originais: ``α = 2 asin(d |c| / 2)``;
- num arco de círculo, a MA de pontos igualmente espaçados cai a ``r · (w_ma,0 + 2 Σ w_ma,k
  cos(α k))`` do centro, para dentro; o fator de correção devolve o ponto ao raio original;
- a segunda soma é outra média, com janela ``±w_cc``, sobre essas correções, e ``u⊥`` aponta para
  fora da curva (o lado oposto ao centro).

Com curvatura nula a correção é zero. Nas pontas do vetor, as janelas encolhem de forma simétrica
(o ponto ``i`` usa no máximo ``i`` vizinhos de cada lado), e a curvatura das duas pontas da MA é
zero.

Processamento incremental (R2): o ponto corrigido ``i`` depende dos pontos originais de
``i - w_ma - w_cc - 1`` a ``i + w_ma + w_cc + 1``. Quando esse último já chegou, o ponto ``i`` é
final e nunca mais muda; ``Ccma.update`` calcula só os pontos que acabaram de ficar finais, e o
resultado é igual ao do lote (``ccma``) na parte já processada.
"""

import math


def gaussian_weights(half):
    """
    Dá os pesos gaussianos ``w_-half .. w_half`` de uma janela, normalizados para somar 1.

    O desvio-padrão é metade da meia janela (``half / 2``), e uma janela nula dá ``[1.0]``.
    """
    if half <= 0:
        return [1.0]
    sigma = half / 2.0
    raw = [math.exp(-0.5 * (k / sigma) ** 2) for k in range(-half, half + 1)]
    total = sum(raw)
    return [w / total for w in raw]


def _window(i, half, n):
    """Dá a meia janela usada em ``i``, que encolhe de forma simétrica perto das pontas."""
    return max(0, min(half, i, n - 1 - i))


def _moving_average(points, i, half):
    """Média ponderada dos pontos em volta de ``i``, com a janela encolhida nas pontas."""
    h = _window(i, half, len(points))
    weights = gaussian_weights(h)
    x = sum(w * points[i + k - h][0] for k, w in enumerate(weights))
    y = sum(w * points[i + k - h][1] for k, w in enumerate(weights))
    return x, y


def curvature(a, b, c):
    """
    Dá a curvatura com sinal do círculo por três pontos (positiva quando a curva vira à esquerda).

    Pontos alinhados ou repetidos dão zero.
    """
    ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
    if ab * bc * ca < 1e-12:
        return 0.0
    cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
    return 2.0 * cross / (ab * bc * ca)


def correction(kappa, d, half):
    """
    Dá o deslocamento que leva um ponto da MA de volta ao raio original, para fora da curva.

    É o termo ``(1 / c) (1 / (w_0 + 2 Σ w_k cos(α k)) - 1)`` da Eq. 2.12, sem sinal.

    Args:
        kappa: Curvatura dos pontos da MA, em 1/m.
        d: Espaçamento médio dos pontos originais, em m.
        half: Meia janela da MA usada no ponto.

    Returns:
        O deslocamento, em m, sempre maior ou igual a zero.
    """
    c = abs(kappa)
    if c < 1e-9 or half <= 0:
        return 0.0
    alpha = 2.0 * math.asin(min(1.0, d * c / 2.0))
    weights = gaussian_weights(half)
    shrink = weights[half] + 2.0 * sum(weights[half + k] * math.cos(alpha * k)
                                       for k in range(1, half + 1))
    if shrink <= 1e-6:
        return 0.0
    return (1.0 / c) * (1.0 / shrink - 1.0)


class Ccma:
    """Aplica o CCMA de forma incremental a um vetor de pontos que só cresce."""

    def __init__(self, w_ma=5, w_cc=3):
        """
        Guarda as janelas.

        Args:
            w_ma: Meia janela da média móvel, em pontos.
            w_cc: Meia janela da média das correções, em pontos.
        """
        if w_ma < 0 or w_cc < 0:
            raise ValueError('as janelas não podem ser negativas')
        self.w_ma, self.w_cc = w_ma, w_cc
        self.points = []
        self.smoothed = []

    @property
    def reach(self):
        """Quantos pontos originais depois de ``i`` o ponto corrigido ``i`` usa."""
        return self.w_ma + self.w_cc + 1

    def add(self, x, y):
        """Acrescenta um ponto original ao fim do vetor."""
        self.points.append((float(x), float(y)))

    def update(self):
        """
        Calcula os pontos que acabaram de ficar finais.

        Returns:
            Os pontos corrigidos novos, na ordem.
        """
        final = len(self.points) - self.reach
        new = [self._point(i) for i in range(len(self.smoothed), final)]
        self.smoothed.extend(new)
        return new

    def pending(self):
        """Os pontos originais que ainda não têm ponto corrigido final."""
        return self.points[len(self.smoothed):]

    def _ma(self, m):
        return _moving_average(self.points, m, self.w_ma)

    def _shift(self, m):
        """O deslocamento, para fora, do ponto da MA em ``m``, com o sinal da curva."""
        n = len(self.points)
        if m <= 0 or m >= n - 1:
            return 0.0
        kappa = curvature(self._ma(m - 1), self._ma(m), self._ma(m + 1))
        h = _window(m, self.w_ma, n)
        lo, hi = m - h, m + h
        d = (sum(math.dist(self.points[k], self.points[k + 1]) for k in range(lo, hi))
             / max(1, hi - lo))
        return math.copysign(correction(kappa, d, h), kappa)

    def _point(self, i):
        n = len(self.points)
        x, y = self._ma(i)
        if 0 < i < n - 1:
            ax, ay = self._ma(i - 1)
            bx, by = self._ma(i + 1)
        else:
            return x, y
        heading = math.atan2(by - ay, bx - ax)
        h = _window(i, self.w_cc, n)
        weights = gaussian_weights(h)
        shift = sum(w * self._shift(i + k - h) for k, w in enumerate(weights))
        # Uma curva à esquerda (curvatura positiva) tem o centro à esquerda: o ponto sai para a
        # direita, contra a normal esquerda (-sin, cos).
        return x + math.sin(heading) * shift, y - math.cos(heading) * shift


def ccma(points, w_ma=5, w_cc=3):
    """
    Aplica o CCMA em lote.

    Args:
        points: ``[(x, y), ...]`` do vetor de pontos.
        w_ma: Meia janela da média móvel, em pontos.
        w_cc: Meia janela da média das correções, em pontos.

    Returns:
        Os pontos corrigidos finais, isto é, todos menos os ``w_ma + w_cc + 1`` últimos.
    """
    filt = Ccma(w_ma, w_cc)
    for x, y in points:
        filt.add(x, y)
    return filt.update()
