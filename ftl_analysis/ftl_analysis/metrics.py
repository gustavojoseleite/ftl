"""
As contas das métricas (seção 8.4 do plano), sobre séries já extraídas dos bags.

Python puro, sem ``rclpy``.
"""

import math

import numpy as np


def summary(errors):
    """
    Resume uma série de erros.

    Args:
        errors: Os erros, na unidade da série.

    Returns:
        Um dicionário com ``rmse``, ``max`` (do valor absoluto) e ``p95`` (do valor absoluto).
    """
    errors = np.asarray(errors, dtype=float)
    return {'rmse': float(np.sqrt(np.mean(errors ** 2))),
            'max': float(np.max(np.abs(errors))),
            'p95': float(np.percentile(np.abs(errors), 95))}


def corner_cut(laterals, turn):
    """
    Dá o maior desvio para dentro da curva.

    Args:
        laterals: Os erros laterais com sinal, em m, positivos à esquerda.
        turn: +1 para a curva à esquerda e -1 para a curva à direita.

    Returns:
        O maior desvio para dentro da curva, em m; 0 se o trator nunca passou por dentro.
    """
    return max(0.0, float(np.max(turn * np.asarray(laterals, dtype=float))))


def string_stability(norms):
    """
    Dá a razão entre a norma do erro de cada veículo e a do precedente.

    Args:
        norms: A norma do erro de cada veículo, na ordem da fila.

    Returns:
        ``norms[i] / norms[i-1]`` para ``i`` de 1 em diante; ``nan`` se o precedente for nulo.
    """
    return [b / a if a else math.nan for a, b in zip(norms, norms[1:])]


def rms_rate(t, values):
    """
    Dá o RMS da derivada de uma série no tempo.

    Args:
        t: Os instantes, em s, crescentes.
        values: Os valores da série.

    Returns:
        O RMS de ``d(values)/dt``, na unidade da série por s.
    """
    rate = np.gradient(np.asarray(values, dtype=float), np.asarray(t, dtype=float))
    return float(np.sqrt(np.mean(rate ** 2)))
