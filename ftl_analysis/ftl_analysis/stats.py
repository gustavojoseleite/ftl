"""Estatística entre as execuções: a média e o intervalo de confiança de 95%."""

import math

import numpy as np
from scipy import stats as scipy_stats


def mean_ci(values):
    """
    Dá a média e o intervalo de confiança de 95% de uma amostra.

    Args:
        values: Um valor por execução.

    Returns:
        ``(média, meia-largura do IC)``, pela distribuição t de Student; a meia-largura é
        ``nan`` com menos de duas execuções.
    """
    values = np.asarray(values, dtype=float)
    mean = float(np.mean(values))
    if len(values) < 2:
        return mean, math.nan
    quantile = scipy_stats.t.ppf(0.975, len(values) - 1)
    return mean, float(quantile * np.std(values, ddof=1) / math.sqrt(len(values)))
