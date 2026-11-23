"""
Perfil de velocidade da rota: o limite de cada ponto pela curvatura, com a frenagem antecipada.

Este módulo é Python puro, sem ``rclpy``, e roda nos testes (``test/test_profile.py``); o nó
``route_driver`` em ``node.py`` o usa.
"""

import math

import numpy as np


def speed_profile(route, cruise, a_lat=1.0, decel=1.0, step=0.5):
    """
    Calcula a velocidade-alvo ao longo da rota.

    Em cada ponto, ``v = min(cruzeiro, sqrt(a_lat / |curvatura|))``. Depois, o perfil é
    antecipado para trás: a velocidade nunca exige frear mais que ``decel`` antes de uma curva.

    Args:
        route: A ``Route``.
        cruise: Velocidade de cruzeiro, em m/s; positiva.
        a_lat: Aceleração lateral máxima nas curvas, em m/s²; positiva.
        decel: Desaceleração máxima antes das curvas, em m/s²; positiva.
        step: Distância entre os pontos do perfil, em m; positiva.

    Returns:
        ``(s, v)``: duas arrays, as distâncias ao longo da rota (m) e as velocidades (m/s).
    """
    count = max(2, math.ceil(route.length / step) + 1)
    s = np.linspace(0.0, route.length, count)
    v = np.empty(count)
    for i, position in enumerate(s):
        kappa = abs(route.curvature(position))
        v[i] = cruise if kappa < 1e-9 else min(cruise, math.sqrt(a_lat / kappa))
    for i in range(count - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * decel * (s[i + 1] - s[i])))
    return s, v


def speed_at(profile, s):
    """
    Interpola o perfil de velocidade.

    Args:
        profile: O par ``(s, v)`` de ``speed_profile``.
        s: Distância ao longo da rota, em m.

    Returns:
        A velocidade-alvo, em m/s; fora da rota vale a da ponta.
    """
    return float(np.interp(s, profile[0], profile[1]))
