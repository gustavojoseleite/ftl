"""
Latitude e longitude em metros, em um referencial que todos os veículos compartilham.

A odometria de cada veículo começa onde ele foi posicionado, então duas odometrias
não se comparam. Um fix GNSS se compara: convertido em metros leste/norte a partir
de um `datum` combinado, cada máquina do comboio cai no mesmo referencial. O
`datum` é um parâmetro que todos os nós do FTL recebem do mesmo arquivo (`ftl.yaml`);
o padrão é a origem de `spherical_coordinates` dos mundos da fazenda de citros.
"""

import math


def metres_per_degree(lat_deg):
    """
    Par (metros por grau de latitude, metros por grau de longitude) no WGS84.

    O atalho esférico usual erra 0,4% nesta latitude, o que em um carreador de
    280 m dá um metro; é a mesma série que o `road_follow` usa.
    """
    phi = math.radians(lat_deg)
    per_lat = 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
    per_lon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)
    return per_lat, per_lon


class LocalFrame:
    """Metros leste/norte a partir de um `datum`. Terra plana: serve para alguns quilômetros."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.per_lat, self.per_lon = metres_per_degree(lat0)

    def to_enu(self, lat, lon):
        return ((lon - self.lon0) * self.per_lon, (lat - self.lat0) * self.per_lat)


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
