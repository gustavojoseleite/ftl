"""Latitude and longitude to metres, in a frame every vehicle shares.

Each vehicle's odometry starts wherever that vehicle was spawned, so two
odometries cannot be compared. A GNSS fix can: turned into east/north metres
from one agreed datum, every machine in the convoy lands in the same frame.
The datum is a parameter all FTL nodes get from the same file (ftl.yaml); the
default is the spherical_coordinates origin of the citrus-farm worlds.
"""

import math


def metres_per_degree(lat_deg):
    """(metres per degree of latitude, metres per degree of longitude) on WGS84.

    The usual spherical shortcut is 0.4% out at this latitude, which over a
    280 m carreador is a metre — the same series road_follow uses.
    """
    phi = math.radians(lat_deg)
    per_lat = 111132.92 - 559.82 * math.cos(2 * phi) + 1.175 * math.cos(4 * phi)
    per_lon = 111412.84 * math.cos(phi) - 93.5 * math.cos(3 * phi)
    return per_lat, per_lon


class LocalFrame:
    """East/north metres from a datum. Flat-earth: fine over a few kilometres."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.per_lat, self.per_lon = metres_per_degree(lat0)

    def to_enu(self, lat, lon):
        return ((lon - self.lon0) * self.per_lon, (lat - self.lat0) * self.per_lat)


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
