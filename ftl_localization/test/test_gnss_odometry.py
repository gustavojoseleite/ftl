"""A conversão do fix GNSS do `gnss_odometry`: antena, covariância e fixes inválidos (F3.1)."""
import math

from ftl_localization.gnss_odometry import base_position, horizontal_variances, valid_fix
import pytest
from sensor_msgs.msg import NavSatFix, NavSatStatus


def _fix(lat=-20.9494, lon=-48.4794, status=NavSatStatus.STATUS_FIX):
    fix = NavSatFix()
    fix.latitude, fix.longitude = lat, lon
    fix.status.status = status
    return fix


def test_invalid_fixes_are_dropped():
    assert valid_fix(_fix())
    assert not valid_fix(_fix(status=NavSatStatus.STATUS_NO_FIX))
    assert not valid_fix(_fix(lat=math.nan))
    assert not valid_fix(_fix(lat=0.0, lon=0.0))


def _yaw_pitch(yaw, pitch=0.0):
    """Quaternion de um rumo e uma arfagem (nariz para baixo é positivo)."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)


def test_antenna_lever_follows_the_heading():
    # Antena 1,075 m à frente: rumo leste, a base fica 1,075 m a oeste da antena.
    assert base_position((10.0, 5.0), _yaw_pitch(0.0), (1.075, 0.0, 0.0)) == \
        pytest.approx((8.925, 5.0))
    # Rumo norte: a base fica 1,075 m ao sul.
    assert base_position((10.0, 5.0), _yaw_pitch(math.pi / 2), (1.075, 0.0, 0.0)) == \
        pytest.approx((10.0, 3.925))
    # Antena 0,2 m à esquerda, rumo leste: a base fica 0,2 m ao sul dela.
    assert base_position((0.0, 0.0), _yaw_pitch(0.0), (0.0, 0.2, 0.0)) == \
        pytest.approx((0.0, -0.2))


def test_antenna_height_on_a_slope():
    # Antena 2,685 m acima da base, veículo arfando 6,6° com o nariz para baixo, rumo leste: a
    # antena fica 2,685 sin(6,6°) = 0,309 m à frente da base na horizontal.
    pitch = math.radians(6.6)
    x, y = base_position((0.0, 0.0), _yaw_pitch(0.0, pitch), (0.0, 0.0, 2.685))
    assert x == pytest.approx(-2.685 * math.sin(pitch), abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_covariance_from_the_fix_or_the_rtk_default():
    fix = _fix()
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    assert horizontal_variances(fix, 0.02, 0.04) == pytest.approx((4e-4, 4e-4, 1.6e-3))
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    fix.position_covariance = [0.01, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.03]
    assert horizontal_variances(fix, 0.02, 0.04) == pytest.approx((0.01, 0.02, 0.03))
    # Conhecida mas zerada, como a do Gazebo: vale a do RTK.
    fix.position_covariance = [0.0] * 9
    assert horizontal_variances(fix, 0.02, 0.04) == pytest.approx((4e-4, 4e-4, 1.6e-3))
