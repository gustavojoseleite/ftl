"""
Testes das rotas do líder (F2.1, R1), sem ROS.

`ftl_route_driver.routes`:
- `ROUTES`: os nomes das rotas, iguais aos dos mundos sem `.sdf` (`citrus_farm`,
  `citrus_farm_30`, `citrus_farm_60`, `citrus_farm_90`);
- `route(nome)` devolve uma `Route`, com:
  - `length` (m), `start` (x, y, rumo em rad) e `turns()` (lista de (graus, raio));
  - `at(s)`, que dá (x, y, rumo) a `s` metros do início;
  - `curvature(s)`, a curvatura com sinal (1/m) em `s`;
  - `project(x, y)`, que dá (s, erro lateral com sinal, positivo à esquerda da rota).

Os perímetros seguem a fórmula do gerador dos mundos (`polygon_perimeter`); a volta do
`citrus_farm` segue a tabela `lap` do `road_follow`.
"""
import ast
import importlib.util
import math
from pathlib import Path

from ftl_route_driver.routes import route, ROUTES
import pytest

PERIMETERS = {'citrus_farm_30': (1315.9, 30.0, 12, -30.0),
              'citrus_farm_60': (1405.5, 26.0, 6, -60.0),
              'citrus_farm_90': (1600.8, 24.0, 4, -90.0)}


def test_names():
    assert sorted(ROUTES) == ['citrus_farm', 'citrus_farm_30', 'citrus_farm_60',
                              'citrus_farm_90']


@pytest.mark.parametrize('name', sorted(PERIMETERS))
def test_perimeters(name):
    length, radius, count, degrees = PERIMETERS[name]
    r = route(name)
    assert r.length == pytest.approx(length, abs=1.0)
    assert r.turns() == [pytest.approx((degrees, radius))] * count
    x, y, heading = r.start
    assert (x, y, heading) == pytest.approx((-205.0, 0.0, math.pi / 2))


def test_lap_of_the_farm():
    r = route('citrus_farm')
    assert r.length == pytest.approx(1102.8, abs=1.0)
    assert [radius for _, radius in r.turns()] == pytest.approx([12.0, 24.0, 12.0, 6.0])
    assert all(degrees == pytest.approx(90.0) for degrees, _ in r.turns())
    assert r.start == pytest.approx((0.0, 0.0, 0.0))
    x, y, _ = r.at(r.length)
    assert math.hypot(x - 6.0, y) < 0.5


def test_curvature_and_position():
    r = route('citrus_farm_90')
    first_leg = (2 * 205 * math.tan(math.pi / 4) - 2 * 24) / 2
    assert r.curvature(first_leg / 2) == pytest.approx(0.0, abs=1e-6)
    assert abs(r.curvature(first_leg + 10.0)) == pytest.approx(1 / 24.0, rel=0.02)
    x, y, heading = r.at(first_leg / 2)
    assert (x, y) == pytest.approx((-205.0, first_leg / 2), abs=0.05)
    assert heading == pytest.approx(math.pi / 2, abs=1e-3)


def test_project():
    r = route('citrus_farm_90')
    s, lateral = r.project(-205.5, 50.0)
    assert s == pytest.approx(50.0, abs=0.1)
    assert lateral == pytest.approx(0.5, abs=0.02)
    s, lateral = r.project(-204.0, 50.0)
    assert lateral == pytest.approx(-1.0, abs=0.02)


def _generator():
    for parent in Path(__file__).resolve().parents:
        candidate = (parent / 'agri_gazebo' / 'agri_gz_worlds' / 'scripts'
                     / 'generate_agriculture_worlds.py')
        if candidate.exists():
            spec = importlib.util.spec_from_file_location('generator', candidate)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    return None


@pytest.mark.parametrize('angle, radius', [(30, 30), (60, 26), (90, 24)])
def test_matches_the_world_generator(angle, radius):
    generator = _generator()
    if generator is None:
        pytest.skip('a fonte do gerador dos mundos não está no workspace')
    reference = generator.polygon_perimeter(angle, radius, 205).points(1.0)
    r = route(f'citrus_farm_{angle}')
    for x, y in reference[::25]:
        assert abs(r.project(x, y)[1]) < 0.05


def test_start_height_on_the_relief():
    """
    `Route.start_z` é a altura do chão no início da rota (relevo dos mundos da fazenda).

    O relevo varia ao longo de X, pelo perfil do gerador; o perímetro começa em x = −205, a
    8,08 m, e a volta do `citrus_farm` começa na origem, a 0 m.
    """
    assert route('citrus_farm_90').start_z == pytest.approx(8.077, abs=0.01)
    assert route('citrus_farm_30').start_z == pytest.approx(8.077, abs=0.01)
    assert route('citrus_farm').start_z == pytest.approx(0.0, abs=0.01)


def test_terrain_matches_the_world_generator():
    generator = _generator()
    if generator is None:
        pytest.skip('a fonte do gerador dos mundos não está no workspace')
    from ftl_route_driver.routes import terrain_height
    source = Path(generator.__file__).read_text()
    profile = ast.literal_eval(source.split('profile = ', 1)[1].split('\n    if extent', 1)[0])
    reference = generator.Terrain(profile)
    for x in range(-250, 251, 25):
        assert terrain_height(float(x)) == pytest.approx(reference.z(float(x)), abs=1e-6)
