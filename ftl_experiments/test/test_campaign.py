"""
Testes do plano de uma campanha de experimentos (F2.3, R1, R2 e R4), sem ROS.

`ftl_experiments.campaign`:
- `plan(campanha, cenário, sobrescritas, sementes, raiz, label=None)` devolve uma `Run` por
  semente, com `path` = `raiz/campanha/label/semente` (o `label` padrão é o cenário), `seed`,
  `scenario` e `overrides` (as sobrescritas lidas, mais `degradation.seed` e `gui` falso);
- `pending(runs)`: as que ainda não terminaram em `done`;
- `time_limit(config)`: 2× o tempo da volta à velocidade de cruzeiro, mais 120 s de partida; sem
  rota, 600 s;
- `write_run(path, reason, duration, seed, commits)` e `read_run(path)` gravam e leem o
  `run.yaml`; `reason` é `done`, `timeout` ou `failed`;
- `topics(config)`: os tópicos que o *bag* grava;
- `commits(src)`: o commit de cada repositório git em `src`.
"""
import subprocess

from ftl_bringup.scenario import parse_overrides, resolve
from ftl_experiments.campaign import (commits, pending, plan, read_run, time_limit, topics,
                                      write_run)
import pytest

ROUTE = 'world=citrus_farm_90.sdf convoy.followers=2 route.enabled=true'


def test_plan_one_run_per_seed(tmp_path):
    runs = plan('c1', 'single_120', ROUTE, [1, 2, 3], tmp_path)
    assert [r.seed for r in runs] == [1, 2, 3]
    assert runs[1].path == tmp_path / 'c1' / 'single_120' / '2'
    assert runs[1].overrides['degradation.seed'] == 2
    assert runs[1].overrides['gui'] is False
    assert runs[1].overrides['world'] == 'citrus_farm_90.sdf'
    labelled = plan('c1', 'single_120', ROUTE, [1], tmp_path, label='p90_10kmh')
    assert labelled[0].path == tmp_path / 'c1' / 'p90_10kmh' / '1'


def test_pending_skips_only_finished_runs(tmp_path):
    runs = plan('c1', 'single_120', ROUTE, [1, 2, 3], tmp_path)
    write_run(runs[0].path, 'done', 812.5, 1, {})
    write_run(runs[1].path, 'timeout', 900.0, 2, {})
    assert [r.seed for r in pending(runs)] == [2, 3]


def test_run_file_round_trip(tmp_path):
    write_run(tmp_path, 'failed', 12.0, 7, {'ftl': 'abc123'})
    assert read_run(tmp_path) == {'reason': 'failed', 'duration': 12.0, 'seed': 7,
                                  'commits': {'ftl': 'abc123'}}
    with pytest.raises(ValueError):
        write_run(tmp_path, 'talvez', 1.0, 1, {})


def test_time_limit():
    config = resolve('single_120', parse_overrides(ROUTE + ' route.cruise_kmh=36'))
    assert time_limit(config) == pytest.approx(2 * 1600.8 / 10.0 + 120.0, rel=0.01)
    assert time_limit(resolve('single_120')) == pytest.approx(600.0)


def test_topics_cover_the_metrics():
    names = topics(resolve('single_120', parse_overrides(ROUTE)))
    for vehicle in ('pickup', 'tractor_1', 'tractor_2'):
        assert f'/ground_truth/{vehicle}/pose' in names
        assert f'/{vehicle}/ftl/state' in names
        assert f'/{vehicle}/cmd_vel' in names
        assert f'/{vehicle}/degradation/state' in names
    for tractor in ('tractor_1', 'tractor_2'):
        assert f'/{tractor}/ftl/gap' in names and f'/{tractor}/ftl/cross_track' in names
    for name in ('/pickup/route/error', '/pickup/route/progress', '/pickup/route/done',
                 '/experiment/config', '/clock'):
        assert name in names
    assert len(names) == len(set(names))


def test_commits(tmp_path):
    repo = tmp_path / 'meu_repo'
    repo.mkdir()
    git = ['git', '-C', str(repo), '-c', 'user.name=t', '-c', 'user.email=t@t']
    subprocess.run(git + ['init', '-q'], check=True)
    subprocess.run(git + ['commit', '-q', '--allow-empty', '-m', 'x'], check=True)
    head = subprocess.run(git + ['rev-parse', 'HEAD'], capture_output=True, text=True,
                          check=True).stdout.strip()
    (tmp_path / 'nao_e_repo').mkdir()
    assert commits(tmp_path) == {'meu_repo': head}
