"""
Testes do conjunto básico e da regra do tempo limite (F2.5, R1, R3 e R4), sem ROS.

`ftl_experiments.suite`:
- `basic()`: os 12 casos da seção 8.1, cada um um `Case` com `label` e `overrides` (o texto de
  sobrescritas do `--set`);
- `quick()`: o caso único do modo rápido;
- `QUICK_DURATION`: os segundos simulados do modo rápido.

`ftl_experiments.campaign.Deadline(limit, stall=60.0, startup=300.0)`:
- `check(sim, wall)` devolve `None` enquanto a execução segue, `'timeout'` quando o tempo simulado
  passa do limite, e `'failed'` quando o `/clock` não começa em `startup` s de parede ou, depois
  de começar, não avança por `stall` s de parede;
- `sim` é `None` enquanto o `/clock` não chegou; o limite conta a partir do primeiro `sim`.
"""
from ftl_bringup.scenario import parse_overrides, resolve
from ftl_experiments.campaign import Deadline
from ftl_experiments.suite import basic, quick, QUICK_DURATION
import pytest


def test_the_twelve_basic_cases():
    cases = basic()
    assert len(cases) == 12
    assert len({c.label for c in cases}) == 12
    assert {c.label for c in cases} >= {'p30_5kmh', 'p60_10_8kmh', 'p90_20kmh', 'p90_40kmh'}
    seen = set()
    for case in cases:
        config = resolve('single_120', parse_overrides(case.overrides))
        assert config['convoy']['followers'] == 4
        assert config['route']['enabled'] is True
        assert config['sensors']['camera_mount'] == 'single'
        angle = case.label.split('_')[0][1:]
        assert config['world'] == f'citrus_farm_{angle}.sdf'
        seen.add((config['world'], config['route']['cruise_kmh']))
    assert seen == {(f'citrus_farm_{a}.sdf', v) for a in (30, 60, 90) for v in (5, 10.8, 20, 40)}


def test_quick_mode():
    case = quick()
    config = resolve('single_120', parse_overrides(case.overrides))
    assert case.label == 'quick'
    assert config['world'] == 'citrus_farm_90.sdf'
    assert config['route']['cruise_kmh'] == pytest.approx(10.8)
    assert config['convoy']['followers'] == 4
    assert QUICK_DURATION == pytest.approx(120.0)


def test_deadline_counts_simulated_time():
    deadline = Deadline(100.0, stall=60.0, startup=300.0)
    assert deadline.check(None, 0.0) is None
    assert deadline.check(None, 30.0) is None
    assert deadline.check(5.0, 40.0) is None
    # A simulação a meia velocidade: 90 s simulados em 180 s de parede ainda estão no prazo.
    assert deadline.check(95.0, 220.0) is None
    assert deadline.check(105.1, 240.0) == 'timeout'


def test_deadline_detects_a_stalled_clock():
    deadline = Deadline(100.0, stall=60.0)
    assert deadline.check(10.0, 0.0) is None
    assert deadline.check(10.0, 59.0) is None
    assert deadline.check(10.0, 61.0) == 'failed'
    never = Deadline(100.0, stall=60.0, startup=300.0)
    assert never.check(None, 0.0) is None
    assert never.check(None, 200.0) is None
    assert never.check(None, 301.0) == 'failed'
    late = Deadline(100.0, stall=60.0, startup=300.0)
    assert late.check(None, 0.0) is None
    assert late.check(1.0, 250.0) is None
    assert late.check(2.0, 300.0) is None
