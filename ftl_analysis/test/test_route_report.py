"""Testes do resumo, da tabela e da figura do erro à rota, sobre execuções montadas à mão."""
import math

from ftl_analysis.analysis import _route_rows
from ftl_analysis.report import route_figure, route_summary, route_table
from ftl_analysis.segments import Segment
import numpy as np
import pandas as pd
import pytest


def _runs():
    rows = []
    for run, scale in (('a/1', 1.0), ('a/2', 3.0)):
        for vehicle, base in (('pickup', 0.2), ('tractor_1', 0.5)):
            rows += [('a', run, 'straight_0', 'straight', vehicle, 'route_lateral_rmse',
                      base * scale),
                     ('a', run, 'straight_1', 'straight', vehicle, 'route_lateral_rmse',
                      base * scale + 1.0),
                     ('a', run, 'curve_0', 'curve', vehicle, 'route_corner_cut', base * scale)]
    columns = ['label', 'run', 'segment', 'kind', 'vehicle', 'metric', 'value']
    return pd.DataFrame(rows, columns=columns)


def test_route_summary_averages_segments_then_runs():
    summary = route_summary(_runs())
    row = summary[(summary.vehicle == 'pickup') & (summary.kind == 'straight')].iloc[0]
    assert row['n'] == 2
    assert row['mean'] == pytest.approx((0.7 + 1.1) / 2)
    assert row['ci95'] > 0


def test_route_summary_keeps_the_queue_order():
    assert list(dict.fromkeys(route_summary(_runs()).vehicle)) == ['pickup', 'tractor_1']


def test_route_table_has_a_row_per_kind():
    text = route_table(route_summary(_runs()))
    assert text.startswith(r'\begin{tabular}{llrr}')
    assert 'retas' in text and 'curvas' in text and r'tractor\_1' in text


def test_route_figure_is_written(tmp_path):
    route_figure(route_summary(_runs()), tmp_path / 'route_error.pdf')
    assert (tmp_path / 'route_error.pdf').stat().st_size > 1000


def test_route_rows_only_cut_corners_in_curves():
    lateral = np.array([-0.1, -0.3, 0.2, 0.0])
    sel = np.ones(4, dtype=bool)
    curve = dict(_route_rows(Segment('curve', 0, 0.0, 10.0, -1), lateral, sel))
    straight = dict(_route_rows(Segment('straight', 0, 0.0, 10.0, 0), lateral, sel))
    assert curve['route_corner_cut'] == pytest.approx(0.3)
    assert curve['route_lateral_mean'] == pytest.approx(-0.05)
    assert curve['route_lateral_max'] == pytest.approx(0.3)
    assert 'route_corner_cut' not in straight
    assert math.isclose(straight['route_lateral_rmse'], math.sqrt(0.14 / 4))
