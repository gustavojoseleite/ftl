"""
Figuras do caso de teste da B0 para o artigo, no tamanho de uma coluna do IEEE.

Uso, no contêiner:
``python3 b0-figuras.py <campanha> --run <rótulo>/<semente> --out <pasta> [--segment curve_0]``.

Grava duas figuras em inglês, como o artigo:
- ``b0_paths.pdf``: (a) a rota ideal e o caminho real de cada veículo, numa janela de 3 m em
  volta do maior corte do último trator; (b) o desvio
  para dentro da curva ao longo da rota, do líder ao T4;
- ``b0_error.pdf``: o erro nas retas e o corte de curva por posição na fila, com o intervalo de
  confiança de 95% entre as sementes, uma série por rótulo da campanha.
"""

import argparse
from pathlib import Path

from ftl_analysis.analysis import paths
from ftl_analysis.report import route_summary
from ftl_analysis.segments import segments
from ftl_route_driver.routes import route as load_route
import matplotlib
from matplotlib.ticker import MaxNLocator
import numpy as np
import pandas as pd
import yaml

COLUMN = 3.5  # largura de uma coluna do IEEE, em polegadas
MARGIN = 12.0  # trecho de reta mostrado antes e depois da curva, em m
HALF = 1.5  # meia largura, em m, da janela em planta, centrada no maior corte do último trator
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
MUTED = '#52514e'
GRID = '#e4e3df'
ROUTE = '#bdbcb6'
LEADER = '#52514e'
# Rampa ordinal de um só tom, do T1 (claro) ao T4 (escuro), validada contra a superfície clara.
TRACTORS = ['#6da7ec', '#3987e5', '#1c5cab', '#0d366b']
# Séries categóricas dos rótulos, na ordem fixa da paleta de referência.
SERIES = ['#2a78d6', '#e0692b', '#2e9e6e', '#8a5cd6', '#c9a227', '#d64f7c']


def _names(data):
    """Dá os veículos na ordem da fila, com o nome usado no artigo."""
    names = {}
    for vehicle in data:
        if vehicle == 'route':
            continue
        tractor = vehicle.startswith('tractor_')
        names[vehicle] = f'T{vehicle.split("_")[1]}' if tractor else 'Leader'
    return names


def _color(vehicle):
    """Dá a cor do veículo: cinza para o líder e a rampa azul para os tratores."""
    if vehicle.startswith('tractor_'):
        return TRACTORS[(int(vehicle.split('_')[1]) - 1) % len(TRACTORS)]
    return LEADER


def _series(label, index):
    """
    Dá a cor, o traço e o nome de um rótulo ``p<ângulo>_<condição>``.

    A cor segue o mundo (90°, 60°, 30°) e o traço, a condição: cheio no c0 e tracejado no campo.
    Outro rótulo leva a cor da sua posição e o traço cheio.
    """
    worlds = {'p90': 0, 'p60': 1, 'p30': 2}
    world, _, cond = label.partition('_')
    if world in worlds:
        dashes = '--' if cond == 'field' else '-'
        return SERIES[worlds[world]], dashes, f'{world[1:]}°, {cond}'
    return SERIES[index % len(SERIES)], '-', label.replace('_', ' ')


def _style(plt):
    """Ajusta o estilo para o artigo: fonte serifada, 8 pt e grade discreta."""
    plt.rcParams.update({
        'font.family': 'serif', 'font.size': 8, 'axes.labelsize': 8, 'legend.fontsize': 7,
        'xtick.labelsize': 7, 'ytick.labelsize': 7, 'axes.edgecolor': MUTED,
        'axes.labelcolor': INK, 'xtick.color': MUTED, 'ytick.color': MUTED,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.5,
        'axes.spines.top': False, 'axes.spines.right': False, 'figure.facecolor': SURFACE,
        'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE, 'pdf.fonttype': 42,
        'legend.frameon': False})


def _inward(route, part, points):
    """
    Projeta um caminho na rota perto do trecho.

    Returns:
        As linhas ``(s desde a entrada da curva, desvio para dentro, x, y)``, ordenadas em ``s``.
    """
    rows = []
    for x, y in points:
        s, lateral = route.project(float(x), float(y))
        if part.s0 - MARGIN <= s <= part.s1 + MARGIN:
            rows.append((s - part.s0, part.turn * lateral, x, y))
    rows = np.array(rows).reshape(-1, 4)
    return rows[np.argsort(rows[:, 0])]


def paths_figure(run, segment, out, plt):
    """Desenha a curva em planta e o desvio para dentro ao longo da rota."""
    config = yaml.safe_load((Path(run) / 'config.yaml').read_text(encoding='utf-8'))
    route = load_route(Path(config['world']).stem)
    part = next(p for p in segments(route) if p.name == segment)
    data = paths(run)
    names = _names(data)
    figure, (top, bottom) = plt.subplots(2, 1, figsize=(COLUMN, 5.0),
                                         gridspec_kw={'height_ratios': [1.4, 1.0]})
    offsets = {vehicle: _inward(route, part, data[vehicle]) for vehicle in names}
    last = list(names)[-1]
    peak = offsets[last][int(np.argmax(offsets[last][:, 1]))]
    center = np.array(peak[2:4])
    along = np.linspace(part.s0 - MARGIN, part.s1 + MARGIN, 2000)
    line = np.array([route.at(float(s))[:2] for s in along])
    low, high = center - HALF, center + HALF
    top.plot(line[:, 0], line[:, 1], color=ROUTE, linewidth=4.0, label='Ideal route', zorder=1)
    for vehicle, name in names.items():
        points = data[vehicle]
        inside = np.all((points >= low - 1.0) & (points <= high + 1.0), axis=1)
        top.plot(points[inside, 0], points[inside, 1], color=_color(vehicle), linewidth=1.0,
                 label=name, zorder=2)
    top.set_xlim(low[0], high[0])
    top.set_ylim(low[1], high[1])
    top.set_aspect('equal')
    top.set_xlabel('x (m)')
    top.set_ylabel('y (m)')
    top.legend(loc='upper left', handlelength=1.5)
    top.xaxis.set_major_locator(MaxNLocator(4))
    top.set_title('(a)', loc='left', fontsize=8)
    bottom.axvspan(0.0, part.s1 - part.s0, color=GRID, alpha=0.6, linewidth=0, zorder=0)
    bottom.axhline(0.0, color=ROUTE, linewidth=1.5, zorder=1)
    for vehicle, name in names.items():
        offset = offsets[vehicle]
        bottom.plot(offset[:, 0], offset[:, 1], color=_color(vehicle), linewidth=1.0, zorder=2)
        peak = int(np.argmax(offset[:, 1])) if len(offset) else None
        if peak is not None and vehicle.startswith('tractor_'):
            bottom.annotate(name, (offset[peak, 0], offset[peak, 1]), xytext=(3, 1),
                            textcoords='offset points', fontsize=7, color=INK)
    bottom.text(0.5 * (part.s1 - part.s0), bottom.get_ylim()[0], 'curve', ha='center',
                va='bottom', fontsize=7, color=MUTED)
    bottom.set_xlim(-MARGIN, part.s1 - part.s0 + MARGIN)
    bottom.set_xlabel('distance from curve entry along the route (m)')
    bottom.set_ylabel('inward deviation (m)')
    bottom.set_title('(b)', loc='left', fontsize=8)
    figure.tight_layout(pad=0.3, h_pad=0.6)
    figure.savefig(out / 'b0_paths.pdf')
    figure.savefig(out / 'b0_paths.png', dpi=200)
    plt.close(figure)


def error_figure(campaign, out, plt):
    """
    Desenha o erro nas retas e o corte de curva por posição na fila, com o intervalo.

    Uma linha de painéis por métrica (retas, curvas) e uma coluna por condição (c0, campo); em
    cada painel, uma série por mundo. A escala do eixo y é a mesma ao longo de cada linha e mostra
    os intervalos inteiros.
    """
    runs = pd.read_csv(Path(campaign) / 'analysis' / 'metrics_runs.csv')
    summary = route_summary(runs)
    vehicles = list(dict.fromkeys(summary.vehicle))
    ticks = ['L' if not v.startswith('tractor_') else f'T{v.split("_")[1]}' for v in vehicles]
    labels = list(dict.fromkeys(summary.label))
    conds = list(dict.fromkeys(label.partition('_')[2] or label for label in labels))
    worlds = list(dict.fromkeys(label.partition('_')[0] for label in labels))
    figure, axes = plt.subplots(2, len(conds), squeeze=False, sharey='row', sharex=True,
                                figsize=(COLUMN, 3.0))
    rows = (('straight', 'Straights: lateral\nRMSE (m)'), ('curve', 'Curves: corner\ncut (m)'))
    shift = np.linspace(-0.15, 0.15, len(worlds)) if len(worlds) > 1 else [0.0]
    titles = {'c0': 'Nominal sensors (c0)', 'field': 'Field sensors'}
    for r, (kind, ylabel) in enumerate(rows):
        for c, cond in enumerate(conds):
            axis = axes[r, c]
            for k, world in enumerate(worlds):
                label = f'{world}_{cond}' if f'{world}_{cond}' in labels else world
                part = summary[(summary.label == label) & (summary.kind == kind)]
                if part.empty:
                    continue
                found = part.set_index('vehicle').reindex(vehicles)
                color, _, _ = _series(label, k)
                name = f'{world[1:]}° curves' if world[1:].isdigit() else world
                axis.errorbar(np.arange(len(vehicles)) + shift[k], found['mean'],
                              yerr=found['ci95'].fillna(0.0), color=color, marker='o',
                              markersize=3, linewidth=1.0, capsize=1.5, elinewidth=0.7,
                              label=name)
            if r == 0:
                axis.set_title(titles.get(cond, cond), fontsize=8)
            if c == 0:
                axis.set_ylabel(ylabel)
            if r == 1:
                axis.set_xticks(range(len(vehicles)))
                axis.set_xticklabels(ticks)
    for r, (kind, _) in enumerate(rows):
        part = summary[summary.kind == kind]
        top = float((part['mean'] + part['ci95'].fillna(0.0)).max())
        axes[r, 0].set_ylim(0.0, 1.05 * top)
    axes[1, 0].legend(loc='upper left', handlelength=1.2)
    figure.tight_layout(pad=0.3, h_pad=0.4, w_pad=0.6)
    figure.savefig(out / 'b0_error.pdf')
    figure.savefig(out / 'b0_error.png', dpi=200)
    plt.close(figure)


def main(argv=None):
    """Grava as figuras do caso de teste da B0."""
    parser = argparse.ArgumentParser(description='Plot the B0 test-case figures for the paper.')
    parser.add_argument('campaign', help='campaign folder, already analyzed')
    parser.add_argument('--run', required=True, help='run for the path figure, as label/seed')
    parser.add_argument('--out', required=True, help='output folder')
    parser.add_argument('--segment', default='curve_0', help='curve to show; default curve_0')
    args = parser.parse_args(argv)
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    _style(plt)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    paths_figure(Path(args.campaign) / args.run, args.segment, out, plt)
    error_figure(args.campaign, out, plt)


if __name__ == '__main__':
    main()
