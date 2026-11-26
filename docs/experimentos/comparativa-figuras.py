"""
Figuras e tabelas da campanha comparativa B0 × E1 para o artigo, no tamanho de uma coluna do IEEE.

Uso, no contêiner, depois do ``analyze`` da campanha:
``python3 comparativa-figuras.py <campanha> --run p90_field/1 --out <pasta> [--segment curve_0]``.

Os rótulos da campanha são ``<caso>_p<ângulo>_<condição>`` (``b0_p90_c0``, ``e1_p30_field``...).
Grava, em inglês como o artigo:
- ``cmp_paths.pdf``: o desvio para dentro da curva ao longo da rota, do líder ao T4, na mesma
  curva e na mesma semente: (a) B0 e (b) E1, com a mesma escala;
- ``cmp_lap_p<ângulo>.pdf``: a volta inteira de cada mundo, desenrolada ao longo da rota: o
  desvio lateral de cada veículo (positivo para dentro das curvas), na mesma semente, uma linha
  por caso e condição;
- ``cmp_error.pdf``: o erro nas retas e o corte de curva por posição na fila, B0 e E1, com o
  intervalo de confiança de 95% entre as execuções dos três mundos; uma coluna por condição;
- ``cmp_corner_cut.tex`` e ``.csv``: o corte de curva por mundo, condição e trator, B0 e E1;
- ``cmp_summary.tex`` e ``.csv``: por caso e condição, o erro nas retas, o corte de curva e o γ
  médios do T1 ao T4 e o RMSE de estimação.
"""

import argparse
from pathlib import Path

from ftl_analysis.analysis import paths
from ftl_analysis.report import route_summary
from ftl_analysis.stats import mean_ci
from ftl_analysis.segments import segments
from ftl_route_driver.routes import route as load_route
import matplotlib
import numpy as np
import pandas as pd
import yaml

COLUMN = 3.5  # largura de uma coluna do IEEE, em polegadas
GAP = 2.0  # salto ao longo da rota que interrompe a linha, em m
MARGIN = 12.0  # trecho de reta mostrado antes e depois da curva, em m
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
MUTED = '#52514e'
GRID = '#e4e3df'
ROUTE = '#bdbcb6'
LEADER = '#52514e'
TRACTORS = ['#6da7ec', '#3987e5', '#1c5cab', '#0d366b']
# Cores categóricas dos casos, na ordem fixa da paleta de referência: B0 laranja, E1 azul.
CASES = {'b0': ('#e0692b', 'B0 (baseline)'), 'e1': ('#2a78d6', 'E1 (proposed)')}
TITLES = {'c0': 'Nominal sensors', 'field': 'Field sensors'}
TRACTOR_SET = ['tractor_1', 'tractor_2', 'tractor_3', 'tractor_4']


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


def split(label):
    """Separa ``<caso>_p<ângulo>_<condição>`` em ``(caso, ângulo, condição)``."""
    case, world, cond = label.split('_', 2)
    return case, int(world[1:]), cond


def _color(vehicle):
    """Dá a cor do veículo: cinza para o líder e a rampa azul para os tratores."""
    if vehicle.startswith('tractor_'):
        return TRACTORS[(int(vehicle.split('_')[1]) - 1) % len(TRACTORS)]
    return LEADER


def _inward(route, part, points):
    """Dá as linhas ``(s desde a entrada da curva, desvio para dentro)`` perto do trecho."""
    rows = []
    for x, y in points:
        s, lateral = route.project(float(x), float(y))
        if part.s0 - MARGIN <= s <= part.s1 + MARGIN:
            rows.append((s - part.s0, part.turn * lateral))
    rows = np.array(rows).reshape(-1, 2)
    return rows[np.argsort(rows[:, 0])]


def paths_figure(campaign, run, segment, out, plt):
    """Desenha o desvio para dentro da curva na B0 e no E1, na mesma curva e semente."""
    world_cond, seed = run.split('/')
    figure, axes = plt.subplots(2, 1, figsize=(COLUMN, 3.6), sharex=True, sharey=True)
    for axis, case, tag in zip(axes, ('b0', 'e1'), ('(a)', '(b)')):
        folder = Path(campaign) / f'{case}_{world_cond}' / seed
        config = yaml.safe_load((folder / 'config.yaml').read_text(encoding='utf-8'))
        route = load_route(Path(config['world']).stem)
        part = next(p for p in segments(route) if p.name == segment)
        data = paths(folder)
        axis.axvspan(0.0, part.s1 - part.s0, color=GRID, alpha=0.6, linewidth=0, zorder=0)
        axis.axhline(0.0, color=ROUTE, linewidth=1.5, zorder=1)
        for vehicle, points in data.items():
            if vehicle == 'route':
                continue
            offset = _inward(route, part, points)
            name = f'T{vehicle.split("_")[1]}' if vehicle.startswith('tractor_') else 'Leader'
            axis.plot(offset[:, 0], offset[:, 1], color=_color(vehicle), linewidth=1.0,
                      zorder=2, label=name)
        axis.set_xlim(-MARGIN, part.s1 - part.s0 + MARGIN)
        axis.set_ylabel('inward deviation (m)')
        axis.set_title(f'{tag} {CASES[case][1]}', loc='left', fontsize=8)
    # As curvas do E1 se sobrepõem, então a identidade vem de uma legenda só, no alto do painel
    # (b), que fica vazio.
    handles, labels = axes[0].get_legend_handles_labels()
    order = sorted(range(len(labels)), key=lambda i: (labels[i] != 'Leader', labels[i]))
    axes[1].legend([handles[i] for i in order], [labels[i] for i in order], ncol=5,
                   loc='upper right', handlelength=1.2, columnspacing=0.8)
    axes[1].set_xlabel('distance from curve entry along the route (m)')
    figure.tight_layout(pad=0.3, h_pad=0.6)
    figure.savefig(out / 'cmp_paths.pdf')
    figure.savefig(out / 'cmp_paths.png', dpi=200)
    plt.close(figure)


def _lateral(route, points):
    """Dá as linhas ``(s ao longo da rota, desvio para dentro)`` da volta inteira."""
    rows = np.array([route.project(float(x), float(y)) for x, y in points]).reshape(-1, 2)
    # Os tratores largam atrás do início da rota, e a rota fechada projeta essas posições perto
    # do fim dela: a volta só conta a partir da primeira passagem pelo começo.
    start = np.flatnonzero(rows[:, 0] < 0.5 * route.length)
    rows = rows[start[0]:] if start.size else rows[:0]
    rows[:, 1] *= -1.0  # as curvas dos mundos são todas à direita: dentro é o lado negativo
    # Onde a pose não avança ao longo da rota (o veículo parado no fim), a linha se interrompe.
    gaps = np.flatnonzero(np.abs(np.diff(rows[:, 0])) > GAP) + 1
    return np.insert(rows, gaps, np.nan, axis=0)


def lap_figure(campaign, world, seed, out, plt):
    """Desenha a volta inteira de um mundo, B0 e E1, nas duas condições de sensores."""
    rows = [(case, cond) for cond in ('c0', 'field') for case in ('b0', 'e1')
            if (Path(campaign) / f'{case}_p{world}_{cond}' / seed).is_dir()]
    figure, axes = plt.subplots(len(rows), 1, figsize=(2 * COLUMN, 1.2 * len(rows) + 0.5),
                                sharex=True, sharey=True, squeeze=False)
    for axis, (case, cond) in zip(axes[:, 0], rows):
        folder = Path(campaign) / f'{case}_p{world}_{cond}' / seed
        config = yaml.safe_load((folder / 'config.yaml').read_text(encoding='utf-8'))
        route = load_route(Path(config['world']).stem)
        for part in segments(route):
            if part.turn:
                axis.axvspan(part.s0, part.s1, color=GRID, alpha=0.6, linewidth=0, zorder=0)
        axis.axhline(0.0, color=ROUTE, linewidth=1.2, zorder=1)
        for vehicle, points in paths(folder).items():
            if vehicle == 'route':
                continue
            lateral = _lateral(route, points)
            name = f'T{vehicle.split("_")[1]}' if vehicle.startswith('tractor_') else 'Leader'
            axis.plot(lateral[:, 0], lateral[:, 1], color=_color(vehicle), linewidth=0.7,
                      zorder=2, label=name)
        axis.set_xlim(0.0, route.length)
        axis.set_ylabel('deviation (m)')
        axis.set_title(f'{CASES[case][1]}, {TITLES[cond].lower()}', loc='left', fontsize=8)
    axes[0, 0].set_ylim(-0.6, 1.0)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    order = sorted(range(len(labels)), key=lambda i: (labels[i] != 'Leader', labels[i]))
    axes[0, 0].legend([handles[i] for i in order], [labels[i] for i in order], ncol=5,
                      loc='upper right', handlelength=1.2, columnspacing=0.8)
    axes[-1, 0].set_xlabel(f'distance along the route (m); {world}° curves shaded, '
                           'positive deviation is toward the inside of the curves')
    figure.tight_layout(pad=0.3, h_pad=0.5)
    figure.savefig(out / f'cmp_lap_p{world}.pdf')
    figure.savefig(out / f'cmp_lap_p{world}.png', dpi=200)
    plt.close(figure)


def error_figure(runs, out, plt):
    """Desenha o erro nas retas e o corte de curva por posição na fila, B0 e E1."""
    pooled = runs.copy()
    parts = pooled.label.map(split)
    pooled['label'] = [f'{case}_{cond}' for case, _, cond in parts]
    pooled['run'] = runs.label + '/' + runs.run.astype(str)
    summary = route_summary(pooled)
    vehicles = list(dict.fromkeys(summary.vehicle))
    ticks = ['L' if not v.startswith('tractor_') else f'T{v.split("_")[1]}' for v in vehicles]
    conds = [c for c in ('c0', 'field') if any(summary.label.str.endswith(c))]
    figure, axes = plt.subplots(2, len(conds), squeeze=False, sharey='row', sharex=True,
                                figsize=(COLUMN, 3.0))
    rows = (('straight', 'Straights: RMSE (m)'), ('curve', 'Curves: corner cut (m)'))
    shift = {'b0': -0.08, 'e1': 0.08}
    for r, (kind, ylabel) in enumerate(rows):
        for c, cond in enumerate(conds):
            axis = axes[r, c]
            for case, (color, name) in CASES.items():
                part = summary[(summary.label == f'{case}_{cond}') & (summary.kind == kind)]
                if part.empty:
                    continue
                found = part.set_index('vehicle').reindex(vehicles)
                axis.errorbar(np.arange(len(vehicles)) + shift[case], found['mean'],
                              yerr=found['ci95'].fillna(0.0), color=color, marker='o',
                              markersize=3, linewidth=1.0, capsize=1.5, elinewidth=0.7,
                              label=name)
            if r == 0:
                axis.set_title(TITLES.get(cond, cond), fontsize=8)
            if c == 0:
                axis.set_ylabel(ylabel)
            if r == 1:
                axis.set_xticks(range(len(vehicles)))
                axis.set_xticklabels(ticks)
        top = float((summary[summary.kind == kind]['mean']
                     + summary[summary.kind == kind]['ci95'].fillna(0.0)).max())
        axes[r, 0].set_ylim(0.0, 1.05 * top)
    axes[1, 0].legend(loc='upper left', handlelength=1.2)
    figure.tight_layout(pad=0.3, h_pad=0.4, w_pad=0.6)
    figure.savefig(out / 'cmp_error.pdf', bbox_inches='tight', pad_inches=0.03)
    figure.savefig(out / 'cmp_error.png', dpi=200, bbox_inches='tight', pad_inches=0.03)
    plt.close(figure)


def _per_run(runs, kind, metric):
    """Dá a média por execução, rótulo e veículo de uma métrica num tipo de trecho."""
    part = runs[(runs.kind == kind) & (runs.metric == metric)]
    return part.groupby(['label', 'vehicle', 'run'])['value'].mean().reset_index()


def corner_cut_table(runs, out):
    """Grava o corte de curva por mundo, condição e trator, B0 e E1."""
    data = _per_run(runs, 'curve', 'route_corner_cut')
    rows = []
    for label, group in data.groupby('label'):
        case, world, cond = split(label)
        for vehicle, values in group.groupby('vehicle'):
            mean, half = mean_ci(values.value)
            rows.append((world, cond, case, vehicle, mean, half, len(values)))
    table = pd.DataFrame(rows, columns=['world', 'cond', 'case', 'vehicle', 'mean', 'ci95', 'n'])
    table = table.sort_values(['world', 'cond', 'case', 'vehicle'],
                              ascending=[False, True, True, True])
    table.to_csv(out / 'cmp_corner_cut.csv', index=False)
    lines = [r'\begin{tabular}{llcrrrr}', r'\hline',
             r'Curves & Sensors & Case & T1 & T2 & T3 & T4 \\', r'\hline']
    for (world, cond, case), group in table.groupby(['world', 'cond', 'case'], sort=False):
        by = group.set_index('vehicle')
        cells = [f"{by.loc[v, 'mean']:.2f}" if v in by.index else '--' for v in TRACTOR_SET]
        lines.append(f'{world}° & {TITLES.get(cond, cond).split()[0]} & {case.upper()} & '
                     + ' & '.join(cells) + r' \\')
    lines += [r'\hline', r'\end{tabular}']
    (out / 'cmp_corner_cut.tex').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return table


def summary_table(runs, out):
    """Grava, por caso e condição, as médias do T1 ao T4 das métricas da comparação."""
    metrics = (('straight', 'route_lateral_rmse', 'Straight RMSE'),
               ('curve', 'route_corner_cut', 'Corner cut'),
               ('lap', 'gamma_route', 'Gamma'),
               ('straight', 'estimation_rmse', 'Estimation RMSE'))
    rows = []
    for kind, metric, name in metrics:
        data = _per_run(runs, kind, metric)
        data = data[data.vehicle.isin(TRACTOR_SET)]
        if data.empty:
            continue
        data['group'] = [split(label)[0] + '_' + split(label)[2] for label in data.label]
        per_run = data.groupby(['group', 'label', 'run']).value.mean().reset_index()
        for group, values in per_run.groupby('group'):
            mean, half = mean_ci(values.value)
            case, cond = group.split('_')
            rows.append((case, cond, name, mean, half, len(values)))
    table = pd.DataFrame(rows, columns=['case', 'cond', 'metric', 'mean', 'ci95', 'n'])
    table.to_csv(out / 'cmp_summary.csv', index=False)
    names = list(dict.fromkeys(table.metric))
    lines = [r'\begin{tabular}{ll' + 'r' * len(names) + '}', r'\hline',
             ' & '.join(['Sensors', 'Case'] + names) + r' \\', r'\hline']
    for (cond, case), group in table.groupby(['cond', 'case']):
        by = group.set_index('metric')
        cells = [f"{by.loc[n, 'mean']:.3f}" if n in by.index else '--' for n in names]
        lines.append(f'{TITLES.get(cond, cond).split()[0]} & {case.upper()} & '
                     + ' & '.join(cells) + r' \\')
    lines += [r'\hline', r'\end{tabular}']
    (out / 'cmp_summary.tex').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return table


def main(argv=None):
    """Grava as figuras e as tabelas da comparação."""
    parser = argparse.ArgumentParser(description='Plot the B0 x E1 comparison for the paper.')
    parser.add_argument('campaign', help='campaign folder, already analyzed')
    parser.add_argument('--run', default='p90_field/1',
                        help='world, condition and seed for the path figure; default p90_field/1')
    parser.add_argument('--out', required=True, help='output folder')
    parser.add_argument('--segment', default='curve_0', help='curve to show; default curve_0')
    parser.add_argument('--seed', default='1', help='seed for the full-lap figures; default 1')
    args = parser.parse_args(argv)
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    _style(plt)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = pd.read_csv(Path(args.campaign) / 'analysis' / 'metrics_runs.csv')
    paths_figure(args.campaign, args.run, args.segment, out, plt)
    for world in (90, 60, 30):
        lap_figure(args.campaign, world, args.seed, out, plt)
    error_figure(runs, out, plt)
    print(corner_cut_table(runs, out).to_string(index=False))
    print(summary_table(runs, out).to_string(index=False))


if __name__ == '__main__':
    main()
