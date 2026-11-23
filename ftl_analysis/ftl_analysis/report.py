"""
O comando ``analyze``: lê uma campanha e grava as tabelas e a figura das métricas.

Uso: ``ros2 run ftl_analysis analyze <campanha> [--out <pasta>]``. A campanha tem as execuções em
``<campanha>/<rótulo>/<semente>/``, como o ``ftl_experiments`` as grava.
"""

import argparse
import logging
from pathlib import Path

from ftl_analysis.analysis import analyze_run
from ftl_analysis.stats import mean_ci
import matplotlib
import pandas as pd
import yaml

LOG = logging.getLogger('ftl_analysis')
SUMMARY_COLUMNS = ['label', 'segment', 'vehicle', 'metric', 'n', 'mean', 'ci95']


def find_runs(campaign, reasons=('done',)):
    """
    Acha as execuções terminadas de uma campanha.

    Args:
        campaign: A pasta da campanha.
        reasons: Os motivos de fim aceitos no ``run.yaml``.

    Returns:
        As pastas ``<campanha>/<rótulo>/<semente>`` com ``run.yaml`` de motivo em ``reasons``,
        em ordem.
    """
    runs = []
    for path in sorted(Path(campaign).glob('*/*/run.yaml')):
        reason = yaml.safe_load(path.read_text(encoding='utf-8')).get('reason')
        if reason in reasons:
            runs.append(path.parent)
        else:
            LOG.warning('skipping %s: reason is %s', path.parent, reason)
    return runs


def summarize(runs):
    """
    Resume as execuções de cada rótulo, trecho, trator e métrica.

    Args:
        runs: O ``DataFrame`` das execuções, com a coluna ``label``.

    Returns:
        Um ``DataFrame`` com ``label``, ``segment``, ``vehicle``, ``metric``, ``n``, ``mean`` e
        ``ci95`` (a meia-largura do intervalo de confiança de 95%).
    """
    rows = []
    for key, group in runs.groupby(['label', 'segment', 'vehicle', 'metric'], sort=False):
        mean, half = mean_ci(group['value'])
        rows.append((*key, len(group), mean, half))
    return pd.DataFrame(rows, columns=SUMMARY_COLUMNS)


def _escape(text):
    """Escapa os sublinhados para o LaTeX."""
    return text.replace('_', r'\_')


def lateral_table(summary):
    """
    Monta a tabela LaTeX do ``lateral_rmse`` por rótulo, trecho e trator.

    Args:
        summary: O resumo devolvido por ``summarize``.

    Returns:
        O texto do ambiente ``tabular``, com a média ± o intervalo de confiança, em m.
    """
    rows = summary[summary.metric == 'lateral_rmse']
    vehicles = list(dict.fromkeys(rows.vehicle))
    lines = [r'\begin{tabular}{ll' + 'r' * len(vehicles) + '}', r'\hline',
             ' & '.join(['Rótulo', 'Trecho'] + [_escape(v) for v in vehicles]) + r' \\',
             r'\hline']
    for (label, segment), group in rows.groupby(['label', 'segment'], sort=False):
        cells = []
        for vehicle in vehicles:
            found = group[group.vehicle == vehicle]
            if found.empty:
                cells.append('-')
                continue
            mean, half = found['mean'].iloc[0], found['ci95'].iloc[0]
            cells.append(f'{mean:.3f}' if pd.isna(half) else f'${mean:.3f} \\pm {half:.3f}$')
        lines.append(' & '.join([_escape(label), _escape(segment)] + cells) + r' \\')
    return '\n'.join(lines + [r'\hline', r'\end{tabular}']) + '\n'


def lateral_figure(summary, path):
    """
    Desenha as barras do ``lateral_rmse`` por trecho e trator, com o intervalo de confiança.

    Args:
        summary: O resumo devolvido por ``summarize``.
        path: O arquivo PDF a gravar.
    """
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows = summary[summary.metric == 'lateral_rmse']
    labels = list(dict.fromkeys(rows.label))
    figure, axes = plt.subplots(len(labels), 1, squeeze=False, sharex=True,
                                figsize=(10, 3 * len(labels)))
    for axis, label in zip(axes[:, 0], labels):
        part = rows[rows.label == label]
        segments = list(dict.fromkeys(part.segment))
        vehicles = list(dict.fromkeys(part.vehicle))
        width = 0.8 / len(vehicles)
        for k, vehicle in enumerate(vehicles):
            found = part[part.vehicle == vehicle].set_index('segment').reindex(segments)
            axis.bar([i + k * width for i in range(len(segments))], found['mean'].fillna(0),
                     width, yerr=found['ci95'].fillna(0), capsize=2, label=vehicle)
        axis.set_xticks([i + 0.4 - width / 2 for i in range(len(segments))])
        axis.set_xticklabels(segments, rotation=90)
        axis.set_ylabel('lateral_rmse (m)')
        axis.set_title(label)
        axis.legend()
    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)


def main(argv=None):
    """Analisa uma campanha e grava as tabelas e a figura na pasta de saída."""
    parser = argparse.ArgumentParser(description='Compute the metrics of a campaign.')
    parser.add_argument('campaign', help='campaign folder, with <label>/<seed>/ runs')
    parser.add_argument('--out', default=None, help='output folder; default <campaign>/analysis')
    parser.add_argument('--reasons', default='done',
                        help='comma-separated run reasons to analyze; default done')
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    campaign = Path(args.campaign)
    out = Path(args.out) if args.out else campaign / 'analysis'
    frames = []
    for folder in find_runs(campaign, tuple(args.reasons.split(','))):
        LOG.info('analyzing %s', folder)
        frame = analyze_run(folder)
        frame.insert(0, 'label', folder.parent.name)
        frames.append(frame)
    if not frames:
        raise SystemExit(f'no finished runs under {campaign}')
    runs = pd.concat(frames, ignore_index=True)
    summary = summarize(runs)
    out.mkdir(parents=True, exist_ok=True)
    runs.to_csv(out / 'metrics_runs.csv', index=False)
    summary.to_csv(out / 'metrics_summary.csv', index=False)
    (out / 'table_lateral.tex').write_text(lateral_table(summary), encoding='utf-8')
    lateral_figure(summary, out / 'lateral_rmse.pdf')
    LOG.info('wrote the tables and the figure to %s', out)
