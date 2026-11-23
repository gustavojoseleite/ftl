"""
O comando ``plot_paths``: desenha a rota ideal e o caminho real de cada veículo de uma execução.

Uso: ``ros2 run ftl_analysis plot_paths <execução> --out <pdf> [--segment <trecho>]``. Com
``--segment``, um segundo painel mostra o trecho pedido, com uma folga de 5 m.
"""

import argparse
from pathlib import Path

from ftl_analysis.analysis import paths
from ftl_analysis.segments import segments
from ftl_route_driver.routes import route as load_route
import matplotlib
import numpy as np
import yaml

MARGIN = 5.0  # folga, em m, em volta do trecho do segundo painel


def _bounds(folder, name):
    """Dá os limites ``(x0, x1, y0, y1)`` de um trecho da rota, com a folga."""
    config = yaml.safe_load((Path(folder) / 'config.yaml').read_text(encoding='utf-8'))
    route = load_route(Path(config['world']).stem)
    for part in segments(route):
        if part.name == name:
            along = np.linspace(part.s0, part.s1, 50)
            points = np.array([route.at(float(s))[:2] for s in along])
            low, high = points.min(axis=0) - MARGIN, points.max(axis=0) + MARGIN
            return low[0], high[0], low[1], high[1]
    raise SystemExit(f'unknown segment {name}')


def _draw(axis, data):
    """Desenha a rota em preto e um traço por veículo, com os eixos na mesma escala."""
    axis.plot(data['route'][:, 0], data['route'][:, 1], color='black', linewidth=1.5,
              label='route')
    for vehicle, points in data.items():
        if vehicle != 'route':
            axis.plot(points[:, 0], points[:, 1], linewidth=1.0, label=vehicle)
    axis.set_aspect('equal')
    axis.set_xlabel('x (m)')
    axis.set_ylabel('y (m)')
    axis.grid(True, linewidth=0.3)


def main(argv=None):
    """Desenha a rota e os caminhos de uma execução num PDF."""
    parser = argparse.ArgumentParser(description='Plot the route and the vehicle paths of a run.')
    parser.add_argument('run', help='run folder, with config.yaml and bag/')
    parser.add_argument('--out', required=True, help='output PDF file')
    parser.add_argument('--segment', default=None, help='segment to zoom in, such as curve_0')
    args = parser.parse_args(argv)
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data = paths(args.run)
    count = 2 if args.segment else 1
    figure, axes = plt.subplots(1, count, squeeze=False, figsize=(8 * count, 7))
    _draw(axes[0, 0], data)
    axes[0, 0].legend()
    if args.segment:
        x0, x1, y0, y1 = _bounds(args.run, args.segment)
        _draw(axes[0, 1], data)
        axes[0, 1].set_xlim(x0, x1)
        axes[0, 1].set_ylim(y0, y1)
        axes[0, 1].set_title(args.segment)
    figure.tight_layout()
    figure.savefig(args.out)
    plt.close(figure)
