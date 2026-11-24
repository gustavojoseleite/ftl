"""
Vídeo do seguimento numa execução, nas curvas, a partir da pose real gravada no bag.

Uso, no contêiner (o ``imageio-ffmpeg`` dá o codificador):
``python3 b0-video.py <execução> --out <mp4> [--curves 0,1,2,3] [--speed 2] [--fps 30]``.

Cada curva vira um trecho: começa quando o líder está 30 m antes da entrada e termina quando o
último trator está 20 m depois da saída. O quadro tem dois painéis, em inglês como o artigo:
- à esquerda, a vista de cima que acompanha o comboio, com a rota ideal, o contorno de cada
  veículo e o rastro dos últimos segundos;
- à direita, o desvio para dentro da curva de cada veículo ao longo da rota, desenhado até o
  instante do quadro, com a posição atual marcada.
"""

import argparse
import math
from pathlib import Path

from ftl_analysis import bag
from ftl_analysis.geometry import reference_point
from ftl_analysis.segments import segments
from ftl_route_driver.protocol import OVERHANGS
from ftl_route_driver.routes import route as load_route
import imageio_ffmpeg
import matplotlib
import numpy as np
import yaml

BEFORE = 30.0  # o trecho começa com o líder a esta distância da entrada da curva, em m
AFTER = 20.0  # e termina com o último trator a esta distância depois da saída, em m
VIEW = 34.0  # meia largura da vista de cima, em m
TRAIL = 12.0  # duração do rastro, em s de simulação
WIDTH = {'pickup': 1.9, 'autonomous_tractor': 1.8}  # largura dos veículos, em m
SURFACE = '#fcfcfb'
INK = '#0b0b0b'
MUTED = '#52514e'
GRID = '#e4e3df'
ROUTE = '#bdbcb6'
LEADER = '#52514e'
TRACTORS = ['#6da7ec', '#3987e5', '#1c5cab', '#0d366b']


def _yaw(q):
    """Dá o rumo, em rad, de um quaternion planar."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _vehicles(config):
    """Dá ``{veículo: tipo}`` na ordem da fila."""
    leader = config['convoy']['leader']
    robots = {leader: leader}
    robots.update({f'tractor_{i + 1}': config['convoy']['follower']
                   for i in range(config['convoy']['followers'])})
    return robots


def _series(samples, robot):
    """Monta ``(t, x, y, rumo, xr, yr)`` de um veículo, com o rumo desembrulhado."""
    rows = []
    for sample in sorted(samples, key=lambda item: item.time):
        pose = sample.message.pose
        yaw = _yaw(pose.orientation)
        xr, yr = reference_point(pose.position.x, pose.position.y, yaw, robot)
        rows.append((sample.time, pose.position.x, pose.position.y, yaw, xr, yr))
    data = np.array(rows).reshape(-1, 6)
    data[:, 3] = np.unwrap(data[:, 3])
    return data


def _at(data, t):
    """Interpola ``(x, y, rumo, xr, yr)`` no instante ``t``."""
    return [float(np.interp(t, data[:, 0], data[:, k])) for k in range(1, 6)]


def _outline(x, y, yaw, robot):
    """Dá os cantos do contorno do veículo em volta da base, em m."""
    rear, front = OVERHANGS[robot]
    half = WIDTH[robot] / 2.0
    corners = np.array([(-rear, -half), (front, -half), (front, half), (-rear, half)])
    c, s = math.cos(yaw), math.sin(yaw)
    return corners @ np.array([[c, s], [-s, c]]) + np.array([x, y])


def _color(vehicle):
    """Dá a cor do veículo: cinza para o líder e a rampa azul para os tratores."""
    if vehicle.startswith('tractor_'):
        return TRACTORS[(int(vehicle.split('_')[1]) - 1) % len(TRACTORS)]
    return LEADER


def _name(vehicle):
    """Dá o nome do veículo usado no artigo."""
    return f'T{vehicle.split("_")[1]}' if vehicle.startswith('tractor_') else 'Leader'


def _window(route, part, data, names):
    """Acha o intervalo de tempo do trecho de uma curva: ``(t0, t1)``."""
    leader, last = names[0], names[-1]

    def first_time(vehicle, target):
        progress = np.array([route.project(x, y)[0] for x, y in data[vehicle][:, 4:6]])
        moving = np.nonzero((progress >= target) & (progress < target + 20.0))[0]
        if not len(moving):
            raise SystemExit(f'{vehicle} never reached s = {target:.0f} m')
        return float(data[vehicle][moving[0], 0])

    return first_time(leader, part.s0 - BEFORE), first_time(last, part.s1 + AFTER)


def _inward(route, part, points):
    """Dá ``(s desde a entrada, desvio para dentro)`` dos pontos perto da curva."""
    rows = []
    for x, y in points:
        s, lateral = route.project(float(x), float(y))
        rows.append((s - part.s0, part.turn * lateral))
    return np.array(rows).reshape(-1, 2)


def render(run, out, curves, speed, fps):
    """Grava o vídeo das curvas pedidas."""
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'serif', 'font.size': 11, 'axes.edgecolor': MUTED,
                         'xtick.color': MUTED, 'ytick.color': MUTED, 'axes.grid': True,
                         'grid.color': GRID, 'grid.linewidth': 0.5, 'axes.facecolor': SURFACE,
                         'figure.facecolor': SURFACE, 'axes.spines.top': False,
                         'axes.spines.right': False, 'legend.frameon': False})
    run = Path(run)
    config = yaml.safe_load((run / 'config.yaml').read_text(encoding='utf-8'))
    route = load_route(Path(config['world']).stem)
    robots = _vehicles(config)
    samples = bag.read(run, [f'/ground_truth/{v}/pose' for v in robots])
    data = {v: _series(samples[f'/ground_truth/{v}/pose'], r) for v, r in robots.items()}
    names = list(robots)
    parts = [p for p in segments(route) if p.kind == 'curve']
    along = np.arange(0.0, route.length, 0.25)
    line = np.array([route.at(float(s))[:2] for s in along])
    figure, (top, side) = plt.subplots(1, 2, figsize=(12.8, 7.2), dpi=100,
                                       gridspec_kw={'width_ratios': [1.0, 1.0]})
    size = figure.canvas.get_width_height()
    writer = imageio_ffmpeg.write_frames(str(out), size, fps=fps, macro_block_size=1,
                                         quality=8)
    writer.send(None)
    for index in curves:
        part = parts[index]
        t0, t1 = _window(route, part, data, names)
        offsets = {v: _inward(route, part, data[v][:, 4:6]) for v in names}
        for t in np.arange(t0, t1, speed / fps):
            top.clear()
            side.clear()
            state = {v: _at(data[v], t) for v in names}
            center = np.mean([state[v][:2] for v in names[1:3]], axis=0)
            top.plot(line[:, 0], line[:, 1], color=ROUTE, linewidth=6.0, zorder=1)
            for vehicle in names:
                d = data[vehicle]
                recent = (d[:, 0] >= t - TRAIL) & (d[:, 0] <= t)
                top.plot(d[recent, 4], d[recent, 5], color=_color(vehicle), linewidth=1.2,
                         zorder=2)
                x, y, yaw = state[vehicle][:3]
                top.fill(*_outline(x, y, yaw, robots[vehicle]).T, color=_color(vehicle),
                         zorder=3, label=_name(vehicle))
            top.set_xlim(center[0] - VIEW, center[0] + VIEW)
            top.set_ylim(center[1] - VIEW, center[1] + VIEW)
            top.set_aspect('equal')
            top.set_xlabel('x (m)')
            top.set_ylabel('y (m)')
            top.legend(loc='upper left', fontsize=10)
            top.set_title(f'B0, curve {index + 1} of {len(parts)}   t = {t - t0:5.1f} s',
                          loc='left', fontsize=12, color=INK)
            side.axvspan(0.0, part.s1 - part.s0, color=GRID, alpha=0.6, linewidth=0)
            side.axhline(0.0, color=ROUTE, linewidth=2.0)
            for vehicle in names:
                past = data[vehicle][:, 0] <= t
                offset = offsets[vehicle][past]
                shown = (offset[:, 0] > -15.0) & (offset[:, 0] < part.s1 - part.s0 + 15.0)
                side.plot(offset[shown, 0], offset[shown, 1], color=_color(vehicle),
                          linewidth=1.6)
                if shown.any():
                    last = np.nonzero(shown)[0][-1]
                    side.plot(*offset[last], 'o', color=_color(vehicle), markersize=6)
            side.set_xlim(-15.0, part.s1 - part.s0 + 15.0)
            side.set_ylim(-0.6, 0.6)
            side.set_xlabel('distance from curve entry along the route (m)')
            side.set_ylabel('inward deviation (m)')
            side.set_title('Corner cut: positive = inside the curve', loc='left', fontsize=12,
                           color=INK)
            side.text(0.5 * (part.s1 - part.s0), -0.57, 'curve', ha='center', color=MUTED)
            figure.tight_layout()
            figure.canvas.draw()
            frame = np.asarray(figure.canvas.buffer_rgba())[:, :, :3]
            writer.send(np.ascontiguousarray(frame))
    writer.close()
    plt.close(figure)


def main(argv=None):
    """Grava o vídeo das curvas de uma execução."""
    parser = argparse.ArgumentParser(description='Render a video of the convoy in the curves.')
    parser.add_argument('run', help='run folder, with config.yaml and bag/')
    parser.add_argument('--out', required=True, help='output MP4 file')
    parser.add_argument('--curves', default='0,1,2,3', help='curve indexes; default all four')
    parser.add_argument('--speed', type=float, default=2.0,
                        help='simulation seconds per video second; default 2')
    parser.add_argument('--fps', type=int, default=30, help='frames per second; default 30')
    args = parser.parse_args(argv)
    curves = [int(c) for c in args.curves.split(',')]
    render(args.run, args.out, curves, args.speed, args.fps)


if __name__ == '__main__':
    main()
