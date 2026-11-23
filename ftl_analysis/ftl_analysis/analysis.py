"""
A análise de uma execução: do bag às métricas por trecho e por trator (seção 8.4 do plano).

Python puro, sem ``rclpy``. O caminho de cada veículo é o estimado (``ftl/state``), no ponto de
referência; o erro lateral de um trator é a distância com sinal ao caminho do líder. A pose real
(``ground_truth``) mede o erro da própria estimativa e, projetada na rota ideal, o erro de cada
veículo à rota (métricas ``route_``), que mostra o problema da B0.
"""

from collections import namedtuple
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ftl_analysis import bag
from ftl_analysis.geometry import ANTENNA, Polyline, reference_point
from ftl_analysis.metrics import corner_cut, rms_rate, string_stability, summary
from ftl_analysis.segments import locate, segments
from ftl_route_driver.routes import route as load_route
import numpy as np
import pandas as pd
import yaml

COLUMNS = ['run', 'seed', 'segment', 'kind', 'vehicle', 'metric', 'value']
START_FRACTION = 0.9  # a partida acaba quando todos passam desta fração do cruzeiro
MIN_SAMPLES = 3  # menos amostras que isto num trecho não dão métrica
ROUTE_STEP = 0.5  # passo, em m, da rota amostrada em ``paths``
EDGE = 1e-6  # folga, em m, para dizer que uma projeção caiu na ponta do caminho

# time e recorded em s; ref é o ponto de referência (n, 2), em m; speed em m/s.
Track = namedtuple('Track', 'time recorded ref speed')
# s: arco no caminho do líder, em m; lateral: erro com sinal, em m; ok: a projeção caiu dentro.
Projection = namedtuple('Projection', 's lateral ok')
# Série do cmd_vel: instante de gravação em s, linear.x em m/s e angular.z em rad/s.
Command = namedtuple('Command', 'recorded linear angular')
Context = namedtuple('Context', 'parts line start target')


def _yaw(q):
    """Dá o rumo, em rad, de um quaternion planar."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _track(samples, robot, odometry):
    """Monta a trilha de um veículo no ponto de referência, ordenada no tempo."""
    rows = []
    for sample in sorted(samples, key=lambda item: item.time):
        message = sample.message
        pose = message.pose.pose if odometry else message.pose
        x, y = reference_point(pose.position.x, pose.position.y, _yaw(pose.orientation), robot)
        speed = message.twist.twist.linear.x if odometry else 0.0
        rows.append((sample.time, sample.recorded, x, y, speed))
    data = np.array(rows).reshape(-1, 5)
    return Track(data[:, 0], data[:, 1], data[:, 2:4], data[:, 4])


def _project(line, track):
    """Projeta a trilha no caminho do líder."""
    data = np.array([line.project(x, y) for x, y in track.ref]).reshape(-1, 2)
    s, lateral = data[:, 0], data[:, 1]
    return Projection(s, lateral, (s > EDGE) & (s < line.length - EDGE))


def _target_gap(config):
    """Lê o espaçamento alvo, em m de base a base, do arquivo de parâmetros do ``ftl``."""
    path = config['ftl']['params_file']
    if not path:
        path = Path(get_package_share_directory('ftl_bringup')) / 'config' / 'ftl.yaml'
    params = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    return float(params['/**/follower']['ros__parameters']['gap'])


def _start_time(tracks, names, cruise):
    """Dá o instante em que todos os tratores passaram de 90% do cruzeiro, ou ``None``."""
    start = -math.inf
    for name in names:
        fast = tracks[name].speed > START_FRACTION * cruise
        if not fast.any():
            return None
        start = max(start, tracks[name].time[np.argmax(fast)])
    return start


def _longitudinal(mine, prev, vehicles, tracks, projections, context):
    """Dá o erro longitudinal de cada amostra do trator, e se ele vale (``ok``)."""
    here, ahead = projections[mine], projections[prev]
    times, s_ahead = tracks[prev].time[ahead.ok], ahead.s[ahead.ok]
    if len(times) < 2:
        return np.zeros(len(here.s)), np.zeros(len(here.s), dtype=bool)
    s_prev = np.interp(tracks[mine].time, times, s_ahead)
    gap = (s_prev - ANTENNA[vehicles[prev]]) - (here.s - ANTENNA[vehicles[mine]])
    inside = (tracks[mine].time >= times[0]) & (tracks[mine].time <= times[-1])
    return gap - context.target, here.ok & inside


def _estimation(track, truth):
    """Dá a distância da estimativa à pose real interpolada no tempo, e se ela vale."""
    ref_x = np.interp(track.time, truth.time, truth.ref[:, 0])
    ref_y = np.interp(track.time, truth.time, truth.ref[:, 1])
    inside = (track.time >= truth.time[0]) & (track.time <= truth.time[-1])
    return np.hypot(track.ref[:, 0] - ref_x, track.ref[:, 1] - ref_y), inside


def _segment_rows(part, data, cmd):
    """Dá as métricas de um trator num trecho, como pares ``(métrica, valor)``."""
    rows = []
    lateral = data['lateral'][data['sel'] & data['lat_ok']]
    if len(lateral) >= 2:
        rows += list(summary(lateral).items())
        rows = [(f'lateral_{name}', value) for name, value in rows]
        if part.kind == 'curve':
            rows.append(('corner_cut', corner_cut(lateral, part.turn)))
    longitudinal = data['longitudinal'][data['sel'] & data['long_ok']]
    if len(longitudinal) >= 2:
        rows.append(('longitudinal_rmse', summary(longitudinal)['rmse']))
    error = data['estimation'][data['sel'] & data['est_ok']]
    if len(error) >= 2:
        rows.append(('estimation_rmse', summary(error)['rmse']))
    if cmd is not None and data['sel'].any():
        recorded = data['recorded'][data['sel']]
        window = (cmd.recorded >= recorded.min()) & (cmd.recorded <= recorded.max())
        if window.sum() >= MIN_SAMPLES:
            rows.append(('steer_rate_rms', rms_rate(cmd.recorded[window], cmd.angular[window])))
            rows.append(('accel_rms', rms_rate(cmd.recorded[window], cmd.linear[window])))
    return rows


def _command(samples):
    """Monta a série do ``cmd_vel``, ordenada no tempo de gravação."""
    samples = sorted(samples, key=lambda item: item.recorded)
    data = np.array([(s.recorded, s.message.linear.x, s.message.angular.z)
                     for s in samples]).reshape(-1, 3)
    return Command(data[:, 0], data[:, 1], data[:, 2])


def _route_rows(part, lateral, sel):
    """Dá as métricas do erro à rota, na pose real, de um veículo num trecho."""
    values = lateral[sel]
    if len(values) < 2:
        return []
    rows = [('route_lateral_mean', float(np.mean(values))),
            ('route_lateral_rmse', summary(values)['rmse']),
            ('route_lateral_max', summary(values)['max'])]
    if part.kind == 'curve':
        rows.append(('route_corner_cut', corner_cut(values, part.turn)))
    return rows


def _route_error(samples, robots, route, parts, start):
    """
    Mede o erro à rota ideal pela pose real, na ordem da fila.

    Returns:
        As linhas ``(trecho, tipo, veículo, métrica, valor)`` e o RMS do erro de cada veículo na
        volta, ``nan`` se o veículo não tem a pose real.
    """
    rows, norms = [], []
    for vehicle, robot in robots.items():
        truth = samples.get(f'/ground_truth/{vehicle}/pose')
        if not truth:
            norms.append(math.nan)
            continue
        track = _track(truth, robot, False)
        data = np.array([route.project(x, y) for x, y in track.ref]).reshape(-1, 2)
        mask = track.time >= start
        place = np.where(mask, locate(parts, data[:, 0]), -1)
        for number, part in enumerate(parts):
            rows += [(part.name, part.kind, vehicle, metric, value)
                     for metric, value in _route_rows(part, data[:, 1], place == number)]
        norms.append(_rms(data[mask, 1]))
    return rows, norms


def _rms(values):
    """Dá o RMS de uma série, ou ``nan`` se ela for vazia."""
    return float(np.sqrt(np.mean(values ** 2))) if len(values) else math.nan


def analyze_run(folder):
    """
    Calcula as métricas de uma execução.

    As amostras da partida são descartadas, até todos os tratores passarem de 90% da velocidade
    de cruzeiro. Uma métrica sem os tópicos que a alimentam fica fora do resultado.

    Args:
        folder: A pasta da execução, com ``config.yaml``, ``run.yaml`` e ``bag/``.

    Returns:
        Um ``DataFrame`` com as colunas ``run``, ``seed``, ``segment``, ``kind``, ``vehicle``,
        ``metric`` e ``value``; o trecho ``lap`` é a volta inteira.
    """
    folder = Path(folder)
    config = yaml.safe_load((folder / 'config.yaml').read_text(encoding='utf-8'))
    seed = yaml.safe_load((folder / 'run.yaml').read_text(encoding='utf-8'))['seed']
    leader = config['convoy']['leader']
    names = [f'tractor_{i + 1}' for i in range(config['convoy']['followers'])]
    robots = {leader: leader}
    robots.update({name: config['convoy']['follower'] for name in names})
    topics = []
    for vehicle in robots:
        topics += [f'/{vehicle}/ftl/state', f'/ground_truth/{vehicle}/pose',
                   f'/{vehicle}/cmd_vel']
    samples = bag.read(folder, topics)
    if any(f'/{vehicle}/ftl/state' not in samples for vehicle in [leader] + names):
        return pd.DataFrame(columns=COLUMNS)
    tracks = {v: _track(samples[f'/{v}/ftl/state'], robots[v], True) for v in robots}
    cruise = config['route']['cruise_kmh'] / 3.6
    start = _start_time(tracks, names, cruise)
    if start is None:
        return pd.DataFrame(columns=COLUMNS)
    route = load_route(Path(config['world']).stem)
    line = Polyline(tracks[leader].ref)
    context = Context(segments(route), line, start, _target_gap(config))
    projections = {v: _project(line, tracks[v]) for v in robots}
    # O caminho do líder é a própria referência: toda amostra dele vale como precedente.
    leader_projection = projections[leader]
    projections[leader] = leader_projection._replace(ok=np.ones(len(leader_projection.s), bool))
    rows = []
    norms = {'lateral': [], 'longitudinal': []}
    for index, name in enumerate(names):
        track, projection = tracks[name], projections[name]
        mask = track.time >= start
        error, valid = _longitudinal(name, ([leader] + names)[index], robots, tracks,
                                     projections, context)
        data = {'recorded': track.recorded, 'lateral': projection.lateral,
                'lat_ok': projection.ok & mask, 'longitudinal': error, 'long_ok': valid & mask,
                'estimation': np.zeros(len(track.time)),
                'est_ok': np.zeros(len(track.time), dtype=bool)}
        truth = samples.get(f'/ground_truth/{name}/pose')
        if truth:
            error, inside = _estimation(track, _track(truth, robots[name], False))
            data['estimation'], data['est_ok'] = error, inside & mask
        commands = samples.get(f'/{name}/cmd_vel')
        commands = _command(commands) if commands else None
        route_s = np.array([route.project(x, y)[0] for x, y in track.ref]).reshape(-1)
        place = np.where(mask, locate(context.parts, route_s), -1)
        for number, part in enumerate(context.parts):
            data['sel'] = place == number
            rows += [(part.name, part.kind, name, metric, value)
                     for metric, value in _segment_rows(part, data, commands)]
        norms['lateral'].append(_rms(data['lateral'][data['lat_ok']]))
        norms['longitudinal'].append(_rms(data['longitudinal'][data['long_ok']]))
    for kind, values in norms.items():
        for name, gamma in zip(names[1:], string_stability(values)):
            rows.append(('lap', 'lap', name, f'gamma_{kind}', gamma))
    route_rows, route_norms = _route_error(samples, robots, route, context.parts, start)
    rows += route_rows
    for name, gamma in zip(names, string_stability(route_norms)):
        rows.append(('lap', 'lap', name, 'gamma_route', gamma))
    frame = pd.DataFrame(rows, columns=['segment', 'kind', 'vehicle', 'metric', 'value'])
    frame = frame[np.isfinite(frame['value'])]
    frame.insert(0, 'seed', seed)
    frame.insert(0, 'run', f'{folder.parent.name}/{folder.name}')
    return frame.reset_index(drop=True)[COLUMNS]


def paths(folder):
    """
    Dá a rota ideal e o caminho real de cada veículo, no ponto de referência.

    Args:
        folder: A pasta da execução, com ``config.yaml`` e ``bag/``.

    Returns:
        Um dicionário com ``route`` e o nome de cada veículo com pose real, cada um uma matriz
        ``(n, 2)`` de pontos, em m; o caminho segue a ordem do tempo.
    """
    folder = Path(folder)
    config = yaml.safe_load((folder / 'config.yaml').read_text(encoding='utf-8'))
    leader = config['convoy']['leader']
    robots = {leader: leader}
    robots.update({f'tractor_{i + 1}': config['convoy']['follower']
                   for i in range(config['convoy']['followers'])})
    samples = bag.read(folder, [f'/ground_truth/{v}/pose' for v in robots])
    route = load_route(Path(config['world']).stem)
    along = np.arange(0.0, route.length, ROUTE_STEP)
    data = {'route': np.array([route.at(float(s))[:2] for s in along]).reshape(-1, 2)}
    for vehicle, robot in robots.items():
        truth = samples.get(f'/ground_truth/{vehicle}/pose')
        if truth:
            data[vehicle] = _track(truth, robot, False).ref
    return data
