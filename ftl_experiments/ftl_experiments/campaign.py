"""
Plano de uma campanha de experimentos: pastas, execuções pendentes, tópicos e commits.

Este módulo é Python puro, sem ``rclpy``; o comando ``run`` em ``cli.py`` o usa.
"""

from dataclasses import dataclass
from pathlib import Path
import subprocess

from ftl_bringup.scenario import parse_overrides
from ftl_route_driver.routes import route
import yaml

REASONS = ('done', 'timeout', 'failed')
NO_ROUTE_LIMIT = 600.0  # tempo limite sem rota, em s
START_MARGIN = 120.0  # tempo de partida somado ao limite, em s


@dataclass
class Run:
    """Uma execução: a pasta, a semente, o cenário e as sobrescritas."""

    path: Path
    seed: int
    scenario: str
    overrides: dict


def plan(campaign, scenario, overrides, seeds, root, label=None):
    """
    Monta uma execução por semente.

    Args:
        campaign: Nome da campanha.
        scenario: Nome do preset do cenário.
        overrides: Sobrescritas como texto ``chave=valor``.
        seeds: Lista de sementes.
        root: Pasta raiz dos experimentos.
        label: Nome da variante; o padrão é o cenário.

    Returns:
        A lista de ``Run``, na ordem das sementes.
    """
    base = Path(root) / campaign / (label or scenario)
    runs = []
    for seed in seeds:
        values = dict(parse_overrides(overrides))
        values['degradation.seed'] = seed
        values['gui'] = False
        runs.append(Run(base / str(seed), seed, scenario, values))
    return runs


def pending(runs):
    """Devolve as execuções que ainda não terminaram em ``done``."""
    result = []
    for run in runs:
        try:
            finished = read_run(run.path)['reason'] == 'done'
        except FileNotFoundError:
            finished = False
        if not finished:
            result.append(run)
    return result


def time_limit(config):
    """
    Dá o tempo limite de uma execução.

    Args:
        config: A configuração resolvida do cenário.

    Returns:
        2 vezes o tempo da volta à velocidade de cruzeiro, mais a partida, em s; sem rota, 600.
    """
    if not config['route']['enabled']:
        return NO_ROUTE_LIMIT
    length = route(Path(config['world']).stem).length
    cruise = config['route']['cruise_kmh'] / 3.6
    return 2.0 * length / cruise + START_MARGIN


def write_run(path, reason, duration, seed, commits, attempts=None):
    """
    Grava o ``run.yaml`` na pasta; ``reason`` é ``done``, ``timeout`` ou ``failed``.

    O campo ``attempts`` só é gravado quando ``attempts`` vem.
    """
    if reason not in REASONS:
        raise ValueError(f'reason must be one of {REASONS}, got {reason!r}')
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    data = {'reason': reason, 'duration': float(duration), 'seed': seed,
            'commits': dict(commits)}
    if attempts is not None:
        data['attempts'] = attempts
    (path / 'run.yaml').write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')


class Deadline:
    """
    O prazo de uma execução, medido no tempo simulado.

    Args:
        limit: O limite, em s simulados, contado a partir do primeiro ``/clock``.
        stall: Quanto o ``/clock`` pode ficar parado, em s de parede, antes de ``failed``.
        startup: Quanto o ``/clock`` pode demorar a começar, em s de parede.
    """

    def __init__(self, limit, stall=60.0, startup=300.0):
        self.limit = limit
        self.stall = stall
        self.startup = startup
        self.first = None  # primeiro tempo simulado visto
        self.last = None  # último tempo simulado visto
        self.moved = 0.0  # tempo de parede em que o relógio avançou pela última vez

    def check(self, sim, wall):
        """
        Avalia o prazo.

        Args:
            sim: O tempo simulado, em s, ou ``None`` se o ``/clock`` ainda não chegou.
            wall: O tempo de parede desde o início da execução, em s.

        Returns:
            ``None`` enquanto segue, ``'timeout'`` ao passar do limite simulado e ``'failed'``
            se o ``/clock`` não começa ou para de avançar.
        """
        if sim is None:
            return 'failed' if wall > self.startup else None
        if self.first is None:
            self.first = sim
        if self.last is None or sim > self.last:
            self.last = sim
            self.moved = wall
        if sim - self.first > self.limit:
            return 'timeout'
        if wall - self.moved > self.stall:
            return 'failed'
        return None


def retry(attempt, retries):
    """
    Repete ``attempt`` enquanto o motivo do fim for ``failed``.

    Args:
        attempt: Função sem argumentos que roda a execução e devolve o motivo do fim.
        retries: O máximo de repetições, além da primeira tentativa.

    Returns:
        A tupla ``(motivo, tentativas)``.
    """
    attempts = 1
    reason = attempt()
    while reason == 'failed' and attempts <= retries:
        attempts += 1
        reason = attempt()
    return reason, attempts


def read_run(path):
    """Lê o ``run.yaml`` gravado por ``write_run``."""
    return yaml.safe_load((Path(path) / 'run.yaml').read_text(encoding='utf-8'))


def topics(config):
    """Devolve, em ordem e sem repetição, os tópicos que o bag grava."""
    leader = config['convoy']['leader']
    tractors = [f'tractor_{i + 1}' for i in range(config['convoy']['followers'])]
    names = []
    for vehicle in [leader] + tractors:
        names += [f'/ground_truth/{vehicle}/pose', f'/{vehicle}/ftl/state',
                  f'/{vehicle}/cmd_vel', f'/{vehicle}/degradation/state']
    for tractor in tractors:
        names += [f'/{tractor}/ftl/gap', f'/{tractor}/ftl/cross_track']
    if config['route']['enabled']:
        names += [f'/{leader}/route/error', f'/{leader}/route/progress',
                  f'/{leader}/route/done']
    names += ['/experiment/config', '/clock']
    return list(dict.fromkeys(names))


def commits(src):
    """Devolve o commit de cada pasta de ``src`` que é um repositório git."""
    result = {}
    for folder in sorted(Path(src).iterdir()):
        if (folder / '.git').exists():
            out = subprocess.run(['git', '-C', str(folder), 'rev-parse', 'HEAD'],
                                 capture_output=True, text=True, check=True)
            result[folder.name] = out.stdout.strip()
    return result
