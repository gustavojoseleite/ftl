"""O comando ``suite``: roda o conjunto básico de casos (ou o modo rápido) e a análise."""

import argparse
from dataclasses import dataclass
import subprocess

ANGLES = (30, 60, 90)  # ângulos do perímetro, em graus
SPEEDS = (5, 10.8, 20, 40)  # velocidades de cruzeiro, em km/h
QUICK_DURATION = 120.0  # duração do modo rápido, em s simulados
SCENARIO = 'single_120'


@dataclass
class Case:
    """Um caso do conjunto: o rótulo da variante e as sobrescritas do ``--set``."""

    label: str
    overrides: str


def _overrides(angle, speed):
    return (f'world=citrus_farm_{angle}.sdf convoy.followers=4 route.enabled=true '
            f'route.cruise_kmh={speed:g} sensors.camera_mount=single')


def basic():
    """Devolve os 12 casos do conjunto básico: 3 ângulos por 4 velocidades."""
    cases = []
    for angle in ANGLES:
        for speed in SPEEDS:
            label = f"p{angle}_{f'{speed:g}'.replace('.', '_')}kmh"
            cases.append(Case(label, _overrides(angle, speed)))
    return cases


def quick():
    """Devolve o caso único do modo rápido: ângulo de 90 graus a 10,8 km/h."""
    return Case('quick', _overrides(90, 10.8))


def main():
    """Roda os casos, uma campanha só, e depois a análise; sai com o código da análise."""
    parser = argparse.ArgumentParser(description='Run the basic set of cases and analyze it.')
    parser.add_argument('--campaign', required=True, help='campaign name')
    parser.add_argument('--reps', type=int, default=3, help='seeds per case; default 3')
    parser.add_argument('--quick', action='store_true',
                        help='run only the quick case, with seed 1')
    parser.add_argument('--root', default='/workspace/experiments', help='output root')
    parser.add_argument('--retries', type=int, default=1,
                        help='repetitions of a run that fails; default 1')
    args = parser.parse_args()
    cases = [quick()] if args.quick else basic()
    seeds = [1] if args.quick else list(range(1, args.reps + 1))
    for case in cases:
        command = ['ros2', 'run', 'ftl_experiments', 'run', '--campaign', args.campaign,
                   '--scenario', SCENARIO, '--set', case.overrides,
                   '--seeds', ','.join(str(seed) for seed in seeds), '--root', args.root,
                   '--label', case.label, '--retries', str(args.retries)]
        if args.quick:
            command += ['--duration', str(QUICK_DURATION)]
        subprocess.run(command, check=False)
    folder = f'{args.root}/{args.campaign}'
    analysis = ['ros2', 'run', 'ftl_analysis', 'analyze', folder, '--out', f'{folder}/analysis']
    if args.quick:
        analysis += ['--reasons', 'done,timeout']
    return subprocess.run(analysis, check=False).returncode


if __name__ == '__main__':
    raise SystemExit(main())
