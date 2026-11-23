"""O comando ``run``: roda um cenário com várias sementes e grava o bag e o ``run.yaml``."""

import argparse
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import time

from ftl_bringup import scenario
from ftl_experiments import campaign
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

STOP_WAIT = 60.0  # espera pelo encerramento de um processo, em s


def overrides_text(overrides):
    """Escreve as sobrescritas como texto ``chave=valor``, com os booleanos em minúsculas."""
    items = []
    for name, value in overrides.items():
        text = str(value).lower() if isinstance(value, bool) else str(value)
        items.append(shlex.quote(f'{name}={text}'))
    return ' '.join(items)


def stop(process, wait=STOP_WAIT):
    """Encerra o grupo do processo com SIGINT e, se demorar, com SIGKILL."""
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=wait)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def execute(run, limit, names, src):
    """Roda uma execução e grava o ``run.yaml``; devolve o motivo do fim."""
    shutil.rmtree(run.path / 'bag', ignore_errors=True)
    run.path.mkdir(parents=True, exist_ok=True)
    config = scenario.resolve(run.scenario, run.overrides)
    limit = limit or campaign.time_limit(config)
    launch = ['ros2', 'launch', 'ftl_bringup', 'experiment.launch.py',
              f'scenario:={run.scenario}', f'set:={overrides_text(run.overrides)}',
              f'output:={run.path}']
    record = ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(run.path / 'bag'),
              '--use-sim-time'] + names
    start = time.monotonic()
    with open(run.path / 'launch.log', 'w') as log:
        launcher = subprocess.Popen(launch, start_new_session=True, stdout=log,
                                    stderr=subprocess.STDOUT)
        recorder = subprocess.Popen(record, start_new_session=True, stdout=log,
                                    stderr=subprocess.STDOUT)
        rclpy.init()
        node = rclpy.create_node('experiment_runner')
        done = []
        leader = config['convoy']['leader']
        node.create_subscription(Bool, f'/{leader}/route/done',
                                 lambda msg: done.append(msg.data),
                                 QoSProfile(depth=1,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:
            reason = None
            while reason is None:
                rclpy.spin_once(node, timeout_sec=0.2)
                if True in done:
                    reason = 'done'
                elif time.monotonic() - start > limit:
                    reason = 'timeout'
                elif launcher.poll() is not None:
                    reason = 'failed'
        finally:
            node.destroy_node()
            rclpy.shutdown()
            stop(recorder)
            stop(launcher)
    campaign.write_run(run.path, reason, time.monotonic() - start, run.seed,
                       campaign.commits(src))
    return reason


def main():
    """Lê os argumentos e roda as execuções pendentes da campanha."""
    parser = argparse.ArgumentParser(description='Run the seeds of a scenario and record bags.')
    parser.add_argument('--campaign', required=True, help='campaign name')
    parser.add_argument('--scenario', default='single_120', help='scenario preset')
    parser.add_argument('--set', default='', help='overrides, e.g. "convoy.followers=2"')
    parser.add_argument('--seeds', default='1,2,3', help='comma-separated seeds')
    parser.add_argument('--root', default='/workspace/experiments', help='output root')
    parser.add_argument('--label', default=None, help='variant name; default is the scenario')
    parser.add_argument('--duration', type=float, default=None,
                        help='time limit in seconds, replacing the computed one')
    args = parser.parse_args()
    seeds = [int(item) for item in args.seeds.split(',')]
    src = Path(os.environ.get('FTL_WORKSPACE_SRC', '/workspace/src'))
    runs = campaign.plan(args.campaign, args.scenario, args.set, seeds, args.root, args.label)
    for run in campaign.pending(runs):
        config = scenario.resolve(run.scenario, run.overrides)
        reason = execute(run, args.duration, campaign.topics(config), src)
        print(f'{run.path} {reason}', flush=True)


if __name__ == '__main__':
    main()
