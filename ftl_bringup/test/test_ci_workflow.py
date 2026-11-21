"""
Testes do workflow de CI do ftl (.github/workflows/ci.yml).

O workflow roda como o do vehicle_control: no container ros:jazzy-ros-base,
com o ftl e os três repositórios da plataforma (os do projects/ftl/ftl.repos).
Ele instala as dependências pelo rosdep, compila os pacotes do ftl e roda os
testes deles, menos os de simulação. O job falha quando um teste falha e
mostra sempre o resultado de cada arquivo de teste.
"""

from pathlib import Path
import re

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
WORKFLOW = REPO / '.github' / 'workflows' / 'ci.yml'
OWNER = 'gustavojoseleite'
PLATFORM = ['robot_description', 'agri_gazebo', 'vehicle_control']
TOKEN = '${{ secrets.WORKSPACE_REPOS_TOKEN }}'
FTL_PACKAGES = '$(colcon list --base-paths src/ftl --names-only)'
TEST_STEP = r'colcon test(?!-result)'


def workflow():
    """Lê o workflow; no YAML 1.1, a chave on vira True."""
    return yaml.safe_load(WORKFLOW.read_text())


def triggers():
    """Devolve os eventos que disparam o workflow."""
    data = workflow()
    return data['on'] if 'on' in data else data[True]


def job():
    """Devolve o único job do workflow."""
    jobs = workflow()['jobs']
    assert len(jobs) == 1, f'o workflow tem {len(jobs)} jobs'
    return next(iter(jobs.values()))


def steps():
    """Devolve os passos do job, na ordem."""
    return job()['steps']


def step_matching(pattern):
    """Devolve a posição e o passo cujo run casa com a expressão."""
    for i, step in enumerate(steps()):
        if re.search(pattern, step.get('run', '')):
            return i, step
    raise AssertionError(f'nenhum passo roda {pattern!r}')


def checkouts():
    """Devolve as posições e as opções dos passos de checkout."""
    return [(i, s.get('with', {})) for i, s in enumerate(steps())
            if str(s.get('uses', '')).startswith('actions/checkout@')]


def test_runs_on_push_to_main_pull_requests_and_on_demand():
    on = triggers()
    assert on['push']['branches'] == ['main']
    assert 'pull_request' in on
    assert 'workflow_dispatch' in on


def test_runs_in_the_same_container_as_vehicle_control():
    assert job()['runs-on'] == 'ubuntu-24.04'
    assert job()['container'] == 'ros:jazzy-ros-base'


def test_runs_the_steps_in_bash():
    defaults = job().get('defaults') or workflow().get('defaults') or {}
    assert defaults.get('run', {}).get('shell') == 'bash'


def test_checks_out_ftl_into_the_workspace():
    own = [w for _, w in checkouts() if 'repository' not in w]
    assert len(own) == 1, f'{len(own)} checkouts do próprio ftl'
    assert own[0].get('path') == 'src/ftl'


@pytest.mark.parametrize('repo', PLATFORM)
def test_checks_out_each_platform_repository_with_the_token(repo):
    found = [w for _, w in checkouts() if w.get('repository') == f'{OWNER}/{repo}']
    assert len(found) == 1, f'{repo}: {len(found)} checkouts'
    assert found[0].get('path') == f'src/{repo}'
    assert found[0].get('token') == TOKEN


def test_checks_out_nothing_else():
    others = {w['repository'] for _, w in checkouts() if 'repository' in w}
    assert others == {f'{OWNER}/{repo}' for repo in PLATFORM}


def test_installs_the_dependencies_with_rosdep():
    _, step = step_matching(r'rosdep install')
    run = step['run']
    for part in ('apt-get update', 'rosdep update', '--from-paths src', '--ignore-src'):
        assert part in run, f'o passo do rosdep não tem {part!r}'


def test_builds_the_ftl_packages_and_what_they_use():
    _, step = step_matching(r'colcon build')
    run = step['run']
    assert 'source /opt/ros/jazzy/setup.bash' in run
    assert f'--packages-up-to {FTL_PACKAGES}' in run


def test_tests_the_ftl_packages_and_fails_on_a_failure():
    _, step = step_matching(TEST_STEP)
    run = step['run']
    assert 'source /opt/ros/jazzy/setup.bash' in run
    assert f'--packages-select {FTL_PACKAGES}' in run
    assert '--return-code-on-test-failure' in run


def test_leaves_the_simulation_tests_out():
    _, step = step_matching(TEST_STEP)
    run = step['run']
    assert re.search(r'--ctest-args\s+-LE\s+simulation\b', run)
    assert re.search(r'--pytest-args\s+-m\s+(["\'])not simulation\1', run)


def test_always_shows_the_result_of_each_test_file():
    _, step = step_matching(r'colcon test-result')
    assert step.get('if') == 'always()'
    assert '--all' in step['run'] and '--verbose' in step['run']


def test_steps_run_in_order():
    order = [step_matching(p)[0]
             for p in (r'rosdep install', r'colcon build', TEST_STEP, r'colcon test-result')]
    assert order == sorted(order)
    assert max(i for i, _ in checkouts()) < order[0]
