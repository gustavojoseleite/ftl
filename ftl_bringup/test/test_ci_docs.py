"""
Testes da documentação do CI, conferida contra o próprio workflow.

A seção CI do README diz o que roda e quando, e explica o segredo com que o
workflow baixa os repositórios privados. A seção Testes do CONTRIBUTING diz
como marcar um teste de simulação para que o CI o deixe de fora.
"""

from pathlib import Path
import re

import yaml

REPO = Path(__file__).resolve().parents[2]
WORKFLOW = REPO / '.github' / 'workflows' / 'ci.yml'
README = REPO / 'README.md'
CONTRIBUTING = REPO / 'CONTRIBUTING.md'


def section(path, title):
    """Devolve o corpo da seção de nível 2 com o título dado."""
    parts = re.split(r'^## +(.+?)\s*$', path.read_text(), flags=re.MULTILINE)
    return dict(zip(parts[1::2], parts[2::2])).get(title, '')


def secrets():
    """Lista os segredos que o workflow usa."""
    return sorted(set(re.findall(r'secrets\.(\w+)', WORKFLOW.read_text())))


def private_repositories():
    """Lista os repositórios que o workflow baixa com o token."""
    steps = next(iter(yaml.safe_load(WORKFLOW.read_text())['jobs'].values()))['steps']
    return sorted(s['with']['repository'].split('/')[1] for s in steps
                  if 'token' in s.get('with', {}))


def simulation_names():
    """Devolve o rótulo do CTest e a marca do pytest que o workflow deixa de fora."""
    text = WORKFLOW.read_text()
    label = re.search(r'-LE\s+(\w+)', text).group(1)
    marker = re.search(r'-m\s+["\']not (\w+)["\']', text).group(1)
    return label, marker


def test_readme_has_a_ci_section_pointing_to_the_workflow():
    body = section(README, 'CI')
    assert body.strip(), 'o README não tem a seção CI'
    assert '.github/workflows/ci.yml' in body


def test_readme_explains_the_secret():
    body = section(README, 'CI')
    assert secrets(), 'o workflow não usa segredo nenhum'
    for name in secrets():
        assert name in body, f'a seção CI não cita o segredo {name}'
    for term in ('fine-grained', 'Contents', 'Settings'):
        assert term in body, f'a seção CI não cita {term!r}'


def test_readme_names_each_private_repository():
    body = section(README, 'CI')
    for repo in private_repositories():
        assert repo in body, f'a seção CI não cita o {repo}'


def test_readme_says_when_it_runs_and_what_stays_out():
    body = section(README, 'CI')
    assert 'pull request' in body.lower()
    assert '`main`' in body
    _, marker = simulation_names()
    assert 'simulação' in body.lower() and marker in body


def test_contributing_explains_how_to_mark_a_simulation_test():
    body = section(CONTRIBUTING, 'Testes')
    label, marker = simulation_names()
    assert f'@pytest.mark.{marker}' in body
    assert re.search(rf'LABELS\s+{label}\b', body), f'falta o rótulo {label} do CTest'
    assert 'CI' in body
