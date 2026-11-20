"""
Testes do README do ftl: a árvore de pacotes e a entrada para os padrões.

A árvore no começo do README lista todo pacote do repositório; a seção
"Pacotes novos" aponta para o CONTRIBUTING.md e para o pacote modelo; e os
testes do ftl_follower saem pelo comando do workspace.
"""

from pathlib import Path
import re

REPO = Path(__file__).resolve().parents[2]
README = REPO / 'README.md'


def tree():
    """Devolve o primeiro bloco de código do README, a árvore de pacotes."""
    found = re.search(r'^```\n(.*?)^```', README.read_text(), re.MULTILINE | re.DOTALL)
    return found.group(1)


def section(title):
    """Devolve o corpo da seção de nível 2 com o título dado."""
    parts = re.split(r'^## +(.+?)\s*$', README.read_text(), flags=re.MULTILINE)
    return dict(zip(parts[1::2], parts[2::2])).get(title, '')


def packages():
    """Lista os pacotes do repositório: as pastas com package.xml."""
    return sorted(p.parent.name for p in REPO.glob('*/package.xml'))


def test_tree_lists_every_package():
    listed = re.findall(r'^(\w+)/\s+\S', tree(), flags=re.MULTILINE)
    assert sorted(listed) == packages()


def test_new_packages_section_points_to_the_standards():
    body = section('Pacotes novos')
    assert 'CONTRIBUTING.md' in body
    assert 'ftl_template' in body
    assert (REPO / 'CONTRIBUTING.md').exists()


def test_follower_tests_use_the_workspace_command():
    assert 'scripts/test.sh ftl_follower' in README.read_text()
