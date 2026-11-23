"""
Guarda da linha de base B0: o lint não mudou o comportamento do ftl_follower_pure_pursuit.

O ftl_follower_pure_pursuit é a linha de base B0 do plano (D2) e fica com o algoritmo
intacto. O lint da F0.4 só mexe na forma do código: aspas, quebras de linha,
a ordem dos imports, um import sem uso e as docstrings. Este teste compara a
forma normalizada de cada arquivo com a que o código tem depois do lint.

A forma normalizada é a árvore sintática sem docstrings e sem imports, seguida
da lista ordenada dos imports. Uma mudança intencional no pacote (a adaptação
da D2, a tradução da F0.6) recalcula os valores com
``python3 test/test_baseline.py``, rodado na pasta do pacote.
"""

import ast
import hashlib
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parents[1]

# sha256 da forma normalizada de cada arquivo, depois do lint da F0.4.
EXPECTED = {
    'ftl_follower_pure_pursuit/__init__.py':
        '3bebd437c9cea372cc2b1f73b1dcc9cf01dc18b48defb40cbf2fb38326f68759',
    'ftl_follower_pure_pursuit/control.py':
        '30480bacc8e875846ea0cbd491222160ee5c22b31733634fa155af4d6c80c127',
    'ftl_follower_pure_pursuit/follower.py':
        '3dfa036fd0157ce243eb5d1779d253efa0869aac95c587ee555211f8d0551862',
    'ftl_follower_pure_pursuit/geo.py':
        '9c7cd6bddc913646111bad8ec98ece03cb5339cf00c51ff0c0aad636789e0dc5',
    'ftl_follower_pure_pursuit/pose_estimator.py':
        'e8ea1bb435195766665dfd65e6ee46124c83d9a6ce7ea6a95c9acea0184d8d95',
    'ftl_follower_pure_pursuit/trail.py':
        'b0d2cb750949f3da5e2af214778018ddafd303532a7d4f7a75ce33c8038ee11e',
    'setup.py':
        '04b5a9ffdea3782e63cd2b64ad00959e6de3a71bf2e927bb9730d8b012e08b36',
}


def _no_behavior(node):
    """Diz se o comando é um import ou uma string solta, como a docstring."""
    if isinstance(node, (ast.Import, ast.ImportFrom)):
        return True
    return (isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant)
            and isinstance(node.value.value, str))


def normalized_form(source):
    """
    Devolve a forma do código que o lint não pode mudar.

    Args:
        source: o código-fonte de um arquivo Python.

    Returns:
        O ``ast.dump`` da árvore sem docstrings e sem imports, que não guarda
        aspas, quebras de linha nem comentários, seguido de uma linha por
        import, em ordem alfabética.
    """
    tree = ast.parse(source)
    imports = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(f'import {a.name} as {a.asname}' for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            dots = '.' * node.level
            imports.update(f'from {dots}{node.module} import {a.name} as {a.asname}'
                           for a in node.names)
        for field, value in ast.iter_fields(node):
            if isinstance(value, list):
                setattr(node, field, [v for v in value if not _no_behavior(v)])
    return ast.dump(tree) + '\n' + '\n'.join(sorted(imports))


def current_files():
    """Lista os arquivos que a guarda cobre: os módulos e o setup.py."""
    modules = sorted((PACKAGE / 'ftl_follower_pure_pursuit').glob('*.py'))
    return [str(p.relative_to(PACKAGE)) for p in modules] + ['setup.py']


def digest(path):
    """Devolve o sha256 da forma normalizada de um arquivo do pacote."""
    source = (PACKAGE / path).read_text()
    return hashlib.sha256(normalized_form(source).encode()).hexdigest()


@pytest.mark.parametrize('path', sorted(EXPECTED))
def test_same_form_as_after_lint(path):
    assert digest(path) == EXPECTED[path], f'{path} mudou além da forma'


def test_no_file_added_or_removed():
    assert sorted(current_files()) == sorted(EXPECTED)


if __name__ == '__main__':
    for path in current_files():
        print(f"    '{path}':\n        '{digest(path)}',")
