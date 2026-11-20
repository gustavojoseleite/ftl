"""
Lint de estilo do pacote: o flake8 com a configuração do ament.

O ``ament_flake8`` confere aspas simples, a ordem dos imports (estilo google),
linhas de até 99 caracteres e nomes não usados ou não definidos. O caminho é o
do pacote, e não a pasta de onde o pytest foi chamado.
"""

from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[str(PACKAGE)])
    assert rc == 0, f'{len(errors)} problema(s) de estilo:\n' + '\n'.join(errors)
