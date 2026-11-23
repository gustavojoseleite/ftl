"""
Lint das docstrings do pacote: o pep257 com a convenção do ament.

As docstrings seguem o estilo Google (``Args:``, ``Returns:``, ``Raises:``).
A convenção do ament lê ``Returns:`` e ``Raises:`` como seções do numpy e
cobra o formato dele, então o teste dispensa as três regras desse formato que
o Google não segue: D406 (título sem dois-pontos), D407 (sublinhado de traços)
e D413 (linha em branco depois da última seção).
"""

from pathlib import Path

from ament_pep257.main import main
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
GOOGLE_STYLE = ['--add-ignore', 'D406', 'D407', 'D413']


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=[str(PACKAGE)] + GOOGLE_STYLE)
    assert rc == 0, 'Há docstrings fora do padrão; a saída acima diz quais'
