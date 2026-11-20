"""
Testes dos registros de decisão em docs/decisoes/.

Todo registro tem as três seções do modelo (Contexto, Decisão, Consequência),
preenchidas, e a consequência diz o que muda no texto da dissertação, com o
número da seção ou da equação. Cada decisão do plano que muda o texto tem o
seu registro, e o índice aponta para todos.
"""

from pathlib import Path
import re

import pytest

REPO = Path(__file__).resolve().parents[2]
DECISIONS = REPO / 'docs' / 'decisoes'
SECTIONS = ['Contexto', 'Decisão', 'Consequência']
RECORD_NAME = re.compile(r'^D(\d{2})-[a-z0-9-]+\.md$')
TEXT_REFERENCE = re.compile(r'(seção|Eq\.) \d+\.\d+', re.IGNORECASE)
MIN_CHARS = 80

# As decisões do plano (seção 12) que mudam o texto da dissertação, e o que o
# registro de cada uma precisa citar.
REQUIRED = {
    4: ['Eq. 4.2', 'arco'],
    5: ['Eq. 4.5', 'linha de visada'],
    7: ['UKF', 'NEES', 'robot_localization'],
    9: ['cascata', 'T1'],
    19: ['40 km/h'],
    20: ['Eq. 4.4', 'd_alvo'],
    21: ['parada segura'],
    22: ['relevo', 'antena'],
}


def records():
    """Lista os arquivos de registro, pelo nome no formato Dnn-assunto.md."""
    return sorted(p for p in DECISIONS.glob('D*.md') if RECORD_NAME.match(p.name))


def number_of(path):
    """Devolve o número da decisão de um arquivo de registro."""
    return int(RECORD_NAME.match(path.name).group(1))


def sections(text):
    """Separa o texto pelos títulos de nível 2, na ordem em que aparecem."""
    parts = re.split(r'^## +(.+?)\s*$', text, flags=re.MULTILINE)
    return [(parts[i], parts[i + 1].strip()) for i in range(1, len(parts), 2)]


def test_template_has_the_three_sections():
    text = (DECISIONS / 'modelo.md').read_text()
    assert [name for name, _ in sections(text)] == SECTIONS


@pytest.mark.parametrize('number', sorted(REQUIRED))
def test_required_decision_has_one_record(number):
    found = [p.name for p in records() if number_of(p) == number]
    assert len(found) == 1, f'D{number}: {found}'


@pytest.mark.parametrize('number', sorted(REQUIRED))
def test_record_cites_what_the_plan_decided(number):
    found = [p for p in records() if number_of(p) == number]
    assert found, f'D{number} sem registro'
    text = found[0].read_text().lower()
    for term in REQUIRED[number]:
        assert term.lower() in text, f'D{number} não cita {term!r}'


@pytest.mark.parametrize('path', records(), ids=lambda p: p.name)
def test_record_has_the_three_sections_filled(path):
    found = sections(path.read_text())
    assert [name for name, _ in found] == SECTIONS, path.name
    for name, body in found:
        assert len(body) >= MIN_CHARS, f'{path.name}: {name} curta demais'


@pytest.mark.parametrize('path', records(), ids=lambda p: p.name)
def test_record_title_starts_with_its_id(path):
    first = path.read_text().splitlines()[0]
    assert re.match(rf'^# D{number_of(path)} — \S', first), first


@pytest.mark.parametrize('path', records(), ids=lambda p: p.name)
def test_consequence_points_to_the_dissertation_text(path):
    consequence = dict(sections(path.read_text())).get('Consequência', '')
    assert 'dissertação' in consequence, f'{path.name}: não fala da dissertação'
    assert TEXT_REFERENCE.search(consequence), f'{path.name}: sem seção nem equação'


def test_index_links_the_template_and_every_record():
    index = (DECISIONS / 'README.md').read_text()
    assert '(modelo.md)' in index, 'o índice não aponta para o modelo'
    for path in records():
        assert f'({path.name})' in index, f'o índice não aponta para {path.name}'
