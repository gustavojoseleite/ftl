"""
Testes do CONTRIBUTING.md do ftl: os padrões da seção 6 do plano.

Cada padrão tem a sua seção, e o que ela afirma confere com o repositório: os
exemplos em Python passam no lint do projeto, o exemplo de parâmetros diz
unidade, faixa e efeito, os comandos de teste são válidos, os caminhos citados
existem, e os números do lint são os da configuração.
"""

import ast
import configparser
from pathlib import Path
import re
import shlex

import ament_flake8
from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257_main
import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
CONTRIBUTING = REPO / 'CONTRIBUTING.md'
TEST_COMMAND = '~/dev/ros2/scripts/test.sh'

# Título de cada seção de nível 2 e os termos que ela precisa citar.
REQUIRED_TERMS = {
    'Idioma': ['D1', 'inglês', 'português'],
    'Docstrings': ['módulo', 'Args:', 'Returns:', 'Raises:'],
    'Equações': ['dissertação'],
    'Parâmetros': ['ParameterDescriptor', 'Unidade:', 'Faixa:', 'Efeito:'],
    'Lógica e ROS': ['rclpy'],
    'Testes': ['pytest', 'gtest', 'launch_testing'],
    'Lint': ['flake8', 'pep257', 'aspas simples', 'D213'],
    'Commits': ['português'],
    'Pacote novo': ['cp -r ftl_template'],
    'Decisões': ['dissertação'],
}

# Caminhos que cada seção precisa citar, e que existem no repositório.
REQUIRED_PATHS = {
    'Parâmetros': ['ftl_bringup/config/ftl.yaml'],
    'Lógica e ROS': ['ftl_template/ftl_template/logic.py', 'ftl_template/ftl_template/node.py'],
    'Pacote novo': ['ftl_bringup/launch/ftl.launch.py', 'ftl_bringup/config/ftl.yaml',
                    'ftl_bringup/package.xml', 'README.md'],
    'Decisões': ['docs/decisoes/modelo.md', 'docs/decisoes/README.md'],
}


def sections():
    """Devolve o corpo de cada seção de nível 2, pelo título."""
    parts = re.split(r'^## +(.+?)\s*$', CONTRIBUTING.read_text(), flags=re.MULTILINE)
    return dict(zip(parts[1::2], parts[2::2]))


def blocks(text, language):
    """Devolve o conteúdo dos blocos de código de uma linguagem."""
    return re.findall(rf'^```{language}\n(.*?)^```', text, flags=re.MULTILINE | re.DOTALL)


def google_style():
    """Lê as regras que o test_pep257.py do modelo dispensa."""
    tree = ast.parse((REPO / 'ftl_template' / 'test' / 'test_pep257.py').read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'GOOGLE_STYLE':
            return ast.literal_eval(node.value)
    raise AssertionError('o test_pep257.py do modelo não tem GOOGLE_STYLE')


def test_every_standard_has_its_section():
    missing = sorted(set(REQUIRED_TERMS) - set(sections()))
    assert not missing, f'faltam as seções {missing}'


@pytest.mark.parametrize('title', sorted(REQUIRED_TERMS))
def test_section_cites_its_terms(title):
    body = sections().get(title, '').lower()
    for term in REQUIRED_TERMS[title]:
        assert term.lower() in body, f'{title}: falta {term!r}'


@pytest.mark.parametrize('title', sorted(REQUIRED_PATHS))
def test_cited_paths_exist(title):
    body = sections().get(title, '')
    for path in REQUIRED_PATHS[title]:
        assert path in body, f'{title}: falta {path}'
        assert (REPO / path).exists(), f'{path} não existe'


def test_equation_citation_shows_the_format():
    assert re.search(r'# Eq\. \d+\.\d+', sections().get('Equações', ''))


def test_python_examples_pass_the_project_lint(tmp_path):
    examples = blocks(CONTRIBUTING.read_text(), 'python')
    assert examples, 'o CONTRIBUTING.md não tem exemplo em Python'
    for i, code in enumerate(examples):
        (tmp_path / f'example_{i}.py').write_text(code)
    rc, errors = main_with_errors(argv=[str(tmp_path)])
    assert rc == 0, '\n'.join(errors)
    assert pep257_main(argv=[str(tmp_path)] + google_style()) == 0


def test_docstring_example_has_the_google_sections():
    examples = blocks(sections().get('Docstrings', ''), 'python')
    sections_used = ('Args:', 'Returns:', 'Raises:')
    assert any(all(s in code for s in sections_used) for code in examples)


def test_parameter_example_states_unit_range_and_effect():
    examples = blocks(sections().get('Parâmetros', ''), 'yaml')
    assert examples, 'a seção Parâmetros não tem exemplo em YAML'
    for code in examples:
        yaml.safe_load(code)
        comments = []
        for line in code.splitlines():
            if line.strip().startswith('#'):
                comments.append(line)
                continue
            if re.match(r'^\s+\w+:\s*[^\s#]', line):
                text = ' '.join(comments) + line.partition('#')[2]
                for label in ('Unidade:', 'Faixa:', 'Efeito:'):
                    assert label in text, f'{line.strip()}: falta {label}'
            comments = []


def test_test_commands_are_valid():
    body = sections().get('Testes', '')
    calls = [line.strip() for code in blocks(body, 'bash') for line in code.splitlines()
             if line.strip().startswith(TEST_COMMAND)]
    assert calls, f'a seção Testes não mostra o {TEST_COMMAND}'
    for call in calls:
        words = shlex.split(call.partition('#')[0])[1:]
        while words:
            word = words.pop(0)
            if word in ('-k', '-R'):
                expression = words.pop(0)
                assert word == '-R' or '*' not in expression, f'{call}: -k não usa curinga'
                continue
            assert not word.startswith('-'), f'{call}: a opção {word} não existe'
            assert (REPO / word / 'package.xml').exists(), f'{call}: {word} não é pacote'


def test_lint_section_matches_the_configuration():
    body = sections().get('Lint', '')
    config = configparser.ConfigParser()
    config.read(Path(ament_flake8.__file__).parent / 'configuration' / 'ament_flake8.ini')
    assert config['flake8']['max-line-length'] in body
    for code in google_style()[1:]:
        assert code in body, f'Lint: falta {code}'
