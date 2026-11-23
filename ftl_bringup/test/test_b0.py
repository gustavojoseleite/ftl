"""A linha de base B0 congelada (F2.2, R1 a R4), conferida nos arquivos do repositório.

- O launch usa o pacote `ftl_follower_pure_pursuit`; não sobra referência ao nome antigo.
- No `ftl.yaml`, o `follower` vai até 11,1 m/s e a zona morta fica desligada (`uncertainty` 0).
- O README tem a seção da B0, que diz o que ela é e como é comparada (seção 8.4 do plano).
- A decisão D4 registra a revisão: arco e distância euclidiana, por parâmetro.
"""
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]


def test_launch_uses_the_frozen_package():
    text = (REPO / 'ftl_bringup' / 'launch' / 'ftl.launch.py').read_text()
    assert 'ftl_follower_pure_pursuit' in text
    assert 'package="ftl_follower"' not in text and "package='ftl_follower'" not in text
    assert not (REPO / 'ftl_follower').exists()


def test_follower_configuration():
    config = yaml.safe_load((REPO / 'ftl_bringup' / 'config' / 'ftl.yaml').read_text())
    params = config['/**/follower']['ros__parameters']
    assert params['v_max'] == 11.1
    assert params['uncertainty'] == 0.0


def test_readme_explains_the_baseline():
    text = (REPO / 'README.md').read_text()
    assert '## Linha de base B0' in text
    section = text.split('## Linha de base B0')[1].split('\n## ')[0]
    for needle in ('ftl_follower_pure_pursuit', 'congelad', 'D2', 'ftl/state', 'uncertainty',
                   '8.4', 'sementes', 'UKF'):
        assert needle in section, needle


def test_decision_d4_is_the_revised_one():
    text = (REPO / 'docs' / 'decisoes' / 'D04-distancia-arco.md').read_text()
    for needle in ('euclidiana', 'parâmetro', 'F7.1'):
        assert needle in text, needle
