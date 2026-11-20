"""O README do repositório ftl apresenta o ftl_msgs (R6)."""
import re
from pathlib import Path

README = Path(__file__).resolve().parents[2] / "README.md"

# Mensagem → pacote que a publica (seção 5.3 do plano).
PUBLICADA_POR = {
    "TrackedVehicle": "ftl_fusion",
    "Rail": "ftl_trajectory",
    "Progress": "ftl_trajectory",
    "FollowerStatus": "ftl_supervisor",
    "V2VEnvelope": "ftl_comms",
}


def secao(texto, padrao):
    """Texto da primeira seção cujo título casa com padrao, até o próximo título de mesmo nível."""
    titulo = re.search(rf"^(#+) .*{padrao}.*$", texto, re.M)
    if not titulo:
        return None
    nivel = len(titulo.group(1))
    resto = texto[titulo.end():]
    fim = re.search(rf"^#{{1,{nivel}}} ", resto, re.M)
    return resto[:fim.start()] if fim else resto


def test_pacote_na_arvore():
    """O ftl_msgs aparece na árvore de pacotes do README, com a sua descrição."""
    assert re.search(r"^ftl_msgs/\s+\S", README.read_text(), re.M), \
        "falta a linha do ftl_msgs/ na árvore de pacotes"


def test_secao_com_cada_mensagem_e_quem_publica():
    """Uma seção com ftl_msgs no título lista cada mensagem e o pacote que a publica."""
    trecho = secao(README.read_text(), r"ftl_msgs")
    assert trecho, "falta uma seção com ftl_msgs no título"
    linhas = trecho.splitlines()
    for mensagem, pacote in PUBLICADA_POR.items():
        com_ela = [linha for linha in linhas if re.search(rf"\b{mensagem}\b", linha)]
        assert com_ela, f"{mensagem} não aparece na seção"
        assert any(pacote in linha for linha in com_ela), \
            f"a linha de {mensagem} não diz que o {pacote} a publica"
    assert "RailPoint" in trecho, "RailPoint não aparece na seção"
