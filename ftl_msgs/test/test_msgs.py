"""Contrato das mensagens do ftl_msgs, visto de Python (R1 a R5).

Roda no container, pelo ~/dev/ros2/scripts/test.sh, contra o pacote instalado.
"""
import re
import subprocess
from pathlib import Path

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import NavSatFix

from ftl_msgs.msg import FollowerStatus, Progress, Rail, RailPoint, TrackedVehicle, V2VEnvelope

MSG_DIR = Path(__file__).resolve().parents[1] / "msg"

# Campo → tipo, como o rosidl os expõe em Python. A ordem não importa; o conjunto, sim.
CONTRATO = {
    TrackedVehicle: {
        "header": "std_msgs/Header",
        "vehicle_id": "string",
        "pose": "geometry_msgs/PoseWithCovariance",
        "twist": "geometry_msgs/TwistWithCovariance",
        "sources": "uint8",
        "gnss_age": "builtin_interfaces/Duration",
        "tag_age": "builtin_interfaces/Duration",
    },
    RailPoint: {
        "index": "uint32",
        "s": "double",
        "position": "geometry_msgs/Point",
        "heading": "double",
        "curvature": "double",
        "speed_limit": "double",
    },
    Rail: {
        "header": "std_msgs/Header",
        "leader_id": "string",
        "built_by": "string",
        "points": "sequence<ftl_msgs/RailPoint>",
    },
    Progress: {
        "header": "std_msgs/Header",
        "vehicle_id": "string",
        "s": "double",
        "released_index": "uint32",
    },
    FollowerStatus: {
        "header": "std_msgs/Header",
        "vehicle_id": "string",
        "state": "uint8",
        "reason": "string",
        "lateral_error": "double",
        "gap_error": "double",
        "heading_error": "double",
        "own_gnss": "boolean",
        "ahead_gnss": "boolean",
        "ahead_tag": "boolean",
    },
    V2VEnvelope: {
        "header": "std_msgs/Header",
        "sender": "string",
        "sequence": "uint64",
        "kind": "uint8",
        "fix": "sequence<sensor_msgs/NavSatFix, 1>",
        "status": "sequence<ftl_msgs/FollowerStatus, 1>",
        "rail": "sequence<ftl_msgs/Rail, 1>",
    },
}

# Os estados da máquina da F8 (nome no plano → constante), na ordem do plano.
ESTADOS = {
    "STATE_INITIALIZING": 0,       # INICIALIZANDO
    "STATE_WAITING_FOR_RAIL": 1,   # AGUARDANDO_TRILHO
    "STATE_NOMINAL": 2,            # NOMINAL
    "STATE_TAG_ONLY": 3,           # SO_TAG
    "STATE_GNSS_ONLY": 4,          # SO_GNSS
    "STATE_ESTIMATED_POSE": 5,     # POSE_ESTIMADA
    "STATE_LOCAL_TRAJECTORY": 6,   # TRAJETORIA_LOCAL
    "STATE_SAFE_STOP": 7,          # PARADA_SEGURA
    "STATE_EMERGENCY": 8,          # EMERGENCIA
}

NOMES = [m.__name__ for m in CONTRATO]


def linhas_de_campo(nome):
    """Linhas de campo e de constante do .msg (sem as linhas só de comentário)."""
    linhas = (MSG_DIR / f"{nome}.msg").read_text().splitlines()
    return [linha for linha in linhas if linha.strip() and not linha.lstrip().startswith("#")]


@pytest.mark.parametrize("msg", list(CONTRATO), ids=NOMES)
def test_campos_e_tipos(msg):
    """R1: cada mensagem tem exatamente os campos e os tipos do contrato."""
    assert msg.get_fields_and_field_types() == CONTRATO[msg]


@pytest.mark.parametrize("nome", NOMES)
def test_interface_show(nome):
    """R1: a interface aparece no ros2 interface show."""
    r = subprocess.run(["ros2", "interface", "show", f"ftl_msgs/msg/{nome}"],
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr


@pytest.mark.parametrize("msg", [m for m in CONTRATO if m is not RailPoint],
                         ids=[n for n in NOMES if n != "RailPoint"])
def test_header_primeiro(msg):
    """R2: toda mensagem publicada começa pelo header."""
    assert list(msg.get_fields_and_field_types())[0] == "header"


@pytest.mark.parametrize("nome", NOMES)
def test_todo_campo_tem_comentario(nome):
    """R2: todo campo e toda constante têm um comentário na mesma linha."""
    for linha in linhas_de_campo(nome):
        assert "#" in linha and linha.split("#", 1)[1].strip(), f"{nome}: sem comentário: {linha}"


@pytest.mark.parametrize("nome", NOMES)
def test_ponto_flutuante_diz_a_unidade(nome):
    """R2: os campos float64 dizem a unidade, entre parênteses, no comentário."""
    for linha in linhas_de_campo(nome):
        if re.match(r"^\s*float64\s", linha):
            comentario = linha.split("#", 1)[1] if "#" in linha else ""
            assert re.search(r"\((m|m/s|rad|rad/s|1/m|s)\)", comentario), f"{nome}: sem unidade: {linha}"


def test_constantes_das_fontes():
    """R3: as fontes de posição são constantes nomeadas (bits)."""
    assert (TrackedVehicle.SOURCE_NONE, TrackedVehicle.SOURCE_GNSS,
            TrackedVehicle.SOURCE_TAG) == (0, 1, 2)


def test_constantes_dos_estados():
    """R3: os estados do supervisor são os da F8, com nomes em inglês."""
    for nome, valor in ESTADOS.items():
        assert getattr(FollowerStatus, nome) == valor, nome


def test_constantes_do_envelope():
    """R4: o tipo de conteúdo do envelope é uma constante nomeada."""
    assert (V2VEnvelope.KIND_FIX, V2VEnvelope.KIND_STATUS, V2VEnvelope.KIND_RAIL) == (1, 2, 3)


def test_envelope_leva_um_conteudo_so():
    """R4: cada conteúdo do envelope cabe no máximo uma vez; com dois, a mensagem não sai.

    O Python do Jazzy só confere o tipo na atribuição com ROS_PYTHON_CHECK_FIELDS=1; sem ele,
    o limite [<=1] é imposto na serialização, antes de a mensagem sair do processo.
    """
    e = V2VEnvelope(kind=V2VEnvelope.KIND_FIX, fix=[NavSatFix()])
    serialize_message(e)  # com um fix, a mensagem sai
    try:
        e.fix = [NavSatFix(), NavSatFix()]
    except (AssertionError, ValueError):
        return
    with pytest.raises(Exception):
        serialize_message(e)


def test_trilho_ida_e_volta_pelo_envelope():
    """R4, R5: um trilho dentro do envelope sai igual da serialização."""
    pontos = [RailPoint(index=i, s=0.5 * i, heading=0.1 * i, curvature=0.04, speed_limit=3.0)
              for i in range(3)]
    trilho = Rail(leader_id="pickup", built_by="tractor_1", points=pontos)
    e = V2VEnvelope(sender="tractor_1", sequence=42, kind=V2VEnvelope.KIND_RAIL, rail=[trilho])
    volta = deserialize_message(serialize_message(e), V2VEnvelope)
    assert (volta.sender, volta.sequence, volta.kind) == ("tractor_1", 42, V2VEnvelope.KIND_RAIL)
    assert len(volta.fix) == 0 and len(volta.status) == 0 and len(volta.rail) == 1
    assert [p.s for p in volta.rail[0].points] == [0.0, 0.5, 1.0]
    assert volta.rail[0].built_by == "tractor_1"


def test_estado_e_estimativa_ida_e_volta():
    """R3, R5: o estado do supervisor e a estimativa do precedente saem iguais da serialização."""
    st = FollowerStatus(vehicle_id="tractor_2", state=FollowerStatus.STATE_TAG_ONLY,
                        reason="perdeu o GNSS do precedente", lateral_error=0.12,
                        own_gnss=True, ahead_gnss=False, ahead_tag=True)
    volta = deserialize_message(serialize_message(st), FollowerStatus)
    assert volta.state == FollowerStatus.STATE_TAG_ONLY
    assert volta.reason == "perdeu o GNSS do precedente"
    assert (volta.own_gnss, volta.ahead_gnss, volta.ahead_tag) == (True, False, True)

    tv = TrackedVehicle(vehicle_id="tractor_1",
                        sources=TrackedVehicle.SOURCE_GNSS | TrackedVehicle.SOURCE_TAG)
    tv.pose.pose.position.x = 12.5
    tv.twist.twist.linear.x = 3.0
    tv.gnss_age.nanosec = 200_000_000
    volta = deserialize_message(serialize_message(tv), TrackedVehicle)
    assert volta.sources == 3
    assert (volta.pose.pose.position.x, volta.twist.twist.linear.x) == (12.5, 3.0)
    assert volta.gnss_age.nanosec == 200_000_000

    pr = Progress(vehicle_id="tractor_3", s=37.25, released_index=120)
    volta = deserialize_message(serialize_message(pr), Progress)
    assert (volta.s, volta.released_index) == (37.25, 120)
