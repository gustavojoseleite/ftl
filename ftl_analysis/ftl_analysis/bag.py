"""
Leitura dos bags MCAP de uma execução.

Cada mensagem ganha dois instantes: o do carimbo do cabeçalho, quando há, e o de gravação.
Também é o comando ``bag_info``, que lista os tópicos de uma execução: ``ros2 run ftl_analysis
bag_info <pasta da execução>``.
"""

import argparse
from collections import namedtuple
from pathlib import Path

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

# time: carimbo do cabeçalho, ou o de gravação sem cabeçalho; recorded: o de gravação, ambos em s.
Sample = namedtuple('Sample', 'time recorded message')


def _open(folder):
    """Abre o bag da pasta da execução e devolve o leitor e o tipo de cada tópico."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    return reader, {meta.name: meta.type for meta in reader.get_all_topics_and_types()}


def read(folder, topics):
    """
    Lê os tópicos pedidos de uma execução.

    Args:
        folder: A pasta da execução, com o bag em ``bag/``.
        topics: Os nomes dos tópicos; os que não estão no bag ficam de fora do resultado.

    Returns:
        Um dicionário de tópico para a lista de ``Sample``, na ordem de gravação.
    """
    reader, types = _open(folder)
    wanted = [name for name in topics if name in types]
    result = {}
    if not wanted:
        return result
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {name: get_message(types[name]) for name in wanted}
    while reader.has_next():
        topic, data, recorded = reader.read_next()
        message = rclpy.serialization.deserialize_message(data, classes[topic])
        time = recorded * 1e-9
        stamp = message.header.stamp if hasattr(message, 'header') else None
        if stamp is not None and (stamp.sec or stamp.nanosec):
            time = stamp.sec + stamp.nanosec * 1e-9
        result.setdefault(topic, []).append(Sample(time, recorded * 1e-9, message))
    return result


def main(argv=None):
    """Lista os tópicos de uma execução, com o tipo e o número de mensagens."""
    parser = argparse.ArgumentParser(description='List the topics of a recorded run.')
    parser.add_argument('run', help='run folder, with the bag in bag/')
    args = parser.parse_args(argv)
    folder = Path(args.run)
    reader, types = _open(folder)
    counts = dict.fromkeys(types, 0)
    while reader.has_next():
        counts[reader.read_next()[0]] += 1
    for name, kind in sorted(types.items()):
        print(f'{name}  {kind}  {counts[name]}')
