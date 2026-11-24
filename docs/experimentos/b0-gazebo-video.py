"""
Grava, pela GUI do Gazebo, o vídeo do comboio nas curvas de uma execução em andamento.

Roda no contêiner com GUI, ao lado do ``ros2 run ftl_experiments run`` e de um ``gz sim -g`` com
a ``b0-gazebo-gui.config`` (o ``b0-gazebo-video.sh`` sobe os três). Acompanha o progresso do
líder em ``/pickup/route/progress``: a câmera passa a seguir o ``--target`` e a gravação começa
quando o líder está 30 m antes da entrada da curva e para quando ele está ``--tail`` metros depois
da saída, com o último trator já fora dela. Um arquivo por curva, ``curve_<n>.mp4``.

O gz-sim 8 só grava vídeo pelo botão da GUI; por isso a gravação captura a janela do Gazebo no
X11 (``ffmpeg -f x11grab``, do ``imageio-ffmpeg``), achada pelo título com o ``python-xlib``. O
vídeo segue o relógio de parede: se a simulação roda abaixo do tempo real, ele fica mais lento.
"""

import argparse
import os
from pathlib import Path
import subprocess
import time

from ftl_analysis.segments import segments
from ftl_route_driver.routes import route as load_route
import imageio_ffmpeg
import rclpy
from std_msgs.msg import Float64
from Xlib import display

BEFORE = 30.0  # a gravação começa com o líder a esta distância da entrada, em m
# Recorte da janela de 1280x720: tira a sombra, a barra de título e a barra do Gazebo.
CROP = 'crop=1280:672:14:97'


def _service(name, reqtype, request):
    """Chama um serviço do gz-transport e devolve se ele respondeu ``true``."""
    result = subprocess.run(
        ['gz', 'service', '-s', name, '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req', request], capture_output=True, text=True)
    return 'true' in result.stdout


def _window(title='Gazebo Sim'):
    """Acha o id da janela do X11 cujo título contém ``title``, ou ``None``."""
    screen = display.Display()
    pending = [screen.screen().root]
    while pending:
        window = pending.pop()
        try:
            name = window.get_wm_name()
            children = window.query_tree().children
        except Exception:  # noqa: B902 - a janela pode sumir durante a busca
            continue
        if isinstance(name, str) and title in name:
            return window.id
        pending.extend(children)
    return None


def _record(window, path, fps=30):
    """Começa a capturar a janela num MP4 e devolve o processo do ffmpeg."""
    return subprocess.Popen(
        [imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error', '-f', 'x11grab',
         '-framerate', str(fps), '-draw_mouse', '0', '-window_id', hex(window),
         '-i', os.environ['DISPLAY'], '-vf', CROP, '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
         '-crf', '20', str(path)], stdin=subprocess.PIPE)


def main(argv=None):
    """Segue o líder pela rota e grava as curvas pedidas."""
    parser = argparse.ArgumentParser(description='Record the convoy in the curves from the GUI.')
    parser.add_argument('--world', default='citrus_farm_90', help='route name')
    parser.add_argument('--curves', default='0,1', help='curve indexes; default 0,1')
    parser.add_argument('--out', required=True, help='output folder')
    parser.add_argument('--target', default='tractor_4', help='model the camera follows')
    parser.add_argument('--offset', default='-14,10,12',
                        help='camera offset from the target, x,y,z in m')
    parser.add_argument('--tail', type=float, default=60.0,
                        help='leader distance past the curve exit when recording stops, in m')
    args = parser.parse_args(argv)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    parts = [p for p in segments(load_route(args.world)) if p.kind == 'curve']
    plan = [(int(c), parts[int(c)]) for c in args.curves.split(',')]
    x, y, z = (float(v) for v in args.offset.split(','))
    rclpy.init()
    node = rclpy.create_node('b0_gazebo_video')
    log = node.get_logger()
    state = {'progress': None}
    node.create_subscription(Float64, '/pickup/route/progress',
                             lambda msg: state.update(progress=msg.data), 10)
    window = None
    while window is None:
        window = _window()
        time.sleep(1.0)
    log.info(f'Gazebo window 0x{window:x}')
    while plan and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        s = state['progress']
        if s is None:
            continue
        index, part = plan[0]
        if 'recording' not in state and s >= part.s0 - BEFORE:
            follow = _service('/gui/follow', 'gz.msgs.StringMsg', f'data: "{args.target}"')
            offset = _service('/gui/follow/offset', 'gz.msgs.Vector3d',
                              f'x: {x}, y: {y}, z: {z}')
            path = out / f'curve_{index}.mp4'
            state['recording'] = (_record(window, path), path)
            log.info(f'curve {index}: follow={follow} offset={offset}, recording {path}')
        elif 'recording' in state and s >= part.s1 + args.tail:
            process, path = state.pop('recording')
            process.communicate(b'q')
            log.info(f'curve {index}: saved {path}, ffmpeg exit {process.returncode}')
            plan.pop(0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
