"""
Grava, pela GUI do Gazebo, o vídeo do comboio nas curvas de uma execução em andamento.

Roda no contêiner com GUI, ao lado do ``ros2 run ftl_experiments run`` e de um ``gz sim -g`` com
a ``b0-gazebo-gui.config`` (o ``b0-gazebo-video.sh`` sobe os três). Acompanha o progresso do
líder em ``/pickup/route/progress``: a câmera passa a seguir o ``--target`` e a gravação começa
quando o líder está 30 m antes da entrada da curva e para quando ele está ``--tail`` metros depois
da saída, com o último trator já fora dela. Um arquivo por curva, ``curve_<n>.mp4``.

Para o corte de curva aparecer, a cena ganha faixas no chão (modelos só visuais): a rota
ideal, numa faixa branca, e o rastro do ponto de referência de cada trator (a antena projetada no
chão, o mesmo das métricas), na rampa de azuis das figuras, do T1 claro ao T4 escuro. Os rastros
recomeçam a cada curva. Com ``--view top`` (o padrão), a câmera segue o ``--target`` de cima, a
``--height`` metros, e a distância entre cada rastro e a faixa branca é o desvio do trator.

O gz-sim 8 só grava vídeo pelo botão da GUI; por isso a gravação captura a janela do Gazebo no
X11 (``ffmpeg -f x11grab``, do ``imageio-ffmpeg``), achada pelo título com o ``python-xlib``. O
vídeo segue o relógio de parede: se a simulação roda abaixo do tempo real, ele fica mais lento.
"""

import argparse
import math
import os
from pathlib import Path
import subprocess
import time

from ftl_analysis.geometry import reference_point
from ftl_analysis.segments import segments
from ftl_route_driver.routes import route as load_route, terrain_height
from geometry_msgs.msg import PoseStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_v_pb2 import EntityFactory_V
from gz.msgs10.entity_pb2 import Entity
from gz.transport13 import Node as GzNode
import imageio_ffmpeg
import rclpy
from std_msgs.msg import Float64
from Xlib import display

BEFORE = 30.0  # a gravação começa com o líder a esta distância da entrada, em m
# Recorte da janela de 1280x720: tira a sombra, a barra de título e a barra do Gazebo.
CROP = 'crop=1280:672:14:97'
AFTER = 80.0  # a faixa da rota ideal vai até esta distância depois da saída da curva, em m
STEP = 0.15  # distância mínima, em m, entre dois pontos de um rastro
# Rampa ordinal das figuras, do T1 (claro) ao T4 (escuro), e a rota ideal em branco.
TRACTORS = ['#6da7ec', '#3987e5', '#1c5cab', '#0d366b']
ROUTE = '#ffffff'
NAMESPACE = 'ftl_trail'


def _rgb(color):
    """Converte ``#rrggbb`` em ``(r, g, b)`` de 0 a 1."""
    return tuple(int(color[i:i + 2], 16) / 255.0 for i in (1, 3, 5))


class Trails:
    """
    Desenha faixas no chão do Gazebo como modelos estáticos, só visuais, criados pelo servidor.

    Cada chamada de ``strip`` cria um modelo com os segmentos novos da faixa (uma caixa fina por
    segmento), pelo serviço ``/world/<mundo>/create_multiple``; ``clear`` remove todos. O serviço
    ``/marker`` da GUI do gz-sim 8 não responde nesta montagem, e por isso as faixas não são
    marcadores.
    """

    def __init__(self, world):
        """Liga ao servidor do mundo ``world``."""
        self.node = GzNode()
        self.world = world
        self.drawn = {}
        self.models = []

    def strip(self, ident, points, width, color, lift):
        """Desenha os segmentos novos da faixa ``ident``, ``lift`` metros acima do chão."""
        start = max(0, self.drawn.get(ident, 0) - 1)
        if len(points) - start < 2:
            return
        r, g, b = _rgb(color)
        rgba = f'{r:.3f} {g:.3f} {b:.3f} 1'
        visuals = []
        for k in range(start, len(points) - 1):
            (ax, ay), (bx, by) = points[k], points[k + 1]
            length = math.hypot(bx - ax, by - ay)
            if length < 1e-3:
                continue
            mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
            yaw = math.atan2(by - ay, bx - ax)
            visuals.append(
                f'<visual name="s{k}"><pose>{mx:.3f} {my:.3f} '
                f'{terrain_height(mx) + lift:.3f} 0 0 {yaw:.4f}</pose><geometry><box><size>'
                f'{length + 0.02:.3f} {width:.3f} 0.02</size></box></geometry><material>'
                f'<ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><emissive>{rgba}</emissive>'
                f'</material><cast_shadows>false</cast_shadows></visual>')
        self.drawn[ident] = len(points)
        if not visuals:
            return
        name = f'{NAMESPACE}_{ident}_{len(self.models)}'
        sdf = (f'<sdf version="1.9"><model name="{name}"><static>true</static><link name="l">'
               + ''.join(visuals) + '</link></model></sdf>')
        request = EntityFactory_V()
        request.data.add(sdf=sdf, name=name, allow_renaming=False)
        self.node.request(f'/world/{self.world}/create_multiple', request, EntityFactory_V,
                          Boolean, 2000)
        self.models.append(name)

    def clear(self):
        """Remove todas as faixas."""
        for name in self.models:
            self.node.request(f'/world/{self.world}/remove',
                              Entity(name=name, type=Entity.MODEL), Entity, Boolean, 2000)
        self.models.clear()
        self.drawn.clear()


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
    parser.add_argument('--target', default='tractor_3', help='model the camera follows')
    parser.add_argument('--view', choices=('top', 'chase'), default='top',
                        help='top: from above, at --height; chase: at --offset')
    parser.add_argument('--height', type=float, default=26.0,
                        help='camera height above the target in the top view, in m')
    parser.add_argument('--offset', default='-14,10,12',
                        help='camera offset from the target in the chase view, x,y,z in m')
    parser.add_argument('--followers', type=int, default=4, help='tractors in the convoy')
    parser.add_argument('--width', type=float, default=0.25, help='trail width, in m')
    parser.add_argument('--tail', type=float, default=60.0,
                        help='leader distance past the curve exit when recording stops, in m')
    args = parser.parse_args(argv)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    parts = [p for p in segments(load_route(args.world)) if p.kind == 'curve']
    plan = [(int(c), parts[int(c)]) for c in args.curves.split(',')]
    if args.view == 'top':
        # Quase na vertical: um recuo pequeno evita a câmera olhar exatamente para baixo.
        x, y, z = -0.5, 0.0, args.height
    else:
        x, y, z = (float(v) for v in args.offset.split(','))
    route = load_route(args.world)
    rclpy.init()
    node = rclpy.create_node('b0_gazebo_video')
    log = node.get_logger()
    state = {'progress': None}
    node.create_subscription(Float64, '/pickup/route/progress',
                             lambda msg: state.update(progress=msg.data), 10)
    tractors = [f'tractor_{k + 1}' for k in range(args.followers)]
    trails = {v: [] for v in tractors}

    def on_pose(msg, vehicle):
        if 'recording' not in state:
            return
        pose = msg.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        point = reference_point(pose.position.x, pose.position.y, yaw, 'autonomous_tractor')
        trail = trails[vehicle]
        if not trail or math.dist(trail[-1], point) >= STEP:
            trail.append(point)

    for vehicle in tractors:
        node.create_subscription(PoseStamped, f'/ground_truth/{vehicle}/pose',
                                 lambda msg, v=vehicle: on_pose(msg, v), 10)
    markers = Trails(args.world)
    drawn = time.monotonic()
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
        if 'recording' in state and time.monotonic() - drawn > 1.0:
            drawn = time.monotonic()
            for k, vehicle in enumerate(tractors):
                # Cada trator um pouco mais alto que o anterior, para as faixas não se cruzarem.
                markers.strip(k + 1, trails[vehicle], args.width,
                              TRACTORS[k % len(TRACTORS)], 0.10 + 0.01 * k)
        if 'recording' not in state and s >= part.s0 - BEFORE:
            markers.clear()
            for trail in trails.values():
                trail.clear()
            ideal = [route.at(float(v))[:2] for v in
                     [part.s0 - BEFORE + 0.5 * i
                      for i in range(int((part.s1 - part.s0 + BEFORE + AFTER) / 0.5))]]
            markers.strip(0, ideal, args.width * 1.6, ROUTE, 0.08)
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
