#!/usr/bin/env python3
"""Dá uma volta pelos carreadores guiado pelos olhos (a câmera) e volta ao meio.

    ros2 run road_follow road_follow
    ros2 run road_follow road_follow --ros-args -p speed:=2.0
    ros2 run road_follow road_follow --ros-args -p laps:=1

Rode num segundo terminal dentro do container já *em execução*, para compartilhar o grafo do ROS
com o simulador:

    docker exec -it ros2-ws-dev bash
    ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor \
        world:=citrus_farm.sdf                  # primeiro terminal
    ros2 run road_follow road_follow             # segundo terminal

O trator sai do cruzamento no centro do mundo rumo leste, pega o carreador principal até a divisa
leste, vira à esquerda, sobe pela divisa e contorna o canto dela, vira à esquerda de novo no
carreador que volta pelo meio da propriedade e chega ao cruzamento de onde partiu. São cerca de
1,1 km, um quarto da malha, e depois ele faz tudo de novo.

**A direção é a câmera e nada mais.** Cada metro de cada reta e de cada curva é dirigido achando a
estrada em /camera_front/image_raw e guiando para pôr a máquina no meio dela. A odometria e o
receptor são lidos, mas só para responder a única pergunta que a estrada não responde: num
entroncamento onde dois carreadores se cruzam, qual deles é o meu. Isso é uma pergunta de mapa, não
de percepção, e fingir que a câmera a responde seria uma mentira disfarçada de demonstração. Por
isso a rota abaixo leva os entroncamentos e a posição da própria máquina diz quando um chegou; quem
dirige é a câmera.

O QUE ELA SEGUE. A pista de rodagem e suas duas bordas. Um carreador aqui tem 12 m de terra nua,
com 7 m de pista nivelada no meio, e essa faixa interna é alguns níveis mais clara que os ombros
dos dois lados: as bordas ficam em +/-3,5 m e o ponto médio entre elas é o eixo. Nada é pintado na
estrada, e nada é desenhado nela que uma estrada não teria -- uma linha branca foi testada e
retirada porque ninguém pinta um caminho de fazenda, e depois dela um par de sulcos de roda, porque
listras de borda dura a 2,1 m uma da outra também parecem tinta e não batiam com a `bitola` de
nenhuma máquina da propriedade. O que sobrou é a própria estrada, e ela se mostra o alvo melhor.

Achar as bordas não exige limiar que dependa do clima. O degrau da pista para o ombro é de cinco a
oito níveis de vermelho, e a própria pista muda com a luz e a inclinação -- 176 níveis a 8 m à
frente neste mundo, 156 a 14 m --, então a linha é suavizada em cinco pixels e comparada com a sua
própria mediana numa janela de 10 m. A pista é a maior parte dessa janela por construção, então a
mediana É a pista, e o que fica alguns níveis abaixo dela é ombro.

Duas faixas da imagem, uma a 6 m à frente e outra a 14 m, escolhidas pela *distância* e não pela
linha: qual linha de pixels é essa vem dos parâmetros intrínsecos da própria câmera em
/camera_front/camera_info e da altura dela no tf, então uma câmera movida na descrição move essas
faixas junto.

Um par de bordas é melhor que uma marca só, e esse é o motivo de usá-las. Duas bordas a 7 m uma da
outra, uma de cada lado do eixo, dão uma medida simétrica: o ponto médio é o eixo esteja a máquina
em esquadro com a estrada ou não, e um entroncamento, onde a pista da estrada que chega alarga a
faixa além de 7 m, falha no teste de espaçamento e é rejeitado em vez de seguido. Onde só uma borda
sobrevive, ela ainda fixa o eixo, 3,5 m ao seu lado, com um portão mais estreito. Sem as duas
significa que a pista passou dos dois extremos da janela, o que é um cruzamento ou um número
pintado sobre a estrada, e um cruzamento é onde a rota já está guiando; o seguidor mantém a
curvatura que tinha, o que numa reta é o certo e no arco de 24 m da divisa é exatamente o certo.

COMO ELA GUIA. Perseguição pura (`pure pursuit`) na faixa distante -- curvatura 2*y/(L^2 + y^2)
para pôr a máquina na estrada a L metros à frente -- mais um termo pequeno no erro próximo, para que
um desvio lateral constante seja de fato removido em vez de seguido em paralelo. O resultado é uma
curvatura, e o que sai é angular.z = speed x curvatura, pelo motivo que o teleop_key explica em
detalhe: uma máquina Ackermann lê angular.z como taxa de guinada, e uma taxa de guinada numa
velocidade é um círculo diferente em outra.

O QUE ELA NÃO FAZ. Ela não acha a estrada de novo sozinha se a perder de vez -- desacelera, mantém
e, depois de `lost_timeout`, para e avisa, que é a falha honesta para uma máquina sem mapa e com uma
câmera só.
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from tf2_ros import Buffer, TransformListener

# A volta, em pernas. Cada uma é um trecho reto de carreador: a direção em que é
# percorrida, o ponto onde termina e o que acontece ali.
#
#   "left"  vira à esquerda no próximo carreador, num arco do raio dado
#   "coast" nada a comandar -- a própria estrada curva aqui e a câmera a dirige; a
#           entrada só existe para que a perna seguinte meça a distância na direção
#           dela e não nesta
#
# Este é o citrus_farm: um quadrado de 560 m com a estrada da divisa em +/-280, o
# carreador principal ao longo de y = 0, o carreador transversal ao longo de x = 0 e
# os cantos da divisa abertos em curva de 24 m a partir de x = 256.
#
# Onde uma curva começa é geometria, não gosto: um arco de raio R que sai de um
# carreador em ângulo reto cai no seguinte R metros adiante, então o arco para a divisa
# em x = 280 tem de começar em x = 280 - R. Errar isso e a máquina cai ao lado da
# estrada e precisa voltar de lado para ela.
# AS ROTAS ESTÃO EM METROS A PARTIR DE ONDE A MÁQUINA COMEÇOU, não da origem do mundo, e
# as duas coisas só são iguais porque toda rota aqui parte da origem do mundo. O
# referencial do fix é a primeira coisa que ele vê, que é a máquina parada na largada
# (veja _on_fix), então um veículo criado 40 m a oeste da origem vira 40 m antes -- que é
# exatamente o que aconteceu na primeira vez que docs/simulation.mp4 foi filmado em
# "origin_bend": o trator saiu do carreador e entrou no talhão a 3 m/s, confiante.
# Comece na origem ou escreva a rota no referencial em que você começa.
ROUTES = {
    # A volta, e o padrão: até a divisa, contorna o canto, volta pelo meio da
    # propriedade até o cruzamento de onde partiu.
    "lap": [
        # direção       a perna termina em   ali         raio
        ((1.0, 0.0), (268.0, 0.0), "left", 12.0),      # principal, leste até a divisa
        ((0.0, 1.0), (280.0, 256.0), "coast", 24.0),   # divisa, norte até o canto
        ((-1.0, 0.0), (12.0, 280.0), "left", 12.0),    # divisa, oeste até o carreador transversal
        ((0.0, -1.0), (0.0, 6.0), "left", 6.0),        # transversal, sul até o cruzamento
    ],
    # Sem rota nenhuma: segue o carreador em que a máquina está até ser parada. "run" é
    # uma perna que nunca termina, então nenhum entroncamento é alcançado e nunca se vira
    # -- o que se quer numa estrada de mão única, como a de 60 graus pelo talhão nordeste,
    # e o que o trabalho de câmera em docs/film usa.
    "follow": [((1.0, 0.0), (0.0, 0.0), "run", 0.0)],
    # Leste a partir do centro do mundo pelo carreador principal e depois para fora
    # dele, na estrada de 30 graus que sai para o talhão sudeste. Uma reta curta pelo
    # cruzamento e depois uma curva, que é o que docs/simulation.mp4 mostra.
    #
    # O arco começa 5,4 m antes do entroncamento e não nele: um arco de raio R que gira
    # um ângulo come R*tan(metade do ângulo) da estrada em cada ponta, o que a 20 m e 30
    # graus dá 5,36. Comece a curva no próprio entroncamento e a máquina a termina cinco
    # metros dentro do talhão.
    "origin_bend": [
        ((1.0, 0.0), (42.7, 0.0), "right", 20.0),          # o principal, leste
        ((0.866, -0.5), (0.0, 0.0), "run", 0.0),           # a estrada de 30 graus
    ],
}
# O último arco é o fechado, e esse é o ponto dele. Contornar o cruzamento a 6 m traz a
# máquina a 2,5 m da origem do mundo de onde partiu; nos 12 m que as outras curvas usam
# ela passaria 5 m longe. Um veículo Ackermann não consegue chegar a um ponto e sair dele
# em outra direção, então "voltar ao meio" é uma curva feita rente, não um ponto
# acertado.


class LineFollow(Node):
    def __init__(self):
        super().__init__("road_follow")

        def param(name, default):
            return self.declare_parameter(name, default).value

        # Velocidades. A máquina chega a 4,2 m/s; um trator num carreador com uma
        # câmera por olhos não anda isso, e este também não.
        self.speed = param("speed", 3.0)
        self.bend_speed = param("bend_speed", 2.0)
        self.turn_speed = param("turn_speed", 1.2)
        self.accel = param("accel", 1.0)

        # Onde as duas faixas olham, em metros de chão à frente. A próxima é tão perto
        # quanto a câmera vê o chão -- 4,5 m na base do quadro -- e a distante é tão
        # longe quanto a estrada fica no quadro pelo canto de 24 m da divisa, cerca de
        # 15 m.
        self.near_d = param("near_distance", 6.0)
        self.far_d = param("far_distance", 14.0)

        # A pista de rodagem, de generate_agriculture_worlds.py: ROAD_W é 7,0 m de pista
        # nivelada no meio de 12 m de terra nua, então as bordas ficam em +/-3,5 m e o
        # ponto médio é o eixo. `step` é quanto abaixo da mediana da própria pista um
        # pixel tem de cair para ser ombro e não pista -- a diferença medida é de 5 a 8
        # níveis, contra uma pista que sozinha varia 176 a 8 m e 156 a 14, então o teste
        # tem de ser relativo e o limiar pequeno. `window` é quanto olhar de cada lado do
        # último centro.
        self.surface_half = param("surface_half", 3.5)
        self.surface_step = param("surface_step", 3.0)
        self.window = param("search_window", 5.0)

        # Direção. min_radius é o da própria máquina, de params.xacro por meio de
        # autonomous_tractor.gz.xacro: 0,60 rad de esterço num entre-eixos de 2,15 m. tau
        # é a rapidez com que a roda pode se mover, o que impede que um centroide ruidoso
        # vire uma sacudida.
        self.min_radius = param("min_radius", 3.2)
        self.near_gain = param("near_gain", 0.06)
        self.tau = param("steer_tau", 0.30)

        # O portão: quanto o meio da estrada pode se mover de um quadro para o seguinte e
        # ainda ser acreditado. Em seguimento, 1,0 m é na verdade um limite de velocidade
        # e não um portão -- quinze quadros por segundo a 3 m/s são 0,2 m de chão por
        # quadro, e a estrada não se move de lado mais rápido que isso, então o que se
        # move é um entroncamento comendo um acostamento e não uma estrada indo a algum
        # lugar. Em aquisição, recém-saída de uma curva e sem histórico algum, ele tem de
        # ser largo o bastante para achar o carreador em que a máquina está.
        self.gate_track = param("gate_track", 1.0)
        self.gate_acquire = param("gate_acquire", 6.0)
        self.reacquire_after = param("reacquire_after", 1.5)

        # Onde a máquina está, só para a rota. A odometria das rodas sozinha não basta:
        # uma volta deste circuito tem 1107 m com três curvas de 90 graus, e a odometria
        # voltou 9 m errada -- 0,8%, número comum para uma máquina Ackermann que patina e
        # suficiente para começar uma curva no lugar errado na terceira volta. O receptor
        # no mastro não deriva, então os dois são fundidos da forma óbvia: a odometria
        # para a suavidade entre fixes, o fix para onde essa suavidade está ancorada. A
        # origem dele é a primeira coisa que vê, que é a máquina parada no centro do
        # mundo, então a rota abaixo está em metros a partir dali e nenhum datum de mundo
        # fica escrito.
        self.use_gnss = param("use_gnss", True)
        self.fix_samples = param("fix_samples", 25)
        self.fix_alpha = param("fix_alpha", 0.05)

        # Qual das ROUTES dirigir. "lap" é o circuito; "follow" só se mantém na
        # estrada em que está.
        self.route = ROUTES[param("route", "lap")]

        self.laps = param("laps", 0)             # 0 = até ser parado
        # Por quanto tempo a máquina pode dirigir na curvatura mantida antes de desistir
        # e parar. Tem de ser maior que a cegueira honesta mais longa deste circuito, que
        # é o número pintado no canto da divisa: 7,8 m de tábua clara atravessada na
        # estrada, cruzada a 1,2 m/s, dão seis segundos e meio, e com seis a volta parou
        # seca em cima dela.
        self.lost_timeout = param("lost_timeout", 12.0)
        self.rate = param("rate", 30.0)

        self.pub = self.create_publisher(Twist, param("topic", "cmd_vel"), 10)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "camera_front/image_raw",
                                 self._on_image, sensor_qos)
        self.create_subscription(CameraInfo, "camera_front/camera_info",
                                 self._on_info, sensor_qos)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(NavSatFix, "gnss/fix", self._on_fix, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # câmera, preenchida a partir de camera_info e tf
        self.fx = self.fy = self.cx = self.cy = None
        self.cam_height = None

        # onde estamos
        self.pos = None                 # odometria mais a correção do fix
        self.odom_xy = None             # só a odometria, como publicada
        self.offset = (0.0, 0.0)        # o que o fix manda somar a ela
        self.fix_origin = None          # (lat, lon) do centro do mundo
        self.fix_seen = []
        self.yaw = 0.0
        self.travelled = 0.0
        self.last_pos = None

        # O que a câmera disse por último, faixa a faixa. Cada faixa segue o seu próprio
        # trecho de estrada: condicionar a distante ao que a próxima vê parece mais
        # arrumado e está errado, porque a distante olha chão que a próxima só alcança
        # daqui a alguns segundos, e num entroncamento esse é exatamente o chão onde
        # falta o acostamento.
        self.track = {"near": None, "far": None}
        self.track_at = {"near": 0.0, "far": 0.0}
        self.seen_at = 0.0
        self.near_y = self.far_y = None
        self.wide = False

        # o que estamos fazendo
        self.leg = 0
        self.lap = 1
        self.state = "wait"
        self.curvature = 0.0
        self.commanded = 0.0
        self.out_speed = 0.0
        self.turn_from = 0.0
        self.turned = 0.0
        self.turn_sign = 0.0
        self.turn_sweep = math.pi / 2
        self.turn_radius = 0.0
        self.lap_start = None
        self.lap_length = 0.0
        self.closest = 1e9
        self.stopped_reason = None

    # -- o que entra -------------------------------------------------------
    def _on_info(self, msg):
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(
                f"camera: {msg.width}x{msg.height}, f = {self.fx:.1f} px, "
                f"centre ({self.cx:.1f}, {self.cy:.1f})")

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        here = (p.x, p.y)
        # A distância percorrida vem da odometria crua: é uma diferença em 30 ms e a
        # correção do fix não tem o que fazer nela.
        if self.last_pos is not None:
            step = math.dist(here, self.last_pos)
            self.travelled += step
            self.lap_length += step
        self.last_pos = self.odom_xy = here
        self.pos = (here[0] + self.offset[0], here[1] + self.offset[1])
        # Quão perto a volta chegou do meio do mundo, que é para o que a volta serve.
        # Só depois que a máquina de fato saiu: uma volta que começa na origem está
        # trivialmente a 0,00 m dela, e relatar isso seria se elogiar.
        if self.lap_length > 50.0:
            self.closest = min(self.closest, math.hypot(*self.pos))

    def _on_fix(self, msg):
        """O receptor, transformado em metros de onde a máquina começou.

        Os primeiros `fix_samples` fixes são tirados em média com a máquina parada, e essa
        média é a origem: o meio do mundo, que é onde sim.launch.py põe um veículo. Tudo
        depois é um deslocamento dela, nos eixos do próprio mundo, porque estes mundos são
        montados em ENU e sem rotação de rumo.
        """
        if not self.use_gnss or math.isnan(msg.latitude):
            return
        if self.fix_origin is None:
            self.fix_seen.append((msg.latitude, msg.longitude))
            if len(self.fix_seen) >= self.fix_samples:
                lat = sum(f[0] for f in self.fix_seen) / len(self.fix_seen)
                lon = sum(f[1] for f in self.fix_seen) / len(self.fix_seen)
                # Metros por grau no WGS84 nesta latitude. O atalho esférico usual
                # erra 0,4% aqui, o que em 280 m de carreador é um metro de onde uma
                # curva começa.
                phi = math.radians(lat)
                self.per_lat = (111132.92 - 559.82 * math.cos(2 * phi)
                                + 1.175 * math.cos(4 * phi))
                self.per_lon = (111412.84 * math.cos(phi)
                                - 93.5 * math.cos(3 * phi))
                self.fix_origin = (lat, lon)
                self.get_logger().info(
                    f"origem do GNSS fixada em {len(self.fix_seen)} amostras: "
                    f"{lat:.6f}, {lon:.6f}")
            return
        if self.odom_xy is None:
            return
        east = (msg.longitude - self.fix_origin[1]) * self.per_lon
        north = (msg.latitude - self.fix_origin[0]) * self.per_lat
        # Um puxão lento, não um salto. O fix traz um metro de ruído e a odometria não
        # traz nenhum, então do fix só se quer a falta de deriva: a 5 Hz e 0,05 a
        # correção tem constante de tempo de quatro segundos e um sexto do ruído de que
        # é feita.
        want = (east - self.odom_xy[0], north - self.odom_xy[1])
        a = self.fix_alpha
        self.offset = (self.offset[0] + (want[0] - self.offset[0]) * a,
                       self.offset[1] + (want[1] - self.offset[1]) * a)

    def _row_for(self, distance):
        """A linha da imagem em que cai o chão a `distance` à frente.

        Uma câmera olhando nivelada a uma altura h vê o chão à distância d num ângulo
        atan(h/d) abaixo do eixo, que é fy*h/d pixels abaixo do ponto principal. Sem
        aproximação de ângulo pequeno e sem calibração a olho: os dois números vêm da
        própria descrição do robô.
        """
        return int(round(self.cy + self.fy * self.cam_height / distance))

    def _surface(self, row, lo, hi):
        """Onde a pista nivelada começa e termina num trecho de uma linha.

        Devolve (início, fim) na linha, ou None. A diferença entre a pista e a terra nua
        ao lado é de cinco a oito níveis de vermelho, que é real e pequena, então a linha
        é suavizada primeiro e depois comparada com a sua própria mediana e não com
        número algum: a pista é a maior parte da janela por construção, então a mediana
        é a pista, e o que fica alguns níveis abaixo dela é ombro.
        """
        red = row[lo:hi, 0].astype(np.float32)
        if red.size < 16:
            return None
        smooth = np.convolve(red, np.ones(5, np.float32) / 5.0, mode="same")
        # A mediana deve ser a pista, e uma coisa nestes mundos é muito mais clara que a
        # pista: a tábua pálida em que o ângulo de uma curva é pintado, 7,8 por 6,0 m e
        # atravessada na estrada. Onde ela cobre parte da janela, arrasta a mediana para
        # cima e a pista de verdade cai do lado errado do limiar, então a mediana é
        # tomada de novo sobre o que sobra depois de excluída a tábua.
        level = np.median(smooth)
        keep = smooth <= level + 8.0
        if keep.any():
            level = np.median(smooth[keep])
        bright = (smooth >= level - self.surface_step) & keep
        if not bright.any():
            return None
        edges = np.flatnonzero(
            np.diff(np.concatenate(([0], bright.astype(np.int8), [0]))))
        runs = [(lo + a, lo + b) for a, b in zip(edges[0::2], edges[1::2])]
        return runs or None

    def _band(self, image, distance, band, now):
        """O meio do carreador numa faixa, como coluna de pixels.

        Devolve (coluna, nada utilizável visto). O segundo não chega a ser falha: nestas
        estradas significa que a pista passou dos dois extremos da janela, o que
        acontece num cruzamento e sobre um número pintado, e quem chamou mantém em vez
        de guiar.
        """
        row_index = self._row_for(distance)
        if not 0 <= row_index < image.shape[0]:
            return None, False

        # Perdida por tempo suficiente, esta faixa esquece onde a estrada estava, o que
        # a deixa achar uma de novo depois de uma curva.
        if now - self.track_at[band] > self.reacquire_after:
            self.track[band] = None
        tracking = self.track[band] is not None
        anchor = self.track[band] if tracking else self.cx

        scale = self.fx / distance                  # pixels por metro lá longe
        lo = max(0, int(anchor - self.window * scale))
        hi = min(image.shape[1], int(anchor + self.window * scale))
        runs = self._surface(image[row_index], lo, hi)
        if not runs:
            return None, True
        here = [r for r in runs if r[0] <= anchor < r[1]]
        start, stop = here[0] if here else max(runs, key=lambda r: r[1] - r[0])

        # Uma borda que cai no fim da janela não é borda: é onde o olhar parou.
        left = None if start <= lo else start
        right = None if stop >= hi else stop - 1

        half_px = self.surface_half * scale
        gate_px = (self.gate_track if tracking else self.gate_acquire) * scale
        if left is not None and right is not None:
            span = right - left
            if not 0.6 * 2 * half_px <= span <= 1.4 * 2 * half_px:
                return None, True
            u = (left + right) / 2.0
        elif left is not None:
            u = left + half_px
            gate_px = min(gate_px, 1.0 * scale)
        elif right is not None:
            u = right - half_px
            gate_px = min(gate_px, 1.0 * scale)
        else:
            return None, True

        if abs(u - anchor) > gate_px:
            return None, True
        self.track[band] = u
        self.track_at[band] = now
        return u, False

    def _on_image(self, msg):
        if self.fx is None or self.cam_height is None:
            return
        if msg.encoding != "rgb8":
            self.get_logger().warn(f"camera says {msg.encoding}, expected rgb8",
                                   once=True)
            return
        image = (np.frombuffer(msg.data, np.uint8)
                 .reshape(msg.height, msg.step)[:, :msg.width * 3]
                 .reshape(msg.height, msg.width, 3))

        now = time.monotonic()
        near_u, near_junction = self._band(image, self.near_d, "near", now)
        far_u, far_junction = self._band(image, self.far_d, "far", now)
        self.wide = near_junction or far_junction

        def lateral(u, distance):
            """Metros à esquerda do eixo da máquina. u à direita do centro é direita."""
            return (self.cx - u) * distance / self.fx

        if near_u is not None:
            self.near_y = lateral(near_u, self.near_d)
            self.seen_at = now
        else:
            self.near_y = None
        self.far_y = lateral(far_u, self.far_d) if far_u is not None else None

    # -- o que sai ----------------------------------------------------------
    def _steer_from_road(self):
        """Curvatura que a câmera pede, ou None se não enxerga."""
        if self.far_y is None and self.near_y is None:
            return None
        # Perseguição pura na faixa distante: o arco daqui até um ponto y metros ao lado,
        # L metros à frente, tem curvatura 2y/(L^2 + y^2), que é exata e não uma
        # aproximação -- ponha o ponto num círculo pela máquina tangente ao eixo dela e a
        # fórmula devolve a curvatura desse mesmo círculo.
        kappa = 0.0
        if self.far_y is not None:
            L = self.far_d
            kappa += 2.0 * self.far_y / (L * L + self.far_y * self.far_y)
        # e um puxão suave no erro próximo, que é o que tira um desvio que só a visada
        # longa dirigiria alegremente em paralelo.
        if self.near_y is not None:
            kappa += self.near_gain * self.near_y
        limit = 1.0 / self.min_radius
        return max(-limit, min(limit, kappa))

    def _remaining(self):
        """Metros que faltam desta perna, medidos na direção da própria perna.

        Na direção da perna e não da máquina, porque uma perna contém o canto de 24 m da
        divisa: medida numa direção que ela mesma gira 90 graus, a distância ao fim da
        perna andaria para trás na metade da curva.
        """
        (dx, dy), (ex, ey), action, _ = self.route[self.leg]
        if action == "run":
            return float("inf")          # uma perna que nunca termina
        return (ex - self.pos[0]) * dx + (ey - self.pos[1]) * dy

    def _sweep_to_next_leg(self):
        """Quanto esta curva gira, a partir das duas pernas que ela une.

        Tirado das direções da própria rota e não escrito ao lado da curva, porque um
        número escrito duas vezes é um número que vai discordar de si mesmo -- e este
        discordou. Na verdade nem estava escrito: toda curva rodava até a máquina ter
        girado 90 graus, o que está certo para três lados de um quadrado e errado para a
        estrada de 30 graus saindo do principal. Pedidos 30, o trator girou 90, saiu do
        carreador e entrou no talhão a 3 m/s, confiante.
        """
        here = self.route[self.leg][0]
        nxt = self.route[(self.leg + 1) % len(self.route)][0]
        sweep = math.atan2(nxt[1], nxt[0]) - math.atan2(here[1], here[0])
        return math.atan2(math.sin(sweep), math.cos(sweep))

    def _next_leg(self):
        self.leg += 1
        if self.leg < len(self.route):
            return
        self.leg = 0
        self.lap += 1
        self.get_logger().info(
            f"volta {self.lap - 1}: {self.lap_length:.0f} m, passou a "
            f"{self.closest:.2f} m da origem, correcao do GNSS "
            f"({self.offset[0]:+.1f}, {self.offset[1]:+.1f}) m")
        self.lap_length = 0.0
        self.closest = 1e9
        if self.laps and self.lap > self.laps:
            self.state = "done"
            self.stopped_reason = f"{self.laps} volta(s) completa(s)"

    def tick(self):
        dt = 1.0 / self.rate

        if self.state == "wait":
            if self.pos is None or self.fx is None:
                return
            if self.use_gnss and self.fix_origin is None:
                return              # ainda parado, tirando a média dos fixes
            if self.cam_height is None and not self._lookup_camera():
                return
            self.state = "follow"
            # O relógio de "há quanto tempo a linha foi vista" começa aqui e não na
            # construção: tudo antes disto era espera pela ponte, e nada disso foi
            # direção às cegas.
            self.seen_at = time.monotonic()
            self.get_logger().info("na estrada, seguindo")

        if self.state == "done":
            self.out_speed = max(0.0, self.out_speed - self.accel * dt)
            self.commanded = 0.0
            self._publish()
            return

        if self.state == "follow":
            asked = self._steer_from_road()
            if asked is None:
                # Nada a ver. Mantém o volante onde está e alivia: num arco isso é a
                # resposta certa, e numa reta não custa nada. O que não pode é endireitar,
                # o que levaria a máquina para fora de uma curva.
                lost_for = time.monotonic() - self.seen_at
                target_speed = self.bend_speed if lost_for < 1.0 else self.turn_speed
                if lost_for > self.lost_timeout:
                    self.state = "done"
                    self.stopped_reason = f"estrada perdida por {lost_for:.1f} s"
            else:
                self.curvature = asked
                target_speed = (self.speed if abs(self.curvature) < 0.02
                                else self.bend_speed)

            if self.pos is not None and self._remaining() <= 0.0:
                action = self.route[self.leg][2]
                if action == "coast":
                    self._next_leg()
                else:
                    self.turn_radius = self.route[self.leg][3]
                    self.turn_sweep = self._sweep_to_next_leg()
                    self.turn_sign = math.copysign(1.0, self.turn_sweep)
                    self.turn_from = self.yaw
                    self.turned = 0.0
                    self.state = "turn"
            elif self.pos is not None:
                # Reduz antes de um entroncamento, como faz qualquer coisa com reboque.
                if self._remaining() < 25.0:
                    target_speed = min(target_speed, self.turn_speed)

        if self.state == "turn":
            # Curvatura constante, e a odometria diz quando 90 graus dela se passaram. A
            # câmera não é consultada: num entroncamento a linha em que ela travaria é a
            # que está sendo deixada.
            step = self.yaw - self.turn_from
            self.turn_from = self.yaw
            self.turned += math.atan2(math.sin(step), math.cos(step))
            self.curvature = self.turn_sign / self.turn_radius
            target_speed = self.turn_speed
            if abs(self.turned) >= abs(self.turn_sweep):
                # A linha que esta faixa seguia é a que está sendo deixada, então as
                # duas faixas recomeçam do meio da imagem.
                self.track = {"near": None, "far": None}
                self.near_y = self.far_y = None
                self.seen_at = time.monotonic()
                self.state = "follow"
                self._next_leg()

        # A roda se move a uma taxa e o acelerador a uma aceleração, porque o plugin
        # Ackermann já tem a própria taxa limitada e comandos em degrau só se acumulam
        # na fila.
        alpha = min(1.0, dt / self.tau)
        self.commanded += (self.curvature - self.commanded) * alpha
        if target_speed > self.out_speed:
            self.out_speed = min(target_speed, self.out_speed + self.accel * dt)
        else:
            self.out_speed = max(target_speed, self.out_speed - self.accel * dt)
        self._publish()

    def _lookup_camera(self):
        """A altura da câmera, perguntada ao robô em vez de suposta."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_footprint", "camera_front_link", rclpy.time.Time())
        except Exception:
            return False
        self.cam_height = tf.transform.translation.z
        self.get_logger().info(
            f"camera a {self.cam_height:.3f} m do solo; bandas em "
            f"{self.near_d:.0f} m (fileira {self._row_for(self.near_d)}) e "
            f"{self.far_d:.0f} m (fileira {self._row_for(self.far_d)})")
        return True

    def _publish(self):
        msg = Twist()
        msg.linear.x = self.out_speed
        msg.angular.z = self.out_speed * self.commanded
        self.pub.publish(msg)

    def status(self):
        if self.pos is None:
            return "esperando odometria"
        radius = 1.0 / self.commanded if abs(self.commanded) > 1e-4 else float("inf")
        where = ("perdida" if self.near_y is None and self.far_y is None
                 else f"{self.near_y:+.2f} m" if self.near_y is not None
                 else f"({self.far_y:+.2f} m)")
        return (f"volta {self.lap} perna {self.leg + 1}/{len(self.route)} {self.state:6s} "
                f"({self.pos[0]:+7.1f},{self.pos[1]:+7.1f}) "
                f"faltam {max(0.0, self._remaining()):6.1f} m  "
                f"v {self.out_speed:4.1f} m/s  R {radius:8.1f} m  "
                f"eixo {where}{'  [cruzamento]' if self.wide else ''}")


def main():
    rclpy.init()
    node = LineFollow()
    period = 1.0 / node.rate
    last_print = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=period)
            node.tick()
            now = time.monotonic()
            if now - last_print > 0.5:
                last_print = now
                print("\r\033[K" + node.status(), end="", flush=True)
            if node.state == "done" and node.out_speed <= 0.0:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print()
        if node.stopped_reason:
            node.get_logger().info(node.stopped_reason)
        # Para a máquina na saída, e só enquanto ainda há um contexto onde publicar: um
        # Ctrl-C ou um SIGTERM já derrubou o rclpy quando isto roda, e publicar nisso
        # levanta erro em vez de parar algo. O que protege o trator nesse caso é o plugin
        # Ackermann, que freia com um comando velho.
        if rclpy.ok():
            stop = Twist()
            for _ in range(5):
                node.pub.publish(stop)
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
