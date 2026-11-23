# ftl

**Follow the leader** (siga o líder): uma fila de máquinas agrícolas em que a primeira é dirigida e
as outras a seguem. Este repositório é o código do próprio projeto, a parte que nenhum outro
projeto usa. A plataforma em que ele roda (veículos, mundos, condução) vem de `robot_description`,
`agri_gazebo` e `vehicle_control`, listados com este repositório no `projects/ftl/ftl.repos` do
workspace.

```
ftl_follower_pure_pursuit/  a B0: pose_estimator (todo veículo) e follower (cada seguidor)
road_follow/      dirige um veículo pelos carreadores pela câmera — o motorista do líder do FTL
ftl_bringup/      os nós do FTL e seus parâmetros, num launch só. Sem simulador.
├── launch/ftl.launch.py
└── config/ftl.yaml           um arquivo por experimento é o uso pretendido
ftl_gz/           cenários de teste do FTL no Gazebo: mundo, líder, seguidores, onde começam
├── launch/sim.launch.py      só o cenário (mundo + comboio)
└── launch/ftl_sim.launch.py  o cenário e os nós do FTL, num comando
ftl_msgs/         as mensagens que ligam os módulos da solução
ftl_template/     o pacote modelo: copie-o para começar um pacote novo
```

## Pacotes novos

A percepção do líder (`ftl_perception`) entra aqui como pacote quando for escrita. Um pacote só sai
para um repositório próprio se um segundo projeto precisar dele. Todo pacote novo nasce de uma cópia
do `ftl_template/`, que já traz a lógica sem ROS, o nó fino, os testes e o lint; o passo a passo
(copiar, trocar o nome, pôr no comboio) e os padrões a seguir estão no `CONTRIBUTING.md`.

Para subir junto com o comboio, o nó entra no `ftl_bringup/launch/ftl.launch.py` (parâmetros no
`ftl_bringup/config/ftl.yaml`) e o `ftl_bringup/package.xml` ganha
`<exec_depend>ftl_perception</exec_depend>`. Um nó novo segue a regra abaixo: fala com o veículo só
pelos tópicos do namespace dele.

## Mensagens (ftl_msgs)

| Mensagem | Para quê | Publicada por |
|---|---|---|
| TrackedVehicle | estimativa do veículo da frente (antena, velocidade, fontes) | ftl_fusion |
| Rail, com RailPoint | trilho virtual do líder | ftl_trajectory |
| Progress | progresso no trilho e liberação de waypoints | ftl_trajectory |
| FollowerStatus | estado do supervisor, erros e fontes | ftl_supervisor |
| V2VEnvelope | envelope do canal V2V (fix, estado ou trilho) | ftl_comms |

Os pacotes que publicam essas mensagens ainda não existem e chegarão nas fases F3 a F8.

## A regra que o mantém portável

Só o `ftl_gz` sabe que existe um simulador. O `ftl_bringup` e todo nó falam com um veículo pelos
tópicos do seu namespace — `cmd_vel`, `odom`, `gnss/fix`, `imu/data`, `camera_front/*` — e pelo
seu TF, então o mesmo launch dirige o comboio no Gazebo ou na fazenda; `use_sim_time` é a única
chave. As dependências apontam para um lado só: `ftl_gz` → `ftl_bringup`, `ftl_gz` →
`agri_gz_bringup`, nunca o contrário.

Cada veículo mantém sua árvore de TF no próprio namespace (`/pickup/tf`, `/pickup/tf_static`),
então um nó que lê TF leva os remaps `/tf → tf` e `/tf_static → tf_static`. O `ftl.launch.py`
faz isso.

## Rodando um teste

Da raiz do workspace (o harness), em containers:

```bash
scripts/up.sh ftl                          # Gazebo + os nós do FTL, um container cada
GUI=false FOLLOWERS=4 scripts/up.sh ftl -d
docker compose -p ftl restart ftl          # código novo do FTL, o Gazebo continua rodando
docker compose -p ftl down
```

O comboio precisa de cerca de um minuto para partir: os controladores de cada veículo sobem um por
vez, e a pickup começa a andar quando o `road_follow` enxerga a estrada. Confira com
`docker compose -p ftl exec ftl bash -c 'ros2 control list_controllers -c /pickup/controller_manager'`
(os três `active`), e `ros2 topic echo /tractor_1/ftl/gap std_msgs/msg/Float64` para um seguidor.

Ou à mão, dentro do `./docker/run.sh`:

```bash
ros2 launch ftl_gz ftl_sim.launch.py                       # os dois de uma vez
ros2 launch ftl_gz sim.launch.py followers:=4 gui:=false   # só o cenário…
ros2 launch ftl_bringup ftl.launch.py use_sim_time:=true   # …e os nós, num segundo shell
ros2 launch ftl_bringup ftl.launch.py params_file:=/workspace/exp_01.yaml
```

O cenário padrão: uma pickup liderando dois tratores autônomos pelo carreador principal do
`citrus_farm.sdf`, a dez metros um do outro; o `road_follow` leva a pickup pela volta e cada trator
segue a máquina à sua frente. Argumentos do cenário: `world`, `leader`, `follower`, `followers`,
`spacing`, `yaw`, `gui` e `sensor_profile`. O `sensor_profile` padrão é o `nominal` (GNSS RTK a
1 Hz, com 2 cm de dispersão; IMU a 20 Hz sem ruído, D13); `default` dá ao GNSS 1 m de ruído, e
`ideal` tira o ruído de tudo.

## CI

O workflow `.github/workflows/ci.yml` roda a cada push no `main`, em todo pull request e sob
demanda (`workflow_dispatch`). Num container `ros:jazzy-ros-base` ele baixa os repositórios da
plataforma, instala as dependências com o rosdep, compila os pacotes do ftl, roda os testes e
mostra o resultado de cada arquivo de teste. Os testes de simulação, com a marca `simulation`,
ficam de fora: o CI não tem simulador nem GPU.

Os repositórios `robot_description`, `agri_gazebo` e `vehicle_control` são privados, então o
workflow precisa de um token: um *fine-grained personal access token* com leitura de Contents
desses três repositórios, salvo neste repositório como o secret `WORKSPACE_REPOS_TOKEN` (Settings
→ Secrets and variables → Actions), o mesmo do `vehicle_control`.

## Como um seguidor segue

```
todo veículo     pose_estimator   odom + imu/data + gnss/fix  ->  ftl/state   (frame comum ftl_map)
cada seguidor    follower         o próprio ftl/state + o ftl/state do veículo da frente  ->  cmd_vel
```

**Um frame para o comboio inteiro.** A odometria de cada veículo começa onde aquele veículo foi
criado, então duas odometrias não se comparam. O `pose_estimator` põe todas as máquinas no mesmo
frame leste/norte, em metros a partir de um datum (`datum_lat`/`datum_lon` no `ftl.yaml`, a origem
dos mundos de citros): rumo pela IMU, que reporta orientação no frame ENU do mundo; posição por
dead-reckoning da odometria ao longo desse rumo, puxada para o fix do GNSS com constante de tempo
de quatro segundos, descontado antes o deslocamento da antena em relação ao `base_footprint`. A
saída, `ftl/state` (`nav_msgs/Odometry`), é o que viajaria pelo rádio entre máquinas reais.

**Caminho, não posição.** O `follower` guarda as posições do veículo da frente como um rastro e
esterça por pure pursuit no ponto do rastro um lookahead (4 m + 1 s × velocidade, no máximo 10 m) à
frente de si. Mirar no próprio veículo cortaria toda curva de um carreador.

**Distância ao longo do caminho.** Quão atrás ele está é o comprimento de arco daquele rastro entre
as duas máquinas, então uma curva não é lida como estar perto demais. A velocidade é a do veículo
da frente mais 0,4 (m/s)/m de erro de distância, em rampa de 1 m/s², limitada a 4 m/s, e zero
abaixo de `min_gap` (5 m) ou quando qualquer um dos `ftl/state` tem mais de um segundo. Cada
seguidor segue o da frente, então o `tractor_2` segue o `tractor_1`, e não a pickup.

**Zona morta.** O parâmetro `uncertainty` (1 m no perfil `default`) é a incerteza da posição
estimada: erros de distância menores que ela não geram correção, e o espaçamento do rastro nunca
fica abaixo dela. Com o veículo da frente parado, o vaivém da estimativa não infla mais o arco.

Todo número acima é um parâmetro no `ftl_bringup/config/ftl.yaml`. Para experimentos, cada seguidor
também publica `ftl/gap` (metros ao longo do caminho) e `ftl/cross_track` (metros fora dele,
esquerda positiva).

**Medido** (2026-11-08, `citrus_farm.sdf`, pickup + 2 tratores, volta do `road_follow` a 3 m/s,
descontado o primeiro quinto de cada execução):

| | 222 m em reta | 358 m com a curva de 90° na divisa leste |
|---|---|---|
| distância, ajustada em 10 m | 9,47 – 10,56 m, média 10,02–10,03 | 8,30 – 10,83 m, média 9,99–10,04 |
| fora do caminho do líder | média 0,12–0,13 m, p95 0,30 m, máx 0,42 m | média 0,14 m, p95 0,36 m, máx 0,66 m |
| estimativa vs pose verdadeira do Gazebo, no fim | 0,14 – 0,41 m | — |

Os veículos são dirigidos pelo ros2_control desde 2026-11-11, com tração nas quatro desde
2026-11-12; os números acima foram tirados primeiro com o plugin do Gazebo, que saiu. O mesmo
experimento nos dois caminhos, com quatro seguidores e na volta completa de 1109 m (a pickup volta
a 2,07 m da origem, distância média de 10,00 / 10,02 m), está no README do `vehicle_control`.

Testes do rastro, das leis de controle e da geodésia: `scripts/test.sh ftl_follower_pure_pursuit`,
que roda os testes de unidade, o lint e a guarda.

Os seguidores levam GNSS e IMU, mas não câmera (`followers_camera:=false` no comboio), então o
comboio custa uma câmera, qualquer que seja o tamanho dele.

## Linha de base B0

A B0 é o pacote `ftl_follower_pure_pursuit`: o seguidor da monografia, congelado (D2), com o
estimador complementar de pose e o pure pursuit sobre o rastro do precedente. O algoritmo não
muda; uma guarda de testes confere isso. Ela não tem câmera, UKF nem trilho comum.

A velocidade do precedente vem do `ftl/state`, como se chegasse pelo rádio. A zona morta
(`uncertainty`) é um recurso do ambiente de teste e fica desligada (0) nos experimentos.

A B0 é comparada com as soluções seguintes como diz a seção 8.4 do plano: os mesmos cenários e as
mesmas sementes, o caminho estimado de cada veículo e as tabelas pareadas.

## road_follow: dirigindo pelas bordas da estrada

```bash
ros2 run road_follow road_follow                      # dá voltas até ser parado
ros2 run road_follow road_follow --ros-args -p laps:=1 -p speed:=2.0
ros2 run road_follow road_follow --ros-args -p route:=follow   # sem rota: só se mantém na estrada
```

O trator sai do cruzamento no centro do mundo rumo leste, pega o carreador principal até a divisa
leste, vira à esquerda, percorre 244 m divisa acima e contorna a sua curva de 24 m, vira à esquerda
de novo no carreador que volta pelo meio da propriedade, e chega ao cruzamento de onde partiu.
**1109 m, um quarto da rede**, e então faz de novo, voltando a cerca de **2 m** da origem do mundo
a cada vez (1,8 m com o antigo plugin do Gazebo, 2,07 m com a tração nas quatro do ros2_control) — uma curva feita de perto, e não um ponto acertado, porque uma máquina Ackermann não
consegue chegar a um ponto e sair dele em outra direção.

**A direção é a câmera e nada mais.** O que ela segue é a **pista de rodagem**: um carreador são
12 m de terra nua com 7 m de pista patrolada no meio, e essa faixa interna é algumas contagens mais
clara que os acostamentos dos dois lados, então as duas bordas ficam a ±3,5 m e o ponto médio delas
é a linha central. Um par de bordas é melhor que uma marca — o ponto médio é o meio da estrada
esteja a máquina alinhada com ela ou não, e um entroncamento, onde a pista da estrada que chega
alarga a faixa para além de 7 m, falha no teste de espaçamento e é rejeitado em vez de seguido.
Quando uma borda se perde, a outra ainda fixa o centro 3,5 m ao lado dela, com uma janela mais
estreita.

Nada aqui é um limiar que dependa do tempo. O degrau da pista para o acostamento é de **cinco a
oito contagens** de vermelho, e a própria pista muda com a luz e a inclinação — 176 contagens a 8 m
à frente, 156 a 14 m —, então a linha é suavizada em cinco pixels e comparada com a **sua própria
mediana** numa janela de 10 m: por construção a pista é a maior parte daquela janela, então a
mediana é a pista. As duas faixas olham 6 m e 14 m à frente, e qual linha de pixels é essa vem do
`/camera_front/camera_info` e da altura da câmera no `tf`, então uma câmera movida na descrição
move as faixas junto. Pure pursuit na faixa distante, um pequeno puxão na próxima, e sai
`angular.z = velocidade × curvatura`.

O `route:=` escolhe qual delas: `lap` é o circuito acima, `follow` é rota nenhuma — sem
entroncamentos, sem curvas, a máquina simplesmente se mantém em qualquer carreador em que esteja
até ser parada, que é o que uma estrada com uma só saída quer — e `origin_bend` anda 43 m para leste
a partir do cruzamento e entra na estrada de 30°. As duas últimas são aquelas em que os vídeos do
`docs/` do repositório do workspace foram filmados.

**Quanto uma curva gira vem das próprias direções da rota**, e não de um número escrito ao lado.
Toda curva girava até a máquina ter virado 90°, o que é certo para três lados de um quadrado e
nunca foi notado até alguém pedir uma curva de 30°: o trator virou 90°, saiu do carreador e entrou
no talhão a confiantes 3 m/s. O giro agora é o ângulo entre o trecho que se deixa e o trecho que se
pega, então não pode discordar da rota a que pertence.

Para o que servem a odometria e o GNSS é a única pergunta que a estrada não responde: num
entroncamento onde dois carreadores se cruzam, qual deles é o meu. Isso é uma pergunta de mapa, não
de percepção. Então a rota carrega as três curvas e a posição da própria máquina diz quando uma
chegou — e **a posição aqui é o receptor no mastro, suavizado pela odometria**, e não a odometria
sozinha. A odometria sozinha foi tentada: uma volta deste circuito voltou **9 m fora**, o que é
normal para uma máquina Ackermann que patina e mais que suficiente para começar uma curva no lugar
errado na terceira volta. O fix não deriva, então os dois são fundidos do jeito óbvio — a odometria
para a suavidade entre fixes, o fix para onde essa suavidade se ancora, quatro segundos de constante
de tempo. A correção aplicada é reportada no fim de cada volta, e não é pequena: **−22,5 m** em x na
volta medida com o plugin do Gazebo, todo erro de odometria que o receptor tirou; com o
ros2_control e a tração nas quatro foi (−7,8, −1,6) m. Sem isso este circuito não
sobrevive à terceira volta.

**Duas marcas no meio da estrada foram tentadas antes disso, e as duas saíram.** Primeiro uma
linha central branca de 30 cm, acrescentada como referência: o seguidor a acompanhava muito bem
até deixar de acompanhar — duas vezes entrou na estrada de 30° em x = 48 e foi para o talhão,
porque a pista de uma estrada que chegava estava por cima da linha em que ela chegava e enterrava
catorze metros dela (veja a nota sobre a ordem em z no README do `agri_gazebo`, que está
corrigida). Depois a linha saiu dos mundos pelo motivo pelo qual nunca deveria ter entrado:
ninguém pinta uma estrada de fazenda. Depois os sulcos de roda que todo carreador tinha, que pelo
menos são algo que uma estrada de terra tem — só que, como duas faixas de borda dura a 2,1 m uma
da outra, também pareciam tinta, e não batiam com a bitola de máquina nenhuma da propriedade.
Saíram também.

O que sobra é a própria estrada, e ela é o alvo melhor. Nada num carreador aqui é desenhado que uma
estrada patrolada não teria.
