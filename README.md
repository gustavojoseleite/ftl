# ftl

**Follow the leader** (siga o líder): uma fila de máquinas agrícolas em que a primeira é dirigida e
as outras a seguem. Este repositório é o código do próprio projeto, a parte que nenhum outro
projeto usa. A plataforma em que ele roda (veículos, mundos, condução) vem de `robot_description`,
`agri_gazebo` e `vehicle_control`, listados com este repositório no `projects/ftl/ftl.repos` do
workspace.

```
ftl_follower_pure_pursuit/  a B0: pose_estimator (todo veículo) e follower (cada seguidor)
road_follow/      dirige um veículo pelos carreadores pela câmera — o motorista do líder do FTL
ftl_analysis/     a análise das campanhas: métricas por trecho e por trator, em tabelas e figura
ftl_bringup/      os nós do FTL e seus parâmetros, num launch só. Sem simulador.
├── launch/ftl.launch.py
└── config/ftl.yaml           um arquivo por experimento é o uso pretendido
ftl_experiments/  o executor de experimentos: roda um cenário com várias sementes e grava os bags
ftl_gz/           cenários de teste do FTL no Gazebo: mundo, líder, seguidores, onde começam
├── launch/sim.launch.py      só o cenário (mundo + comboio)
└── launch/ftl_sim.launch.py  o cenário e os nós do FTL, num comando
ftl_localization/ o UKF por veículo (robot_localization): publica o ftl/state, com covariância
ftl_msgs/         as mensagens que ligam os módulos da solução
ftl_route_driver/ o motorista roteirizado do líder: percorre a rota do mundo sozinho
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

## ftl_route_driver: o líder percorre a rota sozinho

Com `route.enabled=true`, o `experiment.launch.py` sobe o `route_driver` no namespace do líder, e a
fila nasce no início da rota, na altura do chão ali (`convoy_arguments`
envia `x`, `y`, `z` e `yaw`).

- **Rotas** (`routes.py`, o nome é o do mundo sem `.sdf`): `citrus_farm` (a volta `lap` do
  `road_follow`) e os perímetros `citrus_farm_30`, `citrus_farm_60` e `citrus_farm_90`, pela mesma
  fórmula do gerador dos mundos. `route(nome)` devolve uma `Route` com `length`, `start`,
  `start_z`, `turns`, `at`, `curvature` e `project`; `terrain_height(x)` é o relevo dos mundos.
- **Perfil** (`profile.py`): `speed_profile` dá `v = min(cruzeiro, sqrt(a_lat / |curvatura|))`,
  antecipado para trás com desaceleração de 1 m/s²; `speed_at` o interpola.
- **Protocolo** (`protocol.py`): `spacing(gap)` dá a distância entre `base_link` para `gap` metros
  entre para-choques; `finished(route, s)` diz se `s` passou do fim da última curva; `Progress`
  dá o progresso contínuo, mesmo na volta fechada.
- **Nó `route_driver`**: pure pursuit com visada `max(4, v)` m sobre a pose real de
  `/ground_truth/<leader>/pose`, a 20 Hz. Para quando o último trator (`last`) passa o fim da
  última curva. Parâmetros: `route`, `cruise_kmh`, `a_lat`, `leader`, `last` e `wait_for`.
- **Espera da partida** (`wait_for`, lista de tópicos `nav_msgs/Odometry`, vazia por padrão): o nó
  assina cada tópico, e enquanto algum não publicou a velocidade alvo é zero, com o `cmd_vel`
  publicado normalmente. Quando o último chega, o nó registra isso no log. O
  `experiment.launch.py` passa o `ftl/state` do líder e de cada trator (`driver_parameters`, em
  `scenario.py`). O motivo (R6): se o líder arranca antes dos estimadores, eles começam a
  estimar com o líder já em movimento e o erro de estimação chega a 1-5 m.
- **Tópicos**, no namespace do líder: `cmd_vel` (`geometry_msgs/Twist`), `route/error`
  (`std_msgs/Float64`, erro lateral em m, positivo à esquerda), `route/progress` (`Float64`,
  metros desde o início) e `route/done` (`std_msgs/Bool`, QoS `TRANSIENT_LOCAL`, profundidade 1),
  publicado uma vez, com `True`, quando o último trator passa o fim da última curva.

## Linha de base B0

A B0 é o pacote `ftl_follower_pure_pursuit`: o seguidor que já existia no workspace antes desta
pesquisa, congelado (D2), com o estimador complementar de pose e o pure pursuit sobre o rastro do
precedente. Não é a proposta da monografia, e sim o caso da Figura 10(a) dela, em que cada trator
segue o veículo imediatamente à frente. O algoritmo não
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

## ftl_localization: o UKF por veículo

Cada veículo pode estimar a própria pose com o UKF do `robot_localization` em vez do
`pose_estimator` da B0. O resultado é o mesmo `ftl/state` de hoje (`nav_msgs/Odometry`, referencial
`ftl_map`, filho `base_footprint`), agora com a covariância da pose preenchida.

Este UKF estima só a pose do próprio veículo, com GNSS, IMU e odometria. Ele ainda não usa a
câmera nem a tag; a fusão da proposta com a tag do precedente é a F5.

- **O que o UKF funde e de onde**: a velocidade à frente e a de guinada da odometria (`odom`), o
  rumo e a velocidade de guinada da IMU (`imu/data`) e a posição do GNSS (`gnss/fix`), que o nó
  `gnss_odometry` converte na posição da base em `odometry/gps`. Roda em 2D, a 20 Hz, e não
  publica TF.
- **A conversão do GNSS**: o `gnss_odometry` usa a mesma conversão do `pose_estimator` (metros
  leste/norte a partir do `datum`, num plano tangente, como o Gazebo gera o fix) e desconta a
  antena pelo TF `base_footprint -> gnss_link`, girado pelo rumo da IMU. Ele substitui o
  `navsat_transform` do `robot_localization`: pela UTM, o fix ficava 0,18 m deslocado a 205 m do
  `datum`, e o plano tangente dele não punha a origem no `datum`. Quando o fix não traz
  covariância (o do Gazebo não traz), usa a do RTK fixo, σ de 2 cm; com covariância zero o filtro
  ia a NaN. Fixes sem solução ou com latitude e longitude nulas são descartados.
- **A partida**: o UKF nasce na origem do `ftl_map`, longe do veículo. Enquanto o `ftl/state`
  estiver a mais de 3 m do GNSS (`reset_distance`), o `gnss_odometry` manda cada fix também para
  o `set_pose`, com o rumo da IMU, e o filtro recomeça ali.
- **O `datum` comum**: o `gnss_odometry` usa o mesmo `datum` do `pose_estimator`
  (`ftl_bringup/config/ftl.yaml`), então o comboio inteiro fica num referencial só. Os parâmetros
  estão em `ftl_localization/config/localization.yaml`.
- **A escolha**: o argumento `localization:=ukf` do `ftl.launch.py` (padrão `pose_estimator`) ou a
  chave `ftl.localization=ukf` do cenário, que o `experiment.launch.py` repassa ao launch.
- **Sem GNSS**: a covariância da posição só cresce, pela covariância de processo; quando o GNSS
  volta, ela cai.

```bash
ros2 launch ftl_bringup ftl.launch.py localization:=ukf followers:=2
ros2 run ftl_experiments run --campaign loc --set "ftl.localization=ukf" --seeds 1
```

### Comparação com a B0

A B0 e o UKF rodam no mesmo conjunto de sensores de campo: perfil `nominal` (RTK fixo), IMU
`mems` e odometria `field`, no caso `quick` (`citrus_farm_90`, 10,8 km/h, 4 tratores, 120 s
simulados), semente 1. O RMSE de estimação de cada trator nas retas, medido no teste
`ftl_experiments/test/test_sim_localization.py`:

| Trator | B0 (`pose_estimator`) | UKF |
|---|---|---|
| `tractor_1` | 1,26 m | 0,06 m |
| `tractor_2` | 0,94 m | 0,08 m |
| `tractor_3` | 0,90 m | 0,06 m |
| `tractor_4` | 0,91 m | 0,10 m |
| média | 1,00 m | 0,08 m |

Medido em 24/11/2026. A meta do plano é 5 cm; o UKF fica perto dela e dentro do limite do teste
(0,10 m). O erro que sobra é um atraso ao longo da trajetória de 2 a 5 cm, do GNSS a 1 Hz. O
`pose_estimator` da B0 erra cerca de um metro com a IMU `mems` e a odometria `field`, e ainda
carrega 0,2–0,3 m do braço da antena girado só pelo rumo no relevo inclinado; a B0 fica congelada
como está (D2).

## Cenário de experimento

Um experimento sobe com um comando e um cenário. A configuração resolvida é gravada e publicada.

```bash
ros2 launch ftl_bringup experiment.launch.py scenario:=dual_160 \
    set:="convoy.followers=2 degradation.gnss=rtk_float degradation.seed=7" output:=/tmp/exp1
```

O `experiment.launch.py` resolve a configuração (padrões do esquema, depois o preset, depois as
sobrescritas de `set:=`, no formato `chave=valor` separado por espaços, com o valor lido como YAML)
e recusa o que for inválido, nomeando a chave. Depois sobe o comboio do `agri_gz_bringup`, os nós do
FTL e o `apply_scenario`.

- **Presets** (`ftl_bringup/config/scenarios`): `single_120` (uma câmera de 120 graus, o padrão) e
  `dual_160` (duas câmeras de 90 graus).
- **`output`**: a pasta que recebe o `config.yaml`. Vazio vira
  `/workspace/experiments/manual/<AAAAMMDD-HHMMSS>`.
- **`/experiment/config`**: o texto do `config.yaml` (`std_msgs/String`, QoS `TRANSIENT_LOCAL`,
  profundidade 1), para qualquer nó ou gravação saber com que cenário o experimento rodou.
- O `apply_scenario` ajusta, em cada veículo, os parâmetros `profile` e `seed` dos nós `gnss`,
  `imu` e `odometry` (a semente é `degradation.seed` mais o índice do veículo, 0 para o líder), o
  `profile` do `distortion_camera_front` dos seguidores e o atuador do `cmd_vel_limiter`. Tenta até
  4 minutos, até os nós existirem, registra no log o que aplicou e o que faltou e segue vivo.

| Chave | Padrão | Unidade e faixa | Efeito |
|---|---|---|---|
| `world` | `citrus_farm.sdf` | um dos cinco mundos do `agri_gz_worlds` | Mundo do Gazebo. |
| `gui` | `false` | booleano | Abre a janela do Gazebo. |
| `convoy.leader` | `pickup` | `pickup` ou `autonomous_tractor` | Tipo do veículo da frente. |
| `convoy.follower` | `autonomous_tractor` | texto | Tipo de cada seguidor. |
| `convoy.followers` | `4` | veículos, 1 a 4 | Quantos seguidores. |
| `convoy.spacing` | `7.95` | m, 4 a 30 | Distância entre `base_link`; 7,95 dá 4 m de folga. |
| `convoy.x` | `0.0` | m, -500 a 500 | Posição X da fila; com a rota ligada, vale o início dela. |
| `convoy.y` | `0.0` | m, -500 a 500 | Posição Y da fila; com a rota ligada, vale o início dela. |
| `convoy.yaw` | `0.0` | rad, -pi a pi | Rumo da fila inteira. |
| `sensors.profile` | `nominal` | `nominal`, `default` ou `ideal` | Perfil de GNSS e IMU. |
| `sensors.camera_mount` | `single` | `single` ou `dual` | Uma câmera de 120 graus ou duas de 90. |
| `sensors.tag_family` | `36h11` | texto | Família das tags AprilTag. |
| `sensors.tag_size` | `0.5` | m, 0,1 a 2 | Lado da tag. |
| `degradation.gnss` | vazio | vazio ou perfil de gnss | Perfil de degradação do GNSS. |
| `degradation.imu` | vazio | vazio ou perfil de imu | Perfil de degradação da IMU. |
| `degradation.odometry` | vazio | vazio ou perfil de odometry | Perfil de degradação da odom. |
| `degradation.camera` | vazio | vazio ou perfil de camera | Perfil de degradação da câmera. |
| `degradation.seed` | `0` | inteiro | Semente do ruído; cada veículo soma o seu índice. Vai também para o `--seed` do Gazebo, que fixa o ruído dos sensores dele (o GNSS); `0` deixa o Gazebo sortear. |
| `actuator.enabled` | `false` | booleano | Liga o atraso do atuador no `cmd_vel_limiter`. |
| `actuator.steering_rate` | `0.5` | rad/s, 0,05 a 2,0 | Velocidade máxima de giro da direção. |
| `actuator.time_constant` | `0.5` | s, 0,05 a 3,0 | Constante de tempo da resposta do atuador. |
| `route.enabled` | `false` | booleano | O líder percorre a rota do mundo; sem rota, recusa. |
| `route.cruise_kmh` | `10.8` | km/h, 1 a 40 | Velocidade de cruzeiro do líder nas retas. |
| `route.a_lat` | `1.0` | m/s², 0,2 a 3 | Aceleração lateral máxima; limita a velocidade. |
| `ftl.localization` | `pose_estimator` | `pose_estimator` ou `ukf` | Localização de cada veículo: a B0 ou o UKF. |
| `ftl.params_file` | vazio | caminho de um arquivo | Parâmetros dos nós do FTL; vazio: o padrão. |

Os perfis de degradação vêm de `agri_sensor_degradation`. Um booleano não vale onde se espera um
número; um inteiro vale onde se espera um real.

## ftl_experiments: o executor de experimentos

O comando `run` roda um cenário uma vez por semente e grava, por execução, o bag MCAP, a
configuração e o `run.yaml`.

```bash
ros2 run ftl_experiments run --campaign c1 --scenario single_120 \
    --set "world=citrus_farm_90.sdf convoy.followers=2 route.enabled=true" --seeds 1,2,3
```

Argumentos: `--campaign` (obrigatório), `--scenario` (`single_120`), `--set` (as sobrescritas, como
no `experiment.launch.py`), `--seeds` (`1,2,3`), `--root` (`/workspace/experiments`), `--label` (o
nome da variante; o padrão é o cenário), `--duration` (sobrepõe o tempo limite, em s simulados)
e `--retries` (as repetições de uma execução `failed`; padrão 1). Cada execução ganha
`degradation.seed` igual à semente e `gui=false`.

- **Pasta**: `<root>/<campaign>/<label>/<semente>/`, com `config.yaml` (a configuração resolvida),
  `bag/` (o MCAP), `launch.log` e `run.yaml`.
- **`run.yaml`**: `reason` (`done`, `timeout` ou `failed`), `duration` (s), `seed`, `commits` (o
  `git rev-parse HEAD` de cada repositório de `FTL_WORKSPACE_SRC`, padrão `/workspace/src`) e
  `attempts` (as tentativas feitas). O motivo do `attempts` (R7): uma partida do Gazebo falha às
  vezes sem culpa do experimento, e a repetição evita um buraco na campanha; o campo mostra
  quais execuções precisaram dela.
- **Fim da execução**: o `route/done` do líder (`done`), o tempo limite (`timeout`: 2 vezes o
  tempo da volta à velocidade de cruzeiro mais 120 s; sem rota, 600 s) ou a morte do launch
  (`failed`). O tempo limite é medido no tempo simulado, pelo `/clock` (`Deadline`), e conta a
  partir do primeiro `/clock`; o `--duration` também é em segundos simulados. Duas esperas em
  tempo de parede dão `failed`: o `/clock` que não começa em 300 s (partida) e o que não avança
  por 60 s (travamento). O bag é encerrado antes do launch, para o arquivo fechar. O comando
  imprime uma linha por execução, com a pasta e o motivo.
- **Tópicos gravados**, para cada veículo `v` (o líder e os `tractor_N`):
  - `/ground_truth/<v>/pose`: a pose real, que serve ao erro de estimação (seção 8.4 do plano);
  - `/<v>/ftl/state`: o caminho estimado, usado nas métricas;
  - `/<v>/cmd_vel`: o comando enviado ao veículo;
  - `/<v>/degradation/state`: a degradação ativa dos sensores;
  - `/<tractor>/ftl/gap` e `/<tractor>/ftl/cross_track`: a distância e o erro lateral de cada
    seguidor;
  - `/<líder>/route/error`, `/<líder>/route/progress` e `/<líder>/route/done`, com a rota ligada:
    o erro lateral, o progresso e o fim do protocolo;
  - `/experiment/config`: a configuração do experimento;
  - `/clock`: o tempo da simulação.
- **Retomada**: ao rodar de novo o mesmo comando, só as sementes sem `run.yaml` com `done` rodam;
  as que terminaram em `timeout` ou `failed` repetem, e o bag anterior é apagado.

### O conjunto básico: o comando `suite`

```bash
ros2 run ftl_experiments suite --campaign basico
ros2 run ftl_experiments suite --campaign teste --quick
```

O `suite` roda o `run` com `single_120` para cada caso, com as sementes 1 a `--reps` (3), e no
fim roda o `analyze` em `<raiz>/<campanha>`, com a saída em `<raiz>/<campanha>/analysis`. Os
argumentos são `--campaign`, `--reps` (3), `--quick`, `--root` (`/workspace/experiments`) e
`--retries` (1). Sai com o código da análise.

- **Os 12 casos** (`basic()`): o rótulo é `p<ângulo>_<velocidade>kmh`, com o ponto da velocidade
  trocado por `_`. Os ângulos são 30, 60 e 90, e as velocidades, 5, 10,8, 20 e 40 km/h; por
  exemplo, `p30_5kmh`, `p60_10_8kmh` e `p90_40kmh`. Todos têm 4 tratores, rota ligada e uma
  câmera (`camera_mount=single`).
- **Modo rápido** (`--quick`): só o caso `quick` (`quick()`: ângulo 90, 10,8 km/h), com a semente
  1 e `--duration` de `QUICK_DURATION` (120 s simulados). Termina por `timeout`, então a análise
  usa `--reasons done,timeout`.

## ftl_analysis: as métricas de uma campanha

O comando `analyze` transforma a pasta de uma campanha nas tabelas e na figura das métricas, com a
média e o intervalo de confiança de 95% entre as execuções.

```bash
ros2 run ftl_analysis analyze /workspace/experiments/c1 --out /workspace/experiments/c1/analysis
```

A campanha tem as execuções em `<campanha>/<rótulo>/<semente>/`, como o `ftl_experiments` as grava;
só entram as de `run.yaml` com `reason: done`, e as outras são avisadas e puladas. O `--reasons`
(lista separada por vírgula, padrão `done`) muda os motivos aceitos; o modo rápido do `suite` usa
`--reasons done,timeout`. A saída padrão é
`<campanha>/analysis`. O comando `bag_info <pasta da execução>` lista os tópicos de um bag.

O comando `plot_paths <execução> --out <pdf> [--segment <trecho>]` desenha a rota ideal em preto e
o caminho real de cada veículo, com os eixos na mesma escala; com `--segment`, um segundo painel
mostra o trecho pedido (por exemplo, `curve_0`) com uma folga de 5 m. A rota e os caminhos vêm de
`paths(<execução>)`.

- **Saídas**:
  - `metrics_runs.csv`: uma linha por execução, trecho, trator e métrica (`label`, `run`, `seed`,
    `segment`, `kind`, `vehicle`, `metric`, `value`);
  - `metrics_summary.csv`: o resumo por `label`, `segment`, `vehicle` e `metric`, com `n`, `mean`
    e `ci95` (a meia-largura do intervalo, pela distribuição t; vazio com uma só execução);
  - `table_lateral.tex`: o `lateral_rmse` por rótulo, trecho e trator, como média ± intervalo;
  - `lateral_rmse.pdf`: as barras do `lateral_rmse`, com o intervalo de confiança;
  - `table_route.tex` e `route_error.pdf`: o erro à rota por rótulo e veículo, na ordem da fila,
    com o intervalo de confiança; nas retas, a média do `route_lateral_rmse` e, nas curvas, a do
    `route_corner_cut`. Servem para mostrar o problema da B0: o caminho real e a rota ideal
    (plano, seção 8.7).
- **Caminho de cada veículo**: o estimado, o `ftl/state`, no ponto de referência, que é a antena
  projetada no chão (`ANTENNA`: 1,075 m à frente da base no trator e 1,545 m na pickup). A pose
  real, `/ground_truth/<v>/pose`, serve ao `estimation_rmse` e às métricas `route_`; ela nunca
  entra nas métricas do caminho do líder, como na seção 8.4 do plano.
- **Trechos**: cada reta e cada curva da rota do mundo, chamados `straight_<n>` e `curve_<n>`
  (`segments`); a volta inteira é o trecho `lap`. Cada amostra vai ao trecho pela projeção do ponto
  de referência na rota.
- **Partida**: as amostras até todos os tratores passarem de 90% da velocidade de cruzeiro
  (`route.cruise_kmh`) são descartadas.
- **Métricas por trecho e por trator**:
  - `lateral_rmse`, `lateral_max` e `lateral_p95`: a raiz do erro quadrático médio, o máximo e o
    percentil 95 do valor absoluto do erro lateral, a distância com sinal, em m, do ponto de
    referência ao caminho percorrido pelo líder (positiva à esquerda); as amostras que caem antes
    do começo ou depois do fim do caminho do líder ficam de fora;
  - `corner_cut` (só nas curvas): o maior desvio para dentro da curva, em m, ou 0;
  - `longitudinal_rmse`: o RMSE do espaçamento ao longo do caminho do líder, entre o precedente e
    o trator, de base a base, menos o alvo (`gap` do `follower` no `ftl.yaml`, ou no
    `ftl.params_file` do cenário), em m;
  - `steer_rate_rms`: o RMS da derivada da velocidade angular comandada no `cmd_vel`, em rad/s²;
  - `accel_rms`: o RMS da derivada da velocidade linear comandada no `cmd_vel`, em m/s²;
  - `estimation_rmse`: a distância, em m, entre o ponto de referência estimado e o da pose real,
    interpolada no tempo.
- **Erro à rota, pela pose real** (para o líder e para cada trator): a pose real no ponto de
  referência é projetada na rota ideal, com os mesmos trechos e o mesmo descarte da partida:
  - `route_lateral_mean`, `route_lateral_rmse` e `route_lateral_max`: a média com sinal (positiva
    à esquerda), o RMS e o máximo do valor absoluto do erro à rota, em m;
  - `route_corner_cut` (só nas curvas): o maior desvio para dentro da curva, em m, ou 0;
  - `gamma_route` (trecho `lap`, do primeiro trator em diante): a razão entre o RMS do erro à rota
    do trator e o do precedente, na ordem líder, T1, T2…
- **Volta inteira** (trecho `lap`, do segundo trator em diante): `gamma_lateral` e
  `gamma_longitudinal`, a razão entre o RMS do erro do trator e o do precedente
  (`string_stability`); o comboio não amplifica o erro se for no máximo 1.
- **Tempo**: o carimbo do cabeçalho da mensagem, e o de gravação sem cabeçalho; as séries do
  `cmd_vel` usam o de gravação. Uma métrica sem o tópico que a alimenta fica fora da tabela.
- **Comparação com a B0**: as soluções seguintes rodam com os mesmos cenários e as mesmas sementes
  da B0, e a comparação é pareada pela semente, trecho a trecho e trator a trator (seção 8.4 do
  plano).
