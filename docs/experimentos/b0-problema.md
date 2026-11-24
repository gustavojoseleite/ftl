# Caso de teste: o problema da B0

## Objetivo
Mostrar o problema que a proposta resolve. Na B0, cada trator segue o rastro do veículo
imediatamente à frente (Figura 10(a) da monografia), e as posições entram como chegam, sem
vetorização. A hipótese é que:
- cada trator corta as curvas;
- o corte e o desvio crescem do T1 ao T4, porque o erro de um trator vira a referência do
  seguinte (efeito cascata).

## Configuração
| Item | Valor |
|------|-------|
| Seguidor | B0 (`ftl_follower_pure_pursuit`, congelado) |
| Comboio | pickup (líder) + 4 tratores, 10 m entre eles |
| Líder | percorre a rota do mundo pelo `route_driver`, a 10,8 km/h, uma volta |
| Mundos | `citrus_farm_90`, `_60` e `_30` (curvas de 90°, 60° e 30°) |
| Sensores c0 | perfil `nominal`: GNSS RTK fixo a 1 Hz (σ de 2 cm), IMU a 20 Hz, sem degradação |
| Sensores de campo | c0 + IMU `mems` + odometria `field` |
| Sementes | 1, 2 e 3 |

São 3 mundos × 2 condições × 3 sementes = 18 execuções.

**A semente** numera a repetição e inicia o gerador de números aleatórios dos sensores. Nos
sensores de campo, ela define o sorteio do ruído e do viés da IMU e do erro de escala e das
patinagens da odometria; cada veículo soma o seu índice à semente. A mesma semente dá o mesmo
sorteio, e por isso a comparação com a proposta, nas mesmas sementes, é pareada. No c0 não há
degradação a sortear: o ruído do GNSS vem do próprio Gazebo, com sorteio próprio, e a semente só
numera a repetição. O c0 separa as causas: se o erro cresce
na fila mesmo com sensores quase ideais, o problema é a cascata, e não o ruído.

## O que se mede
O caminho de cada veículo é a **pose real** (verdade de solo do simulador), comparada com a
**rota ideal** do mundo:
- nas retas, o erro lateral à rota (`route_lateral_rmse`);
- nas curvas, o corte de curva (`route_corner_cut`), o maior desvio para dentro;
- na volta, o `gamma_route` de cada trator: o erro dele sobre o do veículo da frente. Acima de 1,
  o erro cresce na fila.

## Resultados esperados
- `b0_paths.pdf`: (a) a rota ideal e os caminhos do líder e dos 4 tratores, numa janela de 3 m em
  volta do maior corte do T4; (b) o desvio para dentro da curva ao longo da rota.
- `b0_error.pdf`: o erro nas retas e o corte nas curvas por posição na fila, com o intervalo de
  confiança de 95% das 3 sementes.
- `table_route.tex`: a tabela com os mesmos valores.
- Um vídeo das 4 curvas a partir da pose real gravada: à esquerda, o comboio visto de cima; à
  direita, o desvio de cada veículo, desenhado enquanto ele passa pela curva.

As figuras e o vídeo saem em inglês, como o artigo, no tamanho de uma coluna do IEEE. O T1 ao T4
usam uma rampa de um só tom azul, do claro ao escuro, porque a posição na fila é ordinal.

## Resultados (campanha de 24/11/2026, ftl d32648e)
As 18 execuções completaram a volta. Seis precisaram ser repetidas por falha na partida da
simulação: o controlador de um trator não subiu e ele ficou parado, ou o líder não largou. Todas
passaram na primeira repetição. A tabela dá o corte de curva médio das 4 curvas, em m, com o
intervalo de confiança de 95% das 3 sementes:

| Mundo, sensores | Líder | T1 | T2 | T3 | T4 |
|-----------------|-------|----|----|----|----|
| 90°, c0 | 0,025 | 0,180 ± 0,004 | 0,307 ± 0,008 | 0,420 ± 0,001 | 0,526 ± 0,006 |
| 60°, c0 | 0,021 | 0,152 ± 0,002 | 0,265 ± 0,008 | 0,366 ± 0,010 | 0,458 ± 0,005 |
| 30°, c0 | 0,020 | 0,143 ± 0,007 | 0,249 ± 0,002 | 0,344 ± 0,005 | 0,426 ± 0,006 |
| 90°, campo | 0,025 | 0,259 ± 0,543 | 0,323 ± 0,115 | 0,450 ± 0,099 | 0,572 ± 0,210 |
| 60°, campo | 0,021 | 0,178 ± 0,064 | 0,309 ± 0,145 | 0,527 ± 0,384 | 0,524 ± 0,089 |
| 30°, campo | 0,020 | 0,184 ± 0,145 | 0,315 ± 0,019 | 0,427 ± 0,088 | 0,538 ± 0,036 |

- O corte cresce do T1 ao T4 em todos os mundos: com c0, quase o triplo (0,14–0,18 → 0,43–0,53 m).
  O `gamma_route` do T2 ao T4 fica entre 1,2 e 1,5 com c0: o erro cresce a cada posição.
- Com sensores quase ideais o efeito é o mesmo; a causa é a cascata, e não o ruído.
- Os sensores de campo aumentam o corte e, sobretudo, a variação entre sementes. Com 3 sementes,
  o intervalo fica largo; para o campo, 10 sementes são o indicado.
- Nas retas, o erro fica em 0,06–0,12 m com c0. Na saída de cada curva, os tratores abrem para
  fora, e essa abertura também cresce do T1 ao T4.

## Como rodar
Com os repositórios limpos e enviados, na raiz do workspace (`~/dev/ros2`):

```bash
scripts/experiment.sh b0_problema --dry-run      # grava o repos.lock da campanha
docker/run.sh --no-gui --name b0-problema bash -lc \
  /workspace/src/ftl/docs/experimentos/b0-problema.sh
ros2 run ftl_analysis analyze experiments/b0_problema --out experiments/b0_problema/analysis
ros2 run ftl_analysis plot_paths experiments/b0_problema/p90_c0/1 --out paths.pdf \
  --segment curve_0
```

As figuras e o vídeo do artigo, no contêiner, em `/workspace/experiments/b0_problema`:

```bash
python3 /workspace/src/ftl/docs/experimentos/b0-figuras.py . --run p90_c0/1 --out analysis/artigo
pip install --user --break-system-packages imageio-ffmpeg==0.8.1   # codificador do vídeo
python3 /workspace/src/ftl/docs/experimentos/b0-video.py p90_c0/1 \
  --out analysis/video/b0_p90_c0_1_curves.mp4
```

O vídeo do próprio Gazebo, com a câmera seguindo o T4, sai de uma execução extra da B0 com a
mesma configuração (90°, c0, semente 1), em `experiments/b0_video`. Precisa de tela; a janela do
Gazebo abre durante a gravação e é capturada pelo X11, porque o gz-sim 8 só grava vídeo pelo
botão da GUI:

```bash
docker/run.sh --name b0-video bash -lc /workspace/src/ftl/docs/experimentos/b0-gazebo-video.sh
```

`CURVES` escolhe as curvas (padrão `0,1`), e cada uma vira um `curve_<n>.mp4` em
`experiments/b0_video/video`. O vídeo segue o relógio de parede.

O `b0-video.py` aceita `--curves` (por exemplo, `0` só a primeira), `--speed` (segundos de
simulação por segundo de vídeo; padrão 2) e `--fps`.

O script roda as 18 execuções; `WORLDS`, `CONDS` e `SEEDS` restringem a campanha (por exemplo,
`WORLDS=90 CONDS=c0 SEEDS=1` roda uma só). Os resultados ficam em `experiments/b0_problema/`,
fora do git, com o `repos.lock` dos commits usados.
