# Caso de teste: B0 × E1

## Objetivo
Comparar a B0 com a proposta inicial do artigo (E1), nas mesmas condições e nas mesmas sementes.
O `b0-problema.md` mostra o problema: na B0 cada trator segue o rastro do veículo da frente, e o
corte de curva cresce do T1 ao T4. A hipótese do E1 é que:
- com um trilho único, construído uma só vez a partir do líder, todos os tratores seguem a mesma
  referência e o corte deixa de crescer na fila;
- com o UKF, o erro de estimativa nos sensores de campo cai e o trilho fica limpo.

## Configuração
| Item | B0 | E1 |
|------|----|----|
| Localização | `pose_estimator` | UKF (`ftl_localization`) + relé `gnss_odometry` |
| Seguidor | rastro do veículo da frente | trilho único, montado no T1 |
| Trilho | posições como chegam | amostragem adaptativa + CCMA incremental |

O resto é igual para os dois: pickup (líder) + 4 tratores a 10 m, o líder percorre a rota do
mundo a 10,8 km/h, mundos de 90°, 60° e 30°, sensores c0 (nominais) e de campo (c0 + IMU `mems` +
odometria `field`), sementes 1, 2 e 3. São 2 casos × 3 mundos × 2 condições × 3 sementes = 36
execuções. A semente fixa o sorteio dos sensores, então a B0 e o E1 recebem os mesmos erros e a
comparação é pareada (ver `b0-problema.md`).

## O que se mede
O caminho de cada veículo é a pose real do simulador, comparada com a rota ideal do mundo:
- nas retas, o erro lateral à rota (`route_lateral_rmse`);
- nas curvas, o corte de curva (`route_corner_cut`), o maior desvio para dentro;
- na volta, o erro da estimativa de posição (`estimation_rmse`), a pose estimada contra a real.

## Resultados (campanha de 25 e 26/11/2026)
As 36 execuções completaram a volta. O E1 rodou com o ftl 8dad12e e a B0 com o f9b324d (as duas
voltas refeitas, com o 8dad12e). Entre os dois commits só mudou o código do trilho, que a B0 não
usa. Duas voltas da B0 (60°, c0, sementes 2 e 3) foram refeitas porque rodaram 7–13% mais
devagar que as outras, provavelmente por outro processo disputando a CPU; as métricas delas eram
iguais às da semente 1. Na repetição, as duas ficaram presas na partida por falha de descoberta
do DDS (o líder não recebeu o estado de um trator) e rodaram de novo num domínio ROS isolado. A
razão tempo simulado / tempo de parede ficou em 0,46 em todas as voltas da B0 e entre 0,42 e
0,44 em todas as do E1.

Corte de curva médio das 4 curvas, em m, com o intervalo de confiança de 95% das 3 sementes:

| Mundo, sensores | Caso | Líder | T1 | T2 | T3 | T4 |
|-----------------|------|-------|----|----|----|----|
| 90°, c0 | B0 | 0,02 | 0,18 ± 0,00 | 0,31 ± 0,01 | 0,42 ± 0,01 | 0,53 ± 0,01 |
| 90°, c0 | E1 | 0,02 | 0,19 ± 0,03 | 0,19 ± 0,00 | 0,19 ± 0,03 | 0,18 ± 0,01 |
| 90°, campo | B0 | 0,02 | 0,28 ± 0,56 | 0,34 ± 0,17 | 0,44 ± 0,11 | 0,58 ± 0,24 |
| 90°, campo | E1 | 0,02 | 0,20 ± 0,01 | 0,20 ± 0,03 | 0,20 ± 0,01 | 0,19 ± 0,01 |
| 60°, c0 | B0 | 0,02 | 0,15 ± 0,00 | 0,27 ± 0,00 | 0,37 ± 0,00 | 0,46 ± 0,01 |
| 60°, c0 | E1 | 0,02 | 0,17 ± 0,02 | 0,17 ± 0,01 | 0,17 ± 0,01 | 0,17 ± 0,01 |
| 60°, campo | B0 | 0,02 | 0,21 ± 0,01 | 0,32 ± 0,15 | 0,43 ± 0,09 | 0,52 ± 0,08 |
| 60°, campo | E1 | 0,02 | 0,18 ± 0,02 | 0,17 ± 0,02 | 0,17 ± 0,01 | 0,17 ± 0,01 |
| 30°, c0 | B0 | 0,02 | 0,14 ± 0,01 | 0,25 ± 0,01 | 0,34 ± 0,00 | 0,42 ± 0,01 |
| 30°, c0 | E1 | 0,02 | 0,15 ± 0,01 | 0,15 ± 0,03 | 0,15 ± 0,00 | 0,15 ± 0,01 |
| 30°, campo | B0 | 0,02 | 0,18 ± 0,13 | 0,33 ± 0,04 | 0,43 ± 0,07 | 0,54 ± 0,03 |
| 30°, campo | E1 | 0,02 | 0,15 ± 0,02 | 0,15 ± 0,02 | 0,15 ± 0,02 | 0,15 ± 0,02 |

Média dos 4 tratores nos 3 mundos (9 execuções por linha), em m:

| Métrica | B0, c0 | E1, c0 | B0, campo | E1, campo |
|---------|--------|--------|-----------|-----------|
| Corte de curva | 0,32 | 0,17 | 0,38 | 0,18 |
| Erro nas retas (RMSE) | 0,088 | 0,057 | 0,125 | 0,057 |
| Erro de estimativa (RMSE) | 0,19 | 0,04 | 1,28 | 0,07 |

- **A cascata some.** No E1, o corte do T4 é igual ao do T1 (0,15–0,20 m) em todos os mundos e nas
  duas condições. Na B0 ele quase triplica do T1 ao T4. No T4, o E1 corta de 2,7 a 3,6 vezes
  menos.
- **O T1 corta o mesmo nos dois casos.** O corte que sobra no E1 é o do próprio seguidor
  (*pure pursuit* com *lookahead*), que o T1 já tem na B0. O trilho único tira o acúmulo, não o
  corte de cada trator. Essa é a parte que fica para as próximas etapas.
- **Com c0 o efeito já aparece inteiro.** Com sensores quase ideais, a estimativa da B0 já é boa
  (0,19 m), e mesmo assim o corte do T4 cai de 0,42–0,53 para 0,15–0,19 m. O ganho no corte vem do
  trilho, e não da localização.
- **Nos sensores de campo o UKF segura a estimativa.** O erro de estimativa da B0 vai a 1,28 m e
  o intervalo entre sementes fica largo. No E1 ele fica em 0,07 m, e o corte e o erro nas retas
  são os mesmos do c0.

## Correções do E1 durante a campanha
O E1 rodou três vezes. As duas primeiras mostraram falhas do próprio E1, corrigidas antes da
campanha final (a B0 não mudou):
1. **Rampa sem acúmulo** (ftl 13c4cf2). Um trator cujo controlador subiu tarde acumulava comando
   parado e depois arrancava a 11 m/s. O comando agora fica limitado à velocidade medida mais
   `max_lead` (1 m/s).
2. **Trilho sem ré** (ftl 749b175). Na arrancada, o UKF às vezes faz a estimativa do líder recuar
   alguns metros. Uma vez (30°, campo, semente 1) o recuo foi de 4,8 m: o trilho dobrou sobre si
   mesmo e o T1 parou na dobra. Posições mais de `max_back` (0,5 m) atrás da última aceita agora
   ficam fora do trilho.

## Resultados esperados
- `cmp_paths.pdf`: o desvio para dentro da curva ao longo da rota, na 1ª curva de 90° em campo,
  semente 1: (a) B0, (b) E1.
- `cmp_lap_p90.pdf`, `cmp_lap_p60.pdf` e `cmp_lap_p30.pdf`: a volta inteira de cada mundo,
  desenrolada ao longo da rota, semente 1. Cada linha é um caso numa condição (B0 e E1, c0 e
  campo) e mostra o desvio lateral de todos os veículos, positivo para dentro das curvas. Na
  escala do mapa (centenas de metros) um desvio de 0,5 m não aparece; desenrolada, a volta mostra
  as curvas e as retas na escala do erro.
- `cmp_map_p90.pdf`, `cmp_map_p60.pdf` e `cmp_map_p30.pdf`: o caminho de todos os veículos no
  mapa, semente 1, B0 e E1 lado a lado, sensores c0 e de campo. O desvio à rota está ampliado
  (35× a 40×) e as linhas tracejadas marcam ±0,5 m. A ampliação fica abaixo do raio das curvas
  sobre o corte máximo, senão o pico passa do centro do arco e aparece virado para fora; mesmo
  assim, perto das curvas ela encurta o desvio para dentro, e o valor do corte se lê nas
  `cmp_lap`.
- `cmp_error.pdf`: o erro nas retas e o corte nas curvas por posição na fila, B0 e E1, com o
  intervalo de confiança de 95%.
- `cmp_corner_cut.tex` e `cmp_summary.tex`: as tabelas acima.
- Vídeos do Gazebo com os rastros dos tratores no chão, B0 e E1 na mesma volta (90°, c0,
  semente 1).

## Como rodar
Na raiz do workspace (`~/dev/ros2`), com nada mais rodando na máquina (uma volta já ocupa os
6 núcleos, e outra carga muda o ritmo da simulação):

```bash
docker/run.sh --no-gui --name comparativa bash -lc \
  /workspace/src/ftl/docs/experimentos/comparativa.sh
```

`CASE_LIST`, `WORLDS`, `CONDS` e `SEEDS` restringem a campanha (por exemplo, `CASE_LIST=e1
WORLDS=60 CONDS=c0 SEEDS=2,3`). A análise e as figuras, no contêiner:

```bash
ros2 run ftl_analysis analyze /workspace/experiments/comparativa \
  --out /workspace/experiments/comparativa/analysis --reasons done
cd /workspace/experiments && python3 /workspace/src/ftl/docs/experimentos/comparativa-figuras.py \
  comparativa --out comparativa/figuras
```

As figuras da volta inteira leem os bags de 12 voltas e levam uns 15 minutos; `--seed` escolhe
a semente delas.

O vídeo do E1 com rastros sai do mesmo script da B0:

```bash
NAME=e1_video EXTRA='ftl.localization=ukf ftl.follower=trail' docker/run.sh --name e1-video \
  bash -lc /workspace/src/ftl/docs/experimentos/b0-gazebo-video.sh
```
