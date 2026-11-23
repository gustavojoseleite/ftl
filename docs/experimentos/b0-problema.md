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

São 3 mundos × 2 condições × 3 sementes = 18 execuções. O c0 separa as causas: se o erro cresce
na fila mesmo com sensores quase ideais, o problema é a cascata, e não o ruído.

## O que se mede
O caminho de cada veículo é a **pose real** (verdade de solo do simulador), comparada com a
**rota ideal** do mundo:
- nas retas, o erro lateral à rota (`route_lateral_rmse`);
- nas curvas, o corte de curva (`route_corner_cut`), o maior desvio para dentro;
- na volta, o `gamma_route` de cada trator: o erro dele sobre o do veículo da frente. Acima de 1,
  o erro cresce na fila.

## Resultados esperados
- Uma figura com a rota ideal e os caminhos do líder e dos 4 tratores, com zoom numa curva.
- Uma figura com o erro nas retas e o corte nas curvas, do T1 ao T4.
- Uma tabela com os mesmos valores e o intervalo de confiança das 3 sementes.

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

O script roda as 18 execuções; `WORLDS`, `CONDS` e `SEEDS` restringem a campanha (por exemplo,
`WORLDS=90 CONDS=c0 SEEDS=1` roda uma só). Os resultados ficam em `experiments/b0_problema/`,
fora do git, com o `repos.lock` dos commits usados.
