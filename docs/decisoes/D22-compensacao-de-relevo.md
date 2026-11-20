# D22 — Compensação de terreno pela antena

## Contexto

Nos mundos com relevo, a antena fica acima do chão e o roll e o pitch deslocam a sua posição em relação ao ponto do veículo no solo. Sem compensar, o controle e as métricas medem um ponto que muda com a inclinação.

## Decisão

Usa-se o relevo dos mundos. O ponto de referência do controle e das métricas é a projeção da antena no chão, calculada com roll e pitch. A antena continua sendo o ponto medido.

## Consequência

A dissertação deve citar na seção 4.2 a compensação de terreno pela projeção da antena no chão, igual à dos pilotos automáticos agrícolas, e dizer que em terreno plano nada muda.
