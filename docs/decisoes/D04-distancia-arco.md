# D4 — Distância ao precedente: arco e distância euclidiana, por parâmetro

## Contexto

A Eq. 4.2 da monografia mede a distância ao precedente em linha reta. Na curva essa medida é a corda, e num retorno dois veículos podem estar perto no espaço e longe pelo trilho, então ela representa mal a folga. Em 21/11 a decisão foi revista: antes de abandonar a medida da monografia, convém comparar as duas.

## Decisão

O controle longitudinal implementa as duas distâncias: o comprimento de arco entre as projeções do seguidor e do precedente no trilho, e a distância euclidiana. Um parâmetro escolhe qual delas o controle usa, e as duas são comparadas nos experimentos (seção 8.6, F7.1). A folga mínima vem da frenagem, e a folga alvo é `d_min + 2 m` (seção 5.6 do plano).

## Consequência

A dissertação descreve a Eq. 4.2 nas duas formas, na seção 4.4.1, e apresenta a comparação entre o comprimento de arco e a distância euclidiana na seção 8.6.
