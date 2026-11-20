# D5 — Erro lateral pela linha de visada

## Contexto

A Eq. 4.5 da monografia admite duas leituras para o rumo de referência. A tangente do trilho no waypoint só corrige o rumo e nunca traz de volta um trator deslocado e paralelo ao trilho.

## Decisão

O PID lateral age sobre o erro de rumo até o ponto do trilho a `L_a = max(3 m; 1 s × v)`, ou seja, a linha de visada da antena até esse ponto, e o esterçamento sai da relação com ω.

## Consequência

A dissertação precisa ajustar a Eq. 4.5 na seção 4.4.2, definindo o rumo de referência pela linha de visada com a distância de antecipação `L_a`, no lugar da tangente do trilho.
