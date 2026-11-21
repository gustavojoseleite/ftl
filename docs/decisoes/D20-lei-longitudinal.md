# D20 — Lei longitudinal sobre d_alvo

## Contexto

A Eq. 4.4 da monografia é um PID sobre o erro `e_d` da Eq. 4.3 (`e_d = d_atual − d_alvo`, com `d_alvo` fixo) e não usa a velocidade do precedente. Com a folga fixa, ela não acompanha um perfil de velocidade em que a folga cresce com a velocidade.

## Decisão

Fica a Eq. 4.4 como está, um PID, mas o erro passa a ser `e_d = g − d_alvo(v)`, com `g` pelo arco da D4 e uma folga alvo `d_alvo(v)` que cresce com a velocidade. A lei não usa a velocidade do precedente. Mantêm-se a banda morta, a saturação sem ré, o anti-windup e as rampas.

## Consequência

A dissertação deve trocar, na seção 4.4.1, o erro da Eq. 4.3 por `e_d = g − d_alvo(v)`, deixando a Eq. 4.4 como está. O texto deve dizer que a lei não usa a velocidade do precedente e que, sem ela, só a integral sustenta a velocidade de cruzeiro.
