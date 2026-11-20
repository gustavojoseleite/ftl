# D4 — Distância ao precedente pelo arco no trilho

## Contexto

A Eq. 4.2 da monografia mede a distância ao precedente em linha reta. Na curva essa medida é a corda, e num retorno dois veículos podem estar perto no espaço e longe pelo trilho, então ela representa mal a folga.

## Decisão

No controle, a distância é o comprimento de arco entre as projeções do seguidor e do precedente no trilho. A distância euclidiana fica só como verificação de emergência. A folga mínima vem da frenagem, e a folga alvo é `d_min + 2 m` (seção 5.6 do plano).

## Consequência

A dissertação precisa ajustar a Eq. 4.2 para o comprimento de arco no trilho, na seção 4.4.1, e dizer que a distância euclidiana serve apenas à verificação de emergência.
