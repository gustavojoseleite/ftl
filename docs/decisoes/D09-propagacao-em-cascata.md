# D9 — Propagação do trilho em cascata

## Contexto

A monografia faz o trilho virtual do líder valer para todos, mas não diz como ele chega a cada trator pela comunicação V2V nem o que acontece quando um elo da fila cai.

## Decisão

O trilho se propaga em cascata, T1 → T2 → T3 → T4. Só o T1 suaviza o trilho recebido do líder, e os demais o repassam. Vale a regra do elo caído para quando a comunicação com o precedente se perde.

## Consequência

A dissertação deve descrever na seção 4.3.1 a propagação em cascata do trilho, o papel do T1 como único que suaviza e a regra do elo caído.
