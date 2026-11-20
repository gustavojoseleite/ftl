# D7 — UKF próprio em NumPy

## Contexto

A monografia descreve a fusão de GNSS e tag por um UKF, mas não diz como ele é implementado nem validado. A pose própria do seguidor já vem do GNSS/INS e entra na medida da tag com a sua incerteza.

## Decisão

O UKF é próprio, em NumPy, no referencial global, com a incerteza da pose própria somada ao ruído da tag. Os testes usam o NEES para checar a consistência. O pacote `robot_localization` fica só para a pose própria.

## Consequência

A dissertação deve descrever na seção 4.2 o UKF no referencial global, a composição da incerteza da pose própria com a da tag e o NEES como critério de validação, além do uso do `robot_localization` apenas para a pose própria.
