# Como contribuir com o ftl

Os padrões do repositório, um por seção. O pacote `ftl_template/` aplica todos e é o ponto de
partida de qualquer pacote novo.

## Idioma

A decisão D1 divide o idioma. Em inglês ficam os identificadores: variáveis, funções, classes,
tópicos e parâmetros, e isso vale também nos testes. Em português ficam as docstrings, os
comentários, as mensagens de log e de erro, os READMEs, o `package.xml` e as mensagens de commit.
O código que já existe é traduzido na F0.6.

## Docstrings

Todo módulo abre com uma docstring que diz o que ele faz, a seção da dissertação que ele
implementa e como roda (pelo nó, pelo launch ou só nos testes). Toda função pública leva uma
docstring no estilo Google, com `Args:`, `Returns:` e `Raises:`, e com a unidade de cada grandeza.

```python
"""
Limitador de taxa: impede que um sinal mude mais rápido que um limite.

Este módulo é Python puro e roda nos testes; o nó ``limiter`` em ``node.py``
o envolve para o ROS.
"""


def limit_rate(previous, target, max_rate, dt):
    """
    Aproxima ``previous`` de ``target`` em no máximo ``max_rate * dt``.

    Args:
        previous: Última saída, na unidade do sinal (u).
        target: Valor desejado, na unidade do sinal (u).
        max_rate: Maior variação permitida por segundo, em u/s; zero ou mais.
        dt: Tempo desde a última saída, em segundos; positivo.

    Returns:
        A nova saída, em u.

    Raises:
        ValueError: Se ``dt`` não for positivo ou ``max_rate`` for negativo.
    """
    if dt <= 0.0:
        raise ValueError(f'dt deve ser positivo, recebido {dt}')
    if max_rate < 0.0:
        raise ValueError(f'max_rate não pode ser negativo, recebido {max_rate}')
    step = max_rate * dt
    return previous + max(-step, min(step, target - previous))
```

## Equações

A linha que implementa uma equação da dissertação leva, no fim, o comentário `# Eq. 4.4`, com o
número da equação na dissertação. Assim o código e o texto se acham um a partir do outro. Uma
equação com várias linhas leva o comentário na linha em que o resultado sai.

## Parâmetros

Todo parâmetro é declarado no nó com um `ParameterDescriptor`, cuja `description` diz a unidade, a
faixa e o efeito, e a faixa vai também em `floating_point_range` ou `integer_range`. O valor
padrão de cada experimento mora em `ftl_bringup/config/ftl.yaml`, com um comentário acima de cada
parâmetro que traz as três coisas, nesta forma:

```yaml
/**/limiter:
  ros__parameters:
    # Unidade: u/s (unidade do sinal por segundo).
    # Faixa: de 0 a 100; zero mantém a saída parada.
    # Efeito: quanto menor, mais suave e lenta a saída.
    max_rate: 1.0
```

## Lógica e ROS

A lógica fica num módulo sem `rclpy`, só Python e números, que os testes de unidade rodam sem
ROS: no modelo, `ftl_template/ftl_template/logic.py`. O nó é fino e fica em
`ftl_template/ftl_template/node.py`: declara os parâmetros, liga tópicos e chama a lógica, e nada
mais. Uma regra nova vai para a lógica, não para o nó.

## Testes

Os níveis, do mais barato ao mais caro: testes de unidade da lógica com pytest, sem ROS; testes
do nó com pytest, com `rclpy`; testes de código C++ com gtest, quando o pacote tiver C++; testes
de integração com launch_testing, que sobem os nós de verdade; e os testes de lint, que rodam
junto. Rodam todos no container, pelo `test.sh`, da pasta do repositório ou do worktree:

```bash
~/dev/ros2/scripts/test.sh ftl_follower               # um pacote, tudo
~/dev/ros2/scripts/test.sh ftl_follower ftl_template  # vários pacotes
~/dev/ros2/scripts/test.sh ftl_follower -k lookahead  # só os que têm o nome na expressão
~/dev/ros2/scripts/test.sh ftl_bringup -R test_readme # só os arquivos que casam
```

O `-k` filtra pelo nome os testes pytest dos pacotes Python, sem curinga. O `-R` escolhe pelo
nome os testes registrados no CMake (gtest e `ament_add_pytest_test`), nos pacotes `ament_cmake`.
O comando termina com `RESUMO testes=… falhas=…`.

## Lint

O lint é o do ament: flake8, com linhas de até 99 caracteres, aspas simples, imports em ordem
alfabética e sem espaço sobrando; e pep257, com a convenção do ament e docstrings multilinha que
começam na segunda linha (D213). Os testes `test_flake8.py` e `test_pep257.py` do modelo rodam
os dois.

O estilo Google lê `Returns:` e `Raises:` como seções do numpy, que o ament cobra no formato
dele; por isso o `test_pep257.py` dispensa três regras: D406 (título de seção sem dois-pontos),
D407 (sublinhado de traços sob o título) e D413 (linha em branco depois da última seção). Só
essas três.

Uma armadilha: o D401 exige o primeiro verbo no imperativo, em inglês, e recusa um verbo português
que se parece com um inglês, como "Converte", que ele lê como "Converts". Como as docstrings são
em português, troque de verbo: "Transforma" no lugar de "Converte".

## Commits

A mensagem é em português, com um resumo curto no imperativo ou no presente, e a feature entre
parênteses no fim, como `(F0.4)`. O corpo, quando houver, diz o porquê.

## Pacote novo

1. Copie o modelo para fora de `ftl_template/`, na raiz do repositório:
   `cp -r ftl_template ftl_novo`.
2. Troque o nome em toda parte: a pasta, a pasta de módulo `ftl_novo/ftl_novo`, o arquivo vazio
   em `resource/`, o `package.xml`, o `setup.py` (nome, `packages`, `data_files`, `entry_points`)
   e o `setup.cfg`.
3. Troque a lógica, o nó e os testes pelos do pacote; mantenha as dispensas do `test_pep257.py`.
4. Rode `grep -r ftl_template ftl_novo` e confira que não sobrou nada.
5. Ponha o pacote no comboio pelo `ftl_bringup`: o nó entra em `ftl_bringup/launch/ftl.launch.py`,
   os parâmetros em `ftl_bringup/config/ftl.yaml` e o `<exec_depend>` em
   `ftl_bringup/package.xml`.
6. Liste o pacote na árvore do `README.md`.

## Decisões

Registre uma decisão quando ela mudar o texto da dissertação: uma equação, um limiar, uma escolha
de método. O registro diz o contexto, a decisão e o que muda no texto. O modelo é
`docs/decisoes/modelo.md` e o índice, `docs/decisoes/README.md`; todo registro novo entra no
índice.
