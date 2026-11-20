# Um ponto do trilho virtual do comboio.
# Publicado dentro de Rail, pelo ftl_trajectory.

uint32 index                       # índice do ponto no trilho
float64 s                          # comprimento de arco desde o início do trilho (m)
geometry_msgs/Point position       # posição do ponto no frame ftl_map
float64 heading                    # rumo da tangente ao trilho no ponto (rad)
float64 curvature                  # curvatura do trilho no ponto (1/m)
float64 speed_limit                # limite de velocidade no ponto (m/s)
