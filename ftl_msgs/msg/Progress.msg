# Progresso do veículo ao longo do trilho.
# Publicada pelo ftl_trajectory.

std_msgs/Header header             # stamp do instante da medição
string vehicle_id                  # identificador do veículo
float64 s                          # comprimento de arco do veículo no trilho (m)
uint32 released_index              # último índice de waypoint liberado
