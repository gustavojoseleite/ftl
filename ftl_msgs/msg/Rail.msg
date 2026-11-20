# Trilho virtual que os seguidores percorrem.
# Publicado pelo ftl_trajectory (construído pelo Trator 1 e retransmitido pelos demais).

std_msgs/Header header             # stamp do instante da construção; frame ftl_map
string leader_id                   # identificador do líder cujo caminho gerou o trilho
string built_by                    # identificador do veículo que suavizou o trilho
RailPoint[] points                 # pontos do trilho, em ordem crescente de s
