# Envelope das mensagens trocadas por V2V, com origem, remetente e sequência.
# Publicada pelo ftl_comms; leva um só conteúdo, indicado por kind.

uint8 KIND_FIX=1       # o conteúdo é o fix GNSS bruto (campo fix)
uint8 KIND_STATUS=2    # o conteúdo é o estado do supervisor (campo status)
uint8 KIND_RAIL=3      # o conteúdo é o trilho (campo rail)

std_msgs/Header header                         # stamp de origem da medição
string sender                                  # identificador do remetente
uint64 sequence                                # número de sequência, para medir latência e perda
uint8 kind                                     # tipo do conteúdo (KIND_*)
sensor_msgs/NavSatFix[<=1] fix                 # fix GNSS bruto, se kind for KIND_FIX
FollowerStatus[<=1] status                     # estado do supervisor, se kind for KIND_STATUS
Rail[<=1] rail                                 # trilho, se kind for KIND_RAIL
