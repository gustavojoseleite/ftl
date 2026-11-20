# Estado da máquina de estados do seguidor, com motivo, erros e disponibilidade dos sensores.
# Publicada pelo ftl_supervisor.

uint8 STATE_INITIALIZING=0        # INICIALIZANDO: aguardando os primeiros dados
uint8 STATE_WAITING_FOR_RAIL=1    # AGUARDANDO_TRILHO: sem trilho para seguir
uint8 STATE_NOMINAL=2             # NOMINAL: GNSS e tag disponíveis
uint8 STATE_TAG_ONLY=3            # SO_TAG: só a tag do precedente disponível
uint8 STATE_GNSS_ONLY=4           # SO_GNSS: só o GNSS do precedente disponível
uint8 STATE_ESTIMATED_POSE=5      # POSE_ESTIMADA: segue pela estimativa sem medições novas
uint8 STATE_LOCAL_TRAJECTORY=6    # TRAJETORIA_LOCAL: segue uma trajetória local
uint8 STATE_SAFE_STOP=7           # PARADA_SEGURA: parada controlada
uint8 STATE_EMERGENCY=8           # EMERGENCIA: parada de emergência

std_msgs/Header header            # stamp do instante da medição
string vehicle_id                 # identificador do seguidor
uint8 state                       # estado atual (STATE_*)
string reason                     # motivo da última transição de estado
float64 lateral_error             # erro lateral ao trilho (m)
float64 gap_error                 # erro de distância ao precedente (m)
float64 heading_error             # erro de rumo em relação ao trilho (rad)
bool own_gnss                     # o GNSS próprio está disponível
bool ahead_gnss                   # o GNSS do precedente está disponível
bool ahead_tag                    # a tag do precedente está disponível
