# Estimativa do veículo precedente, com a antena GNSS como ponto de referência.
# Publicada pelo ftl_fusion (UKF do precedente).

uint8 SOURCE_NONE=0   # nenhuma fonte ativa
uint8 SOURCE_GNSS=1   # bit: o fix GNSS do precedente (via V2V) está ativo
uint8 SOURCE_TAG=2    # bit: a pose da tag do precedente está ativa

std_msgs/Header header                              # stamp do instante da medição; frame ftl_map
string vehicle_id                                   # identificador do veículo precedente
geometry_msgs/PoseWithCovariance pose               # pose da antena do precedente e covariância
geometry_msgs/TwistWithCovariance twist             # velocidade estimada pelo UKF e covariância
uint8 sources                                       # máscara de bits das fontes ativas (SOURCE_*)
builtin_interfaces/Duration gnss_age                # idade da última medição GNSS
builtin_interfaces/Duration tag_age                 # idade da última medição da tag
