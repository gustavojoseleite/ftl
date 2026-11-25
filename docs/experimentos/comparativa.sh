#!/usr/bin/env bash
# Campanha comparativa do artigo (comparativa.md): a B0 e o E1 (proposta inicial) nos mundos de
# 90, 60 e 30 graus, a 10,8 km/h, com 4 tratores, sensores nominais (c0) e de campo (IMU mems,
# odometria field), sementes 1, 2 e 3. A semente fixa também o ruído do GNSS do Gazebo, então a B0
# e o E1 recebem os mesmos erros de sensor.
#   B0: pose_estimator + rastro do veículo da frente (pure_pursuit)
#   E1: UKF (ftl_localization) + trilho único do líder com amostragem adaptativa e CCMA
# Rótulos: <caso>_p<ângulo>_<condição>, por exemplo e1_p90_field. Uma execução que termina em
# `failed` (falha na partida da simulação) é guardada em <campanha>_falhas e refeita até 3 vezes.
source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash
set -uo pipefail
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-74} GZ_PARTITION=comparativa
unset DISPLAY
campaign=${CAMPAIGN:-comparativa}
root=/workspace/experiments
base='convoy.followers=4 route.enabled=true route.cruise_kmh=10.8'
base+=' sensors.camera_mount=single sensors.profile=nominal'
declare -A CASES=(
  [b0]='ftl.localization=pose_estimator ftl.follower=pure_pursuit'
  [e1]='ftl.localization=ukf ftl.follower=trail'
)

run() {  # run <caso> <mundo> <condição> <sementes>
  local extra=''
  [[ $3 == field ]] && extra='degradation.imu=mems degradation.odometry=field'
  ros2 run ftl_experiments run --campaign "$campaign" --root "$root" \
    --set "world=citrus_farm_${2}.sdf ${base} ${extra} ${CASES[$1]}" --seeds "$4" \
    --label "${1}_p${2}_${3}"
}

for w in ${WORLDS:-90 60 30}; do
  for cond in ${CONDS:-c0 field}; do
    for case in ${CASE_LIST:-b0 e1}; do
      run "$case" "$w" "$cond" "${SEEDS:-1,2,3}"
    done
  done
done

for attempt in 1 2 3; do
  failed=$(grep -l '^reason: failed' "$root/$campaign"/*/*/run.yaml 2>/dev/null)
  [[ -z $failed ]] && break
  for file in $failed; do
    dir=$(dirname "$file"); seed=$(basename "$dir"); label=$(basename "$(dirname "$dir")")
    mkdir -p "$root/${campaign}_falhas/$label"
    mv "$dir" "$root/${campaign}_falhas/$label/${seed}_$(date +%H%M%S)"
    case=${label%%_*}; rest=${label#*_}; w=${rest%%_*}; w=${w#p}; cond=${rest#*_}
    echo "refazendo $label/$seed (tentativa $attempt)"
    run "$case" "$w" "$cond" "$seed"
  done
done
