#!/usr/bin/env bash
# Caso de teste de b0-problema.md: a B0 nos mundos de 90, 60 e 30 graus, a 10,8 km/h, com 4
# tratores, sensores ideais (c0: perfil nominal, sem degradação) e de campo (IMU mems,
# odometria field).
source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash
set -uo pipefail
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-74} GZ_PARTITION=b0_problema
unset DISPLAY
base='convoy.followers=4 route.enabled=true route.cruise_kmh=10.8'
base+=' sensors.camera_mount=single sensors.profile=nominal'
for w in ${WORLDS:-90 60 30}; do
  for cond in ${CONDS:-c0 field}; do
    extra=''
    [[ $cond == field ]] && extra='degradation.imu=mems degradation.odometry=field'
    ros2 run ftl_experiments run --campaign b0_problema --root /workspace/experiments \
      --set "world=citrus_farm_${w}.sdf ${base} ${extra}" --seeds "${SEEDS:-1,2,3}" \
      --label "p${w}_${cond}"
  done
done
