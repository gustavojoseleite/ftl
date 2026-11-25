#!/bin/bash
# Grava, pela GUI do Gazebo, o vídeo da B0 nas curvas.
# Uso, na raiz do workspace e com tela:
#   docker/run.sh --name b0-video bash -lc /workspace/src/ftl/docs/experimentos/b0-gazebo-video.sh
# Argumentos extras vão para o b0-gazebo-video.py (por exemplo, --view chase).
# Roda uma execução da B0 com a mesma configuração da campanha b0_problema (90°, c0, semente 1),
# em experiments/b0_video, abre o cliente gráfico do Gazebo e grava as curvas (CURVES, padrão
# 0,1) em experiments/b0_video/video. O servidor roda sem tela, como na campanha; só o cliente
# gráfico usa o DISPLAY. Para outro caso, NAME dá a pasta (experiments/<NAME>) e EXTRA, as chaves
# de cenário a mais; o E1 é NAME=e1_video EXTRA='ftl.localization=ukf ftl.follower=trail'.
source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash
# O download às vezes chega corrompido (erro de hash do pip); três tentativas bastam.
for _ in 1 2 3; do
  pip install -q --no-cache-dir --break-system-packages --user imageio-ffmpeg==0.8.1 \
    python-xlib==0.33 && break
done
set -uo pipefail
here=/workspace/src/ftl/docs/experimentos
name=${NAME:-b0_video}
out=/workspace/experiments/$name
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-75} GZ_PARTITION=$name
base='convoy.followers=4 route.enabled=true route.cruise_kmh=10.8'
base+=' sensors.camera_mount=single sensors.profile=nominal'
mkdir -p "$out"
(unset DISPLAY; ros2 run ftl_experiments run --campaign "$name" --root /workspace/experiments \
  --set "world=citrus_farm_${WORLD:-90}.sdf ${base} ${EXTRA:-}" --seeds "${SEED:-1}" \
  --label "p${WORLD:-90}_c0") > "$out/run.log" 2>&1 &
runner=$!
until gz topic -l 2>/dev/null | grep -q '^/world/.*/clock'; do sleep 2; done
gz sim -g --gui-config "$here/b0-gazebo-gui.config" > "$out/gui.log" 2>&1 &
gui=$!
python3 "$here/b0-gazebo-video.py" --world "citrus_farm_${WORLD:-90}" --curves "${CURVES:-0,1}" \
  --out "$out/video" "$@"
sleep 5
kill -INT "$runner" "$gui" 2>/dev/null
wait "$runner"
ls -la "$out/video"
