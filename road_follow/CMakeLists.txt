cmake_minimum_required(VERSION 3.8)
project(road_follow)

find_package(ament_cmake REQUIRED)

# Em lib/<pacote>, então o comando é `ros2 run road_follow road_follow`. RENAME
# faz o colcon copiar em vez de criar link: recompile depois de editar o script.
install(
  PROGRAMS scripts/road_follow.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME road_follow
)

ament_package()
