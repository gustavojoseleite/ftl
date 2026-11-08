cmake_minimum_required(VERSION 3.8)
project(road_follow)

find_package(ament_cmake REQUIRED)

# lib/<package>, so the command is `ros2 run road_follow road_follow`. RENAME
# makes colcon copy instead of symlink: rebuild after editing the script.
install(
  PROGRAMS scripts/road_follow.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME road_follow
)

ament_package()
