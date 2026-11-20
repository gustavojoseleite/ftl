cmake_minimum_required(VERSION 3.8)
project(ftl_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedVehicle.msg"
  "msg/RailPoint.msg"
  "msg/Rail.msg"
  "msg/Progress.msg"
  "msg/FollowerStatus.msg"
  "msg/V2VEnvelope.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_add_pytest_test(test_msgs_py test/test_msgs.py)

  ament_add_gtest(test_msgs_cpp test/test_msgs.cpp)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(test_msgs_cpp "${cpp_typesupport_target}")
  ament_target_dependencies(test_msgs_cpp rclcpp sensor_msgs)

  ament_add_pytest_test(test_readme test/test_readme.py)
endif()

ament_package()
