cmake_minimum_required(VERSION 3.8)
project(ftl_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(name test_decisions test_contributing test_readme)
    ament_add_pytest_test(${name} test/${name}.py)
  endforeach()
endif()

ament_package()
