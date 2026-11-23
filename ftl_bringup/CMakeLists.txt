cmake_minimum_required(VERSION 3.8)
project(ftl_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/apply_scenario DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(name test_decisions test_contributing test_readme test_ci_workflow test_ci_docs test_b0
               test_sim_launch test_scenario test_scenario_route
               test_driver_parameters)
    ament_add_pytest_test(${name} test/${name}.py)
  endforeach()
  ament_add_pytest_test(test_sim_experiment test/test_sim_experiment.py TIMEOUT 900)
  set_tests_properties(test_sim_experiment PROPERTIES LABELS simulation)
  ament_add_pytest_test(test_sim_route test/test_sim_route.py TIMEOUT 900)
  set_tests_properties(test_sim_route PROPERTIES LABELS simulation)
endif()

ament_package()
