cmake_minimum_required(VERSION 3.16)
project(vc_safety)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(safety_state src/safety_state.cpp)
target_compile_features(safety_state PUBLIC cxx_std_17)
target_include_directories(safety_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(input_validation src/input_validation.cpp)
target_compile_features(input_validation PUBLIC cxx_std_17)
target_include_directories(input_validation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(input_validation geometry_msgs nav_msgs sensor_msgs)

add_library(ecu_diagnostics src/ecu_diagnostics.cpp)
target_compile_features(ecu_diagnostics PUBLIC cxx_std_17)
target_include_directories(ecu_diagnostics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ecu_diagnostics diagnostic_msgs)

add_library(safety_gate_component SHARED src/safety_gate_node.cpp)
target_compile_features(safety_gate_component PUBLIC cxx_std_17)
target_link_libraries(
  safety_gate_component safety_state input_validation ecu_diagnostics)
ament_target_dependencies(safety_gate_component
  diagnostic_msgs geometry_msgs nav_msgs rcl_interfaces rclcpp
  rclcpp_components rclcpp_lifecycle sensor_msgs std_msgs std_srvs)
rclcpp_components_register_nodes(
  safety_gate_component "vc_safety::SafetyGateNode")

add_executable(safety_gate_node src/safety_gate_main.cpp)
target_compile_features(safety_gate_node PUBLIC cxx_std_17)
target_link_libraries(safety_gate_node safety_gate_component)
ament_target_dependencies(safety_gate_node rclcpp)

install(TARGETS
  safety_state input_validation ecu_diagnostics
  safety_gate_component safety_gate_node
  EXPORT export_vc_safety
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(test_safety_config test/test_safety_config.py)
  ament_add_gtest(test_safety_state test/test_safety_state.cpp)
  target_link_libraries(test_safety_state safety_state)
  ament_add_gtest(test_input_validation test/test_input_validation.cpp)
  target_link_libraries(test_input_validation input_validation)
  ament_add_gtest(test_ecu_diagnostics test/test_ecu_diagnostics.cpp)
  target_link_libraries(test_ecu_diagnostics ecu_diagnostics)
  add_launch_test(test/test_safety_gate_launch.py TIMEOUT 30)
endif()

ament_export_include_directories(include)
ament_export_libraries(safety_state input_validation ecu_diagnostics)
ament_export_targets(export_vc_safety HAS_LIBRARY_TARGET)
ament_export_dependencies(
  diagnostic_msgs geometry_msgs nav_msgs rclcpp rclcpp_lifecycle sensor_msgs)
ament_package()
