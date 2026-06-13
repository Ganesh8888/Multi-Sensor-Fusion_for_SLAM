cmake_minimum_required(VERSION 3.12)
project(fusion_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # 1. Test URDF Xacro parsing
  ament_add_pytest_test(test_urdf test/test_urdf.py
    TIMEOUT 15
  )

  # 2. Test Simulation Launch File
  add_launch_test(test/test_sim_launch.py
    TIMEOUT 25
  )
endif()

# Add C++ targets
add_executable(fault_injector_node src/fault_injector_node.cpp)
ament_target_dependencies(fault_injector_node rclcpp nav_msgs geometry_msgs)

# Install compiled targets
install(TARGETS
  fault_injector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories for non-compiled files (Launch, URDF, configs, etc.)
install(DIRECTORY
  launch
  description
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
