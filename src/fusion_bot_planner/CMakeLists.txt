cmake_minimum_required(VERSION 3.8)
project(fusion_bot_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#1. Tell CMake to compile our .cpp file into a shared library
add_library(${PROJECT_NAME} SHARED src/fusion_bot_planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 2. Link the ROS 2 libraries we depend on to our new library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_core
  pluginlib
  nav_msgs
  geometry_msgs
)

# 3. Tell ROS 2 where to find our XML plugin file
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

# 4. Tell CMake to "install" the library so Nav2 can find it at runtime
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)


ament_package()
