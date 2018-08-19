cmake_minimum_required(VERSION 3.5)
project(costmap)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_library(costmap_ros
        src/costmap.cpp
        src/costmap_ros.cpp
        src/layer.cpp
        src/map_cell.cpp)

ament_target_dependencies(costmap_ros
        rclcpp
        tf2
        tf2_ros
        pluginlib
        nav_msgs
        geometry_msgs
        tf2_geometry_msgs)

add_library(layers SHARED
        plugins/map_layer.cpp
        plugins/obstacle_layer.cpp
        plugins/inflation_layer.cpp)
ament_target_dependencies(layers rclcpp)
target_link_libraries(layers costmap_ros)
pluginlib_export_plugin_description_file(costmap plugins_description.xml)

add_executable(costmap_node src/costmap_node.cpp)
target_link_libraries(costmap_node costmap_ros)
ament_target_dependencies(costmap_node rclcpp )

install(TARGETS costmap_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS costmap_ros layers
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}/)

ament_package()