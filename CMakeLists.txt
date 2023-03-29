cmake_minimum_required(VERSION 3.5)
project(nonpersistent_voxel_layer)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_voxel_grid REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
	geometry_msgs
	laser_geometry
	map_msgs
	message_filters
	nav_msgs
	pluginlib
	sensor_msgs
  nav2_msgs
	rclcpp
	std_msgs
	tf2
	tf2_ros
	visualization_msgs
	nav2_voxel_grid
	nav2_costmap_2d)

set(library_name nonpersistent_voxel_layer_core)

add_definitions(${EIGEN3_DEFINITIONS})

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(${library_name} SHARED
	plugins/nonpersistent_voxel_layer.cpp
)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_link_libraries(${library_name}
  ${EIGEN3_LIBRARIES}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY example
  DESTINATION share/${PROJECT_NAME}
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_package()
