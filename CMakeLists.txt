cmake_minimum_required(VERSION 3.5)
project(nonpersistent_voxel_layer)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
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
find_package(pcl_conversions REQUIRED)
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

find_package(PCL REQUIRED COMPONENTS filters common)
if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # This package fails to build on Debian Stretch with a linking error against
  # 'Qt5::Widgets'.  This is a transitive dependency that comes in to PCL via
  # the PCL dependency on VTK.  However, we don't actually care about the Qt
  # dependencies for this package, so just remove them.  This is similar to the
  # workaround in https://github.com/ros-perception/perception_pcl/pull/151,
  # and can be removed when Stretch goes out of support.
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt")
endif()

set(dependencies
	geometry_msgs
	laser_geometry
	map_msgs
	message_filters
	nav_msgs
	pcl_conversions
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
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(${library_name} SHARED
	plugins/nonpersistent_voxel_layer.cpp
)

target_link_libraries(${library_name}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

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
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY example
  DESTINATION share
)

install(FILES costmap_plugins.xml
  DESTINATION share
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
