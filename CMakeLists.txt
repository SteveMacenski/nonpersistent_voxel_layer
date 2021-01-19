cmake_minimum_required(VERSION 2.8.3)
project(nonpersistent_voxel_layer)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            dynamic_reconfigure
            geometry_msgs
            laser_geometry
            map_msgs
            message_filters
            message_generation
            nav_msgs
            pluginlib
            roscpp
            sensor_msgs
            std_msgs
            tf
            visualization_msgs
            voxel_grid
            costmap_2d
        )

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/NonPersistentVoxelPlugin.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES nonpersistent_voxel_layer
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        laser_geometry
        map_msgs
        costmap_2d
        message_filters
        message_runtime
        nav_msgs
        pluginlib
        roscpp
        sensor_msgs
        std_msgs
        tf
        visualization_msgs
        voxel_grid
    DEPENDS
        EIGEN3
        Boost
)

add_library(nonpersistent_voxel_layer
  plugins/nonpersistent_voxel_layer.cpp
)
add_dependencies(nonpersistent_voxel_layer ${PROJECT_NAME}_gencfg)
target_link_libraries(nonpersistent_voxel_layer ${catkin_LIBRARIES})

install(TARGETS
    nonpersistent_voxel_layer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/nonpersistent_voxel_layer/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
