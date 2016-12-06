cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_msgs
    pcl_ros
    sensor_msgs
    roscpp
    rospy
    std_msgs
    dynamic_reconfigure
    )

find_package(PCL REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

generate_dynamic_reconfigure_options(cfg/reconfig.cfg)

catkin_package()

add_executable(pcl_process src/pcl_process.cpp)
add_executable(pcl_read src/pcl_read.cpp)
add_executable(pcl_visualize src/pcl_visualize.cpp)

target_link_libraries(pcl_process ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_read ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_visualize ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(pcl_process ${PROJECT_NAME}_gencfg)


