cmake_minimum_required(VERSION 3.0.2)
project(soma_mse)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    dwa_local_planner
    tf2_ros
    tf
    nav_msgs
    )

catkin_package(
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

add_executable(harvest_behavior_node
    src/node/behavior_node.cpp
    src/definition.h
    src/StateBase/StateBase.cpp
    src/StateBase/StateBase.h
    src/States/Stop.cpp
    src/States/Stop.h
    src/States/MoveTo.cpp
    src/States/MoveTo.h
    src/States/Home.cpp
    src/States/Home.h
    )

target_link_libraries(harvest_behavior_node
    ${catkin_LIBRARIES}
    )

add_executable(sample_odom_subscriber
src/node/sample_slam_odom.cpp
)

target_link_libraries(sample_odom_subscriber
${catkin_LIBRARIES}
)