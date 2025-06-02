cmake_minimum_required(VERSION 3.10)
project(quad_gap)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  pluginlib
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  # benchmarking_tools
  # pips_trajectory_msgs
  dynamic_reconfigure
  message_generation
  mbf_costmap_core
  # pips
  # pips_trajectory_testing
  # turtlebot_trajectory_testing
  # pips_egocylindrical
  # pips_egocircle
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/qg.cfg
  cfg/CollisionChecker.cfg
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_message_files(
  FILES
  TrajPlan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS 
    include 
    ${EIGEN3_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
 LIBRARIES 
    quad_gap 
    yaml-cpp
  CATKIN_DEPENDS 
    base_local_planner dynamic_reconfigure geometry_msgs nav_msgs roscpp rospy std_msgs sensor_msgs message_runtime #  pips_trajectory_msgs pips_egocircle
)

add_library(quad_gap
  src/robot_geo_parser.cpp
  src/planner.cpp
  src/trajectory_follower.cpp
  src/quad_gap.cpp
  src/gap_detection/GapDetector.cpp
  
  src/potentialgap_config.cpp
  src/visualization.cpp
  src/goal_selector.cpp
  src/trajectory_scoring.cpp
  src/gap_trajectory_generator.cpp
  src/gap_manip.cpp
  src/trajectory_controller.cpp
  ) 

add_dependencies(quad_gap ${PROJECT_NAME}_gencfg)
                  target_compile_options(quad_gap PRIVATE ${OpenMP_FLAGS})
                  target_link_libraries(quad_gap
                  ${catkin_LIBRARIES}
                  ${OpenMP_LIBS}
                  yaml-cpp
)
