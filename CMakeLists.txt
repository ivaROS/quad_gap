cmake_minimum_required(VERSION 3.10)
project(quad_gap)

add_compile_options(-std=c++17)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  pluginlib
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  pips_trajectory_msgs
  message_generation
  mbf_costmap_core
  pips_trajectory_testing
  turtlebot_trajectory_testing
  pips_egocylindrical
  pips_egocircle
  # dynamic_reconfigure
  # dynamic_reconfigure
  # benchmarking_tools
  # pips
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

# generate_dynamic_reconfigure_options(
#   cfg/qg.cfg
#   cfg/CollisionChecker.cfg
# )

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
  DEPENDS OpenMP 
  CATKIN_DEPENDS 
    base_local_planner geometry_msgs nav_msgs roscpp rospy std_msgs sensor_msgs message_runtime pips_trajectory_msgs #   pips_egocircle dynamic_reconfigure
)

add_library(quad_gap
  src/gap_detection/GapDetector.cpp
  src/config/QuadGapConfig.cpp
  src/visualization/GapVisualizer.cpp
  src/visualization/GoalVisualizer.cpp
  src/visualization/TrajectoryVisualizer.cpp
  src/global_plan_management/GlobalPlanManager.cpp
  src/trajectory_evaluation/TrajectoryEvaluator.cpp
  src/trajectory_generation/GapTrajectoryGenerator.cpp
  src/trajectory_generation/GapManipulator.cpp
  src/trajectory_tracking/TrajectoryController.cpp
  src/Planner.cpp
  src/QuadGapPlanner.cpp 
  src/TimeKeeper.cpp 
  ) 

add_dependencies(quad_gap ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)

target_compile_options(quad_gap PRIVATE ${OpenMP_FLAGS})

target_link_libraries(quad_gap
                  ${catkin_LIBRARIES}
                  ${OpenMP_LIBS}
                  yaml-cpp
)
