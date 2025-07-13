cmake_minimum_required(VERSION 3.10)
project(quad_gap)

add_compile_options(-std=c++17)

##########
# OpenMP #
##########

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

################
# Dependencies #
################
set(dependencies
    rclcpp
    geometry_msgs
    nav2_costmap_2d
    pluginlib
    nav_msgs
    nav2_util
    nav2_core
    tf2
  # pips_trajectory_testing
  # turtlebot_trajectory_testing
  # pips_egocylindrical
  # pips_egocircle
  # dynamic_reconfigure
  # dynamic_reconfigure
  # benchmarking_tools
  # pips
  # pips_trajectory_msgs
)

find_package(ament_cmake REQUIRED)

foreach(dependency IN ITEMS ${dependencies})
  find_package(${dependency} REQUIRED)
endforeach()

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

###########
# Library #
###########

add_library(quad_gap
  # src/gap_detection/GapDetector.cpp
  # src/config/QuadGapConfig.cpp
  # src/visualization/GapVisualizer.cpp
  # src/visualization/GoalVisualizer.cpp
  # src/visualization/TrajectoryVisualizer.cpp
  # src/global_plan_management/GlobalPlanManager.cpp
  # src/trajectory_evaluation/TrajectoryEvaluator.cpp
  # src/trajectory_generation/GapGoalPlacer.cpp
  # src/trajectory_generation/GapTrajectoryGenerator.cpp
  # src/trajectory_generation/GapManipulator.cpp
  # src/trajectory_tracking/TrajectoryController.cpp
  # src/Planner.cpp
  src/QuadGapPlanner.cpp 
  # src/TimeKeeper.cpp 
  ) 

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core ${PROJECT_NAME}.xml)

ament_package()

