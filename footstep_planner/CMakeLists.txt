cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# ROS fuerte system install:
#find_package(sbpl REQUIRED)
# hardcoded fix for fuerte?
include_directories(/opt/ros/fuerte/include/sbpl)

set(FOOTSTEP_PLANNER_FILES src/FootstepPlanner.cpp
	src/FootstepNavigation.cpp
    src/FootstepPlannerNode.cpp
    src/FootstepPlannerEnvironment.cpp 
    src/Footstep.cpp
    src/PlanningState.cpp
    src/Heuristic.cpp 
    src/helper.cpp
    src/PathCostHeuristic.cpp
    src/PlanningStateChangeQuery.cpp
)

rosbuild_add_library(${PROJECT_NAME} ${FOOTSTEP_PLANNER_FILES})
target_link_libraries(${PROJECT_NAME} ${sbpl_LIBRARIES})
rosbuild_add_executable(footstep_planner_node src/footstep_planner.cpp)
target_link_libraries(footstep_planner_node ${PROJECT_NAME} ${sbpl_LIBRARIES})
rosbuild_add_executable(footstep_navigation_node src/footstep_navigation.cpp)
target_link_libraries(footstep_navigation_node ${PROJECT_NAME} ${sbpl_LIBRARIES})
