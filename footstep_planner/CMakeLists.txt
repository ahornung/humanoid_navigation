cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# ROS fuerte system install:
#find_package(sbpl REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)

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
    src/State.cpp
)

rosbuild_add_library(${PROJECT_NAME} ${FOOTSTEP_PLANNER_FILES})
target_link_libraries(${PROJECT_NAME} ${SBPL_LIBRARIES})
rosbuild_add_executable(footstep_planner_node src/footstep_planner.cpp)
target_link_libraries(footstep_planner_node ${PROJECT_NAME} ${SBPL_LIBRARIES})
rosbuild_add_executable(footstep_planner_walls src/footstep_planner_walls.cpp)
target_link_libraries(footstep_planner_walls ${PROJECT_NAME} ${SBPL_LIBRARIES} ${OpenCV_LIBS})
rosbuild_add_executable(footstep_navigation_node src/footstep_navigation.cpp)
target_link_libraries(footstep_navigation_node ${PROJECT_NAME} ${SBPL_LIBRARIES})
