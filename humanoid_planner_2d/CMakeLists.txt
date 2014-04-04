cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

## CMake config:
#find_package(sbpl REQUIRED)
## pkg-config:
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)

include_directories(${SBPL_INCLUDE_DIRS})
# not needed for SBPL with CMake config
link_directories(${SBPL_LIBRARY_DIRS})


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/SBPLPlanner2D.cpp)
target_link_libraries(${PROJECT_NAME} ${SBPL_LIBRARIES})
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(sbpl_2d_planner_node src/humanoid_planner_2d.cpp)
target_link_libraries(sbpl_2d_planner_node ${PROJECT_NAME})
