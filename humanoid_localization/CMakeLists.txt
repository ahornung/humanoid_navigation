cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(octomap REQUIRED)                                                                        
include_directories(${OCTOMAP_INCLUDE_DIRS})                                                          
link_directories(${OCTOMAP_LIBRARY_DIRS})

find_package(dynamicEDT3D REQUIRED)                                                                        
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})                                                          
link_directories(${DYNAMICEDT3D_LIBRARY_DIRS})

LINK_LIBRARIES(${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})                                                    

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_library(${PROJECT_NAME} src/HumanoidLocalization.cpp src/MotionModel.cpp src/MapModel.cpp
		src/ObservationModel.cpp src/EndpointModel.cpp src/RaycastingModel.cpp)
rosbuild_add_boost_directories()
rosbuild_add_openmp_flags(${PROJECT_NAME})

rosbuild_add_executable(localization_node src/localization_node.cpp)
#rosbuild_link_boost(${PROJECT_NAME} random)
target_link_libraries(localization_node ${PROJECT_NAME} ${OCTOMAP_LIBRARIES})
