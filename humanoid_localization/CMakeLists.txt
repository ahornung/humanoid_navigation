cmake_minimum_required(VERSION 2.8.3)
project(humanoid_localization)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_conversions pcl_ros message_filters sensor_msgs)
find_package(OpenMP)
find_package(Boost REQUIRED COMPONENTS random)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

find_package(dynamicEDT3D)
if (${dynamicEDT3D_FOUND})                                                                       
  include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})                                                          
  link_directories(${DYNAMICEDT3D_LIBRARY_DIRS})
  LINK_LIBRARIES(${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})
  list(APPEND LIB_FILES src/EndpointModel.cpp)                                      
else (${dynamicEDT3D_FOUND})
  MESSAGE(WARNING "dynamicEDT3D library (part of OctoMap >1.5) not found, skipping endpoint model")
  add_definitions(-DSKIP_ENDPOINT_MODEL)
endif (${dynamicEDT3D_FOUND})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mapmodel motionmodel observationmodel raycastingmodel observationmodel humanoidlocalization
  CATKIN_DEPENDS tf
  DEPENDS octomap OpenMP
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

if (${dynamicEDT3D_FOUND})
  add_library(endpointmodel src/EndpointModel.cpp)
endif (${dynamicEDT3D_FOUND})

add_library(mapmodel src/MapModel.cpp)
add_library(motionmodel src/MotionModel.cpp)
add_library(observationmodel src/ObservationModel.cpp)

add_library(raycastingmodel src/RaycastingModel.cpp)
target_link_libraries(raycastingmodel observationmodel)

add_library(humanoidlocalization src/HumanoidLocalization.cpp)
target_link_libraries(humanoidlocalization mapmodel motionmodel observationmodel raycastingmodel ${catkin_LIBRARIES})

add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node humanoidlocalization ${catkin_LIBRARIES})

# install
install(TARGETS mapmodel motionmodel observationmodel raycastingmodel humanoidlocalization
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
) 
install(TARGETS localization_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
