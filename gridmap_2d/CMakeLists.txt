cmake_minimum_required(VERSION 2.8.3)
project(gridmap_2d)

find_package(catkin REQUIRED COMPONENTS nav_msgs)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(${PROJECT_NAME} src/GridMap2D.cpp)

#define some target ...
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# install
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
) 
