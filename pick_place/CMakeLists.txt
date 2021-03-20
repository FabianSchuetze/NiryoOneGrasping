
cmake_minimum_required(VERSION 3.1.3)
project(pick_place)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  niryo_one_msgs
  pose_detection
  tf2_ros
  genmsg
  #tf2_eigen
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
      roscpp
      std_msgs
      actionlib
      actionlib_msgs
      niryo_one_msgs
      pose_detection
      tf2_geometry_msgs
      tf2_ros
      genmsg
  #DEPENDS 
)

###########
## Build ##
###########
set(${PROJECT_NAME}_SRC
    src/picking.cpp
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
include_directories(include)
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRC})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pick_place_exe src/main.cpp)
target_link_libraries(pick_place_exe ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(simple_pick_place_exe src/simple_pick_place.cpp)
target_link_libraries(simple_pick_place_exe ${catkin_LIBRARIES} ${PROJECT_NAME})
