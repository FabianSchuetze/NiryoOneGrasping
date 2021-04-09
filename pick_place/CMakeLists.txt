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
  tf2_ros
  tf_conversions
  genmsg
  tf2_eigen
  tf2_geometry_msgs
  cv_bridge
  pose_detection
)

add_action_files(
  DIRECTORY action
  FILES MoveJoints.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
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
      tf2_geometry_msgs
      tf_conversions
      tf2_ros
      genmsg
      pcl_ros
      cv_bridge
      pose_detection
  DEPENDS 
    EIGEN3
    OpenCV
)


###########
## Build ##
###########
set(${PROJECT_NAME}_SRC
    src/picking.cpp
)
find_package(OpenCV REQUIRED PATHS /usr/local/lib/cmake/opencv4
             NO_DEFAULT_PATH)
include_directories(/usr/local/include/opencv4)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#include_directories("/root/generate_samples/devel/include/")
include_directories(include)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)


add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pick_place_exe src/main.cpp)
target_link_libraries(pick_place_exe ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(simple_pick_place_exe src/simple_pick_place.cpp)
target_link_libraries(simple_pick_place_exe ${catkin_LIBRARIES})

add_executable(move_joints src/move_joints.cpp)
target_link_libraries(move_joints ${catkin_LIBRARIES} ${PROJECT_NAME})
