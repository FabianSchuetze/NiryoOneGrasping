
cmake_minimum_required(VERSION 3.1.3)
project(integration)

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
  pcl_ros
  cv_bridge
  new_pick_place
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
      new_pick_place
  DEPENDS 
    EIGEN3
    OpenCV
    PCL
)

###########
## Build ##
###########
set(${PROJECT_NAME}_SRC
    src/integrate.cpp
    src/communication.cpp
)
find_package(Open3D REQUIRED)
list(APPEND Open3D_LIBRARIES dl)
find_package(OpenCV REQUIRED PATHS /usr/local/lib/cmake/opencv4
             NO_DEFAULT_PATH)
include_directories(${Open3D_LIBRARIES})
include_directories(/usr/local/include/opencv4)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#include_directories("/root/generate_samples/devel/include/")
include_directories(include)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)


add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Open3D_LIBRARIES})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(integrate src/main.cpp)
target_link_libraries(integrate ${catkin_LIBRARIES} ${PCL_LIBRARIES}
    ${Open3D_LIBRARIES} ${PROJECT_NAME})

add_executable(move_joints_client src/move_joints_client.cpp)
target_link_libraries(move_joints_client ${catkin_LIBRARIES} ${PCL_LIBRARIES}
    ${Open3D_LIBRARIES} ${PROJECT_NAME})
