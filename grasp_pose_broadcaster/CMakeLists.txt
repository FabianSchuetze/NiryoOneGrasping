cmake_minimum_required(VERSION 3.1.3)
project(grasp_pose_broadcast)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
## Find dependencies
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtle_tf2
 CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## C++ examples
add_executable(grasp_pose_broadcaster src/main.cpp)
target_link_libraries(grasp_pose_broadcaster ${catkin_LIBRARIES})
