cmake_minimum_required(VERSION 3.1.3)
project(pick_place)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    rviz_visual_tools
    moveit_visual_tools
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)


catkin_package(
  LIBRARIES ${PROJECT_NAME}
  #INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
#include_directories("/home/fabian/Documents/work/github/gpd_catkin/devel")
add_executable(pick_place src/pick_place_tutorial.cpp)
target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
