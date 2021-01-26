cmake_minimum_required(VERSION 3.1.3)
project(generate_images)

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

find_package(OpenCV REQUIRED)
find_package(Eigen3)


catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
    OpenCV
)

###########
## Build ##
###########

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
add_executable(DisplayImage src/main.cpp )
target_link_libraries(DisplayImage ${OpenCV_LIBS} 
    ${catkin_LIBRARIES})
