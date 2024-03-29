cmake_minimum_required(VERSION 3.1.3)
project(gpd)

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
  object_pose
  gpd_ros
  eigen_conversions
  utils
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES turtle_tf2
 CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros message_runtime
 object_pose gpd_ros eigen_conversions utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

find_package(Open3D REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS} ${Open3D_LIBRARIES} include
)

## C++ examples
add_executable(gpd_interaction src/gpd_interaction.cpp src/main.cpp)
add_dependencies(gpd_interaction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gpd_interaction ${catkin_LIBRARIES}
    ${Open3D_LIBRARIES})
