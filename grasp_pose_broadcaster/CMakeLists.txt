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
  object_pose
  utils
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  #INCLUDE_DIRS devel/include
#  LIBRARIES turtle_tf2
 CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros message_runtime object_pose
                utils
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(${PROJECT_NAME}_SRC
    src/grasp_pose_broadcaster.cpp
)

find_package(Open3D REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS} ${Open3D_LIBRARIES}
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Open3D_LIBRARIES})

## C++ examples
add_executable(grasp_pose_broadcaster src/main.cpp)
add_dependencies(grasp_pose_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(grasp_pose_broadcaster ${catkin_LIBRARIES}
    ${Open3D_LIBRARIES} ${PROJECT_NAME})
