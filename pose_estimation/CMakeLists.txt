cmake_minimum_required(VERSION 3.11)
project(pose_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    moveit_core
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    pcl_ros
    actionlib_msgs
    cv_bridge
    utils
    tf_conversions
    object_pose
)

find_package(OpenCV REQUIRED PATHS /usr/local/lib/cmake/opencv4
             NO_DEFAULT_PATH)
include_directories(/usr/local/include/opencv4)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)

add_action_files(
  DIRECTORY action
  FILES BroadcastPose.action
)


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs # Or other packages containing msgs
)


catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    actionlib_msgs
    cv_bridge
    utils
    tf_conversions
    object_pose
  DEPENDS
    EIGEN3
    OpenCV
    PCL
)

###########
## Build ##
###########
set(${PROJECT_NAME}_SRC
    src/target.cpp
    src/scence.cpp
    src/match.cpp
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} 
                           ${PCL_INCLUDE_DIRS})
include_directories(include)
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRC})

add_executable(visual_pose src/main.cpp )
target_link_libraries(visual_pose ${OpenCV_LIBS} 
    ${catkin_LIBRARIES} ${PROJECT_NAME} realsense2 ${PCL_LIBRARIES})
