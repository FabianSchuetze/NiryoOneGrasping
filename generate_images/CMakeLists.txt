cmake_minimum_required(VERSION 3.11)
project(generate_images)

set(CMAKE_CXX_STANDARD 17)
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

find_package(OpenCV REQUIRED PATHS /usr/local/lib/cmake/opencv4
             NO_DEFAULT_PATH)
set(OpenCV_INCLUDE_DIRS /usr/local/include/opencv4)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)


catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
    OpenCV
)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        set(CXX_FILESYSTEM_LIBRARIES "stdc++fs")
endif()

###########
## Build ##
###########
set(${PROJECT_NAME}_SRC
    src/io_helper_functions.cpp
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
include_directories(${REALSENSE2_INCLUDE_DIR})
include_directories(include)
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRC})

add_executable(DisplayImage src/main.cpp )
add_executable(WriteImagesToFile src/write_images_to_file.cpp)
target_link_libraries(DisplayImage ${OpenCV_LIBS} 
    ${catkin_LIBRARIES}
    ${CXX_FILESYSTEM_LIBRARIES})
target_link_libraries(WriteImagesToFile ${OpenCV_LIBS} 
    ${catkin_LIBRARIES} realsense2 ${PROJECT_NAME}
    ${CXX_FILESYSTEM_LIBRARIES})
