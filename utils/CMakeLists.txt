cmake_minimum_required(VERSION 3.1.3)
project(utils)

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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros message_runtime
#  DEPENDS system_lib
)
include_directories(include)

###########
## Build ##
###########

set(${PROJECT_NAME}_SRC
    src/utils.cpp
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

message("TTUTU")
message(${CATKIN_PACKAGE_INCLUDE_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp")
  #PATTERN "" EXCLUDE)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
