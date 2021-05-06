cmake_minimum_required(VERSION 3.11)
project(generate_images)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS 
  CATKIN_DEPENDS
  DEPENDS
)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        set(CXX_FILESYSTEM_LIBRARIES "stdc++fs")
endif()

###########
## Build ##
###########
