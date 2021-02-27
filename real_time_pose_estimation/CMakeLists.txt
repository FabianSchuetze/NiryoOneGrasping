cmake_minimum_required(VERSION 3.15)
project(textured_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(realsense2 REQUIRED)


catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  DEPENDS
    EIGEN3
    OpenCV
)

###########
## Build ##
###########
set(${PROJECT_NAME}_SRC
    src/Model.cpp
    src/Mesh.cpp
    src/ModelRegistration.cpp
    src/PnPProblem.cpp
    src/RobustMatcher.cpp
    src/Utils.cpp
    src/CsvReader.cpp
    src/CsvReader.cpp
)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )
include_directories(include)
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRC})

add_executable(detection src/main_detection.cpp )
target_link_libraries(detection ${OpenCV_LIBS} 
    ${catkin_LIBRARIES} ${PROJECT_NAME})
