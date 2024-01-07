cmake_minimum_required(VERSION 3.0.2)
project(analysis_mp_chaser)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
        tf
)

catkin_package(
    LIBRARIES
        INCLUDE_DIRS include
        CATKIN_DEPENDS
)

include_directories( include ${catkin_INCLUDE_DIRS})

file(GLOB ANALYZER_SRC src/analysis_mp_chaser/Analyzer.cpp)

add_executable(analyzer test/main_airsim.cpp ${ANALYZER_SRC})
target_link_libraries(analyzer ${catkin_LIBRARIES})