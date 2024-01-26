cmake_minimum_required(VERSION 3.0.2)
project(analysis_mp_chaser)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
        tf
        los_keeper
        pcl_ros
        pcl_conversions
        )

catkin_package(
    LIBRARIES
        INCLUDE_DIRS include
        CATKIN_DEPENDS
)

include_directories( include ${catkin_INCLUDE_DIRS})

file(GLOB ANALYZER_MPCHASER_SRC src/analysis_mp_chaser/Analyzer.cpp)
file(GLOB ANALYZER_LOSKEEPER_SRC src/analysis_los_keeper/Analyzer.cpp)

add_executable(analyzer_mp_chaser test/main_airsim.cpp ${ANALYZER_MPCHASER_SRC})
target_link_libraries(analyzer_mp_chaser ${catkin_LIBRARIES})

add_executable(analyzer_los_keeper test/main_simulator.cpp ${ANALYZER_LOSKEEPER_SRC})
target_link_libraries(analyzer_los_keeper ${catkin_LIBRARIES})