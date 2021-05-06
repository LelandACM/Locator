cmake_minimum_required(VERSION 3.0.0)
project(XINGSONG_DRIVER VERSION 0.1.0)

include(CTest)
enable_testing()

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_BUILD_TYPE Debug )
if( CMAKE_BUILD_TYPE STREQUAL "Release")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -fPIC")
else()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -std=c++11 -fPIC")
endif()

file(GLOB_RECURSE HEADER_FILES *.h *.hpp)

find_package(Boost REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${Boost_INCLUDE_DIR})

find_package(Boost COMPONENTS thread system REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${Boost_INCLUDE-_DIR})


add_executable(xs_driver main.cpp hins/xingsong_driver.cpp hins/laser_data_receiver.cpp)

target_link_libraries(xs_driver ${Boost_LIBRARIES})
