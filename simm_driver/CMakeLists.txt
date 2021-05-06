cmake_minimum_required(VERSION 3.0.0)
project(SIMM_DRIVER VERSION 0.1.0)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR})


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost COMPONENTS thread system REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR})

add_executable(simm_driver main.cpp data_filters.cpp pavo_driver.cpp utils.cpp)

target_link_libraries(simm_driver ${Boost_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

