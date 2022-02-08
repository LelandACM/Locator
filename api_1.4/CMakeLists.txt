cmake_minimum_required(VERSION 3.19)
project(testBoschCode)

set(CMAKE_CXX_STANDARD 14)
find_package(CURL REQUIRED)

include_directories(${CURL_INCLUDE_DIR})

add_executable(testBoschCode
        BinaryInterfaceClient.h
        BinaryInterfaceDummyLaserData.h
        BinaryInterfacePoseStruct.h
        BinaryInterfaceServer.h
        BinaryInterfaceStruct.h
        bosch.cpp
        bosch.h
        main.cpp)

target_link_libraries(testBoschCode ${CURL_LIBRARY})
