cmake_minimum_required(VERSION 2.8)
project(ClassicalControl)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(classicalcontrol INTERFACE)
target_include_directories(classicalcontrol INTERFACE ${PROJECT_SOURCE_DIR}/include)

add_executable(main src/main.cpp)
target_link_libraries(main classicalcontrol ${EIGEN3_LIBS})
