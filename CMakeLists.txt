cmake_minimum_required(VERSION 3.20)
project(depthRender)

set(CMAKE_CXX_STANDARD 14)

find_package( Open3D  REQUIRED)

include_directories(${Open3D_INCLUDE_DIRS})
link_directories(${Open3D_LIBRARY_DIRS})


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(depthRender main.cpp  util.h util.cpp)
target_link_libraries(depthRender ${OpenCV_LIBS} ${Open3D_LIBRARIES} )
