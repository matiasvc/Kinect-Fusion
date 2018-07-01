cmake_minimum_required(VERSION 3.0)
project(Kinect-Fusion)

include_directories(${SDL2_INCLUDE_DIRS})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")
set(SOURCE_FILES main.cpp VoxelGrid.cpp Raytracer.cpp ImplicitSurface.hpp Pose.cpp)

add_executable(Kinect-Fusion ${SOURCE_FILES})
target_link_libraries(Kinect-Fusion Eigen3::Eigen)
target_link_libraries(Kinect-Fusion ${OpenCV_LIBS})
