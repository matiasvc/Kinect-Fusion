cmake_minimum_required(VERSION 3.0)
project(Kinect-Fusion)

include_directories(${SDL2_INCLUDE_DIRS})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# OpenCV
find_package(OpenCV REQUIRED)

# Ceres Solver
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# FreeImage
find_package(FreeImage REQUIRED)
include_directories(${FREEIMAGE_INCLUDE_DIRS})

# Set c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")

# Set header files 
set(
	HEADER_FILES
	ImplicitSurface.hpp
	KinectOptimizer.h
	VirtualSensor.h)

# Set source files
set(
	SOURCE_FILES 
	main.cpp 
	VoxelGrid.cpp 
	Raytracer.cpp  
	Pose.cpp 
	FreeImageHelper.cpp
	Vertex.cpp)

# executable files
add_executable(Kinect-Fusion ${SOURCE_FILES})

# target libraries 
target_link_libraries(Kinect-Fusion Eigen3::Eigen)
target_link_libraries(Kinect-Fusion ${OpenCV_LIBS})
target_link_libraries(Kinect-Fusion freeimage)
target_link_libraries(Kinect-Fusion ${CERES_LIBRARIES})
