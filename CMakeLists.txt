cmake_minimum_required (VERSION 3.0)
project (Kinect-Fusion)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -O3")
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)



list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres Solver
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# FreeImage
find_package(FreeImage REQUIRED)
include_directories(${FREEIMAGE_INCLUDE_DIRS})

# Set c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -O3")

# Set header files 
set(
	HEADER_FILES
	ImplicitSurface.hpp
	SimpleMesh.h
	PointCloud.h
	KinectOptimizer.h
	VirtualSensor.h)

# Set source files
set(
	SOURCE_FILES 
	main.cpp
	FreeImageHelper.cpp)

# executable files
add_executable(Kinect-Fusion ${SOURCE_FILES} ${HEADER_FILES})

# target libraries 
target_link_libraries(Kinect-Fusion Eigen3::Eigen)
target_link_libraries(Kinect-Fusion ${OpenCV_LIBS})
target_link_libraries(Kinect-Fusion freeimage)
target_link_libraries(Kinect-Fusion ${CERES_LIBRARIES})
