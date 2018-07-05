cmake_minimum_required(VERSION 3.0)
project(Kinect-Fusion)

include_directories(${SDL2_INCLUDE_DIRS})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

# Find path: free image library
find_path(
        FREEIMAGE_INCLUDE_DIR
        NAMES FreeImage.h
        PATHS
        /usr/include
        /usr/local/include
        /sw/include
        /opt/local/include
        DOC "The directory where FreeImage.h resides")

# Find library files: free image library
find_library(
        FREEIMAGE_LIBRARY
        NAMES freeimage
        PATHS
        /usr/lib64
        /usr/lib
        /usr/local/lib64
        /usr/local/lib
        /sw/lib
        /opt/local/lib
        ${FREEIMAGE_ROOT_DIR}/lib
        DOC "The FreeImage library")

# Set c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11")

# Set source files
set(
	SOURCE_FILES 
	main.cpp 
	VoxelGrid.cpp 
	Raytracer.cpp 
	ImplicitSurface.hpp 
	Pose.cpp 
	FreeImageHelper.cpp)

# executable files
add_executable(Kinect-Fusion ${SOURCE_FILES})

# target libraries 
target_link_libraries(Kinect-Fusion Eigen3::Eigen)
target_link_libraries(Kinect-Fusion ${OpenCV_LIBS})
target_link_libraries(Kinect-Fusion freeimage)
