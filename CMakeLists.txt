cmake_minimum_required (VERSION 3.0)
project (Kinect-Fusion)
set(CMAKE_CXX_STANDARD 11)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)


find_path(
        FREEIMAGE_INCLUDE_DIR
        NAMES FreeImage.h
        PATHS
        /usr/include
        /usr/local/include
        /sw/include
        /opt/local/include
        DOC "The directory where FreeImage.h resides")

# Find library files
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



add_executable (Kinect-Fusion main.cpp VoxelGrid.cpp ImplicitSurface.hpp Raytracer.cpp Raytracer.hpp ModelReconstructor.cpp ModelReconstructor.hpp FreeImageHelper.cpp FreeImageHelper.h VirtualSensor.h Eigen.h)
target_link_libraries (Kinect-Fusion Eigen3::Eigen freeimage)
