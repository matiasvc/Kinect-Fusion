cmake_minimum_required (VERSION 3.0)
project (Kinect-Fusion)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable (Kinect-Fusion main.cpp VoxelGrid.cpp ImplicitSurface.hpp Raytracer.cpp Raytracer.hpp)
target_link_libraries (Kinect-Fusion Eigen3::Eigen)
