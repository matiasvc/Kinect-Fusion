cmake_minimum_required (VERSION 3.0)
project (Kinect-Fusion)
set(CMAKE_CXX_STANDARD 11)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable (Kinect-Fusion main.cpp VoxelGrid.cpp ImplicitSurface.hpp Raytracer.cpp Raytracer.hpp ModelReconstructor.cpp ModelReconstructor.hpp)
target_link_libraries (Kinect-Fusion Eigen3::Eigen)
