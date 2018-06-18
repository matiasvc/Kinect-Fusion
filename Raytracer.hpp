
#include <Eigen/Eigen>

#include "VoxelGrid.hpp"

#ifndef KINECT_FUSION_RAYTRACER_HPP
#define KINECT_FUSION_RAYTRACER_HPP

Eigen::MatrixXd raytraceImage(VoxelGrid& voxelGrid, Eigen::Matrix4d& cameraPose, Eigen::Matrix3d cameraIntrisic,
                              unsigned int resolutionWidth, unsigned int resolutionHeight);

#endif //KINECT_FUSION_RAYTRACER_HPP
