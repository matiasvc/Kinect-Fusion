#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include "VoxelGrid.hpp"
#include "Pose.hpp"

#ifndef KINECT_FUSION_RAYTRACER_HPP
#define KINECT_FUSION_RAYTRACER_HPP

void raytraceImage(VoxelGrid& voxelGrid, Pose cameraPose, Eigen::Matrix3d cameraIntrisic,
                      unsigned int resolutionWidth, unsigned int resolutionHeight,
                      const double stepSizeVoxel, const double epsilon,
                      cv::Mat& depthImage, cv::Mat& normalImage);

#endif //KINECT_FUSION_RAYTRACER_HPP
