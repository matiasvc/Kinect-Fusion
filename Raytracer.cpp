#include <Eigen/Eigen>

#include "Raytracer.hpp"

Eigen::MatrixXd raytraceImage(VoxelGrid& voxelGrid, Eigen::Matrix4d& cameraPose, Eigen::Matrix3d cameraIntrisic,
                              unsigned int resolutionWidth, unsigned int resolutionHeight)
{
	Eigen::Matrix<double, resolutionHeight, resolutionWidth> image;


	return image;
}
