#include <Eigen/Core>

#include "Raytracer.hpp"

Eigen::MatrixXd raytraceImage(VoxelGrid& voxelGrid, Eigen::Matrix4d& cameraPose, Eigen::Matrix3d cameraIntrisic,
                              const unsigned int resolutionWidth, const unsigned int resolutionHeight)
{
	Eigen::MatrixXd image(resolutionWidth, resolutionHeight);


	return image;
}
