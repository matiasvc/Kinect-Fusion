#include <iostream>

#include <Eigen/Core>

#include "Raytracer.hpp"

bool searchRay(VoxelGrid& voxelGrid, Eigen::Vector3d origin, Eigen::Vector3d ray, double& length)
{
	Eigen::Vector3d point = origin + ray*length;
	float pointValue = voxelGrid.getValueAtPoint(point);
	double voxelSize = voxelGrid.size/voxelGrid.resolution;

	while (pointValue > 0.0f) {
		length += ((double)pointValue);
		//length += 0.05;
		point = origin + ray*length;

		if (not voxelGrid.withinGrid(point)) { return false; }

		pointValue = voxelGrid.getValueAtPoint(point);
	}
	return true;
}


cv::Mat raytraceImage(VoxelGrid& voxelGrid, Pose cameraPose, Eigen::Matrix3d cameraIntrisic,
                      const unsigned int resolutionWidth, const unsigned int resolutionHeight)
{
	cv::Mat image = cv::Mat::zeros(resolutionHeight, resolutionWidth, CV_32F);

	double fx = cameraIntrisic(0, 0)*resolutionWidth;
	double fy = cameraIntrisic(1, 1)*resolutionHeight;
	double cx = cameraIntrisic(0, 2)*resolutionWidth;
	double cy = cameraIntrisic(1, 2)*resolutionHeight;

	Eigen::Vector3d origin = cameraPose.translation;

//	int u = 400;
//	int v = 800;

	for (int v = 0; v < resolutionHeight; ++v)
	{
		std::cout << v << std::endl;
		for (int u = 0; u < resolutionWidth; ++u)
		{
			double rayX = ((double)u - cx)/fx;
			double rayY = ((double)v - cy)/fy;
			Eigen::Vector3d ray(rayX, rayY, 1);
			ray.normalize();

			ray = cameraPose.transformVector(ray);

			double length;


			if (voxelGrid.projectRayToVoxelPoint(origin, ray, length) and // Does the ray hit the voxel grid
			    searchRay(voxelGrid, origin, ray, length)) // Does the ray hit a zero crossing
			{
				image.at<float>(u, v) = (float)length;
			}
			else
			{
				image.at<float>(u, v) = 0.0f;
			}
		}
	}

	double min, max;
	cv::minMaxLoc(image, &min, &max);
	image = image * (1/max);

	return image;
}
