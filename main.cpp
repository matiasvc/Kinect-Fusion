#include <iostream>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "VoxelGrid.hpp"
#include "ImplicitSurface.hpp"
#include "Raytracer.hpp"
#include "Pose.hpp"

int main (int argc, char* argv[])
{
	VoxelGrid grid(128, 3.0);
	Torus surface(Eigen::Vector3d::Ones()*1.5, 1.0, 0.40);

	fillVoxelGrid(grid, surface);

	Eigen::Vector3d camEuler;
	camEuler << 0.5, 0.1, 0.03;
	Eigen::Vector3d camPos;
	camPos << 1.5, 4.0, -3.0;
	Pose camPose = Pose::PoseFromEuler(camEuler, camPos);

	Eigen::Matrix3d K;
	K << 1.5, 0.0, 0.5,
	     0.0, 1.5, 0.5,
	     0.0, 0.0, 1.0;

	cv::Mat mat = raytraceImage(grid, camPose, K, 512, 512);
//	cv::Mat mat = cv::Mat::zeros(512, 512, CV_32F);

	cv::Mat grayImage;
	mat.convertTo(grayImage, CV_8UC3, 255.0);

	cv::imwrite("image.png", grayImage);

	cv::namedWindow("Display Window", cv::WINDOW_AUTOSIZE);
	cv::imshow("Display window", mat);

	cv::waitKey(0);

	return 0;
}
