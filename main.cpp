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
	Torus surface(Eigen::Vector3d::Ones()*1.5, 1.0, 0.20);
//	Sphere surface(Eigen::Vector3d::Ones()*1.5, 1.0);

	fillVoxelGrid(grid, surface);

	Eigen::Vector3d camEuler;
	camEuler << 0.0, 0.0, 0.00;
	Eigen::Vector3d camPos;
	camPos << 0.0, 0.0, -3.0;

	Eigen::Matrix3d K;
	K << 0.8203125, 0.0,     0.5,
	     0.0,       1.09375, 0.5,
	     0.0,       0.0,     1.0;

	const unsigned int resolutionWidth = 640;
	const unsigned int resolutionHeight = 480;

	cv::Mat depthImage = cv::Mat::zeros(resolutionHeight, resolutionWidth, CV_32F);
	cv::Mat normalMap = cv::Mat::zeros(resolutionHeight, resolutionWidth, CV_32FC3);

	const double posDelta = 0.1;
	const double rotDelta = 0.05;

	while (cv::waitKey(1) != 27)
	{
		Pose camPose = Pose::PoseFromEuler(camEuler, camPos);
		raytraceImage(grid, camPose, K, resolutionWidth, resolutionHeight,
		              6.0, 1e-3, depthImage, normalMap);

		double min, max;
		cv::minMaxLoc(depthImage, &min, &max);

		cv::Mat displayImg;
		normalMap.convertTo(displayImg, CV_8UC3, 127, 127);

		//cv::imshow("Display window", depthImage * (1/max));
		cv::imshow("Display window", displayImg);


		switch (cv::waitKey(0))
		{
			case 119: // W - Up
			{
				camPos.y() -= posDelta;
			} break;
			case 100: // D - Right
			{
				camPos.x() += posDelta;
			} break;
			case 115: // S - Down
			{
				camPos.y() += posDelta;
			} break;
			case 97: // A - Left
			{
				camPos.x() -= posDelta;
			} break;
			case 113: // Q - Forewards
			{
				camPos.z() += posDelta;
			} break;
			case 101: // E - Backwards
			{
				camPos.z() -= posDelta;
			} break;

			case 105: // I - X+
			{
				camEuler.x() += rotDelta;
			} break;
			case 106: // J - Y-
			{
				camEuler.y() -= rotDelta;
			} break;
			case 107: // K - X-
			{
				camEuler.x() -= rotDelta;
			} break;
			case 108: // L - Y+
			{
				camEuler.y() += rotDelta;
			} break;
			case 117: // U - Z+
			{
				camEuler.z() += rotDelta;
			} break;
			case 111: // O - Z-
			{
				camEuler.z() -= rotDelta;
			} break;
		}
	}

	return 0;
}
