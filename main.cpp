#pragma once

#include <iostream>
#include <fstream>

#include "ModelReconstructor.hpp"
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ImplicitSurface.hpp"
#include "Raytracer.hpp"
#include "Pose.hpp"

#include "Eigen.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "PointCloud.h"

#define PROJECT 1

#include "KinectOptimizer.h"

int kinectFusion_v4() {
	std::string filenameIn = "./data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "./results/mesh_";

	// Load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.init(filenameIn)) {
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	Eigen::Matrix3d cameraIntrinsic = sensor.getDepthIntrinsics().cast<double>();
	int depthCols = sensor.getDepthImageWidth();
	int depthRows = sensor.getDepthImageHeight();
	Eigen::Vector2i camResolution(depthCols, depthRows);

	//INIT TSDF
	unsigned int resolution = 300; //num voxels in each dimension of TSDF
	double size = 3.0;  //size of model in meters
	float truncationDistance = 0.08;
	ModelReconstructor model(truncationDistance, resolution, size, cameraIntrinsic, camResolution);

	// We store a first frame as an inital frame
	sensor.processNextFrame();

	// Setup the optimizer.
	KinectFusionOptimizer optimizer;

	// Setup the optimizer's parameters
	std::vector<int> iterNumbers = std::vector<int>{ 10, 5, 4 };
	optimizer.setNbOfIterationsInPyramid(iterNumbers); // Bottom Pyramid to Top Pyramid Level = Fine to Corse(left->right)

	// Create variables to store old values of depth frame and normals map
	unsigned int depthFrameSize = sensor.getDepthImageWidth()* sensor.getDepthImageHeight();
	float* prev_depthMap;// = new float[depthFrameSize]; //target depth map
	cv::Mat depthImage = cv::Mat::zeros(sensor.getDepthImageHeight(), sensor.getDepthImageWidth(), CV_32F);
	cv::Mat normalMap = cv::Mat::zeros(sensor.getDepthImageHeight(), sensor.getDepthImageWidth(), CV_32FC3);

	const Matrix4f zeroPose = sensor.getTrajectory(); // Matrix4f::Identity() in case of an unknown dataset
	Matrix4f currentCameraToWorld = zeroPose.inverse(); //Matrix4f::Identity(); //zeroPose.inverse();
	Matrix4d currentCameraPose = currentCameraToWorld.inverse().cast<double>(); //sensor.getTrajectory().cast<double>();
	float* depthMapArr = sensor.getDepth();
	Eigen::MatrixXd depthMap = Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(depthMapArr, depthRows, depthCols).cast<double>();

	//FUSE FRAME
	model.fuseFrame(depthMap, currentCameraPose);
	Pose curCamPose = Pose(currentCameraPose);//Pose::PoseFromEuler(camEuler, camPos);
    VoxelGrid grid = *model.getModel();
    Eigen::Matrix3d normalized_Intrinsics = sensor.getDepthIntrinsics().cast<double>();
    normalized_Intrinsics.row(0) = normalized_Intrinsics.row(0)/sensor.getDepthImageWidth();
    normalized_Intrinsics.row(1) = normalized_Intrinsics.row(1)/sensor.getDepthImageHeight();

    raytraceImage(grid, curCamPose, normalized_Intrinsics, sensor.getDepthImageWidth(), sensor.getDepthImageHeight(), 1.5, 1e-3, depthImage, normalMap);

	// Initialize the previous frame variables (Use this if you want to use the depth frame from the camera instead of the model)
	//std::copy(sensor.getDepth(), sensor.getDepth() + depthFrameSize, prev_depthMap);

    // This function was used if the created depthArray was continous as it is not guaranteed with opencv
    // However the way this matrix was created and changed doesn't lead to making it not continuous thus
    // we use a pointer to the underlying data instead
    /*std::vector<float> prev_depthArray;
    if (depthImage.isContinuous()) {
        prev_depthArray.assign((float*)depthImage.datastart, (float*)depthImage.dataend);
        std::cout << "Yes it is continuous!" << std::endl;
    } else {
        for (int i = 0; i < depthImage.rows; ++i) {
            prev_depthArray.insert(prev_depthArray.end(), depthImage.ptr<float>(i), depthImage.ptr<float>(i)+depthImage.cols);
        }
    }*/

   /* for(int iii = 0; iii <  sensor.getDepthImageWidth()*sensor.getDepthImageHeight(); iii++)
    {
        int row = iii/sensor.getDepthImageWidth();
        int col = iii%sensor.getDepthImageWidth();
        std::cout<< "Original value: "<< sensor.getDepth()[iii] << " opencv value :" <<  depthImage.at<float>(row, col) << "Vectorized value: "<< prev_depthArray[iii] << std::endl;
    }*/

    // Pointer to the previoud depth frame
    prev_depthMap = depthImage.ptr<float>(0);

	if ( sensor.getDepthImageWidth() % (int)pow(2, iterNumbers.size()-1) != 0 || sensor.getDepthImageHeight() % (int)pow(2, iterNumbers.size()-1) != 0)
	{
		std::cout << "Error: Invalid pyramid level size for the current depth frame!" << std::endl;
		return -1;
	}

	int i = 0;
	const int iMax = 60000;
	while (i < iMax && sensor.processNextFrame()) {

		std::cout << "Current Frame #: " << i << std::endl;
		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		currentCameraToWorld = optimizer.estimatePoseInPyramid(sensor, prev_depthMap, currentCameraToWorld /* T_g,k-1 */);	// = T_g,k
																										// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current calculated camera pose: " << std::endl << currentCameraPose << std::endl;

		Eigen::MatrixXd depthMap = Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(sensor.getDepth(), depthRows, depthCols).cast<double>();
		model.fuseFrame(depthMap, currentCameraPose.cast<double>());

		if ( i % 1 == 0) {//i % 5 == 0) {
				   // We write out the mesh to file for debugging.
			SimpleMesh currentDepthMesh{ sensor, currentCameraPose, 0.1f };
			SimpleMesh currentCameraMesh = SimpleMesh::camera(currentCameraPose, 0.0015f);
			SimpleMesh resultingMesh = SimpleMesh::joinMeshes(currentDepthMesh, currentCameraMesh, Matrix4f::Identity());

			std::stringstream ss;
			ss << filenameBaseOut << sensor.getCurrentFrameCnt() << ".off";
			if (!resultingMesh.writeMesh(ss.str())) {
				std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
				return -1;
			}
		}

		// Update the previous frame variables
		// std::copy(sensor.getDepth(), sensor.getDepth() + depthFrameSize, prev_depthMap);
		// TODO: MATIAS RAYCAST to prev_depthMap

		Pose curCamPose = Pose(currentCameraPose.cast<double>());//Pose::PoseFromEuler(camEuler, camPos);
		raytraceImage(*model.getModel(), curCamPose, normalized_Intrinsics,
                      sensor.getDepthImageWidth(), sensor.getDepthImageHeight(),
                      1.5, 1e-3, depthImage, normalMap);

		prev_depthMap = depthImage.ptr<float>(0);

		i++;
	}


	//delete prev_depthMap;
	std::cout << "Done!" << std::endl;

	return 0;
}

int main() {
	int result = -1;

	result = kinectFusion_v4();
	
	return result;
}
