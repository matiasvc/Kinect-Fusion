#pragma once

#include <iostream>
#include <fstream>

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

	// We store a first frame as a reference frame. All next frames are tracked relatively to the first frame.
	sensor.processNextFrame();

	// Setup the optimizer.
	KinectFusionOptimizer optimizer;

	// Setup the optimizer's parameters
	//optimizer.setNbOfIterations(8);
	std::vector<int> iterNumbers = std::vector<int>{ 10, 5, 4 };
	optimizer.setNbOfIterationsInPyramid(iterNumbers); // Bottom Pyramid to Top Pyramid Level = Fine to Corse(left->right)

	// Create variables to store old values of depth frame and normals map
	unsigned int depthFrameSize = sensor.getDepthImageWidth()* sensor.getDepthImageHeight();
	float* prev_depthMap = new float[depthFrameSize]; //target depth map

													  // Initialize the previous frame variables
	std::copy(sensor.getDepth(), sensor.getDepth() + depthFrameSize, prev_depthMap);

	if ( sensor.getDepthImageWidth() % (int)pow(2, iterNumbers.size()-1) != 0 || sensor.getDepthImageHeight() % (int)pow(2, iterNumbers.size()-1) != 0)
	{
		std::cout << "Error: Invalid pyramid level size for the current depth frame!" << std::endl;
		return -1;
	}
	
	const Matrix4f zeroPose = sensor.getTrajectory();
	Matrix4f currentCameraToWorld = Matrix4f::Identity();

	int i = 0;
	const int iMax = 6;
	while (i < iMax && sensor.processNextFrame()) {

		std::cout << "Current Frame #: " << i << std::endl;
		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		PointCloud source{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };  // Downsample(Originallly): 8
		currentCameraToWorld = optimizer.estimatePoseInPyramid(sensor, prev_depthMap, currentCameraToWorld /* T_g,k-1 */);	// = T_g,k
																										// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();	// INFO: Originally currentCameraToWorld.inverse();
		std::cout << "Current calculated camera pose: " << std::endl << currentCameraPose << std::endl;

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
		std::copy(sensor.getDepth(), sensor.getDepth() + depthFrameSize, prev_depthMap);

		i++;
	}


	delete prev_depthMap;
	std::cout << "Done!" << std::endl;

	return 0;
}

int main() {
	int result = -1;

	result = kinectFusion_v4();
	
	return result;
}
