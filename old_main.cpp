#pragma once

#include <iostream>
#include <fstream>

#include "Eigen.h"
#include "VirtualSensor.h"
#include "SimpleMesh.h"
#include "ProcrustesAligner.h"
#include "PointCloud.h"

#define PROJECT 1

#ifndef PROJECT
#	include "ICPOptimizer.h"
#else
	#include "KinectOptimizer.h"
#endif

#define USE_POINT_TO_PLANE	1

#define RUN_PROCRUSTES		0
#define RUN_SHAPE_ICP		0
#define RUN_SEQUENCE_ICP	0
#define RUN_KINECT_FUSION	0	// Projective Association
#define RUN_KINECT_FUSION_v2 0	// ICP 
#define RUN_KINECT_FUSION_v3 0	// Projective Association + Analytical Derivatives
#define RUN_KINECT_FUSION_v4 1	// Projective Association + Coarse-to-Fine

#ifndef PROJECT
 
void debugCorrespondenceMatching() {
	// Load the source and target mesh.
	const std::string filenameSource = "./data/bunny/bunny_part1.off";
	const std::string filenameTarget = "./data/bunny/bunny_part2_trans.off";

	SimpleMesh sourceMesh;
	if (!sourceMesh.loadMesh(filenameSource)) {
		std::cout << "Mesh file wasn't read successfully." << std::endl;
		return;
	}

	SimpleMesh targetMesh;
	if (!targetMesh.loadMesh(filenameTarget)) {
		std::cout << "Mesh file wasn't read successfully." << std::endl;
		return;
	}

	PointCloud source{ sourceMesh };
	PointCloud target{ targetMesh };
	
	// Search for matches using FLANN.
	std::unique_ptr<NearestNeighborSearch> nearestNeighborSearch = std::make_unique<NearestNeighborSearchFlann>();
	nearestNeighborSearch->setMatchingMaxDistance(0.0001f);
	nearestNeighborSearch->buildIndex(target.getPoints());
	auto matches = nearestNeighborSearch->queryMatches(source.getPoints());

	// Visualize the correspondences with lines.
	SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, Matrix4f::Identity());
	auto sourcePoints = source.getPoints();
	auto targetPoints = target.getPoints();

	for (unsigned i = 0; i < 100; ++i) { // sourcePoints.size()
		const auto match = matches[i];
		if (match.idx >= 0) {
			const auto& sourcePoint = sourcePoints[i];
			const auto& targetPoint = targetPoints[match.idx];
			resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::cylinder(sourcePoint, targetPoint, 0.002f, 2, 15), resultingMesh, Matrix4f::Identity());
		}
	}

	resultingMesh.writeMesh("./results/correspondences.off");
}

int alignBunnyWithProcrustes() {
	// Load the source and target mesh.
	const std::string filenameSource = "./data/bunny/bunny.off";
	const std::string filenameTarget = "./data/bunny/bunny_trans.off";

	SimpleMesh sourceMesh;
	if (!sourceMesh.loadMesh(filenameSource)) {
		std::cout << "Mesh file wasn't read successfully." << std::endl;
		return -1;
	}

	SimpleMesh targetMesh;
	if (!targetMesh.loadMesh(filenameTarget)) {
		std::cout << "Mesh file wasn't read successfully." << std::endl;
		return -1;
	}

	// Fill in the matched points: sourcePoints[i] is matched with targetPoints[i].
	std::vector<Vector3f> sourcePoints = { 
		{ -0.0106867f, 0.179756f, -0.0283248f }, // left ear
		{ -0.0639191f, 0.179114f, -0.0588715f }, // right ear
		{ 0.0590575f, 0.066407f, 0.00686641f }, // tail
		{ -0.0789843f, 0.13256f, 0.0519517f } // mouth
	};
	std::vector<Vector3f> targetPoints = { 
		{ -0.02744f, 0.179958f, 0.00980739f }, // left ear
		{ -0.0847672f, 0.180632f, -0.0148538f }, // right ear
		{ 0.0544159f, 0.0715162f, 0.0231181f }, // tail
		{ -0.0854079f, 0.10966f, 0.0842135f } // mouth
	};
		
	// Estimate the pose from source to target mesh with Procrustes alignment.
	ProcrustesAligner aligner;
	Matrix4f estimatedPose = aligner.estimatePose(sourcePoints, targetPoints);

	// Visualize the resulting joined mesh. We add triangulated spheres for point matches.
	SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, estimatedPose);
	for (const auto& sourcePoint : sourcePoints) {
		resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::sphere(sourcePoint, 0.002f), resultingMesh, estimatedPose);
	}
	for (const auto& targetPoint : targetPoints) {
		resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::sphere(targetPoint, 0.002f), resultingMesh, Matrix4f::Identity());
	}
	resultingMesh.writeMesh("./results/bunny_procrustes.off");

	return 0;
}

int alignBunnyWithICP() {
	// Load the source and target mesh.
	const std::string filenameSource = "./data/bunny/bunny_part1.off";
	const std::string filenameTarget = "./data/bunny/bunny_part2_trans.off";

	SimpleMesh sourceMesh;
	if (!sourceMesh.loadMesh(filenameSource)) {
		std::cout << "Mesh file wasn't read successfully." << std::endl;
		return -1;
	}

	SimpleMesh targetMesh;
	if (!targetMesh.loadMesh(filenameTarget)) {
		std::cout << "Mesh file wasn't read successfully." << std::endl;
		return -1;
	}

	// Estimate the pose from source to target mesh with ICP optimization.
	ICPOptimizer optimizer;
	optimizer.setMatchingMaxDistance(0.0003f);
	if (USE_POINT_TO_PLANE) {
		optimizer.usePointToPlaneConstraints(true);
		optimizer.setNbOfIterations(10);
	}
	else {
		optimizer.usePointToPlaneConstraints(false);
		optimizer.setNbOfIterations(20);
	}

	PointCloud source{ sourceMesh };
	PointCloud target{ targetMesh };

	Matrix4f estimatedPose = optimizer.estimatePose(source, target);
	
	// Visualize the resulting joined mesh. We add triangulated spheres for point matches.
	SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, estimatedPose);
	resultingMesh.writeMesh("./results/bunny_icp.off");

	return 0;
}

int reconstructRoom() {
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
	PointCloud target{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };
	
	// Setup the optimizer.
	ICPOptimizer optimizer;
	//KinectFusionOptimizer optimizer;

	optimizer.setMatchingMaxDistance(0.1f);
	if (USE_POINT_TO_PLANE) {
		optimizer.usePointToPlaneConstraints(true);
		optimizer.setNbOfIterations(10);
	}
	else {
		optimizer.usePointToPlaneConstraints(false);
		optimizer.setNbOfIterations(20);
	}

	// We store the estimated camera poses.
	std::vector<Matrix4f> estimatedPoses;
	Matrix4f currentCameraToWorld = Matrix4f::Identity();
	estimatedPoses.push_back(currentCameraToWorld.inverse());

	int i = 0;
	const int iMax = 3;
	while (sensor.processNextFrame() && i <= iMax) {

		float* depthMap = sensor.getDepth();
		Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
		Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();

		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		PointCloud source{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight(), 8 };  // Downsample(Originallly): 8
		currentCameraToWorld = optimizer.estimatePose(source, target, currentCameraToWorld);
		
		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		if (true ){//i % 5 == 0) {
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
		
		i++;
	}

	return 0;
}

#endif

int kinectFusion() {
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
	PointCloud target{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };

	// Setup the optimizer.
	KinectFusionOptimizer optimizer;

	// Setup the optimizer's parameters
	optimizer.setMatchingMaxDistance(0.1f);
	optimizer.usePointToPlaneConstraints(true);	// INFO: Unnecessary, hard-coded to do so anyways
	optimizer.setNbOfIterations(20);

	// We store the estimated camera poses.		INFO: Actually we don't have to unless we compare against groundtruth...
	std::vector<Matrix4f> estimatedPoses;
	Matrix4f currentCameraToWorld = Matrix4f::Identity();
	/*const Vector3f groundTrtTrans = Vector3f(1.3563, 0.6305, 1.6380);
	Quaternionf groundTrtRot = Quaternionf(0.6132, 0.5962, -0.3311, -0.3986);
	groundTrtRot.normalize();
	currentCameraToWorld.block(0, 0, 3, 3) = groundTrtRot.toRotationMatrix();
	currentCameraToWorld.block(0, 3, 3, 1) = groundTrtTrans;*/


	estimatedPoses.push_back(currentCameraToWorld.inverse());

	// Create variables to store old values of depth frame and normals map
	unsigned int depthFrameSize = sensor.getDepthImageWidth()* sensor.getDepthImageHeight();
	float* prev_depthMap = new float[depthFrameSize]; //target depth map

	// Initialize the previous frame variables
	std::copy(sensor.getDepth(), sensor.getDepth() + depthFrameSize, prev_depthMap);
	std::vector<Vector3f> prevNormals(target.getRawNormals());

	std::cout << "Target points size: " << target.getPoints().size() << std::endl;
	std::cout << "Target normals size: " << target.getRawNormals().size() << std::endl;

	//Matrix4f cumPose = Matrix4f::Identity();

	int i = 0;
	const int iMax = 20;
	while (i < iMax + 1 && sensor.processNextFrame()) {

		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		PointCloud source{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };  // Downsample(Originallly): 8
		currentCameraToWorld = optimizer.estimatePose(source, prev_depthMap, prevNormals, sensor, currentCameraToWorld /* T_g,k-1 */);	// = T_g,k

		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();	// INFO: Originally currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		//getchar();

		if (true) {//i % 5 == 0) {
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
		prevNormals = source.getRawNormals();
		std::copy(sensor.getDepth(), sensor.getDepth() + depthFrameSize, prev_depthMap);
		
		//currentCameraToWorld = Matrix4f::Identity();

		i++;
	}


	delete prev_depthMap;

	std::cout << "Done!" << std::endl;

	return 0;
}

#ifndef PROJECT
int kinectFusion_v2() {
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
	PointCloud target{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight()};

	// Setup the optimizer.
	KinectFusionOptimizer_v2 optimizer;

	// Setup the optimizer's parameters
	optimizer.setMatchingMaxDistance(0.1f);
	optimizer.usePointToPlaneConstraints(true);	// INFO: Unnecessary, hard-coded to do so anyways
	optimizer.setNbOfIterations(20);

	// We store the estimated camera poses.		INFO: Actually we don't have to unless we compare against groundtruth...
	std::vector<Matrix4f> estimatedPoses;
	Matrix4f currentCameraToWorld = Matrix4f::Identity();
	/*const Vector3f groundTrtTrans = Vector3f(1.3563, 0.6305, 1.6380);
	Quaternionf groundTrtRot = Quaternionf(0.6132, 0.5962, -0.3311, -0.3986);
	groundTrtRot.normalize();
	currentCameraToWorld.block(0, 0, 3, 3) = groundTrtRot.toRotationMatrix();
	currentCameraToWorld.block(0, 3, 3, 1) = groundTrtTrans;
	const Matrix4f initialPose = currentCameraToWorld;*/
	
	estimatedPoses.push_back(currentCameraToWorld.inverse());
	
	int i = 0;
	const int iMax = 5;
	while (i < iMax + 1 && sensor.processNextFrame()) {

		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		PointCloud source{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() , 8 };  // Downsample(Originallly): 8
		currentCameraToWorld = optimizer.estimatePose(source, target, currentCameraToWorld /*Matrix4f::Identity() /* T_g,k-1 */);	// = T_g,k
																																		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();	// INFO: Originally currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		//getchar();

		if (true) {//i % 5 == 0) {
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
		target = source;
		i++;
	}

	std::cout << "Done!" << std::endl;

	return 0;
}
#endif

int kinectFusion_v3() {
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
	PointCloud target{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };

	// Setup the optimizer.
	KinectFusionOptimizer_v3 optimizer;

	// Setup the optimizer's parameters
	optimizer.setMatchingMaxDistance(0.1f);
	optimizer.usePointToPlaneConstraints(true);	// INFO: Unnecessary, hard-coded to do so anyways
	optimizer.setNbOfIterations(25);

	// We store the estimated camera poses.		INFO: Actually we don't have to unless we compare against groundtruth...
	std::vector<Matrix4f> estimatedPoses;
	Matrix4f currentCameraToWorld = Matrix4f::Identity();

	/*const Vector3f initialPose_Trans = Vector3f(1.3563, 0.6305, 1.6380);
	const Vector3f poseTrans = Vector3f(1.3543, 0.6306, 1.6360);
	Quaternionf initialPose_Rot = Quaternionf(0.6132, 0.5962, -0.3311, -0.3986);
	Quaternionf poseRot = Quaternionf(0.6129, 0.5966, -0.3316, -0.3980);
	initialPose_Rot.normalize();
	poseRot.normalize();
	Matrix4f initialPose;
	initialPose.block(0, 0, 3, 3) = initialPose_Rot.toRotationMatrix();
	initialPose.block(0, 3, 3, 1) = initialPose_Trans;
	Matrix4f poseUpdate;
	poseUpdate.block(0, 0, 3, 3) = poseRot.toRotationMatrix();
	poseUpdate.block(0, 3, 3, 1) = poseTrans;
		
	currentCameraToWorld = initialPose.inverse()*poseUpdate;
	std::cout << "initialPose: " << initialPose << std::endl;
	std::cout << "poseUpdate: " << poseUpdate << std::endl;
	std::cout << "initialPose inverse: " << initialPose.inverse() << std::endl; // This is ill-conditioned!!
	std::cout << "currentCameraToWorld: " << currentCameraToWorld << std::endl;*/
	

	estimatedPoses.push_back(currentCameraToWorld.inverse());

	// Create variables to store old values of depth frame and normals map
	unsigned int depthFrameSize = sensor.getDepthImageWidth()* sensor.getDepthImageHeight();
	float* prev_depthMap = new float[depthFrameSize]; //target depth map

													  // Initialize the previous frame variables
	std::copy(sensor.getDepth(), sensor.getDepth() + depthFrameSize, prev_depthMap);
	std::vector<Vector3f> prevNormals(target.getRawNormals());

	std::cout << "Target points size: " << target.getPoints().size() << std::endl;
	std::cout << "Target normals size: " << target.getRawNormals().size() << std::endl;

	//Matrix4f cumPose = Matrix4f::Identity();

	int i = 0;
	const int iMax = 6;
	while (i < iMax && sensor.processNextFrame()) {

		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		PointCloud source{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };  // Downsample(Originallly): 8
		currentCameraToWorld = optimizer.estimatePose(source, prev_depthMap, prevNormals, sensor, currentCameraToWorld /* T_g,k-1 */);	// = T_g,k

																																		// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();	// INFO: Originally currentCameraToWorld.inverse();
		std::cout << "Current camera pose: " << std::endl << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		//getchar();

		if (true) {//i % 5 == 0) {
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
		prevNormals = source.getRawNormals();
		std::copy(sensor.getDepth(), sensor.getDepth() + depthFrameSize, prev_depthMap);

		//currentCameraToWorld = Matrix4f::Identity();

		i++;
	}


	delete prev_depthMap;

	std::cout << "Done!" << std::endl;

	return 0;
}

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
	//PointCloud target{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };

	// Setup the optimizer.
	KinectFusionOptimizer optimizer;

	// Setup the optimizer's parameters
	optimizer.setMatchingMaxDistance(0.1f);
	optimizer.usePointToPlaneConstraints(true);	// INFO: Unnecessary, hard-coded to do so anyways
	optimizer.setNbOfIterations(8);

	// We store the estimated camera poses.		INFO: Actually we don't have to unless we compare against groundtruth...
	std::vector<Matrix4f> estimatedPoses;
	Matrix4f currentCameraToWorld = Matrix4f::Identity();

	estimatedPoses.push_back(currentCameraToWorld.inverse());

	// Create variables to store old values of depth frame and normals map
	unsigned int depthFrameSize = sensor.getDepthImageWidth()* sensor.getDepthImageHeight();
	float* prev_depthMap = new float[depthFrameSize]; //target depth map

													  // Initialize the previous frame variables
	std::copy(sensor.getDepth(), sensor.getDepth() + depthFrameSize, prev_depthMap);
	//std::vector<Vector3f> prevNormals(target.getRawNormals());

	//std::cout << "Target points size: " << target.getPoints().size() << std::endl;
	//std::cout << "Target normals size: " << target.getRawNormals().size() << std::endl;

	//Matrix4f cumPose = Matrix4f::Identity();

	std::vector<int> iterNumbers{10,5,4,2,1}; // Bottom Pyramid to Top Pyramid Level = Fine to Corse(left->right)

	if ( sensor.getDepthImageWidth() % (int)pow(2, iterNumbers.size()-1) != 0 || sensor.getDepthImageHeight() % (int)pow(2, iterNumbers.size()-1) != 0)
	{
		std::cout << "Error: Invalid pyramid level size for the current depth frame!" << std::endl;
		return -1;
	}


	const Matrix4f zeroPose = sensor.getTrajectory();

	int i = 0;
	const int iMax = 6;
	while (i < iMax && sensor.processNextFrame()) {

		std::cout << "Current Frame #: " << i << std::endl;
		// Estimate the current camera pose from source to target mesh with ICP optimization.
		// We downsample the source image to speed up the correspondence matching.
		PointCloud source{ sensor.getDepth(), sensor.getDepthIntrinsics(), sensor.getDepthExtrinsics(), sensor.getDepthImageWidth(), sensor.getDepthImageHeight() };  // Downsample(Originallly): 8
		currentCameraToWorld = optimizer.estimatePoseInPyramid(sensor, prev_depthMap, iterNumbers, currentCameraToWorld /* T_g,k-1 */);	// = T_g,k
																										// Invert the transformation matrix to get the current camera pose.
		Matrix4f currentCameraPose = currentCameraToWorld.inverse();	// INFO: Originally currentCameraToWorld.inverse();
		std::cout << "Current calculated camera pose: " << std::endl << currentCameraPose << std::endl;
		estimatedPoses.push_back(currentCameraPose);

		//const Matrix4f cur_GrndTrthPose = sensor.getTrajectory()*zeroPose.inverse();

		//getchar();

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
		//prevNormals = source.getRawNormals();
		std::copy(sensor.getDepth(), sensor.getDepth() + depthFrameSize, prev_depthMap);

		//currentCameraToWorld = Matrix4f::Identity();

		i++;
	}


	delete prev_depthMap;

	std::cout << "Done!" << std::endl;

	return 0;
}


int main() {
	int result = -1;

	/*if (RUN_PROCRUSTES)
		result = alignBunnyWithProcrustes();
	else if (RUN_SHAPE_ICP)
		result = alignBunnyWithICP();
	else if (RUN_SEQUENCE_ICP)
		result = reconstructRoom();
	else if (RUN_KINECT_FUSION)
		result = kinectFusion();
	else if (RUN_KINECT_FUSION_v2)
		result = kinectFusion_v2();
	else if (RUN_KINECT_FUSION_v3)
		result = kinectFusion_v3();
	else if (RUN_KINECT_FUSION_v4)
		result = kinectFusion_v4();*/

	if (RUN_KINECT_FUSION)
		result = kinectFusion();
	else if (RUN_KINECT_FUSION_v2)
		return result; // result = kinectFusion_v2();
	else if (RUN_KINECT_FUSION_v3)
		result = kinectFusion_v3();
	else if (RUN_KINECT_FUSION_v4)
		result = kinectFusion_v4();
	
	return result;
}
