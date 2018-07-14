#include <iostream>
#include "ModelReconstructor.hpp"
#include "VirtualSensor.h"
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ImplicitSurface.hpp"
#include "Raytracer.hpp"
#include "Pose.hpp"


//todo: x forveksles med y
//todo: camera pose between frames has a traslational offset? related to xy confusion
//todo: speed up local reconstruction
//todo: init with negative TSDF
//todo: fix half voxel offset


int main (int argc, char* argv[])
{
    // INIT CAMERA
	std::string filenameIn = "/home/opeide/TUM/3D scanning/Kinect-Fusion/data/rgbd_dataset_freiburg1_xyz/";
    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.Init(filenameIn))
    {
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
        return -1;
    }
    std::cout << "Virtual sensor initialized!" << std::endl;
    Eigen::Matrix3d cameraIntrinsic = sensor.GetDepthIntrinsics().cast<double>();
    int depthCols = sensor.GetDepthImageWidth();
    int depthRows = sensor.GetDepthImageHeight();
    Eigen::Vector2i camResolution(depthCols, depthRows);

    //INIT TSDF
	unsigned int resolution = 200; //num voxels in each dimension of TSDF
    double size = 3.0;  //size of model in meters
    float truncationDistance = 0.08;
    ModelReconstructor model(truncationDistance, resolution, size, cameraIntrinsic, camResolution);



    Eigen::Matrix4d camEx0;
    for (int i=0; i<2; ++i){
        //LOAD FRAME
        for (int j=0; j<5; ++j) {
            sensor.ProcessNextFrame();
        }
        float* depthMapArr = sensor.GetDepth();
        Eigen::MatrixXf depthMapf = Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(depthMapArr, depthRows, depthCols);
        Eigen::MatrixXd depthMap = depthMapf.cast<double>();
        Eigen::Matrix4d cameraExtrinsic = sensor.GetTrajectory().cast<double>();//Eigen::Matrix4d::Identity(); //

        if (i==0){
            camEx0 = cameraExtrinsic;
        }

        //FUSE FRAME
        model.fuseFrame(depthMap, cameraExtrinsic);
    }
    //model.writeTSDFToFile("TSDF.csv");


    //RAYCAST
    std::cout << "Starting raycast" << std::endl;

//    Eigen::Vector3d camEuler;
//    camEuler << 0.0, 0.0, 0.00;
//    Eigen::Vector3d camPos(camPose0.col(3).head(3));
//    camPos << 0.0, 0.0, -3.0;

	Pose camPose = Pose(camEx0);//Pose::PoseFromEuler(camEuler, camPos);


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

	const Eigen::Vector3d xVector(1,0,0);
	const Eigen::Vector3d yVector(0,1,0);
	const Eigen::Vector3d zVector(0,0,1);

    VoxelGrid grid = *model.getModel();
    std::cout << "Retrieved model" << std::endl;

    cv::namedWindow("disp window", cv::WINDOW_AUTOSIZE);
    while (cv::waitKey(1) != 27)
    {
        raytraceImage(grid, camPose, K, resolutionWidth, resolutionHeight,
                      1.5, 1e-3, depthImage, normalMap);

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
                camPose.translate(-yVector*posDelta);
            } break;
            case 100: // D - Right
            {
	            camPose.translate(xVector*posDelta);
            } break;
            case 115: // S - Down
            {
	            camPose.translate(yVector*posDelta);
            } break;
            case 97: // A - Left
            {
	            camPose.translate(-xVector*posDelta);
            } break;
            case 113: // Q - Forewards
            {
	            camPose.translate(zVector*posDelta);
            } break;
            case 101: // E - Backwards
            {
	            camPose.translate(-zVector*posDelta);
            } break;

            case 105: // I - X+
            {
                camPose.rotateEuler(xVector*rotDelta);
            } break;
            case 106: // J - Y-
            {
	            camPose.rotateEuler(-yVector*rotDelta);
            } break;
            case 107: // K - X-
            {
	            camPose.rotateEuler(-xVector*rotDelta);
            } break;
            case 108: // L - Y+
            {
	            camPose.rotateEuler(yVector*rotDelta);
            } break;
            case 117: // U - Z+
            {
	            camPose.rotateEuler(zVector*rotDelta);
            } break;
            case 111: // O - Z-
            {
	            camPose.rotateEuler(-zVector*rotDelta);
            } break;
        }
    }
    std::cout << "Exiting!" << std::endl;
    return 0;
}
