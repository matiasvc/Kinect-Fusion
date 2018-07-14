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
	unsigned int resolution = 100; //num voxels in each dimension of TSDF
    double size = 3.0;  //size of model in meters
    float truncationDistance = 0.08;
    ModelReconstructor model(truncationDistance, resolution, size, cameraIntrinsic, camResolution);




    for (int i=0; i<2; ++i){
        //LOAD FRAME
        for (int j=0; j<5; ++j) {
            sensor.ProcessNextFrame();
        }
        float* depthMapArr = sensor.GetDepth();
        Eigen::MatrixXf depthMapf = Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(depthMapArr, depthRows, depthCols);
        Eigen::MatrixXd depthMap = depthMapf.cast<double>();
        Eigen::Matrix4d cameraExtrinsic = sensor.GetTrajectory().cast<double>();//Eigen::Matrix4d::Identity(); //

        //FUSE FRAME
        model.fuseFrame(depthMap, cameraExtrinsic);
    }
    model.writeTSDFToFile("TSDF.csv");


    //RAYCAST
    std::cout << "Starting raycast" << std::endl;

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



    VoxelGrid grid = *model.getModel();
    std::cout << "Retrieved model" << std::endl;

    cv::namedWindow("disp window", cv::WINDOW_AUTOSIZE);
    while (cv::waitKey(1) != 27)
    {
        Pose camPose = Pose::PoseFromEuler(camEuler, camPos);
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
    std::cout << "Exiting!" << std::endl;
    return 0;
}
