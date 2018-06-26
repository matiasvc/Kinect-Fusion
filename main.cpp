#include <iostream>
#include "VoxelGrid.hpp"
#include "ModelReconstructor.hpp"
#include "VirtualSensor.h"

#include <Eigen/Core>

int main (int argc, char* argv[])
{
	Eigen::Vector3i resolution = Eigen::Vector3i::Ones () * 17;
	Eigen::Vector3d size = Eigen::Vector3d::Ones () * 1.0;  //todo
	Eigen::Vector3d offset = Eigen::Vector3d::Zero ();
	float truncationDistance = 1;




    std::string filenameIn = "/home/opeide/TUM/3D scanning/Kinect-Fusion/data/rgbd_dataset_freiburg1_xyz/";

    // load video
    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.Init(filenameIn))
    {
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
        return -1;
    }

    sensor.ProcessNextFrame();


    Eigen::Matrix3d cameraIntrinsic = sensor.GetDepthIntrinsics().cast<double>();
    Eigen::Vector2i camResolution(sensor.GetDepthImageWidth(), sensor.GetDepthImageWidth());
    ModelReconstructor fuser(truncationDistance, resolution, size, offset, cameraIntrinsic, camResolution);




    std::cout << "Completed!" << std::endl;
    return 0;


//	double fx = 1;
//    double fy = 1;
//    double u0 = 1;
//    double v0 = 1;
//    Eigen::Matrix3d cameraIntrinsic;
//    cameraIntrinsic << 	fx, 0, u0,
//						0, fy, v0,
//						0, 0, 1;
//
//    Eigen::Vector2i camResolution(3,3);
//    Eigen::Matrix3d depthMap1 = Eigen::Matrix3d::Ones() * 6;
//    Eigen::Matrix3d depthMap2 = Eigen::Matrix3d::Ones() * 7;
//
//    Eigen::Matrix4d cameraPose1 = Eigen::Matrix4d::Identity();
//    Eigen::Matrix4d cameraPose2 = Eigen::Matrix4d::Identity();
//    cameraPose2(1,3) = resolution.y();


    //Fuse depthmaps
//    sensor.ProcessNextFrame();
//
//    float* depthMapArr = sensor.GetDepth();
//    Eigen::MatrixXd depthMap = Eigen::Map<Eigen::MatrixXf>(depthMapArr, camResolution(1), camResolution(0)).cast<Eigen::MatrixXd>();
//
//    Eigen::Matrix4d cameraPose = Eigen::Matrix4d::Ones();//sensor.GetTrajectory().cast<double>();
//
//    fuser.fuseFrame(depthMap,cameraPose);

}
