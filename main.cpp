#include <iostream>
#include "VoxelGrid.hpp"
#include "ModelReconstructor.hpp"
#include "VirtualSensor.h"

#include <Eigen/Core>

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
    Eigen::Vector3i resolution = Eigen::Vector3i::Ones () * 100; //num voxels in each dimension of TSDF
    Eigen::Vector3d size = Eigen::Vector3d::Ones () * 3.0;  //size of model in meters
    Eigen::Vector3d offset(2,1,0);
    float truncationDistance = 0.3;
    ModelReconstructor model(truncationDistance, resolution, size, offset, cameraIntrinsic, camResolution);

    for (int i=0; i<2; ++i){
        //LOAD FRAME
        sensor.ProcessNextFrame();
        float* depthMapArr = sensor.GetDepth();
        Eigen::MatrixXf depthMapf = Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(depthMapArr, depthRows, depthCols);
        Eigen::MatrixXd depthMap = depthMapf.cast<double>();
        Eigen::Matrix4d cameraPose = Eigen::Matrix4d::Identity();
        cameraPose = sensor.GetTrajectory().cast<double>();
        //cameraPose.col(3) = Eigen::Vector4d::Ones();

        //FUSE FRAME
        model.fuseFrame(depthMap,cameraPose);
        std::cout << cameraPose << std::endl;
    }
    model.printTSDF();

    std::cout << "Exiting!" << std::endl;
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




}
