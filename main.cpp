#include <iostream>
#include "VoxelGrid.hpp"
#include "ModelReconstructor.hpp"
#include "VirtualSensor.h"

#include <Eigen/Core>


//todo: x forveksles med y

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
    Eigen::Vector3i resolution = Eigen::Vector3i::Ones () * 70; //num voxels in each dimension of TSDF
    Eigen::Vector3d size = Eigen::Vector3d::Ones () * 5.0;  //size of model in meters
    Eigen::Vector3d offset(0,0,0); //camera pose is relative to this point
    float truncationDistance = 0.2;
    ModelReconstructor model(truncationDistance, resolution, size, offset, cameraIntrinsic, camResolution);



    for (int i=0; i<2; ++i){
        //LOAD FRAME
        for (int j=0; j<5; ++j) {
            sensor.ProcessNextFrame();
        }
        if (i ==0){
            continue;
        }

        float* depthMapArr = sensor.GetDepth();
        Eigen::MatrixXf depthMapf = Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(depthMapArr, depthRows, depthCols);
        Eigen::MatrixXd depthMap = depthMapf.cast<double>();
        Eigen::Matrix4d cameraPose = sensor.GetTrajectory().cast<double>();//Eigen::Matrix4d::Identity();
        cameraPose.col(3) += Eigen::Vector4d(3,3.5,1.5,0);

        //FUSE FRAME
        model.fuseFrame(depthMap,cameraPose);
        std::cout << cameraPose << std::endl;
    }
    model.writeTSDFToFile("TSDF.csv");

    std::cout << "Exiting!" << std::endl;
    return 0;
}
