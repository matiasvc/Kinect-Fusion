#include <iostream>
#include "VoxelGrid.hpp"
#include "ModelReconstructor.hpp"
#include "VirtualSensor.h"

#include <Eigen/Core>


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
    std::cout << cameraIntrinsic << std::endl;

    //INIT TSDF
    Eigen::Vector3i resolution = Eigen::Vector3i::Ones () * 100; //num voxels in each dimension of TSDF
    Eigen::Vector3d size = Eigen::Vector3d::Ones () * 2.0;  //size of model in meters
    Eigen::Vector3d offset(0,0,0); //camera pose is relative to this point
    float truncationDistance = 0.02;
    ModelReconstructor model(truncationDistance, resolution, size, offset, cameraIntrinsic, camResolution);
    ModelReconstructor model0(truncationDistance, resolution, size, offset, cameraIntrinsic, camResolution);
    ModelReconstructor model1(truncationDistance, resolution, size, offset, cameraIntrinsic, camResolution);

    std::ofstream camFile;
    camFile.open("cams.csv");

    for (int i=0; i<2; ++i){
        //LOAD FRAME
        for (int j=0; j<10; ++j) {
            sensor.ProcessNextFrame();
        }

        std::cout << sensor.GetCurrentColorFile() << std::endl;
        std::cout << sensor.GetCurrentDepthFile() << std::endl;

        float* depthMapArr = sensor.GetDepth();
        Eigen::MatrixXf depthMapf = Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(depthMapArr, depthRows, depthCols);
        Eigen::MatrixXd depthMap = depthMapf.cast<double>();

        Eigen::Matrix4d cameraExtrinsic = sensor.GetTrajectory().cast<double>();//Eigen::Matrix4d::Identity(); //

        //cameraExtrinsic.col(3) = Eigen::Vector4d(-1,0.5,1.5,1);

//
//        Eigen::Matrix3d rotx30;
//        rotx30 <<
//        1.0000,         0.,         0.,
//        0.,    0.8660,   -0.5000,
//        0.,    0.5000,    0.8660;


//        FUSE FRAME
        model.fuseFrame(depthMap, cameraExtrinsic);
        camFile << cameraExtrinsic << "\n";
        std::cout << cameraExtrinsic << std::endl;

//        if (i==0){
//            camFile << cameraExtrinsic << "\n";
//            std::cout << cameraExtrinsic << std::endl;
//
//            model0.fuseFrame(depthMap,cameraExtrinsic);
//            model0.writeTSDFToFile("TSDF0.csv");
//        }
//        if (i==0){
//            cameraExtrinsic.block(0,0,3,3) = rotx30; //Eigen::Matrix3d::Identity();
//            cameraExtrinsic.col(3) = Eigen::Vector4d(-1,0,-1,1);
//
//            camFile << cameraExtrinsic << "\n";
//            std::cout << cameraExtrinsic << std::endl;
//
//            model1.fuseFrame(depthMap,cameraExtrinsic);
//            model1.writeTSDFToFile("TSDF1.csv");
//        }

    }
    model.writeTSDFToFile("TSDF.csv");

    camFile.close();
    std::cout << "Exiting!" << std::endl;
    return 0;
}
