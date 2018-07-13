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
    Eigen::Vector3i resolution = Eigen::Vector3i::Ones() * 150; //num voxels in each dimension of TSDF
    Eigen::Vector3d size = Eigen::Vector3d::Ones() * 3.0;  //size of model in meters
    Eigen::Vector3d offset(1,1,0); //camera pose is relative to this point
    float truncationDistance = 0.08;
    ModelReconstructor model(truncationDistance, resolution, size, offset, cameraIntrinsic, camResolution);



    for (int i=0; i<5; ++i){
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

    std::cout << "Exiting!" << std::endl;
    return 0;
}
