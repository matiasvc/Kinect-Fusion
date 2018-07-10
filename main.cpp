#include <iostream>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "VoxelGrid.hpp"
#include "ImplicitSurface.hpp"
#include "Raytracer.hpp"
#include "Pose.hpp"
#include "Vertex.hpp"

int main (int argc, char* argv[])
{

    std::string filenameIn = "./data/rgbd_dataset_freiburg1_xyz/";
    std::string filenameBaseOut = "mesh_";
    
    // load video
    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.init(filenameIn))
    {
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
        return -1;
    }
    
    // max sqrdDistance between point and neightbor in order to normal
    float edgeThresholdSqrd = 0.05f*0.05f; // 5cm apart
    
    while (sensor.processNextFrame())
    {
        // get vertices of current frame, vertices will be in CAMERA SPACE.
        std::vector<Vertex> vertices = GetVertices(sensor, edgeThresholdSqrd);


        
         // write mesh file
//         std::stringstream filenameOut;
//         filenameOut << "./mesh/" << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".obj";
//
//         if (!WriteToFile(vertices, filenameOut.str()))
//         {
//         std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
//         return -1;
//         }
    }

	return 0;
}
