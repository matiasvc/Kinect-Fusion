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
	VoxelGrid grid(128, 3.0);
	Torus surface(Eigen::Vector3d::Ones()*1.5, 1.0, 0.40);

	fillVoxelGrid(grid, surface);

	Eigen::Vector3d camEuler;
	camEuler << 0.5, 0.1, 0.03;
	Eigen::Vector3d camPos;
	camPos << 1.5, 4.0, -3.0;
	Pose camPose = Pose::PoseFromEuler(camEuler, camPos);

	Eigen::Matrix3d K;
	K << 1.5, 0.0, 0.5,
	     0.0, 1.5, 0.5,
	     0.0, 0.0, 1.0;

	cv::Mat mat = raytraceImage(grid, camPose, K, 512, 512);
//	cv::Mat mat = cv::Mat::zeros(512, 512, CV_32F);

	cv::Mat grayImage;
	mat.convertTo(grayImage, CV_8UC3, 255.0);

	cv::imwrite("image.png", grayImage);

	cv::namedWindow("Display Window", cv::WINDOW_AUTOSIZE);
	cv::imshow("Display window", mat);

	cv::waitKey(0);
    
    
    //////////////////////////////////////////////////////////////////////////
    /** isakrs' part. Geting vertices with normals in camera space. **/
    
    std::string filenameIn = "./data/rgbd_dataset_freiburg1_xyz/";
    std::string filenameBaseOut = "mesh_";
    
    // load video
    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.Init(filenameIn))
    {
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
        return -1;
    }
    
    // max sqrdDistance between point and neightbor in order to normal
    float edgeThresholdSqrd = 0.05f*0.05f; // 5cm apart
    
    while (sensor.ProcessNextFrame())
    {
        // get vertices of current frame, vertices will be in CAMERA SPACE.
        std::vector<Vertex> vertices = GetVertices(sensor, edgeThresholdSqrd);
        
         // write mesh file
         std::stringstream filenameOut;
         filenameOut << "./mesh/" << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".obj";
         
         if (!WriteToFile(vertices, filenameOut.str()))
         {
         std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
         return -1;
         }
    }

	return 0;
}
