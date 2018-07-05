#include <iostream>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "VoxelGrid.hpp"
#include "ImplicitSurface.hpp"
#include "Raytracer.hpp"
#include "Pose.hpp"

// isakrs' include statements
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    // position stored as 4 floats (4th component is supposed to be 1.0)
    // (MINF, MINF, MINF, MINF) if it does not exist
    Vector4f position;
    
    // color stored as 4 unsigned char
    // (0, 0, 0, 0) if it does not exist
    Vector4uc color;
    
    // normal normalized, stored as 3 floats
    // (MINF, MINF, MINF) if it does not exist
    Vector3f normal;
};

inline float sqrdDist(Vector4f p1, Vector4f p2)
{
    float x = p1(0) - p2(0);
    float y = p1(1) - p2(1);
    float z = p1(2) - p2(2);
    
    return x*x + y*y + z*z;
}

// A cross two Vector4f vectors, with last element always is 1
Vector3f cross(Vector4f v1, Vector4f v2)
{
    Vector3f v31 = v1.head<3>();
    Vector3f v32 = v2.head<3>();
    Vector3f normal = (v31).cross(v32);
    return normal;
}


std::vector<Vertex> GetVertices(VirtualSensor& sensor, float edgeThresholdSqrd)
{
    // get ptr to the current depth frame
    // depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
    float* depthMap = sensor.GetDepth();
    
    // get ptr to the current color frame
    // color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
    BYTE* colorMap = sensor.GetColorRGBX();
    
    // depth intrinsics. From image to camera space
    Matrix3f depthIntrinsicsInv = sensor.GetDepthIntrinsics().inverse();
    
    // depth extrinsics. To get camera RDB and depth on top each other
    Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();
    
    //  trajectory. To map camera points to world space
    Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();
    
    // if the depth value at idx is invalid (MINF) write the following values to the vertices array
    // vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
    // vertices[idx].color = Vector4uc(0,0,0,0);
    // otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
    
    unsigned int imageWidth = sensor.GetDepthImageWidth();
    unsigned int imageHeight = sensor.GetDepthImageHeight();
    
    std::vector<Vertex> vertices(imageWidth * imageHeight);
    
    for (int i = 0; i < imageWidth * imageHeight; ++i)
    {
        int u = i % imageWidth; // from top left and rightwards, ie column
        int v = i / imageWidth; // from top left and downwards, ie row
        
        float d  = depthMap[i]; // depth value at (v, u)
        Vector4f pos;           // position (x,y,z) seen from the camera screen at (v, u),
        Vector4uc c;            // color value at (v, u)
        
        if (d == MINF)
        {
            pos = Vector4f(MINF, MINF, MINF, MINF);
            c = Vector4uc(0, 0, 0, 0);
        }
        else
        {
            Vector4f posImage = {v, u, 1.0, 1.0};
            
            Matrix4f depthIntrinsicsInvExtended; depthIntrinsicsInvExtended.setIdentity();
            depthIntrinsicsInvExtended.block(0, 0, 3, 3) = depthIntrinsicsInv;
            
            pos = depthExtrinsicsInv * depthIntrinsicsInvExtended * (d * posImage);
            
            BYTE color = colorMap[i];
            c(0) = colorMap[i*4 + 0];
            c(1) = colorMap[i*4 + 1];
            c(2) = colorMap[i*4 + 2];
            c(3) = 255;
        }
        vertices[i].position = pos;
        vertices[i].color = c;
    }
    
    // edge indices
    std::vector<unsigned int> edgeIndices;
    for (int u = 0; u < imageWidth; ++u)
    {
        unsigned int upperEdgeIdx = u;
        unsigned int lowerEdgeIdx = u + imageWidth * (imageHeight - 1);
        edgeIndices.push_back(upperEdgeIdx);
        edgeIndices.push_back(lowerEdgeIdx);
    }
    for (int v = 0; v < imageHeight; ++v)
    {
        unsigned int rightEdgeIdx = v * imageWidth;
        unsigned int leftEdgeIdx  = v * imageWidth + (imageWidth - 1);
        edgeIndices.push_back(rightEdgeIdx);
        edgeIndices.push_back(leftEdgeIdx);
    }
    
    // calculate vertex normals
    Vector3f normalInf(MINF, MINF, MINF);
    
    std::cout << "NormalInf: " << normalInf << std::endl;
    
    // The edge vertices have no normal
    for (auto const& edgeIdx: edgeIndices)
    {
        vertices[edgeIdx].normal = normalInf;
    }
    
    // Inside the edge vertices
    for (int u = 1; u < imageWidth - 1; ++u)
    {
        for (int v = 1; v < imageHeight - 1; ++v)
        {
            int idx      = v * imageWidth + u;
            int rightIdx = v * imageWidth + (u + 1);
            int leftIdx  = v * imageWidth + (u - 1);
            int lowerIdx = (v + 1) * imageWidth + u;
            int upperIdx = (v - 1) * imageWidth + u;
            if ( !(vertices[idx].position.allFinite()       &&
                   vertices[rightIdx].position.allFinite()  &&
                   vertices[leftIdx].position.allFinite()   &&
                   vertices[lowerIdx].position.allFinite()  &&
                   vertices[upperIdx].position.allFinite()) )  // then one or more vertex doesn't exist
            {
                vertices[idx].normal = normalInf;
            }
            else // then all 5 points exist
            {
                Vector4f p      = vertices[idx].position;
                Vector4f pRight = vertices[rightIdx].position;
                Vector4f pLeft  = vertices[leftIdx].position;
                Vector4f pLower = vertices[lowerIdx].position;
                Vector4f pUpper = vertices[rightIdx].position;
                if (sqrdDist(p, pRight) >= edgeThresholdSqrd ||
                    sqrdDist(p, pLeft)  >= edgeThresholdSqrd ||
                    sqrdDist(p, pLower) >= edgeThresholdSqrd ||
                    sqrdDist(p, pUpper) >= edgeThresholdSqrd)    // then points are too far from each other
                {
                    vertices[idx].normal = normalInf;
                }
                else // then normal makes sense and can be computed
                {
                    Vector4f v1 = pRight - pLeft;
                    Vector4f v2 = pUpper - pLower;
                    Vector3f normal = cross(v1, v2).normalized();
                    vertices[idx].normal = normal;
                    
                }
            }
        }
    }
    
    return vertices;
}


bool WriteToFile(std::vector<Vertex>& vertices, unsigned int width, unsigned int height, const std::string& filename)
{
    // file format http://paulbourke.net/dataformats/obj/minobj.html
    
    // Write off file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;
    
    // Save vertecies: the position (point cloud and normal)
    for (auto const& v: vertices)
    {
        if (v.normal.allFinite())
        {
            outFile << "v"   << " " << v.position(0) << " " << v.position(1) << " " << v.position(2) << " " << std::endl;
            outFile << "vn " << " " << v.normal(0)   << " " << v.normal(1)   << " " << v.normal(2)   << " " << std::endl;
        }
    }
    
    // close file
    outFile.close();
    
    return true;
}


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
    // isakrs' part
    
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
    
    // max sqrdDistance between points to in order to compute a normal between them
    float edgeThresholdSqrd = 0.05f*0.05f; // 5cm distance
    
    // convert video to meshes
    while (sensor.ProcessNextFrame())
    {
        // get global vertices of frame
        std::vector<Vertex> vertices = GetVertices(sensor, edgeThresholdSqrd);
        
         // write mesh file
         std::stringstream filenameOut;
         filenameOut << "./mesh/"
         << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".obj";
         
         if (!WriteToFile(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), filenameOut.str()))
         {
         std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
         return -1;
         }
    }

	return 0;
}
