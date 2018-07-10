//
// Created by Isak Rathe Stoere on 05/07/2018.
//

#include "Vertex.hpp"

/** A squared distance function of two Vector4f vectors with last element equal to 1.0 **/
inline float sqrdDist(Vector4f p1, Vector4f p2)
{
    float x = p1(0) - p2(0);
    float y = p1(1) - p2(1);
    float z = p1(2) - p2(2);
    
    return x*x + y*y + z*z;
}

/** A cross of two Vector4f vectors with last element equal to 1.0 **/
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
    float* depthMap = sensor.getDepth();
    
    // get ptr to the current color frame
    // color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
    BYTE* colorMap = sensor.getColorRGBX();
    
    // depth intrinsics. From image to camera space
    Matrix3f depthIntrinsicsInv = sensor.getDepthIntrinsics().inverse();
    
    // depth extrinsics. To get camera RDB and depth on top each other
    Matrix4f depthExtrinsicsInv = sensor.getDepthExtrinsics().inverse();
    
    // trajectory. To map camera points to world space
    //Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();
    // trajectory will not be done. Just keeping the comment.
    
    // if the depth value at idx is invalid (MINF), then
        // vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
        // vertices[idx].color = Vector4uc(0,0,0,0);
    // otherwise
        // apply back-projection and transform the vertex to CAMERA space,
        // use the corresponding color from the colormap
    
    unsigned int imageWidth = sensor.getDepthImageWidth();
    unsigned int imageHeight = sensor.getDepthImageHeight();
    
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
    
    // Normals are computed according to this alg:
    // if vertex is on the egde of camera image (an edgeIdx), then
        // vertices[edgeIdx].normal will be (MINF, MINF, MINF)
    // else if vertex and neigbors is too far from each other
        // vertex[idx].normal will be (MINF, MINF, MINF)
    // else, now it only makes sence to compute a normal
        // vertex[idx].normal will be cross product of neighbors' positons
    
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
    
    // Edge vertices
    Vector3f normalInf(MINF, MINF, MINF);
    for (auto const& edgeIdx: edgeIndices)
    {
        vertices[edgeIdx].normal = normalInf;
    }
    
    // None edge vertices
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


bool WriteToFile(std::vector<Vertex>& vertices, const std::string& filename)
{
    // file format obj: http://paulbourke.net/dataformats/obj/minobj.html
    
    // open file
    std::ofstream outFile(filename);
    if (!outFile.is_open()) return false;
    
    // Save vertecies with position and normal
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
