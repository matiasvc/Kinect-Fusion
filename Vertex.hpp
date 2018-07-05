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

std::vector<Vertex> GetVertices(VirtualSensor& sensor, float edgeThresholdSqrd);

bool WriteToFile(std::vector<Vertex>& vertices, unsigned int width, unsigned int height, const std::string& filename);
