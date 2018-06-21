#include "VoxelGrid.hpp"

VoxelGrid::VoxelGrid(Eigen::Vector3i resolution, Eigen::Vector3d size, Eigen::Vector3d offset)
: resolution(resolution), size(size), offset(offset)
{
    numElements = resolution.x ()*resolution.y ()*resolution.z ();
	voxelData = new float[numElements];
}

VoxelGrid::~VoxelGrid ()
{
	delete [] voxelData;
}

float VoxelGrid::getValue (unsigned int x, unsigned int y, unsigned int z)
{
	if (x >= resolution.x() and y >= resolution.y() and z >= resolution.z()) { return 0.0f; }
	return voxelData[x + y*resolution.y() + z*resolution.y()*resolution.z()];
}

void VoxelGrid::setValue (unsigned int x, unsigned int y, unsigned int z, float value)
{
	if (x >= resolution.x() and y >= resolution.y() and z >= resolution.z()) { return; }
	voxelData[x + y*resolution.y() + z*resolution.y()*resolution.z()] = value;
}

Eigen::Vector3d VoxelGrid::getPointAtIndex(Eigen::Vector3i index){
	Eigen::Vector3d point( double(index.x()) *size.x(), double(index.y()) *size.y(), double(index.z()) *size.z());
    point += offset;
	return point;
}

void VoxelGrid::print() {
    std::cout.precision(1);
    for (unsigned int x = 0; x < resolution.x(); ++x) {
        for(unsigned int y = 0; y < resolution.y(); ++y){
            for (unsigned int z = 0; z < resolution.z(); ++z){
                std::cout << getValue(x, y, z) << "   \t  \t";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}

float VoxelGrid::getValueAtPoint (Eigen::Vector3d point)
{
	// Clamp point to within the voxel volume
	point.cwiseMin (offset);
	point.cwiseMax (offset + size);

	auto x_local = float((point.x() - offset.x())/size.x());
	auto y_local = float((point.y() - offset.y())/size.y());
	auto z_local = float((point.z() - offset.z())/size.z());

	auto x1 = (unsigned int) x_local;
	auto y1 = (unsigned int) y_local;
	auto z1 = (unsigned int) z_local;

	float xd = x_local - x1;
	float yd = y_local - y1;
	float zd = z_local - z1;

	float c000 = getValue (x1,   y1,   z1);
	float c001 = getValue (x1,   y1,   z1+1);
	float c010 = getValue (x1,   y1+1, z1);
	float c011 = getValue (x1,   y1+1, z1+1);
	float c100 = getValue (x1+1, y1,   z1);
	float c101 = getValue (x1+1, y1,   z1+1);
	float c110 = getValue (x1+1, y1+1, z1);
	float c111 = getValue (x1+1, y1+1, z1+1);

	float c00 = c000*(1 - xd) + c100*xd;
	float c01 = c001*(1 - xd) + c101*xd;
	float c10 = c010*(1 - xd) + c110*xd;
	float c11 = c011*(1 - xd) + c111*xd;

	float c0 = c00*(1 - yd) + c10*yd;
	float c1 = c01*(1 - yd) + c11*yd;

	float c = c0*(1 - zd) + c1*zd;

	return c;
}

void VoxelGrid::setAllValues(float val) {
	for (int i=0; i<numElements; ++i){
		voxelData[i] = val;
	}
}

void VoxelGrid::operator= (const VoxelGrid& rhs){
    for(int i=0; i<numElements; ++i){
        voxelData[i] = rhs.voxelData[i];
    }
}

VoxelGrid VoxelGrid::operator+ (const VoxelGrid &rhs)
{
    VoxelGrid summed = VoxelGrid(resolution, size, offset);
    for(int i=0; i<numElements; ++i){
        summed.voxelData[i] = voxelData[i] + rhs.voxelData[i];
    }
    return summed;
}

VoxelGrid VoxelGrid::operator*(const VoxelGrid & rhs)
{
    VoxelGrid elemProd = VoxelGrid(resolution, size, offset);
    for(int i=0; i<numElements; ++i){
        elemProd.voxelData[i] = voxelData[i] * rhs.voxelData[i];
    }
    return elemProd;
}

VoxelGrid VoxelGrid::operator/(const VoxelGrid & rhs)
{
    VoxelGrid elemDiv = VoxelGrid(resolution, size, offset);
    for(int i=0; i<numElements; ++i){
        if (voxelData[i]==0 or rhs.voxelData[i]==0){
            elemDiv.voxelData[i] = 0;
        }else{
            elemDiv.voxelData[i] = voxelData[i] / rhs.voxelData[i];
        }
    }
    return elemDiv;
}

