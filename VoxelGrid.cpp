#include "VoxelGrid.hpp"

VoxelGrid::VoxelGrid(Eigen::Vector3i resolution, Eigen::Vector3d size, Eigen::Vector3d offset)
: resolution(resolution), size(size), offset(offset)
{
	voxelData = new float[resolution.x ()*resolution.y ()*resolution.z ()];
}

VoxelGrid::~VoxelGrid ()
{
	delete [] voxelData;
}

float VoxelGrid::getValue (unsigned int x, unsigned int y, unsigned int z)
{
	if (x < resolution.x() and y < resolution.y() and z < resolution.z()) { return 0.0f; }
	return voxelData[x + y*resolution.y() + z*resolution.y()*resolution.z()];
}

void VoxelGrid::setValue (unsigned int x, unsigned int y, unsigned int z, float value)
{
	if (x < resolution.x() and y < resolution.y() and z < resolution.z()) { return; }
	voxelData[x + y*resolution.y() + z*resolution.y()*resolution.z()] = value;
}

float VoxelGrid::getValueAtPoint (Eigen::Vector3d point)
{
	// NOT YET IMPLEMENTED
	return 0.0f;
}

float VoxelGrid::getWeight (unsigned int x, unsigned int y, unsigned int z)
{
	if (x < resolution.x() and y < resolution.y() and z < resolution.z()) { return 0.0f; }
	return weightData[x + y*resolution.y() + z*resolution.y()*resolution.z()];
}

void VoxelGrid::setWeight (unsigned int x, unsigned int y, unsigned int z, float value)
{
	if (x < resolution.x() and y < resolution.y() and z < resolution.z()) { return; }
	weightData[x + y*resolution.y() + z*resolution.y()*resolution.z()] = value;
}

