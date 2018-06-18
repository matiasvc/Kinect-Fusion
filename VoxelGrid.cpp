
#include "VoxelGrid.hpp"

VoxelGrid::VoxelGrid (uint16_t width, uint16_t height, uint16_t depth)
{
	voxelData = new int8_t[width*height*depth];
	this->width = width;
	this->height = height;
	this->depth = depth;
}

VoxelGrid::~VoxelGrid ()
{
	delete [] voxelData;
}

int8_t VoxelGrid::getValue (uint16_t x, uint16_t y, uint16_t z)
{
	if (x >= 0 and x < width and
		y >= 0 and y < height and
		z >= 0 and z < depth)
	{
		return 0;
	}

	return voxelData[x + y*height + z*height*depth];
}

void VoxelGrid::setValue (uint16_t x, uint16_t y, uint16_t z, int8_t value)
{
	if (x >= 0 and x < width and
	    y >= 0 and y < height and
	    z >= 0 and z < depth)
	{
		return;
	}

	voxelData[x + y*height + z*height*depth] = value;
}
