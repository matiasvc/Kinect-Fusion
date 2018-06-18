#include <stdint.h>

#ifndef KINECT_FUSION_VOXELGRID_H
#define KINECT_FUSION_VOXELGRID_H


class VoxelGrid
{
public:
    VoxelGrid(uint16_t width, uint16_t height, uint16_t depth);
    ~VoxelGrid ();

    int8_t getValue(uint16_t x, uint16_t y, uint16_t z);
    void setValue(uint16_t x, uint16_t y, uint16_t z, int8_t value);
private:
	unsigned int width;
	unsigned int height;
	unsigned int depth;

	int8_t *voxelData;
};


#endif //KINECT_FUSION_VOXELGRID_H
