#include <Eigen/Core>
#include <iostream>

#ifndef KINECT_FUSION_VOXELGRID_H
#define KINECT_FUSION_VOXELGRID_H


class VoxelGrid
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VoxelGrid(unsigned int resolution, double size);
    ~VoxelGrid ();

    float getValue(unsigned int x, unsigned int y, unsigned int z);
	void setValue(unsigned int x, unsigned int y, unsigned int z, float value);


    bool projectRayToVoxelPoint (Eigen::Vector3d origin, Eigen::Vector3d direction, double& length);
    bool  withinGrid(Eigen::Vector3d point);
    Eigen::Vector3d getPointAtIndex(Eigen::Vector3i index);

    float getValueAtPoint(Eigen::Vector3d point);
    void setAllValues(float);

    void operator= (const VoxelGrid&);
    VoxelGrid operator+ (const VoxelGrid&);
    VoxelGrid operator* (const VoxelGrid&);
    VoxelGrid operator/ (const VoxelGrid&);

    const unsigned int resolution;
    const double size;
    const double voxelSize;
    int numElements;

private:

	float *voxelData;
};



#endif //KINECT_FUSION_VOXELGRID_H
