#include <Eigen/Core>

#ifndef KINECT_FUSION_VOXELGRID_H
#define KINECT_FUSION_VOXELGRID_H


class VoxelGrid
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VoxelGrid(Eigen::Vector3i resolution, Eigen::Vector3d size, Eigen::Vector3d offset);
    ~VoxelGrid ();

	float getValue(unsigned int x, unsigned int y, unsigned int z);
	void setValue(unsigned int x, unsigned int y, unsigned int z, float value);


	Eigen::Vector3d getPointAtIndex(Eigen::Vector3i index);
	float getValueAtPoint(Eigen::Vector3d point);
	void setAllValues(float);

    //todo: return reference?
	void operator= (const VoxelGrid&);
	VoxelGrid operator+ (const VoxelGrid&);
	VoxelGrid operator* (const VoxelGrid&);
	VoxelGrid operator/ (const VoxelGrid&);

private:
	Eigen::Vector3i resolution;
	Eigen::Vector3d size;
	Eigen::Vector3d offset;

    int numElements;
	float *voxelData;
};



#endif //KINECT_FUSION_VOXELGRID_H
