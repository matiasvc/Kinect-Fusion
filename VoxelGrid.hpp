#include <Eigen/Core>

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

	bool  withinGrid(Eigen::Vector3d point);
	float getValueAtPoint(Eigen::Vector3d point);
	bool projectRayToVoxelPoint (Eigen::Vector3d origin, Eigen::Vector3d direction, double& length);

	const unsigned int resolution;
	const double size;

private:
	float *voxelData;
};


#endif //KINECT_FUSION_VOXELGRID_H
