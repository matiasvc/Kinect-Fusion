#include <iostream>
#include "VoxelGrid.hpp"
#include "ModelReconstructor.hpp"

#include <Eigen/Core>

int main (int argc, char* argv[])
{
	Eigen::Vector3i resolution = Eigen::Vector3i::Ones () * 4;
	Eigen::Vector3d size = Eigen::Vector3d::Ones () * 3.0;
	Eigen::Vector3d offset = Eigen::Vector3d::Zero ();

	float truncationDistance = 1.0;

	double fx = 1;
	double fy = 1;
	double u0 = 1;
	double v0 = 1;
	Eigen::Matrix3d cameraIntrinsic;
	cameraIntrinsic << 	fx, 0, u0,
						0, fy, v0,
						0, 0, 1;

    Eigen::Matrix3d depthMap;
    depthMap <<     3,3,3,
                    3,3,3,
                    3,3,3;

    Eigen::Matrix4d cameraPose;
    cameraPose <<   1,0,0,0,
                    0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;

    ModelReconstructor fuser(truncationDistance, resolution, size, offset, cameraIntrinsic);
    fuser.fuseFrame(depthMap, cameraPose);
    fuser.printTSDF();

    //VoxelGrid *model = fuser.getModel();



	std::cout << "Finishing" << std::endl;
	return 0;
}
