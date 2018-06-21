#include <iostream>
#include "VoxelGrid.hpp"
#include "ModelReconstructor.hpp"

#include <Eigen/Core>

int main (int argc, char* argv[])
{
	Eigen::Vector3i resolution = Eigen::Vector3i::Ones () * 17;
	Eigen::Vector3d size = Eigen::Vector3d::Ones () * 1.0;
	Eigen::Vector3d offset = Eigen::Vector3d::Zero ();

	float truncationDistance = 1;

    Eigen::Vector2i camResolution(3,3);
	double fx = 1;
	double fy = 1;
	double u0 = 1;
	double v0 = 1;
	Eigen::Matrix3d cameraIntrinsic;
	cameraIntrinsic << 	fx, 0, u0,
						0, fy, v0,
						0, 0, 1;

    Eigen::Matrix3d depthMap1 = Eigen::Matrix3d::Ones() * 6;
    Eigen::Matrix3d depthMap2 = Eigen::Matrix3d::Ones() * 7;

    Eigen::Matrix4d cameraPose1 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d cameraPose2 = Eigen::Matrix4d::Identity();
    cameraPose2(1,3) = resolution.y();


    ModelReconstructor fuser(truncationDistance, resolution, size, offset, cameraIntrinsic, camResolution);

    fuser.fuseFrame(depthMap1, cameraPose1);
    fuser.fuseFrame(depthMap2, cameraPose2);
    fuser.printTSDF();




	std::cout << "Finishing" << std::endl;
	return 0;
}
