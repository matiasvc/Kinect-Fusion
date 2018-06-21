#include <iostream>
#include "VoxelGrid.hpp"
#include "ModelReconstructor.hpp"

#include <Eigen/Core>

int main (int argc, char* argv[])
{
	Eigen::Vector3i resolution = Eigen::Vector3i::Ones () * 4;
	Eigen::Vector3d size = Eigen::Vector3d::Ones () * 3.0;
	Eigen::Vector3d offset = Eigen::Vector3d::Zero ();

	double truncationDistance = .1;

	double fx = 1;
	double fy = 1;
	double u0 = 1;
	double v0 = 1;
	Eigen::Matrix3d cameraIntrinsic;
	cameraIntrinsic << 	fx, 0, u0,
						0, fy, v0,
						0, 0, 1;

	//auto *grid = new VoxelGrid(resolution, size, offset);

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
    std::cout << "Fusing Frame..." << std::endl;
    fuser.fuseFrame(depthMap, cameraPose);
    std::cout << "Frame fused!" << std::endl;

    VoxelGrid *model = fuser.getModel(); //todo: why necessary to pass pointer??
    model->setAllValues(6.0);
	for (unsigned int x = 0; x < 4; ++x) {
		for(unsigned int y = 0; y < 4; ++y){
			for (unsigned int z = 0; z < 4; ++z){
				std::cout << model->getValue(x, y, z);
			}
            std::cout << std::endl;
		}
        std::cout << std::endl;
	}
	std::cout << "Finishing" << std::endl;
	return 0;
}
