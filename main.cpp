#include <iostream>
#include "VoxelGrid.hpp"

#include <Eigen/Core>

int main (int argc, char* argv[])
{
	Eigen::Vector3i resolution = Eigen::Vector3i::Ones () * 512;
	Eigen::Vector3d size = Eigen::Vector3d::Ones () * 3.0;
	Eigen::Vector3d offset = Eigen::Vector3d::Zero ();

	auto *grid = new VoxelGrid(resolution, size, offset);

	for (unsigned int i = 50; i < 300; ++i)
	{
		std::cout << (int) grid->getValue (45, i, 400);
	}

    std::cout << "Hello world" << std::endl;

	return 0;
}
