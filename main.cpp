#include <iostream>
#include "VoxelGrid.hpp"


int main (int argc, char* argv[])
{
	auto *grid = new VoxelGrid(512, 512, 512);

	for (int i = 50; i < 300; ++i)
	{
		std::cout << (int) grid->getValue (45, i, 400);
	}

    std::cout << "Hello world" << std::endl;
}
