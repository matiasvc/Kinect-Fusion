#ifndef KINECT_FUSION_IMPLICITSURFACE_HPP
#define KINECT_FUSION_IMPLICITSURFACE_HPP

#include <iostream>

#include <Eigen/Eigen>
#include "VoxelGrid.hpp"


class ImplicitSurface
{
public:
	virtual double eval(const Eigen::Vector3d& x) = 0;
};


void fillVoxelGrid(VoxelGrid& voxelGrid, ImplicitSurface& surface)
{
	double voxelSize = voxelGrid.size/voxelGrid.resolution;

	for (unsigned int x = 0; x < voxelGrid.resolution; ++x)
	{
		for (unsigned int y = 0; y < voxelGrid.resolution; ++y)
		{
			for (unsigned int z = 0; z < voxelGrid.resolution; ++z)
			{
				Eigen::Vector3d point;
				point << x*voxelSize, y*voxelSize, z*voxelSize;

				auto value = (float) surface.eval(point);
				voxelGrid.setValue(x, y, z, value);
			}
		}
	}
}

class Sphere : public ImplicitSurface
{
public:
	Sphere(const Eigen::Vector3d& center, double radius)
			: m_center(center), m_radius(radius)
	{ }

	double eval(const Eigen::Vector3d& _x) override
	{
		double x = _x(0) - this->m_center(0);
		double y = _x(1) - this->m_center(1);
		double z = _x(2) - this->m_center(2);

		return x*x + y*y + z*z - this->m_radius*this->m_radius;
	}


private:
	Eigen::Vector3d m_center;
	double m_radius;
};


class Torus : public ImplicitSurface
{
public:
	Torus(const Eigen::Vector3d& center, double radius, double a)
			: m_center(center), m_radius(radius), m_a(a)
	{
	}

	double eval(const Eigen::Vector3d& _x) override
	{
		const double x = _x(0) - this->m_center(0);
		const double y = _x(1) - this->m_center(1);
		const double z = _x(2) - this->m_center(2);

		double sum = x*x + y*y + z*z + this->m_radius*this->m_radius - this->m_a*this->m_a;

		return sum*sum - 4*this->m_radius*this->m_radius*(x*x + y*y);
	}

private:
	Eigen::Vector3d m_center;
	double m_radius;
	double m_a;
};


#endif //KINECT_FUSION_IMPLICITSURFACE_HPP
