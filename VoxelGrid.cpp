#include "VoxelGrid.hpp"

#include <math.h>

#include <iostream>

VoxelGrid::VoxelGrid(unsigned int resolution, double size)
: resolution(resolution), size(size), voxelSize(size/resolution)
{
    numElements = resolution*resolution*resolution;
	voxelData = new float[numElements];
}

VoxelGrid::~VoxelGrid()
{
	delete [] voxelData;
}

float VoxelGrid::getValue(unsigned int x, unsigned int y, unsigned int z)
{
	if (x >= resolution or y >= resolution or z >= resolution) { return 0.0f; }
	return voxelData[x + y*resolution + z*resolution*resolution];
}

void VoxelGrid::setValue(unsigned int x, unsigned int y, unsigned int z, float value)
{
	if (x >= resolution or y >= resolution or z >= resolution) { return; }
	voxelData[x + y*resolution + z*resolution*resolution] = value;
}

bool  VoxelGrid::withinGrid(Eigen::Vector3d point)
{
    double x = point.x();
    double y = point.y();
    double z = point.z();

    return x >= 0 and x <= size and
           y >= 0 and y <= size and
           z >= 0 and z <= size;
}


Eigen::Vector3d VoxelGrid::getPointAtIndex(Eigen::Vector3i index){
	Eigen::Vector3d point = index.cast<double>()/(double(resolution-1));
	point = point*size;
    //todo: fix half voxel error
	return point;
}



float VoxelGrid::getValueAtPoint(Eigen::Vector3d point)
{
	// Clamp point to within the voxel volume

	auto x_local = float(point.x()/size)*resolution;
	auto y_local = float(point.y()/size)*resolution;
	auto z_local = float(point.z()/size)*resolution;

	if (x_local < 0.0) { x_local = 0.0; }
	if (y_local < 0.0) { y_local = 0.0; }
	if (z_local < 0.0) { z_local = 0.0; }

	if (x_local > resolution - 2) { x_local = float(resolution - 2); }
	if (y_local > resolution - 2) { y_local = float(resolution - 2); }
	if (z_local > resolution - 2) { z_local = float(resolution - 2); }

	auto x1 = (unsigned int) x_local;
	auto y1 = (unsigned int) y_local;
	auto z1 = (unsigned int) z_local;

	if (x1 > resolution - 2) { x1 = resolution - 2; }
	if (y1 > resolution - 2) { y1 = resolution - 2; }
	if (z1 > resolution - 2) { z1 = resolution - 2; }

	float xd = x_local - x1;
	float yd = y_local - y1;
	float zd = z_local - z1;

	float c000 = getValue (x1,   y1,   z1);
	float c001 = getValue (x1,   y1,   z1+1);
	float c010 = getValue (x1,   y1+1, z1);
	float c011 = getValue (x1,   y1+1, z1+1);
	float c100 = getValue (x1+1, y1,   z1);
	float c101 = getValue (x1+1, y1,   z1+1);
	float c110 = getValue (x1+1, y1+1, z1);
	float c111 = getValue (x1+1, y1+1, z1+1);

	float c00 = c000*(1 - xd) + c100*xd;
	float c01 = c001*(1 - xd) + c101*xd;
	float c10 = c010*(1 - xd) + c110*xd;
	float c11 = c011*(1 - xd) + c111*xd;

	float c0 = c00*(1 - yd) + c10*yd;
	float c1 = c01*(1 - yd) + c11*yd;

	return c0*(1 - zd) + c1*zd;
}


void VoxelGrid::setAllValues(float val) {
	for (int i=0; i<numElements; ++i){
		voxelData[i] = val;
	}
}

void VoxelGrid::operator= (const VoxelGrid& rhs){
    for(int i=0; i<numElements; ++i){
        voxelData[i] = rhs.voxelData[i];
    }
}

VoxelGrid VoxelGrid::operator+ (const VoxelGrid &rhs)
{
    VoxelGrid summed = VoxelGrid(resolution, size);
    for(int i=0; i<numElements; ++i){
        summed.voxelData[i] = voxelData[i] + rhs.voxelData[i];
    }
    return summed;
}

VoxelGrid VoxelGrid::operator*(const VoxelGrid & rhs)
{
    VoxelGrid elemProd = VoxelGrid(resolution, size);
    for(int i=0; i<numElements; ++i){
        elemProd.voxelData[i] = voxelData[i] * rhs.voxelData[i];
    }
    return elemProd;
}

VoxelGrid VoxelGrid::operator/(const VoxelGrid & rhs)
{
    float defaultValue = 1.0;
    VoxelGrid elemDiv = VoxelGrid(resolution, size);
    for(int i=0; i<numElements; ++i){
        if (rhs.voxelData[i]==0){
            elemDiv.voxelData[i] = defaultValue;
        }else{
            elemDiv.voxelData[i] = voxelData[i] / rhs.voxelData[i];
        }
    }
    return elemDiv;
}

bool VoxelGrid::projectRayToVoxelPoint(Eigen::Vector3d origin, Eigen::Vector3d direction, double& length)
{
	double dirfrac_x = 1.0 / direction.x();
	double dirfrac_y = 1.0 / direction.y();
	double dirfrac_z = 1.0 / direction.z();

	Eigen::Vector3d lb = Eigen::Vector3d::Zero();
	Eigen::Vector3d rt = Eigen::Vector3d::Ones()*(size);

	double t1 = (lb.x() - origin.x())*dirfrac_x;
	double t2 = (rt.x() - origin.x())*dirfrac_x;
	double t3 = (lb.y() - origin.y())*dirfrac_y;
	double t4 = (rt.y() - origin.y())*dirfrac_y;
	double t5 = (lb.z() - origin.z())*dirfrac_z;
	double t6 = (rt.z() - origin.z())*dirfrac_z;

	double tmin = std::fmax( std::fmax( std::fmin( t1, t2 ), std::fmin( t3, t4 ) ), std::fmin( t5, t6 ) );
	double tmax = std::fmin( std::fmin( std::fmax( t1, t2 ), std::fmax( t3, t4 ) ), std::fmax( t5, t6 ) );

	// if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
	if (tmax < 0)
	{
		length = tmax;
		return false;
	}

	// if tmin > tmax, ray doesn't intersect AABB
	if (tmin > tmax)
	{
		length = tmax;
		return false;
	}


	length = tmin;
	return true;
}


