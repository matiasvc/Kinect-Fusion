//
// Created by Matias Christensen on 24/06/2018.
//

#include "Pose.hpp"

Pose::Pose(Eigen::Matrix3d orientation, Eigen::Vector3d translation)
: orientation(orientation), translation(translation)
{}

Pose::Pose(Eigen::Matrix4d extrinsic){

	orientation = extrinsic.block(0,0,3,3).transpose();
	translation = extrinsic.col(3).head(3);
	translation = -orientation*translation;
	std::cout << "init pose with position " << translation << std::endl;
}


Eigen::Vector3d Pose::transformPoint(Eigen::Vector3d point)
{
	return orientation*point + translation;
}

Eigen::Vector3d Pose::transformVector(Eigen::Vector3d vector)
{
	return orientation*vector;
}

void Pose::translate(Eigen::Vector3d translationVector)
{
	translation += orientation * translationVector;
}

void Pose::rotateEuler(Eigen::Vector3d eulerAngles)
{
	Eigen::Quaterniond q;
	q = Eigen::AngleAxisd(eulerAngles.x(), Eigen::Vector3d::UnitX()) *
	    Eigen::AngleAxisd(eulerAngles.y(), Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(eulerAngles.z(), Eigen::Vector3d::UnitZ());

	Eigen::Matrix3d rot = q.matrix();

	orientation = orientation * rot;
}

Pose Pose::PoseFromEuler(Eigen::Vector3d eulerAngles, Eigen::Vector3d translation)
{
	Eigen::Quaterniond q;
	q = Eigen::AngleAxisd(eulerAngles.x(), Eigen::Vector3d::UnitX()) *
	    Eigen::AngleAxisd(eulerAngles.y(), Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(eulerAngles.z(), Eigen::Vector3d::UnitZ());

	Eigen::Matrix3d rot = q.matrix();

	return {rot, translation};
}
