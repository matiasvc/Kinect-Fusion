#ifndef KINECT_FUSION_POSE_HPP
#define KINECT_FUSION_POSE_HPP

#include <Eigen/Eigen>

class Pose
{
public:
	Pose(Eigen::Matrix3d rotation, Eigen::Vector3d translation);

	Eigen::Matrix3d orientation;
	Eigen::Vector3d translation;

	Eigen::Vector3d transformPoint(Eigen::Vector3d point);
	Eigen::Vector3d transformVector(Eigen::Vector3d translationVector);

	void translate(Eigen::Vector3d translate);
	void rotateEuler(Eigen::Vector3d eulerAngles);

	static Pose PoseFromEuler(Eigen::Vector3d eulerAngles, Eigen::Vector3d translation);
private:

};


#endif //KINECT_FUSION_POSE_HPP
