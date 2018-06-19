//
// Created by opeide on 14.06.18.
//

#include <Eigen/Core>
#include "VoxelGrid.hpp"

#ifndef KINECT_FUSION_TSDF_fuser_H
#define KINECT_FUSION_TSDF_fuser_H


class ModelReconstructor
{
private:
    Eigen::Matrix3d cameraIntrinsic;

    VoxelGrid TSDF_global;
    VoxelGrid weights_global;

    VoxelGrid & calculate_TSDF_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d& cameraPose);
    VoxelGrid & calculate_weights_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d& cameraPose); //needs surface normal map

public:
    ModelReconstructor( Eigen::Vector3i resolution,
                        Eigen::Vector3d size,
                        Eigen::Vector3d offset,
                        Eigen::Matrix3d cameraIntrinsic);

    void fuseFrame(Eigen::MatrixXd depthMap, Eigen::Matrix4d& cameraPose);

    VoxelGrid getModel();  //reference?
};

#endif //KINECT_FUSION_TSDF_H
