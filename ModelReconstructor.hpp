//
// Created by opeide on 14.06.18.
//

#include <Eigen/Core>
#include "VoxelGrid.hpp"

#ifndef KINECT_FUSION_TSDF_fuser_H
#define KINECT_FUSION_TSDF_fuser_H


class ModelReconstructor
{
public:
    ModelReconstructor( Eigen::Vector3i resolution,
                        Eigen::Vector3d size,
                        Eigen::Vector3d offset,
                        Eigen::Matrix3d cameraIntrinsic);

    void fuseFrame(Eigen::MatrixXd depthMap, Eigen::Matrix4d& cameraPose);

    VoxelGrid & getModel();  //reference?

private:
    VoxelGrid _TSDF_global;
    VoxelGrid _weights_global;

    Eigen::Matrix3d _cameraIntrinsic;
    Eigen::Vector3i _resolution;
    Eigen::Vector3d _size;
    Eigen::Vector3d _offset;

    VoxelGrid & get_empty_voxelGrid();
    VoxelGrid & calculate_TSDF_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d& cameraPose);
    VoxelGrid & calculate_weights_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d& cameraPose); //needs surface normal map
};

#endif //KINECT_FUSION_TSDF_H
