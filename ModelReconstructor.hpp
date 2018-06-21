//
// Created by opeide on 14.06.18.
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <algorithm>

#include "VoxelGrid.hpp"

#ifndef KINECT_FUSION_TSDF_fuser_H
#define KINECT_FUSION_TSDF_fuser_H


class ModelReconstructor
{
public:
    ModelReconstructor( float truncationDistance,
                        Eigen::Vector3i resolution,
                        Eigen::Vector3d size,
                        Eigen::Vector3d offset,
                        Eigen::Matrix3d cameraIntrinsic,
                        Eigen::Vector2i camResolution);

    void printTSDF();

    void fuseFrame(Eigen::MatrixXd depthMap, Eigen::Matrix4d& cameraPose);

    VoxelGrid *getModel();  //reference?

private:
    VoxelGrid* _TSDF_global;
    VoxelGrid* _weights_global;

    float _truncationDistance;

    Eigen::Matrix3d _cameraIntrinsic;
    Eigen::Vector3i _resolution;
    Eigen::Vector3d _size;
    Eigen::Vector3d _offset;
    Eigen::Vector2i _camResolution;

    VoxelGrid get_empty_voxelGrid();
    VoxelGrid calculate_TSDF_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d& cameraPose);
    VoxelGrid calculate_weights_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d& cameraPose); //needs surface normal map
};

#endif //KINECT_FUSION_TSDF_H
