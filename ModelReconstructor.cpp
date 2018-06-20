//
// Created by opeide on 14.06.18.
//

#include "ModelReconstructor.hpp"


ModelReconstructor::ModelReconstructor(Eigen::Vector3i resolution, Eigen::Vector3d size, Eigen::Vector3d offset,
                                       Eigen::Matrix3d cameraIntrinsic)
:_resolution(resolution), _size(size), _offset(offset), _cameraIntrinsic(cameraIntrinsic)
{
    _TSDF_global = VoxelGrid(resolution, size, offset);
    _weights_global = VoxelGrid(resolution, size, offset);
}

VoxelGrid& ModelReconstructor::get_empty_voxelGrid()
{
    return VoxelGrid(_resolution, _size, _offset);
}

VoxelGrid& ModelReconstructor::getModel()
{
    return _TSDF_global;
}

VoxelGrid& ModelReconstructor::calculate_TSDF_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d &cameraPose)
{
    VoxelGrid TSDF_local = get_empty_voxelGrid();
    //todo: Loop over every voxel and calculate a signed distance value

    return TSDF_local;
}

VoxelGrid& ModelReconstructor::calculate_weights_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d &cameraPose)
{
    VoxelGrid weights_local = get_empty_voxelGrid();    //todo: init to 1
    return weights_local;
}

void ModelReconstructor::fuseFrame(Eigen::MatrixXd depthMap, Eigen::Matrix4d &cameraPose)
{
    VoxelGrid weights_local = calculate_weights_local(depthMap, cameraPose);
    VoxelGrid TSDF_local = calculate_TSDF_local(depthMap, cameraPose);
    //todo: update global TSDF and weights using a running average
}



