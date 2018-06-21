//
// Created by opeide on 14.06.18.
//

#include <algorithm>
#include "ModelReconstructor.hpp"


ModelReconstructor::ModelReconstructor(double truncationDistance,
                                       Eigen::Vector3i resolution,
                                       Eigen::Vector3d size,
                                       Eigen::Vector3d offset,
                                       Eigen::Matrix3d cameraIntrinsic)
:   _TSDF_global_ref(new VoxelGrid(resolution, size, offset)),
    _weights_global_ref(new VoxelGrid(resolution, size, offset))
{
    _truncationDistance = truncationDistance;
    _resolution = resolution;
    _size = size;
    _offset = offset;
    _cameraIntrinsic = cameraIntrinsic;

    _weights_global_ref->setAllValues(1);
}

VoxelGrid *ModelReconstructor::get_empty_voxelGrid()
{
    VoxelGrid *emptyGrid = new VoxelGrid(_resolution, _size, _offset);
    return emptyGrid;
}

VoxelGrid *ModelReconstructor::getModel()
{
    return _TSDF_global_ref;
}

//Loop over every world point and calculate a truncated signed distance value (TSDF)
VoxelGrid* ModelReconstructor::calculate_TSDF_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d &cameraPose)
{
    VoxelGrid* TSDF_local = get_empty_voxelGrid();
//    for (int xi=0; xi<_resolution.x(); ++xi){
//        for (int yi=0; yi<_resolution.z(); ++yi){
//            for (int zi=0; zi<_resolution.z(); ++zi){
//                //convert voxel indexes to world coordinates
//                Eigen::Vector3i voxelIndex(xi,yi,zi);
////                Eigen::Vector3d worldPoint = TSDF_local.getPointAtIndex(voxelIndex);
////                Eigen::Vector3d worldPointHomo(worldPoint, 1);
//
//                //project model point to pix coords
////                Eigen::Vector3d cameraPointHomo = (cameraPose.inverse() * worldPointHomo);
////                Eigen::Vector3d cameraPoint = cameraPointHomo.head(3);
////                Eigen::Vector3d pixPointHomo = _cameraIntrinsic * cameraPoint;
////                Eigen::Vector2i pixPoint(pixPointHomo.head(2)/pixPointHomo.z());
//
//                //create 'lambda', measurement scaling. (2 norm of camera point?)
//
//                //calc SDF value. use camera pos - point as depth (camerapoint again??).
//                float TSDF = 1;
//
//                //turncate SDF value
//                if (TSDF >= -_truncationDistance){
//                    TSDF = std::min(float(1), TSDF/_truncationDistance);
//                }
//                else{
//                    TSDF = 9; //todo: set to NULL?
//                }
//
//                TSDF_local->setValue(voxelIndex.x(), voxelIndex.y(), voxelIndex.z(), TSDF);
//            }
//        }
//    }
    TSDF_local->setAllValues(5.0);
    return TSDF_local;
}

//requires normalmap
VoxelGrid* ModelReconstructor::calculate_weights_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d &cameraPose)
{
    VoxelGrid* weights_local = get_empty_voxelGrid();
    weights_local->setAllValues(1.0);
    return weights_local;
}

void ModelReconstructor::fuseFrame(Eigen::MatrixXd depthMap, Eigen::Matrix4d &cameraPose)
{
    //VoxelGrid weights_local = calculate_weights_local(depthMap, cameraPose);
    VoxelGrid* TSDF_local_ref = calculate_TSDF_local(depthMap, cameraPose);

    //todo: update global TSDF and weights using a running average
//    _TSDF_global_ref = _weights_global_ref*_TSDF_global_ref + weights_local*TSDF_local;
//    _weights_global_ref = _weights_global_ref + weights_local;
//    _TSDF_global_ref = _TSDF_global_ref / _weights_global_ref;
    _TSDF_global_ref = TSDF_local_ref;
}



