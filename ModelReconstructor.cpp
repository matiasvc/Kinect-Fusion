//
// Created by opeide on 14.06.18.
//


#include "ModelReconstructor.hpp"


ModelReconstructor::ModelReconstructor(float truncationDistance,
                                       Eigen::Vector3i resolution,
                                       Eigen::Vector3d size,
                                       Eigen::Vector3d offset,
                                       Eigen::Matrix3d cameraIntrinsic,
                                        Eigen::Vector2i camResolution)
:   _TSDF_global(new VoxelGrid(resolution, size, offset)),
    _weights_global(new VoxelGrid(resolution, size, offset))
{
    _truncationDistance = truncationDistance;
    _resolution = resolution;
    _size = size;
    _offset = offset;
    _cameraIntrinsic = cameraIntrinsic;
    _camResolution = camResolution;

    _weights_global->setAllValues(0);
}

VoxelGrid ModelReconstructor::get_empty_voxelGrid()
{
    VoxelGrid emptyGrid = VoxelGrid(_resolution, _size, _offset);
    return emptyGrid;
}

VoxelGrid *ModelReconstructor::getModel()
{
    return _TSDF_global;
}


void ModelReconstructor::printTSDF() {
    std::cout.precision(2);
    for (unsigned int x = 0; x < 4; ++x) {
        for(unsigned int y = 0; y < 4; ++y){
            for (unsigned int z = 0; z < 4; ++z){
                std::cout << _TSDF_global->getValue(x, y, z) << "\t";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}


//Loop over every world point and calculate a truncated signed distance value (TSDF)
VoxelGrid ModelReconstructor::calculate_TSDF_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d &cameraPose)
{
    VoxelGrid TSDF_local = get_empty_voxelGrid();
    for (int xi=0; xi<_resolution.x(); ++xi){
        for (int yi=0; yi<_resolution.z(); ++yi){
            for (int zi=0; zi<_resolution.z(); ++zi){
                Eigen::Vector3i voxelIndex(xi,yi,zi);

                //convert voxel indexes to world coordinates
                Eigen::Vector3d worldPoint = TSDF_local.getPointAtIndex(voxelIndex);
                Eigen::Vector4d worldPointHomo;
                worldPointHomo << worldPoint, 1;

                //project model point to pix coords
                Eigen::Matrix4d cameraPoseInv = cameraPose.inverse();
                Eigen::Vector4d cameraPointHomo = (cameraPoseInv * worldPointHomo);
                Eigen::Vector3d cameraPoint = cameraPointHomo.head(3);
                Eigen::Vector3d pixPointHomo = _cameraIntrinsic * cameraPoint;
                Eigen::Vector2i pixPoint;
                pixPoint << pixPointHomo.x()/pixPointHomo.z(), pixPointHomo.y()/pixPointHomo.z();
                if (bool(((pixPoint-_camResolution).array() >= 0).any()) or bool(((pixPoint).array() < 0).any())){
                    continue; //todo: affect weight??
                }

                //todo:create 'lambda', to scale ray to depth. (2 norm of camera point?)
                double lambda = 1.;

                //calc SDF value. use camera pos - point as depth (camerapoint again??).
                Eigen::Vector3d camPos = cameraPose.col(3).head(3);

                float TSDF = (float) depthMap(pixPoint.x(), pixPoint.y()) - (float) lambda*(camPos-worldPoint).norm();

//                //turncate SDF value
//                if (TSDF >= -_truncationDistance){
//                    TSDF = std::min(float(1), TSDF/_truncationDistance);
//                }
//                else{
//                    TSDF = 0;
//                }

                TSDF_local.setValue(xi, yi, zi, TSDF);
            }
        }
    }
    TSDF_local.setValue(0,0,0,99);
    return TSDF_local;
}

//requires normalmap
VoxelGrid ModelReconstructor::calculate_weights_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d &cameraPose)
{
    VoxelGrid weights_local = get_empty_voxelGrid();
    weights_local.setAllValues(1.0);
    return weights_local;
}

void ModelReconstructor::fuseFrame(Eigen::MatrixXd depthMap, Eigen::Matrix4d &cameraPose)
{
    std::cout << "Fusing Frame... " << std::endl;

    VoxelGrid weights_local = calculate_weights_local(depthMap, cameraPose);
    VoxelGrid TSDF_local = calculate_TSDF_local(depthMap, cameraPose);

    //update global TSDF and weights using a running average
    *_TSDF_global = (*_weights_global)*(*_TSDF_global) + (weights_local*TSDF_local);
    *_weights_global = (*_weights_global) + weights_local;
    *_TSDF_global = (*_TSDF_global) / (*_weights_global);

    std::cout << "Frame Fused!" << std::endl;
}

