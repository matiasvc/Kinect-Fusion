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
    _TSDF_global->setAllValues(0);
}


VoxelGrid *ModelReconstructor::getModel()
{
    return _TSDF_global;
}


void ModelReconstructor::printTSDF() {
    std::cout.precision(1);
    for (unsigned int x = 0; x < _resolution.x(); ++x) {
        for(unsigned int y = 0; y < _resolution.y(); ++y){
            for (unsigned int z = 0; z < _resolution.z(); ++z){
                if (_weights_global->getValue(x, y, z) != 0){
                    std::cout << _TSDF_global->getValue(x, y, z);
                }
                else{
                    std::cout << " ";
                }
                std::cout << "   \t  \t";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}


//Loop over every world point and calculate a truncated signed distance value (TSDF), along with a weight
void ModelReconstructor::reconstruct_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d cameraPose, VoxelGrid* TSDF, VoxelGrid* weights)
{
    weights->setAllValues(1.0f);
    TSDF->setAllValues(1.0f);

    for (int xi=0; xi<_resolution.x(); ++xi){
        for (int yi=0; yi<_resolution.z(); ++yi){
            for (int zi=0; zi<_resolution.z(); ++zi){
                Eigen::Vector3i voxelIndex(xi,yi,zi);
                //convert voxel indexes to world coordinates
                Eigen::Vector3d worldPoint = TSDF->getPointAtIndex(voxelIndex);
                Eigen::Vector4d worldPointHomo;
                worldPointHomo << worldPoint, 1;

                //project model point to pix coords
                Eigen::Matrix4d cameraPoseInv = cameraPose.inverse();
                Eigen::Vector4d cameraPointHomo = (cameraPoseInv * worldPointHomo);
                Eigen::Vector3d cameraPoint = cameraPointHomo.head(3);
                Eigen::Vector3d pixPointHomo = _cameraIntrinsic * cameraPoint;
                Eigen::Vector2i pixPoint;
                pixPoint << pixPointHomo.x()/pixPointHomo.z(), pixPointHomo.y()/pixPointHomo.z();

                //if point not in view
                if (bool(((pixPoint-_camResolution).array() >= 0).any()) or bool(((pixPoint).array() < 0).any())){
                    TSDF->setValue(xi, yi, zi, 0.0f);
                    weights->setValue(xi,yi,zi,0.0f);
                    continue;
                }

                //calc SDF value.
                double pointDepth = cameraPoint.z();
                float TSDF_val = (float) depthMap(pixPoint.x(), pixPoint.y()) - (float) pointDepth;

                //turncate SDF value
                if (TSDF_val >= -_truncationDistance){
                    TSDF_val = std::min(1.0f, fabsf(TSDF_val)/_truncationDistance)*copysignf(1.0f, TSDF_val);
                }
                else{ //too far behind obstacle
                    TSDF_val = 0.0f;
                    weights->setValue(xi,yi,zi,0.0f);
                }

                TSDF->setValue(xi, yi, zi, TSDF_val);
            }
        }
    }
}



void ModelReconstructor::fuseFrame(Eigen::MatrixXd depthMap, Eigen::Matrix4d cameraPose)
{
    std::cout << "Fusing Frame... " << std::endl;

    VoxelGrid *TSDF_local( new VoxelGrid(_resolution, _size, _offset));
    VoxelGrid *weights_local( new VoxelGrid(_resolution, _size, _offset));
    reconstruct_local(depthMap, cameraPose, TSDF_local, weights_local);

    //update global TSDF and weights using a running average
    *_TSDF_global = (*_weights_global)*(*_TSDF_global) + (*weights_local)*(*TSDF_local);
    *_weights_global = (*_weights_global) + (*weights_local);
    *_TSDF_global = (*_TSDF_global) / (*_weights_global);

    std::cout << "Frame Fused!" << std::endl;
}

