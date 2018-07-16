//
// Created by opeide on 14.06.18.
//


#include "ModelReconstructor.hpp"


ModelReconstructor::ModelReconstructor(float truncationDistance,
                                       unsigned int resolution,
                                       double size,
                                       Eigen::Matrix3d cameraIntrinsic,
                                        Eigen::Vector2i camResolution)
:   _TSDF_global(new VoxelGrid(resolution, size)),
    _weights_global(new VoxelGrid(resolution, size))
{
    _truncationDistance = truncationDistance;
    _resolution = resolution;
    _size = size;
    _cameraIntrinsic = cameraIntrinsic;
    _camResolution = camResolution;

    _weights_global->setAllValues(0.0);
    _TSDF_global->setAllValues(-1.0);
}


VoxelGrid *ModelReconstructor::getModel()
{
    return _TSDF_global;
}


void ModelReconstructor::writeTSDFToFile(std::string fileName){
    std::cout << "Writing to file..." << std::endl;
    std::cout << "\t" << fileName << std::endl;
    std::remove(fileName.c_str());
    std::ofstream file;
    file.open(fileName);
    for (unsigned int x = 0; x < _resolution; ++x) {
        for(unsigned int y = 0; y < _resolution; ++y){
            for (unsigned int z = 0; z < _resolution; ++z){
                file << x << "," << y << "," << z << "," << _TSDF_global->getValue(x, y, z) << "\n";
            }
        }
    }
    file.close();
    std::cout << "Wrote to file!" << std::endl;
}


//Loop over every world point and calculate a truncated signed distance value (TSDF), along with a weight
void ModelReconstructor::reconstruct_local(Eigen::MatrixXd depthMap, Eigen::Matrix4d cameraExtrinsic, VoxelGrid* TSDF, VoxelGrid* weights)
{
    weights->setAllValues(0);
    TSDF->setAllValues(1.0f);


    //Eigen::Matrix4d cameraPoseInv = cameraPose.inverse();
    Eigen::Vector3d cameraPoint;
    Eigen::Vector3d pixPointHomo;

    for (int xi=0; xi<_resolution; ++xi){
        for (int yi=0; yi<_resolution; ++yi){
            for (int zi=0; zi<_resolution; ++zi){
                Eigen::Vector3i voxelIndex(xi,yi,zi);

                //convert voxel indexes to world coordinates
                Eigen::Vector4d worldPointHomo;
                worldPointHomo << TSDF->getPointAtIndex(voxelIndex), 1;

                //transform world point to cam coords
                cameraPoint = (cameraExtrinsic * worldPointHomo).head(3);

                //point behind camera or too far away
                if(cameraPoint.z() <= 0 || cameraPoint.z() > 3.0){
                    continue;
                }

                //Project point to pixel in depthmap
                pixPointHomo = _cameraIntrinsic * cameraPoint;
                Eigen::Vector2i pixPoint;
                pixPoint << pixPointHomo.x()/pixPointHomo.z(), pixPointHomo.y()/pixPointHomo.z();

                //if pix outisde depthmap
                if (bool(((pixPoint-_camResolution).array() >= 0).any()) || bool(((pixPoint).array() < 0).any())){
                    continue;
                }

                //calc SDF value.
                int depthRow = pixPoint.y();//_camResolution.y()-1 - pixPoint.y(); //row0 is at top. y0 is at bottom.
                int depthCol = pixPoint.x();
                double pointDepth = cameraPoint.z();
                float TSDF_val = (float) depthMap.coeff(depthRow,depthCol) - (float) pointDepth;

                //truncate SDF value
                if (TSDF_val >= -_truncationDistance){
                    TSDF_val = std::min(1.0f, fabsf(TSDF_val)/_truncationDistance)*copysignf(1.0f, TSDF_val);
                }
                else{ //too far behind obstacle
                    continue;
                }
                TSDF->setValue(xi, yi, zi, TSDF_val);
                weights->setValue(xi, yi, zi, 1.0);
            }
        }
    }
}


//todo: update global model voxels directly. Might be more efficient.
void ModelReconstructor::fuseFrame(Eigen::MatrixXd depthMap, Eigen::Matrix4d cameraExtrinsic)
{
    std::cout << "Fusing Frame... " << std::endl;

    //Create TSDF for new frame
    VoxelGrid *TSDF_local( new VoxelGrid(_resolution, _size));
    VoxelGrid *weights_local( new VoxelGrid(_resolution, _size));
    reconstruct_local(depthMap, cameraExtrinsic, TSDF_local, weights_local);

    //update global TSDF and weights using a running average
    *_TSDF_global = (*_weights_global)*(*_TSDF_global) + (*weights_local)*(*TSDF_local);
    *_weights_global = (*_weights_global) + (*weights_local);
    *_TSDF_global = (*_TSDF_global) / (*_weights_global);

    delete [] TSDF_local;
    delete [] weights_local;

    std::cout << "Frame Fused!" << std::endl;
}

