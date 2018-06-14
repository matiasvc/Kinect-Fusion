//
// Created by opeide on 14.06.18.
//

#ifndef KINECT_FUSION_TSDF_fuser_H
#define KINECT_FUSION_TSDF_fuser_H

#include "VoxelGrid.h"


class TSDF_fuser{
    private:
        VoxelGrid::VoxelGrid TSDF_global;
        VoxelGrid::VoxelGrid weights_global;

        VoxelGrid::VoxelGrid calculate_TSDF_local(int depthMap, int cameraPose);    //needs camera calibration matrix
        VoxelGrid::VoxelGrid calculate_weights_local(int depthMap, int cameraPose); //needs surface normal map

public:
        TSDF_fuser(int gridSize);   //init voxel grids

        void fuse_depthmap(int depthMap, int cameraPose);
        VoxelGrid::VoxelGrid get_TSDF(){ return TSDF_global;}
};

#endif //KINECT_FUSION_TSDF_H
