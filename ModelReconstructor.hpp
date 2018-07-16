//
// Created by opeide on 14.06.18.
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#include <tuple>
#include <fstream>

#include "VoxelGrid.hpp"
#include <stdio.h>
#ifndef KINECT_FUSION_TSDF_fuser_H
#define KINECT_FUSION_TSDF_fuser_H


class ModelReconstructor
{
public:
    ModelReconstructor(float truncationDistance,
                        unsigned int resolution,
                        double size,
                        Eigen::Matrix3d cameraIntrinsic,
                        Eigen::Vector2i camResolution);

	~ModelReconstructor();
    void writeTSDFToFile(std::string fileName);

    void fuseFrame(Eigen::MatrixXd depthMap, Eigen::Matrix4d cameraPose);

    VoxelGrid *getModel();  //reference?

private:
    VoxelGrid* _TSDF;
    VoxelGrid* _weights;

    float _truncationDistance;

    Eigen::Matrix3d _cameraIntrinsic;
	unsigned int _resolution;
    double _size;
    Eigen::Vector2i _camResolution;

};

#endif //KINECT_FUSION_TSDF_H
