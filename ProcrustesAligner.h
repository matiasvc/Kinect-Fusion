#pragma once
#include "SimpleMesh.h"
#include <Eigen/StdVector>

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;
		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.

		Vector3f meanPoints = Vector3f::Zero();

		unsigned int n = points.size();
		for (unsigned int i = 0; i < n; i++)
			meanPoints = meanPoints + points[i];

		meanPoints = (1.0 / float(n))*meanPoints;

		return meanPoints;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm. 
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.

		unsigned int n = sourcePoints.size();
		/*Vector3f sourceX = 
		Vector3f crossCovMat = ;
		unsigned int n = points.size();*/

		MatrixXf sourceX(3, n);
		MatrixXf targetX(3, n);
		for (unsigned int i = 0; i < n; i++)
		{
			sourceX.col(i) = sourcePoints[i] - sourceMean;
			targetX.col(i) = targetPoints[i] - targetMean;
		}

		sourceX.transposeInPlace();
		targetX.transposeInPlace();

		/*const Eigen::Vector3f *ptr = sourcePoints.data();
		const int size = sourcePoints.size();
		Eigen::Vector3f  sourceX = Eigen::Map<int, Vector3f> ptr;
		Eigen::Vector3f temp1 = sourceX.colwise() - sourceMean;

		Eigen::Matrix3f targetX(targetPoints.data());
		Eigen::Matrix3f temp2 = targetX.colwise() - targetMean;*/

		Eigen::Matrix3f crossCovMat = targetX.transpose()*sourceX;

		JacobiSVD<MatrixXf> svd(crossCovMat, ComputeThinU | ComputeThinV);

		double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
		if (d > 0)
			d = 1.0;
		else
			d = -1.0;

		Eigen::Matrix3f I = Eigen::Matrix3f::Identity(3, 3);
		I(2, 2) = d;

		//Eigen::Matrix3f R = svd.matrixV() * I * svd.matrixU().transpose();
		Eigen::Matrix3f R = svd.matrixU() * I * svd.matrixV().transpose();

		return R;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target opints.
		
		//Matrix3f rT = rotation.transpose();
		Vector3f t = targetMean - rotation*sourceMean;

		return t;
	}
};