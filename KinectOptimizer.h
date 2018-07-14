#pragma once

// The Google logging library (GLOG), used in Ceres, has a conflict with Windows defined constants. This definitions prevents GLOG to use the same constants
#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "PointCloud.h"


struct Match {
	int idx;
	float weight;
};

#undef ITER_INFO
#define TIMING_INFO

#ifdef TIMING_INFO
	double totalJacobianEvalutionTime = 0;
	double totalSolvingTime = 0;
#endif


/**
 * Helper methods for writing Ceres cost functions.
 */
template <typename T>
static inline void fillVector(const Vector3f& input, T* output) {
	output[0] = T(input[0]);
	output[1] = T(input[1]);
	output[2] = T(input[2]);
}


/**
 * Pose increment is only an interface to the underlying array (in constructor, no copy
 * of the input array is made).
 * Important: Input array needs to have a size of at least 6.
 */
template <typename T>
class PoseIncrement {
public:
	explicit PoseIncrement(T* const array) : m_array{ array } { }
	
	void setZero() {
		for (int i = 0; i < 6; ++i)
			m_array[i] = T(0);
	}

	T* getData() const {
		return m_array;
	}

	/**
	 * Applies the pose increment onto the input point and produces transformed output point.
	 * Important: The memory for both 3D points (input and output) needs to be reserved (i.e. on the stack)
	 * beforehand).
	 */
	void apply(T* inputPoint, T* outputPoint) const {
		// pose[0,1,2] is angle-axis rotation.
		// pose[3,4,5] is translation.
		const T* rotation = m_array;
		const T* translation = m_array + 3;

		T temp[3];
		ceres::AngleAxisRotatePoint(rotation, inputPoint, temp);

		outputPoint[0] = temp[0] + translation[0];
		outputPoint[1] = temp[1] + translation[1];
		outputPoint[2] = temp[2] + translation[2];
	}

	/**
	 * Converts the pose increment with rotation in SO3 notation and translation as 3D vector into
	 * transformation 4x4 matrix.
	 */
	static Matrix4f convertToMatrix(const PoseIncrement<double>& poseIncrement) {
		// pose[0,1,2] is angle-axis rotation.
		// pose[3,4,5] is translation.
		double* pose = poseIncrement.getData();
		double* rotation = pose;
		double* translation = pose + 3;

		// Convert the rotation from SO3 to matrix notation (with column-major storage).
		double rotationMatrix[9];
		ceres::AngleAxisToRotationMatrix(rotation, rotationMatrix);

		// Create the 4x4 transformation matrix.
		Matrix4f matrix;
		matrix.setIdentity();
		matrix(0, 0) = float(rotationMatrix[0]);	matrix(0, 1) = float(rotationMatrix[3]);	matrix(0, 2) = float(rotationMatrix[6]);	matrix(0, 3) = float(translation[0]);
		matrix(1, 0) = float(rotationMatrix[1]);	matrix(1, 1) = float(rotationMatrix[4]);	matrix(1, 2) = float(rotationMatrix[7]);	matrix(1, 3) = float(translation[1]);
		matrix(2, 0) = float(rotationMatrix[2]);	matrix(2, 1) = float(rotationMatrix[5]);	matrix(2, 2) = float(rotationMatrix[8]);	matrix(2, 3) = float(translation[2]);
		
		return matrix;
	}

private:
	T* m_array;
};


/**
 * Optimization constraints.
 */
class PointToPointConstraint {
public:
	PointToPointConstraint(const Vector3f& sourcePoint, const Vector3f& targetPoint, const float weight) :
		m_sourcePoint{ sourcePoint },
		m_targetPoint{ targetPoint },
		m_weight{ weight }
	{ }

	template <typename T>
	bool operator()(const T* const pose, T* residuals) const {
		// TODO: Implemented the point-to-point cost function.
		// The resulting 3D residual should be stored in residuals array. To apply the pose 
		// increment (pose parameters) to the source point, you can use the PoseIncrement
		// class.
		// Important: Ceres automatically squares the cost function.
		
		//auto poseIncrement = PoseIncrement<const T>(pose);
		//T temp[3] = {};
		T pt[3] = { T(m_sourcePoint[0]),  T(m_sourcePoint[1]), T(m_sourcePoint[2]) };
		//poseIncrement.apply(pt, temp);
				
		//T result[3] = { T(m_targetPoint(0) - temp(0)), T(m_targetPoint(1) - temp(1)), T(m_targetPoint(2) - temp(2)) };

		const T* rotation = pose;
		const T* translation = pose + 3;

		T temp[3];
		ceres::AngleAxisRotatePoint(rotation, pt, temp);

		temp[0] = temp[0] + translation[0];
		temp[1] = temp[1] + translation[1];
		temp[2] = temp[2] + translation[2];

		const T w = T(sqrt(m_weight));

		residuals[0] = w*(-T(m_targetPoint(0)) + temp[0]);
		residuals[1] = w*(-T(m_targetPoint(1)) + temp[1]);
		residuals[2] = w*(-T(m_targetPoint(2)) + temp[2]);
	
		return true;
	}

	static ceres::CostFunction* create(const Vector3f& sourcePoint, const Vector3f& targetPoint, const float lambda) {
		return new ceres::AutoDiffCostFunction<PointToPointConstraint, 3, 6>(
			new PointToPointConstraint(sourcePoint, targetPoint, lambda)
		);
	}

protected:
	const Vector3f m_sourcePoint;
	const Vector3f m_targetPoint;
	const float m_weight;
	const float LAMBDA = 0.1f;
};

class PointToPlaneConstraint {
public:
	PointToPlaneConstraint(const Vector3f& sourcePoint, const Vector3f& targetNormal, const Vector3f& targetPoint, const float weight) :
		m_sourcePoint{ sourcePoint },
		m_targetPoint{ targetPoint },
		m_targetNormal{ targetNormal },
		m_weight{ weight }
	{ }

	template <typename T>
	bool operator()(const T* const pose, T* residuals) const {
		// TODO: Implemented the point-to-plane cost function.
		// The resulting 1D residual should be stored in residuals array. To apply the pose 
		// increment (pose parameters) to the source point, you can use the PoseIncrement
		// class.
		// Important: Ceres automatically squares the cost function.

		T pt[3] = { T(m_sourcePoint[0]),  T(m_sourcePoint[1]), T(m_sourcePoint[2]) };
		T nrm[3] = { T(m_targetNormal[0]),  T(m_targetNormal[1]), T(m_targetNormal[2]) };
		//poseIncrement.apply(pt, temp);

		//T result[3] = { T(m_targetPoint(0) - temp(0)), T(m_targetPoint(1) - temp(1)), T(m_targetPoint(2) - temp(2)) };

		const T* rotation = pose;
		const T* translation = pose + 3;

		T src_pt[3];
		ceres::AngleAxisRotatePoint(rotation, pt, src_pt);

		src_pt[0] = src_pt[0] + translation[0];
		src_pt[1] = src_pt[1] + translation[1];
		src_pt[2] = src_pt[2] + translation[2];

		const T w = T(sqrt(m_weight));

		T x, y, z;
		x = w*(src_pt[0] - T(m_targetPoint[0])) * nrm[0];
		y = w*(src_pt[1] - T(m_targetPoint[1])) * nrm[1];
		z = w*(src_pt[2] - T(m_targetPoint[2])) * nrm[2];
		
		residuals[0] = x + y + z;

		return true;
	}

	static ceres::CostFunction* create(const Vector3f& sourcePoint, const Vector3f& sourceNormal, const Vector3f& targetPoint, const float lambda) {
		return new ceres::AutoDiffCostFunction<PointToPlaneConstraint, 1, 6>(
			new PointToPlaneConstraint(sourcePoint, sourceNormal,targetPoint, lambda)
		);
	}

protected:
	const Vector3f m_sourcePoint;
	const Vector3f m_targetPoint;
	const Vector3f m_targetNormal;
	const float m_weight;
	const float LAMBDA = 1.0f;
};

/**
 * ICP optimizer, using Ceres for optimization.
 */
class KinectFusionOptimizer {
public:
	KinectFusionOptimizer() :
		m_nIterations{ 20 },
		m_distThreshold{ 0.2f }, // 0.07f
		m_angleThreshold{ 0.6f }, // 0.866025f
		m_nIterationsPyramid{ std::vector<int>{10,5,4} },
		m_oldPose{ Matrix4f::Identity() }
	{ }

	void setMatchingMaxDistance(const float &maxDistance) {
		m_distThreshold = maxDistance;
	}

	void setNbOfIterationsInPyramid(const std::vector<int> &iters) {
		m_nIterationsPyramid = iters;
	}

	void setMatchingNormalsDistance(const float &maxDistance) {
		m_angleThreshold = maxDistance;
	}

	void setNbOfIterations(unsigned nIterations) {
		m_nIterations = nIterations;
	}

	void setOldPose(const Matrix4f &pose) {
		m_oldPose = pose;
	}

	const double calcError(const MatrixXd& sourcePoints, const MatrixXd& targetPoints, const MatrixXd& targetNormals, const VectorXd& weights, const unsigned int& size, const VectorXd& deltaX) {

		//std::cout << "Will solve the constraints now..\n";

		VectorXd err(size);
		const Vector3d r = deltaX.segment(0, 3);
		const Vector3d t = deltaX.segment(3, 3);

		//#pragma omp parallel for shared(sourcePoints, targetNormals, targetPoints, err)  
		for (int i = 0; i < size; i++)
		{
			const Vector3d src_pts = sourcePoints.col(i);
			const Vector3d trgt_nrm = targetNormals.col(i);
			err(i) = weights(i)*((src_pts - targetPoints.col(i)).dot(trgt_nrm) + r.dot(src_pts.cross(trgt_nrm)) + t.dot(trgt_nrm));
		}
		//#pragma omp barrier  

		return err.squaredNorm() / (double)size;;
	}

	Matrix4f estimatePose(const PointCloud& source, float* previousDepthMap, const std::vector<Vector3f>& previousNormalMap, VirtualSensor& sensor, const Matrix4f & oldPose = Matrix4f::Identity()) {

		// The initial estimate can be given as an argument.
		//Matrix4f estimatedPose = Matrix4f::Identity();	// INFO: Normally this should be true ???
		Matrix4f estimatedPose = oldPose;

		// We optimize on the transformation in SE3 notation: 3 parameters for the axis-angle vector of the rotation (its length presents
		// the rotation angle) and 3 parameters for the translation vector. 
		double incrementArray[6];
		auto poseIncrement = PoseIncrement<double>(incrementArray);
		poseIncrement.setZero();

		// Rotate old normals into the global frame N_g,k-1
		auto rotatedNormals = rotatePoints(previousNormalMap, oldPose.block(0,0,3,3));
		//auto rotatedNormals = previousNormalMap;

		std::cout << "Transformed normals from previous frame into the global frame ..." << std::endl;

		const Matrix3f& depthIntrinsics = sensor.getDepthIntrinsics();
		const Matrix4f& depthExtrinsics = sensor.getDepthExtrinsics();
		float fovX = depthIntrinsics(0, 0);
		float fovY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);
		
		Matrix4f oldPoseInv = oldPose.inverse();
		Matrix4f depthExtInv = depthExtrinsics.inverse();

		const unsigned int dWidth = sensor.getDepthImageWidth();
		const unsigned int dHeight = sensor.getDepthImageHeight();

		for (int i = 0; i < m_nIterations; ++i) {
			// Compute the matches.
			std::cout << "Doing projective association of points ..." << std::endl;
			clock_t begin = clock();

			// TODO transform points from vertex_k into vertex_k-1
			auto tempTransform = depthExtrinsics*oldPoseInv*estimatedPose;
			auto transformedPoints = transformPoints(source.getPoints(), tempTransform);				// Transformation with left-side multiplication
			//auto transformedPoints = transformPoints(transformPoints(transformPoints(source.getPoints(), estimatedPose), oldPoseInv), depthExtrinsics);

			std::cout << "Transformed source points into global frame ..." << std::endl;

			// TODO project points to pixels & dehomogenize down to depth frame pixels u_hat (u, v, 1)
			const unsigned nMatches = transformedPoints.size();
			std::vector<Match> matches(nMatches);	// Acts like a boolean for valid matches, -1 otherwise
			std::vector<Vector3f> target(nMatches);
			int k = 0;
			unsigned int cnt = 0;
			for (const auto& pt : transformedPoints) {
				
				const Vector3f temp = depthIntrinsics * pt;

				unsigned int pixel_u = (unsigned int)std::floor(temp.x() / temp.z());
				unsigned int pixel_v = (unsigned int)std::floor(temp.y() / temp.z());

				unsigned int idx = pixel_v * dWidth + pixel_u;

				if (pixel_u >= dWidth || pixel_u < 0 ||
					pixel_v >= dHeight || pixel_v < 0 ||
					previousDepthMap[idx] == MINF )
				{
					target[k] = Vector3f(MINF, MINF, MINF);
					matches[k].idx = -1; 
				}
				else 
				{
					const float depth = previousDepthMap[idx];

					// TODO backproject pixels to points V_g,k-1
					target[k] = (oldPose*depthExtInv *Vector4f((pixel_u - cX)*depth / fovX, (pixel_v - cY)*depth / fovY, depth, 1)).block(0, 0, 3, 1);
					/*const float depthScale = depth / temp.z();
					target[k] = (oldPose*depthExtInv *Vector4f(pt.x()*depthScale, pt.y()*depthScale, pt.z()*depthScale, 1)).block(0, 0, 3, 1);*/
						
					Vector4f temp_srcPt = Vector4f(source.getPoints()[k].x(), source.getPoints()[k].y(), source.getPoints()[k].z(), 1);
					Vector4f temp_trgtPt = Vector4f(target[k].x(), target[k].y(), target[k].z(), 1);
						
					Vector3f dist_Test = (estimatedPose*temp_srcPt - temp_trgtPt).block(0, 0, 3, 1);
					if (dist_Test.norm() > 0.1f )
					{
						//target[k] = Vector3f(MINF, MINF, MINF);
						matches[k].idx = -1;
					}
					else
					{
						if (idx > rotatedNormals.size())
						{
							std::cout << "Attempted to index: " << idx << " which is bigger than the size of Normals Vector: " << rotatedNormals.size() << std::endl;
							std::cout << "u: " << pixel_u << "v: " << pixel_v << "width: " << sensor.getDepthImageWidth() << "height: " << sensor.getDepthImageHeight() << "\n\n";
							//target[k] = Vector3f(MINF, MINF, MINF);
							matches[k].idx = -1;
						}
						else
						{
							Vector3f tempNrm = rotatedNormals[idx];
							if (!tempNrm.allFinite())
							{
								//target[k] = Vector3f(MINF, MINF, MINF);
								matches[k].idx = -1;
							}
							else
							{
								const Vector3f curPt = depthIntrinsics*source.getPoints()[k];
								const unsigned int px_u = (unsigned int)std::floor(curPt.x() / curPt.z());
								const unsigned int px_v = (unsigned int)std::floor(curPt.y() / curPt.z());
								unsigned int px_idx = px_v * dWidth + px_u;

								matches[k].idx = idx;
								cnt++;

								if (px_u < dWidth && px_u >= 0 &&
									px_v < dHeight && px_v >= 0 &&
									source.getRawNormals()[px_idx].allFinite())
								{
									Vector3f curNrm = estimatedPose.block(0,0,3,3)*source.getRawNormals()[px_idx];
									const float nrmAngl = curNrm.dot(tempNrm);

									if (nrmAngl < 0.707107f )	// 0.707107 = cos(45)=sin(45)
									{
										matches[k].idx = -1;	// No need to set weight as this will be discarded anyways
										cnt--;
									}
									matches[k].weight = 0.5f*(0.1f - dist_Test.norm()) / 0.1f + 0.5f*(nrmAngl - 0.707107f)/ (1.0f - 0.707107f);
								}
								else
									matches[k].weight = 0.5f*(0.1f - dist_Test.norm()) / 0.1f;
							}
						}
					}
				}
				k++;
			}

			auto transformedSrcPoints = transformPoints(source.getPoints(), estimatedPose);

			std::cout << "Projected source points into the previous frame ..." << std::endl;
			std::cout << "# of valid matches: "<< cnt << std::endl;

			if (cnt == 0)
			{
				std::cout << "WARNING: 0 Valid Matches! Optimization will not run at this iteration!" << std::endl;
				break;
			}

			clock_t end = clock();
			double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
			std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

			std::cout << "Ceres constraints will be created now ..." << std::endl;

			// Prepare point-to-point and point-to-plane constraints.
			ceres::Problem problem;
			prepareConstraints(transformedSrcPoints, target, rotatedNormals, matches, poseIncrement, problem); // INFO: target normals need index of u_hat

			std::cout << "Ceres constraints are created. Ceres will try to solve now ..." << std::endl;

			// Configure options for the solver.
			ceres::Solver::Options options;
			configureSolver(options);

			std::cout << "Solver has run. It will report results now ..." << std::endl;

			// Run the solver (for one iteration).
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			//std::cout << summary.BriefReport() << std::endl;
			//std::cout << summary.FullReport() << std::endl;

			// Update the current pose estimate (we always update the pose from the left, using left-increment notation).
			Matrix4f matrix = PoseIncrement<double>::convertToMatrix(poseIncrement);
			estimatedPose = PoseIncrement<double>::convertToMatrix(poseIncrement) * estimatedPose;
			poseIncrement.setZero();

			std::cout << "Optimization iteration done." << std::endl;
		}

		return estimatedPose;
	}

	Matrix4f estimatePose_v2(const PointCloud& source, float* previousDepthMap, const std::vector<Vector3f>& previousNormalMap, const Matrix3f &depthIntrinsics, const Matrix4f &depthExtrinsics,
		const unsigned int &dWidth, const unsigned int &dHeight, const Matrix4f & currentEstimate, const unsigned int& numIters, const Matrix4f & oldPose = Matrix4f::Identity()) {
		// The initial estimate can be given as an argument.
		//Matrix4f estimatedPose = Matrix4f::Identity();	// INFO: Normally this should be true ???
		Matrix4f estimatedPose = currentEstimate;

		// We optimize on the transformation in SE3 notation: 3 parameters for the axis-angle vector of the rotation (its length presents
		// the rotation angle) and 3 parameters for the translation vector. 

		double incrementArray[6];
		auto poseIncrement = PoseIncrement<double>(incrementArray);
		poseIncrement.setZero();

		// Rotate old normals into the global frame N_g,k-1
		auto rotatedNormals = rotatePoints(previousNormalMap, oldPose.block(0, 0, 3, 3));
		//auto rotatedNormals = previousNormalMap;


		#ifdef LOG_INFO
		std::cout << "Transformed normals from previous frame into the global frame ..." << std::endl;
		#endif

		const float distThres = 0.05f;		// 7 cm - Threshold on distance
		const float angleThres = 0.866025f;	// 0.866025 = cos(30), 0.707107 = cos(45) - Threshold between normals

		float fovX = depthIntrinsics(0, 0);
		float fovY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		Matrix4f oldPoseInv = oldPose.inverse();
		Matrix4f depthExtInv = depthExtrinsics.inverse();

		for (int i = 0; i < numIters; ++i) {
			// Compute the matches.

			#ifdef LOG_INFO
			std::cout << "Doing projective association of points ..." << std::endl;
			#endif LOG_INFO

			clock_t begin = clock();

			// TODO transform points from vertex_k into vertex_k-1
			auto tempTransform = depthExtrinsics * oldPoseInv*estimatedPose;
			auto transformedPoints = transformPoints(source.getPoints(), tempTransform);				// Transformation with left-side multiplication
																										//auto transformedPoints = transformPoints(transformPoints(transformPoints(source.getPoints(), estimatedPose), oldPoseInv), depthExtrinsics);

			#ifdef LOG_INFO
			std::cout << "Transformed source points into global frame ..." << std::endl;
			#endif LOG_INFO
	
			// TODO project points to pixels & dehomogenize down to depth frame pixels u_hat (u, v, 1)
			const unsigned nMatches = transformedPoints.size();
			std::vector<Match> matches(nMatches);	// Acts like a boolean for valid matches, -1 otherwise
			std::vector<Vector3f> target(nMatches);
			int k = 0;
			unsigned int cnt = 0;
			for (const auto& pt : transformedPoints) {

				const Vector3f temp = depthIntrinsics * pt;

				unsigned int pixel_u = (unsigned int)std::floor(temp.x() / temp.z());
				unsigned int pixel_v = (unsigned int)std::floor(temp.y() / temp.z());

				unsigned int idx = pixel_v * dWidth + pixel_u;

				if (pixel_u >= dWidth || pixel_u < 0 ||
					pixel_v >= dHeight || pixel_v < 0 ||
					previousDepthMap[idx] == MINF)
				{
					target[k] = Vector3f(MINF, MINF, MINF);
					matches[k].idx = -1;
				}
				else
				{
					const float depth = previousDepthMap[idx];

					// TODO backproject pixels to points V_g,k-1
					target[k] = (oldPose*depthExtInv *Vector4f((pixel_u - cX)*depth / fovX, (pixel_v - cY)*depth / fovY, depth, 1)).block(0, 0, 3, 1);
					/*const float depthScale = depth / temp.z();
					target[k] = (oldPose*depthExtInv *Vector4f(pt.x()*depthScale, pt.y()*depthScale, pt.z()*depthScale, 1)).block(0, 0, 3, 1);*/

					Vector4f temp_srcPt = Vector4f(source.getPoints()[k].x(), source.getPoints()[k].y(), source.getPoints()[k].z(), 1);
					Vector4f temp_trgtPt = Vector4f(target[k].x(), target[k].y(), target[k].z(), 1);

					Vector3f dist_Test = (estimatedPose*temp_srcPt - temp_trgtPt).block(0, 0, 3, 1);
					if (dist_Test.norm() > distThres)
					{
						//target[k] = Vector3f(MINF, MINF, MINF);
						matches[k].idx = -1;
					}
					else
					{
						if (idx > rotatedNormals.size())
						{
							std::cout << "Attempted to index: " << idx << " which is bigger than the size of Normals Vector: " << rotatedNormals.size() << std::endl;
							std::cout << "u: " << pixel_u << "v: " << pixel_v << "width: " << dWidth << "height: " << dHeight << "\n\n";
							//target[k] = Vector3f(MINF, MINF, MINF);
							matches[k].idx = -1;
						}
						else
						{
							Vector3f tempNrm = rotatedNormals[idx];
							if (!tempNrm.allFinite())
							{
								//target[k] = Vector3f(MINF, MINF, MINF);
								matches[k].idx = -1;
							}
							else
							{
								const Vector3f curPt = depthIntrinsics * temp_srcPt.head<3>();
								const unsigned int px_u = (unsigned int)std::floor(curPt.x() / curPt.z());
								const unsigned int px_v = (unsigned int)std::floor(curPt.y() / curPt.z());
								unsigned int px_idx = px_v * dWidth + px_u;

								matches[k].idx = idx;
								cnt++;

								if (px_u < dWidth && px_u >= 0 &&
									px_v < dHeight && px_v >= 0 &&
									source.getRawNormals()[px_idx].allFinite())
								{
									Vector3f curNrm = estimatedPose.block(0, 0, 3, 3)*source.getRawNormals()[px_idx];
									const float nrmAngl = curNrm.dot(tempNrm);

									if (nrmAngl < 0.866025f)	// 0.707107 = cos(45)=sin(45), 0.866025 = cos(30)
									{
										matches[k].idx = -1;	// No need to set weight as this will be discarded anyways
										cnt--;
									}
									matches[k].weight = (0.5f*(distThres - dist_Test.norm()) / distThres + 0.5f*(nrmAngl - angleThres) / (1.0f - angleThres));
								}
								else
									matches[k].weight = (0.5f*(distThres - dist_Test.norm()) / distThres);
							}
						}
					}
				}
				k++;
			}

			auto transformedSrcPoints = transformPoints(source.getPoints(), estimatedPose);

			#ifdef LOG_INFO
			std::cout << "Projected source points into the previous frame ..." << std::endl;
			std::cout << "# of valid matches: " << cnt << std::endl;
			#endif
			
			if (cnt == 0)
			{
				std::cout << "WARNING: 0 Valid Matches! Optimization will not run at this iteration!" << std::endl;
				break;
			}

			clock_t end = clock();
			double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;

			#ifdef LOG_INFO
			std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;
			#endif

			if (false)
			{
				std::cout << "Will debug the frame" << std::endl;

				std::ofstream sourcePoints("sourcePoints.txt");
				if (!sourcePoints.is_open())
					continue;
				sourcePoints << "Source Points: " << std::endl;
				for (int dd = 0; dd < transformedPoints.size(); dd++)
					sourcePoints << transformedPoints[dd] << std::endl;
				sourcePoints.close();

				std::ofstream targetPoints("targetPoints.txt");
				if (!targetPoints.is_open())
					continue;
				targetPoints << "Target Points: " << std::endl;
				for (int dd = 0; dd < transformedPoints.size(); dd++)
					targetPoints << transformedPoints[dd] << std::endl;
				targetPoints.close();

				std::ofstream matchedPoints("Matches.txt");
				if (!matchedPoints.is_open())
					continue;
				matchedPoints << "Matches: " << std::endl;
				for (int dd = 0; dd < matches.size(); dd++)
					matchedPoints << matches[dd].idx << std::endl;
				matchedPoints.close();

				std::ofstream normalMapFile("NormalMap.txt");
				if (!normalMapFile.is_open())
					continue;
				normalMapFile << "Previous Normal Map: " << std::endl;
				for (int dd = 0; dd < rotatedNormals.size(); dd++)
					normalMapFile << rotatedNormals[dd] << std::endl;
				normalMapFile.close();

				std::cout << "Debugging texts has been created!" << std::endl;
			}

			#ifdef LOG_INFO
			std::cout << "Ceres constraints will be created now ..." << std::endl;
			#endif

			// Prepare point-to-point and point-to-plane constraints.
			ceres::Problem problem;
			prepareConstraints(transformedSrcPoints, target, rotatedNormals, matches, poseIncrement, problem); // INFO: target normals need index of u_hat

			#ifdef LOG_INFO
			std::cout << "Ceres constraints are created. Ceres will try to solve now ..." << std::endl;
			#endif

			// Configure options for the solver.
			ceres::Solver::Options options;
			configureSolver(options);

			#ifdef LOG_INFO
			std::cout << "Solver has run. It will report results now ..." << std::endl;
			#endif

			// Run the solver (for one iteration).
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);

			#ifdef ITER_INFO
			std::cout << "Cost:         " << summary.final_cost<< std::endl;
			std::cout << "Spent time at iteration:       " << summary.total_time_in_seconds << std::endl;
			//std::cout << "Percent of time spent evaluating the jacobian: " << summary.jacobian_evaluation_time_in_seconds/summary.total_time_in_seconds << std::endl;
			#endif

			#ifdef TIMING_INFO
			totalJacobianEvalutionTime += summary.jacobian_evaluation_time_in_seconds;
			totalSolvingTime += summary.total_time_in_seconds;
			#endif

			// Update the current pose estimate (we always update the pose from the left, using left-increment notation).
			Matrix4f matrix = PoseIncrement<double>::convertToMatrix(poseIncrement);
			estimatedPose = PoseIncrement<double>::convertToMatrix(poseIncrement) * estimatedPose;
			poseIncrement.setZero();

			#ifdef LOG_INFO
			std::cout << "Optimization iteration done." << std::endl;
			#endif
		}

		return estimatedPose;
	}

	Matrix4f estimatePose_v3(const PointCloud& source, float* previousDepthMap, const std::vector<Vector3f>& previousNormalMap, const Matrix3f &depthIntrinsics, const Matrix4f &depthExtrinsics,
		const unsigned int &dWidth, const unsigned int &dHeight, const Matrix4f & currentEstimate, const unsigned int& numIters, const bool& analyticJacobian, const Matrix4f & oldPose = Matrix4f::Identity()) {
		// The initial estimate can be given as an argument.
		//Matrix4f estimatedPose = Matrix4f::Identity();	// INFO: Normally this should be true ???
		Matrix4f estimatedPose = currentEstimate;

		// We optimize on the transformation in SE3 notation: 3 parameters for the axis-angle vector of the rotation (its length presents
		// the rotation angle) and 3 parameters for the translation vector. 

		VectorXd poseIncrement(6);

		// Rotate old normals into the global frame N_g,k-1
		auto rotatedNormals = rotatePoints(previousNormalMap, oldPose.block(0, 0, 3, 3));
		//auto rotatedNormals = previousNormalMap;


#ifdef LOG_INFO
		std::cout << "Transformed normals from previous frame into the global frame ..." << std::endl;
#endif

		const float distThres = m_distThreshold;		// 7 cm - Threshold on distance
		const float angleThres = m_angleThreshold; // 0.866025f;	// 0.866025 = cos(30), 0.707107 = cos(45) - Threshold between normals

		float fovX = depthIntrinsics(0, 0);
		float fovY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		Matrix4f oldPoseInv = oldPose.inverse();
		Matrix4f depthExtInv = depthExtrinsics.inverse();

		double curError = std::numeric_limits<double>::infinity();
		double prevError;
		double lambda = 0.7;

		for (int i = 0; i < numIters; ++i) {
			// Compute the matches.

#ifdef LOG_INFO
			std::cout << "Doing projective association of points ..." << std::endl;
#endif LOG_INFO

			clock_t begin = clock();

			// TODO transform points from vertex_k into vertex_k-1
			auto tempTransform = depthExtrinsics * oldPoseInv*estimatedPose;
			auto transformedPoints = transformPoints(source.getPoints(), tempTransform);				// Transformation with left-side multiplication
																										//auto transformedPoints = transformPoints(transformPoints(transformPoints(source.getPoints(), estimatedPose), oldPoseInv), depthExtrinsics);

#ifdef LOG_INFO
			std::cout << "Transformed source points into global frame ..." << std::endl;
#endif LOG_INFO

			// TODO project points to pixels & dehomogenize down to depth frame pixels u_hat (u, v, 1)
			const unsigned nMatches = transformedPoints.size();
			std::vector<Match> matches(nMatches);	// Acts like a boolean for valid matches, -1 otherwise
			std::vector<Vector3f> target(nMatches);
			int k = 0;
			unsigned int cnt = 0;
			for (const auto& pt : transformedPoints) {

				const Vector3f temp = depthIntrinsics * pt;

				unsigned int pixel_u = (unsigned int)std::floor(temp.x() / temp.z());
				unsigned int pixel_v = (unsigned int)std::floor(temp.y() / temp.z());

				unsigned int idx = pixel_v * dWidth + pixel_u;

				if (pixel_u >= dWidth || pixel_u < 0 ||
					pixel_v >= dHeight || pixel_v < 0 ||
					previousDepthMap[idx] == MINF)
				{
					target[k] = Vector3f(MINF, MINF, MINF);
					matches[k].idx = -1;
				}
				else
				{
					const float depth = previousDepthMap[idx];

					// TODO backproject pixels to points V_g,k-1
					target[k] = (oldPose*depthExtInv *Vector4f((pixel_u - cX)*depth / fovX, (pixel_v - cY)*depth / fovY, depth, 1)).block(0, 0, 3, 1);
					/*const float depthScale = depth / temp.z();
					target[k] = (oldPose*depthExtInv *Vector4f(pt.x()*depthScale, pt.y()*depthScale, pt.z()*depthScale, 1)).block(0, 0, 3, 1);*/

					Vector4f temp_srcPt = Vector4f(source.getPoints()[k].x(), source.getPoints()[k].y(), source.getPoints()[k].z(), 1);
					Vector4f temp_trgtPt = Vector4f(target[k].x(), target[k].y(), target[k].z(), 1);

					Vector3f dist_Test = (estimatedPose*temp_srcPt - temp_trgtPt).block(0, 0, 3, 1);
					if (dist_Test.norm() > distThres)
					{
						//target[k] = Vector3f(MINF, MINF, MINF);
						matches[k].idx = -1;
					}
					else
					{
						if (idx > rotatedNormals.size())
						{
							std::cout << "Attempted to index: " << idx << " which is bigger than the size of Normals Vector: " << rotatedNormals.size() << std::endl;
							std::cout << "u: " << pixel_u << "v: " << pixel_v << "width: " << dWidth << "height: " << dHeight << "\n\n";
							//target[k] = Vector3f(MINF, MINF, MINF);
							matches[k].idx = -1;
						}
						else
						{
							Vector3f tempNrm = rotatedNormals[idx];
							if (!tempNrm.allFinite())
							{
								//target[k] = Vector3f(MINF, MINF, MINF);
								matches[k].idx = -1;
							}
							else
							{
								const Vector3f curPt = depthIntrinsics * source.getPoints()[k];
								const unsigned int px_u = (unsigned int)std::floor(curPt.x() / curPt.z());
								const unsigned int px_v = (unsigned int)std::floor(curPt.y() / curPt.z());
								unsigned int px_idx = px_v * dWidth + px_u;

								matches[k].idx = idx;
								cnt++;

								if (px_u < dWidth && px_u >= 0 &&
									px_v < dHeight && px_v >= 0 &&
									source.getRawNormals()[px_idx].allFinite())
								{
									Vector3f curNrm = estimatedPose.block(0, 0, 3, 3)*source.getRawNormals()[px_idx];
									const float nrmAngl = curNrm.dot(tempNrm);

									if (nrmAngl < 0.866025f)	// 0.707107 = cos(45)=sin(45), 0.866025 = cos(30)
									{
										matches[k].idx = -1;	// No need to set weight as this will be discarded anyways
										cnt--;
									}
									matches[k].weight = (0.5f*(distThres - dist_Test.norm()) / distThres + 0.5f*(nrmAngl - angleThres) / (1.0f - angleThres));
								}
								else
									matches[k].weight = (0.5f*(distThres - dist_Test.norm()) / distThres);
							}
						}
					}
				}
				k++;
			}

			clock_t end = clock();
			double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
			std::cout << "Completed in " << elapsedSecs << " seconds." << std::endl;

			MatrixXd srcPts(3, cnt);
			MatrixXd trgtPts(3, cnt);
			MatrixXd NrmlPts(3, cnt);
			VectorXd weights(cnt);
			const Matrix3f rot_Pose = estimatedPose.block(0, 0, 3, 3);
			const Vector3f trans_Pose = estimatedPose.block(0, 3, 3, 1);

			unsigned int m = 0;
			for (unsigned int n = 0; n < nMatches; n++)
			{
				//std::cout << "n: " << n << "col: " << col * 3 << "\n";
				if (matches[n].idx > -1)
				{
					srcPts.col(m) = (rot_Pose*source.getPoints()[n] + trans_Pose).cast<double>();
					trgtPts.col(m) = target[n].cast<double>();
					NrmlPts.col(m) = rotatedNormals[matches[n].idx].cast<double>();
					weights[m] = sqrt(double(matches[n].weight));

					m++;
			}
			}

			//std::cout << "Projected source points into the previous frame ..." << std::endl;
			std::cout << "# of valid matches: " << cnt << std::endl;

			if (cnt == 0)
			{
				std::cout << "WARNING: 0 Valid Matches! Optimization will not run at this iteration!" << std::endl;
				break;
			}

			if (i == 0)
			{
				VectorXd zeroPose(6);
				zeroPose.setZero();
				prevError = calcError(srcPts, trgtPts, NrmlPts, weights, cnt, zeroPose);
				std::cout << "Initial Error: " << prevError << std::endl;
			}

			// Prepare point-to-point and point-to-plane constraints.
			if (analyticJacobian)
				poseIncrement = solveConstraints_v2(srcPts, trgtPts, NrmlPts, weights, cnt, curError, lambda);
			else
				poseIncrement = solveConstraints_Numeric(srcPts, trgtPts, NrmlPts, weights, cnt, curError, lambda); // INFO: target normals need index of u_hat

			totalJacobianEvalutionTime += double(clock() - end) / CLOCKS_PER_SEC;
			totalSolvingTime += double(clock() - end) / CLOCKS_PER_SEC;

			/*std::cout << "Current Error: " << curError << std::endl;
			std::cout << "Pose Increment: \n" << poseIncrement << std::endl;
			std::cout << "Update Matrix: \n" << toUpdateMatrix(poseIncrement) << std::endl;
			std::cout << "Estimated Pose: \n" << toUpdateMatrix(poseIncrement) * estimatedPose << std::endl;*/

			if (curError >= prevError)
			{
				//std::cout << "Current error: " << curError << std::endl;
				lambda = lambda * 5.0;		//L0=5, Lup=2.0, Ldwn=1.125 5,1.5,6
				if (lambda > 25)
				{
					//std::cout << "Loop has terminated early!\n";
					break;
				}

				continue;
			}
			else
			{
				prevError = curError;
				std::cout << "New best error found: " << prevError << std::endl;
				lambda = lambda / 1.25;
			}

			/*if (curError/prevError > 0.995)
			{
			estimatedPose = toUpdateMatrix(poseIncrement) * estimatedPose;
			break;
			}*/
			//prevError = curError;


			// Update the current pose estimate (we always update the pose from the left, using left-increment notation).
			auto poseInc = PoseIncrement<double>(poseIncrement.data());
			estimatedPose = PoseIncrement<double>::convertToMatrix(poseInc) * estimatedPose;

			//estimatedPose = toUpdateMatrix(poseIncrement) * estimatedPose;
		}

		return estimatedPose;
	}

	Matrix4f estimatePoseInPyramid(VirtualSensor& sensor, float* previousDepthMap, const Matrix4f & oldPose = Matrix4f::Identity()) {

		#ifdef TIMING_INFO
		clock_t begin = clock();
		#endif

		Matrix4f estimatedPose = oldPose;

		const unsigned int level = m_nIterationsPyramid.size();
		unsigned int curLevel = level;

		std::vector<std::vector<float>> depthPyramid_source;
		std::vector<std::vector<float>> depthPyramid_target;
		std::vector<PointCloud> sourcePtsPyramid;
		std::vector<std::vector<Vector3f>> normalsPyramid;

		const int dWidth = sensor.getDepthImageWidth();
		const int dHeight = sensor.getDepthImageHeight();
		const Matrix3f depthIntrinsics = sensor.getDepthIntrinsics();
		const Matrix4f depthExtrinsics = sensor.getDepthExtrinsics();

		#ifdef LOG_INFO
		std::cout << "Creating pyramids.." << std::endl;
		#endif

		#ifdef TIMING_INFO
		clock_t buildingPyramid_start = clock();
		#endif

		for (int i = 0; i < level; i++)
		{
			const unsigned int lvlScale = pow(2, i);

			if (i != 0)
			{
				unsigned int temp = lvlScale / 2;

				depthPyramid_source.push_back(downScaleDepth_Average(depthPyramid_source.back(), dWidth / temp, dHeight / temp));
				depthPyramid_target.push_back(downScaleDepth_Average(depthPyramid_target.back(), dWidth / temp, dHeight / temp));
			}
			else {
				depthPyramid_source.push_back(std::vector<float>(&(sensor.getDepth()[0]), &(sensor.getDepth()[dWidth*dHeight])));
				depthPyramid_target.push_back(std::vector<float>(&(previousDepthMap[0]), &(previousDepthMap[dWidth*dHeight])));
			}
		
			Matrix3f newIntrinsic = depthIntrinsics / (float)lvlScale;
			newIntrinsic(8) = 1.0f;

			PointCloud source{ depthPyramid_source.back().data(), newIntrinsic, depthExtrinsics, dWidth / lvlScale, dHeight / lvlScale };
			PointCloud target{ depthPyramid_target.back().data(), newIntrinsic, depthExtrinsics, dWidth / lvlScale, dHeight / lvlScale };
			
			sourcePtsPyramid.push_back(source);
			normalsPyramid.push_back(target.getRawNormals());
		}

		#ifdef LOG_INFO
		std::cout << "Finished creating pyramids. Will try to solve now.." << std::endl;
		#endif

		#ifdef TIMING_INFO
		std::cout << "Built pyramid in [secs]:       " << double(clock() - buildingPyramid_start) / CLOCKS_PER_SEC << std::endl;
		#endif

		/*if (curLevel == 3)
		{
			int cnt = 0;
			/*for (const auto& pt : sourcePtsPyramid.back().getPoints())
				if (pt.allFinite())
					cnt++;
			std::cout << "Number of finite points in source: " << cnt << std::endl;

			cnt = 0;
			for (const auto& pt : normalsPyramid.back())
				if (pt.allFinite())
					cnt++;
			std::cout << "Number of finite normals: " << cnt << std::endl;

			std::vector<float> tempDepth = depthPyramid_target.back();

			std::cout << "Top pyramid level size: " << depthPyramid_target.back().size() << std::endl;
			std::cout << "Current dimensions: " << dHeight / 4 << " " << dWidth / 4 << std::endl;

			cnt = 0;
			for (int v = 0; v < dHeight/4; v++)
				for (int u = 0; u < dWidth/4; u++)
					if (depthPyramid_target.back()[v * dWidth + u] != MINF)
						cnt++;
			std::cout << "Number of finite depths: " << cnt << std::endl;

		}*/

		#ifdef TIMING_INFO
		clock_t estimationPyramid_start = clock();
		#endif

		while (curLevel > 0)
		{
			//std::cout << "Current Pyramid Level : " << curLevel << std::endl;

			const unsigned int lvlScale = pow(2, (curLevel - 1));
			Matrix3f newIntrinsic = depthIntrinsics / (float)lvlScale;
			newIntrinsic(8) = 1.0f;
			unsigned int iterNum = m_nIterationsPyramid.at(curLevel-1);

			#ifdef ITER_INFO
			std::cout << "Pyramid Level:	" << curLevel << std::endl;
			#endif


			/*estimatedPose = estimatePose_v2(sourcePtsPyramid.back(), depthPyramid_target.back().data(), normalsPyramid.back(),
				newIntrinsic, depthExtrinsics, dWidth / lvlScale, dHeight / lvlScale,
				estimatedPose, iterNum, oldPose);*/
			estimatedPose = estimatePose_v3(sourcePtsPyramid.back(), depthPyramid_target.back().data(), normalsPyramid.back(),
				newIntrinsic, depthExtrinsics, dWidth / lvlScale, dHeight / lvlScale,
				estimatedPose, iterNum, true, oldPose);


			depthPyramid_target.pop_back();
			depthPyramid_source.pop_back();
			sourcePtsPyramid.pop_back();
			normalsPyramid.pop_back();

			#ifdef LOG_INFO
			std::cout << "Pyramid Level: " << curLevel << " current estimate: " << estimatedPose.inverse() << std::endl;
			#endif

			curLevel--;
		}

		#ifdef TIMING_INFO
		clock_t end = clock();

		std::cout << "Optimization only took [secs]: " << double(end - estimationPyramid_start) / CLOCKS_PER_SEC << std::endl;
		std::cout << "   Total jacobian evaluation:  " << totalJacobianEvalutionTime << std::endl;
		std::cout << "   Total ceres solution:       " << totalSolvingTime << std::endl;
		std::cout << "   Jacobian eval./solution:    " << (int)(100.0*totalJacobianEvalutionTime / totalSolvingTime) << " %" << std::endl;
		std::cout << "Whole Estimation elapsed secs: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

		totalSolvingTime = 0;
		totalJacobianEvalutionTime = 0;
		#endif

		return estimatedPose;
	}

	std::vector<float> downScaleDepth_Median(float* depthFrame, const unsigned int& width, const unsigned int& height)
	{
		const unsigned int new_width = width / 2;
		const unsigned int new_height = height / 2;

		std::vector<float> result(new_width*new_height);

		for (int v = 0; v < new_height; v++)
		{
			for (int u = 0; u < new_width; u++)
			{
				const unsigned int idx_down = v * new_width + u;
				result[idx_down] = MINF;

				const unsigned int idx_orgnl = 2 * new_width*v * 2 + u * 2;
				std::vector<float> depthVals;

				if (depthFrame[idx_orgnl] != MINF)
					depthVals.push_back(depthFrame[idx_orgnl]);
				if (depthFrame[idx_orgnl + 1] != MINF)
					depthVals.push_back(depthFrame[idx_orgnl + 1]);
				if (depthFrame[idx_orgnl + width] != MINF)
					depthVals.push_back(depthFrame[idx_orgnl + width]);
				if (depthFrame[idx_orgnl + width + 1] != MINF)
					depthVals.push_back(depthFrame[idx_orgnl + width + 1]);
					
				if (depthVals.size() > 0)
				{
					std::sort(depthVals.begin(), depthVals.end());
					result[idx_down] = depthVals[depthVals.size() / 2];
				}

			}
		}
		return result;
	}

	std::vector<float> downScaleDepth_Median(std::vector<float>& depthFrame, const unsigned int& width, const unsigned int& height)
	{

		return downScaleDepth_Median(depthFrame.data(), width, height);
	}

	std::vector<float> downScaleDepth_Average(float* depthFrame, const unsigned int& width, const unsigned int& height)
	{
		const unsigned int new_width = width / 2;
		const unsigned int new_height = height / 2;

		std::vector<float> result(new_width*new_height);

		for (int v = 0; v < new_height; v++)
		{
			for (int u = 0; u < new_width; u++)
			{
				const unsigned int idx_down = v * new_width + u;
				
				const unsigned int idx_orgnl = 2 * new_width*v * 2 + u * 2;
				std::vector<float> depthVals;

				if (depthFrame[idx_orgnl] != MINF)
					depthVals.push_back(depthFrame[idx_orgnl]);
				if (depthFrame[idx_orgnl + 1] != MINF)
					depthVals.push_back(depthFrame[idx_orgnl + 1]);
				if (depthFrame[idx_orgnl + width] != MINF)
					depthVals.push_back(depthFrame[idx_orgnl + width]);
				if (depthFrame[idx_orgnl + width + 1] != MINF)
					depthVals.push_back(depthFrame[idx_orgnl + width + 1]);


				if (depthVals.size() > 0)
				{
					result[idx_down] = std::accumulate(depthVals.begin(), depthVals.end(), 0.0f)/(float)depthVals.size();
				}
				else
					result[idx_down] = MINF;

			}
		}
		return result;
	}

	std::vector<float> downScaleDepth_Average(std::vector<float>& depthFrame, const unsigned int& width, const unsigned int& height)
	{

		return downScaleDepth_Average(depthFrame.data(), width, height);
	}

	std::vector<float> downScaleDepth_Fixed(float* depthFrame, const unsigned int& width, const unsigned int& height)
	{
		const unsigned int new_width = width / 2;
		const unsigned int new_height = height / 2;

		std::vector<float> result(new_width*new_height);

		for (int v = 0; v < new_height; v++)
		{
			for (int u = 0; u < new_width; u++)
			{
				const unsigned int idx_down = v * new_width + u;
				result[idx_down] = MINF;

				const unsigned int idx_orgnl = 2* new_width*v*2 + u*2;

				if (depthFrame[idx_orgnl] != MINF)
					result[idx_down] = depthFrame[idx_orgnl];
				else if (depthFrame[idx_orgnl + 1] != MINF)
					result[idx_down] = depthFrame[idx_orgnl + 1];
				else if (depthFrame[idx_orgnl + width] != MINF)
					result[idx_down] = depthFrame[idx_orgnl + width];
				else if (depthFrame[idx_orgnl + width + 1] != MINF)
					result[idx_down] = depthFrame[idx_orgnl + width + 1];
			}
		}
		return result;
	}

	std::vector<float> downScaleDepth_Fixed(std::vector<float>& depthFrame, const unsigned int& width, const unsigned int& height)
	{
		
		return downScaleDepth_Fixed(depthFrame.data(), width, height);
	}

	Matrix3d toSkewSymm(const Vector3d &pt)
	{
		Matrix3d result;
		result << 0, -pt.z(), pt.y(),
			pt.z(), 0, -pt.x(),
			-pt.y(), pt.x(), 0;

		return result;
	}



private:
	unsigned int m_nIterations;
	std::vector<int> m_nIterationsPyramid;
	float m_distThreshold;
	float m_angleThreshold;
	Matrix4f m_oldPose; 

	std::vector<Vector3f> transformPoints(const std::vector<Vector3f>& sourcePoints, const Matrix4f& pose) {
		std::vector<Vector3f> transformedPoints;
		transformedPoints.reserve(sourcePoints.size());

		const auto rotation = pose.block(0, 0, 3, 3);
		const auto translation = pose.block(0, 3, 3, 1);

		for (const auto& point : sourcePoints) {
			transformedPoints.push_back(rotation * point + translation);
		}

		return transformedPoints;
	}

	std::vector<Vector3f> rotatePoints(const std::vector<Vector3f>& sourcePoints, const Matrix3f& rotation) {
		std::vector<Vector3f> rotatedPoints;
		rotatedPoints.reserve(sourcePoints.size());

		for (const auto& point : sourcePoints) {
			rotatedPoints.push_back(rotation * point);
		}

		return rotatedPoints;
	}

	void configureSolver(ceres::Solver::Options& options) {
		// Ceres options.
		options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
		options.use_nonmonotonic_steps = false;
		options.linear_solver_type = ceres::DENSE_QR;
		options.minimizer_progress_to_stdout = 0;
		options.max_num_iterations = 1; // INFO: Originally 1
		options.num_threads = 8;
		options.logging_type = ceres::LoggingType::SILENT;
	}

	void prepareConstraints(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints, const std::vector<Vector3f>& targetNormals, const std::vector<Match> matches, const PoseIncrement<double>& poseIncrement, ceres::Problem& problem) const {
		const unsigned nPoints = sourcePoints.size();
		const unsigned nNormals = targetNormals.size();

		//static unsigned int invCnt = 0;

		for (unsigned i = 0; i < nPoints; ++i) {
			const auto match = matches[i];
			if (match.idx >= 0) {
				const auto& sourcePoint = sourcePoints[i];
				const auto& targetPoint = targetPoints[i];	// INFO: Originally  ...= targetPoints[match.idx];

				const auto& targetNormal = targetNormals[match.idx];

				if (!sourcePoint.allFinite() || !targetPoint.allFinite() || !targetNormal.allFinite())
				{
					if (!sourcePoint.allFinite())
						std::cout << "Warning: A wrong source point while adding constraints! Point is: " << sourcePoint << std::endl;
					if (!targetPoint.allFinite())
						std::cout << "Warning: A wrong target point while adding constraints! Point is: " << targetPoint << std::endl;
					else
						std::cout << "Warning: A wrong target normal while adding constraints! Normal is: " << targetNormal << std::endl;

					continue;
				};

				ceres::CostFunction* pt2pl_Cost = PointToPlaneConstraint::create(sourcePoint, targetNormal, targetPoint, match.weight);
				problem.AddResidualBlock(pt2pl_Cost, NULL, poseIncrement.getData());

				/*ceres::CostFunction* pt2pt_Cost = PointToPointConstraint::create(sourcePoint, targetPoint, 1.0f);
				problem.AddResidualBlock(pt2pt_Cost, NULL, poseIncrement.getData());*/


			}
		}
	}

	VectorXd solveConstraints_v2(const MatrixXd& sourcePoints, const MatrixXd& targetPoints, const MatrixXd& targetNormals, const VectorXd& weights, const unsigned int& size, double & error, const double&lambda) {

		//std::cout << "Will solve the constraints now..\n";

		MatrixXd A(size, 6);

		for (int i = 0; i < size; i++)
			A.block<1, 3>(i, 0) = sourcePoints.block<3, 1>(0, i).cross(targetNormals.block<3, 1>(0, i));

		A.block(0, 3, size, 3) = targetNormals.transpose();

		/*for (int i = 0; i < 6; i++)
		{
			A.block(0,i,size,1) = weights.cwiseProduct(A.block(0, i, size, 1));
		}*/

		A = weights.asDiagonal()*A;

		MatrixXd H = A.transpose() * A;

		H += lambda * H.diagonal().asDiagonal();

		const VectorXd B = weights.cwiseProduct((targetNormals.transpose()*(targetPoints - sourcePoints)).diagonal());

		VectorXd deltaX(6);

		deltaX = H.ldlt().solve(A.transpose()*B);

		const Vector3d r = deltaX.segment(0, 3);
		const Vector3d t = deltaX.segment(3, 3);

		//error = (-B + weights.cwiseProduct(-B + (r.transpose()*A.block(0, 0, size, 3)).diagonal() + (t.transpose()*A.block(0, 3, size, 3)).diagonal())).squaredNorm() / (double)size;

		VectorXd err(size);  
		for (int i = 0; i < size; i++)
			err(i) = -B(i) + weights(i)*(r.dot(sourcePoints.block<3, 1>(0, i).cross(targetNormals.block<3, 1>(0, i))) + t.dot(targetNormals.block<3, 1>(0, i)));
			//err(i) = weights(i)*(-B(i) + r.dot(Vector3d(sourcePoints.col(i)).cross(Vector3d(targetNormals.col(i)))) + t.dot(Vector3d(targetNormals.col(i))));

		error = err.squaredNorm() / (double)size;

		//std::cout << "Current Error: " << error << std::endl;

		return deltaX;
	}

	VectorXd solveConstraints_Numeric(const MatrixXd& sourcePoints, const MatrixXd& targetPoints, const MatrixXd& targetNormals, const VectorXd& weights, const unsigned int& nPoints, double & error, const double&lambda) {
	
		VectorXd deltaX(6);
		double epsilon = 1e-9;
		const unsigned int size = nPoints;

		VectorXd err(size);
		for (int i = 0; i < size; i++)
		{
			const Vector3d src_pts = sourcePoints.col(i);
			const Vector3d trgt_nrm = targetNormals.col(i);
			const double curErr = (-sourcePoints.col(i) + targetPoints.col(i)).dot(targetNormals.col(i));// +Vector3d(0.0, 0.0, 0.0).dot(src_pts.cross(trgt_nrm)) + Vector3d(0.0, 0.0, 0.0).dot(trgt_nrm);
			err(i) = weights(i)*curErr;
		}

		const Vector3d r_x(epsilon, 0.0, 0.0);
		const Vector3d r_y(0.0, epsilon, 0.0);
		const Vector3d r_z(0.0, 0.0, epsilon);
		const Vector3d t_x(epsilon, 0.0, 0.0);
		const Vector3d t_y(0.0, epsilon, 0.0);
		const Vector3d t_z(0.0, 0.0, epsilon);

		std::vector<Vector3d> rr;
		rr.push_back(r_x);
		rr.push_back(r_y);
		rr.push_back(r_z);
		std::vector<Vector3d> tt;
		tt.push_back(t_x);
		tt.push_back(t_y);
		tt.push_back(t_z);

		MatrixXd J(size, 6);

		for (int j = 0; j < 6; j++) {
			Vector3d r, t;

			if (j < 3)
			{
				r = rr.at(j);
				t = Vector3d(0.0, 0.0, 0.0);
			}
			else
			{
				r = Vector3d(0.0, 0.0, 0.0);
				t = tt.at(j - 3);
			}

			for (int i = 0; i < size; i++)
			{
				const Vector3d src_pts = sourcePoints.col(i);
				const Vector3d trgt_nrm = targetNormals.col(i);
				const double fwd = (sourcePoints.col(i) - targetPoints.col(i)).dot(targetNormals.col(i)) + r.dot(src_pts.cross(trgt_nrm)) + t.dot(trgt_nrm);
				const double bwd = (sourcePoints.col(i) - targetPoints.col(i)).dot(targetNormals.col(i)) + (-r).dot(src_pts.cross(trgt_nrm)) + (-t).dot(trgt_nrm);
				J(i, j) = (weights(i)*weights(i))*(fwd*fwd - bwd * bwd) / (2.0 * epsilon);
			}
		}

		MatrixXd H = J.transpose()*J;
		const MatrixXd damping = lambda * H.diagonal().asDiagonal();
		H += damping;

		//VectorXd B = J.transpose()*err;	// Actually here err is equal to -error

		deltaX = (H.ldlt().solve(J.transpose()*err));

		const Vector3d r = deltaX.segment(0, 3);
		const Vector3d t = deltaX.segment(3, 3);
		for (int i = 0; i < size; i++)
		{
			const Vector3d src_pts = sourcePoints.col(i);
			const Vector3d trgt_nrm = targetNormals.col(i);
			const double curErr = (sourcePoints.col(i) - targetPoints.col(i)).dot(targetNormals.col(i)) + r.dot(src_pts.cross(trgt_nrm)) + t.dot(trgt_nrm);
			err(i) = weights(i)*curErr;
		}

		error = err.squaredNorm() / (double)size;

		return deltaX;
	}

	const Matrix4f toUpdateMatrix(const VectorXd& poseIncrement) {
		Matrix4f result;
		result << 1.0, poseIncrement(2), -poseIncrement(1), poseIncrement(3),
			-poseIncrement(2), 1.0, poseIncrement(0), poseIncrement(4),
			poseIncrement(1), -poseIncrement(0), 1.0, poseIncrement(5),
			0.0, 0.0, 0.0, 1.0;

		return result;
	}

};
