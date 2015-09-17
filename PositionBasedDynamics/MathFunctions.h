#ifndef MATH_FUNCTIONS_H
#define MATH_FUNCTIONS_H

#include <Eigen/Dense>

// ------------------------------------------------------------------------------------
namespace PBD
{
	class MathFunctions
	{
	private:
		static void jacobiRotate(Eigen::Matrix3f &A,
			Eigen::Matrix3f &R,
			int p,
			int q);

	public:
		static float infNorm(const Eigen::Matrix3f &A);
		static float oneNorm(const Eigen::Matrix3f &A);

		static void eigenDecomposition(const Eigen::Matrix3f &A,
			Eigen::Matrix3f &eigenVecs,
			Eigen::Vector3f &eigenVals);

		static void polarDecomposition(const Eigen::Matrix3f &A,
			Eigen::Matrix3f &R,
			Eigen::Matrix3f &U,
			Eigen::Matrix3f &D);

		static void polarDecompositionStable(const Eigen::Matrix3f &M,
			const float tolerance,
			Eigen::Matrix3f &R);

		static void svdWithInversionHandling(const Eigen::Matrix3f &A,
			Eigen::Vector3f &sigma,
			Eigen::Matrix3f &U,
			Eigen::Matrix3f &VT);

		static float cotTheta(const Eigen::Vector3f &v, const Eigen::Vector3f &w);
	};
}

#endif