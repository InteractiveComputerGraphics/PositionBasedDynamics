#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>

#define USE_DOUBLE

#ifdef USE_DOUBLE
typedef double Real;

#define REAL_MAX DBL_MAX
#define REAL_MIN DBL_MIN
#else
typedef float Real;

#define REAL_MAX FLT_MAX
#define REAL_MIN FLT_MIN
#endif

namespace PBD
{
	using Vector2r = Eigen::Matrix<Real, 2, 1>;
	using Vector3r = Eigen::Matrix<Real, 3, 1>;
	using Vector4r = Eigen::Matrix<Real, 4, 1>;
	using Matrix2r = Eigen::Matrix<Real, 2, 2>;
	using Matrix3r = Eigen::Matrix<Real, 3, 3>;
	using Matrix4r = Eigen::Matrix<Real, 4, 4>;
	using AlignedBox2r = Eigen::AlignedBox<Real, 2>;
	using AlignedBox3r = Eigen::AlignedBox<Real, 3>;
	using AngleAxisr = Eigen::AngleAxis<Real>;
	using Quaternionr = Eigen::Quaternion<Real>;
}

#endif
