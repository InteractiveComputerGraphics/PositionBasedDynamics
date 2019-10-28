#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include <float.h>

//#define USE_DOUBLE
#define MIN_PARALLEL_SIZE 64

#ifdef USE_DOUBLE
typedef double Real;

#define REAL_MAX DBL_MAX
#define REAL_MIN DBL_MIN
#define RealParameter DoubleParameter
#define RealParameterType ParameterBase::DOUBLE
#define RealVectorParameterType ParameterBase::VEC_DOUBLE
#else
typedef float Real;

#define REAL_MAX FLT_MAX
#define REAL_MIN FLT_MIN
#define RealParameter FloatParameter
#define RealParameterType ParameterBase::FLOAT
#define RealVectorParameterType ParameterBase::VEC_FLOAT
#endif

using Vector2r = Eigen::Matrix<Real, 2, 1, Eigen::DontAlign>;
using Vector3r = Eigen::Matrix<Real, 3, 1, Eigen::DontAlign>;
using Vector4r = Eigen::Matrix<Real, 4, 1, Eigen::DontAlign>;
using Vector5r = Eigen::Matrix<Real, 5, 1, Eigen::DontAlign>;
using Vector6r = Eigen::Matrix<Real, 6, 1, Eigen::DontAlign>;
using Matrix2r = Eigen::Matrix<Real, 2, 2, Eigen::DontAlign>;
using Matrix3r = Eigen::Matrix<Real, 3, 3, Eigen::DontAlign>;
using Matrix4r = Eigen::Matrix<Real, 4, 4, Eigen::DontAlign>;
using AlignedBox2r = Eigen::AlignedBox<Real, 2>;
using AlignedBox3r = Eigen::AlignedBox<Real, 3>;
using AngleAxisr = Eigen::AngleAxis<Real>;
using Quaternionr = Eigen::Quaternion<Real, Eigen::DontAlign>;

#if defined(WIN32) || defined(_WIN32) || defined(WIN64)
	// Enable memory leak detection
#ifdef _DEBUG
	#define _CRTDBG_MAP_ALLOC 
	#include <stdlib.h>
	#include <crtdbg.h>
	#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__) 	
	#define REPORT_MEMORY_LEAKS _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );
#else
	#define REPORT_MEMORY_LEAKS
#endif
#else
	#define REPORT_MEMORY_LEAKS
	#define DEBUG_NEW new
#endif

#endif


#if defined(WIN32) || defined(_WIN32) || defined(WIN64)
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE __attribute__((always_inline))
#endif


