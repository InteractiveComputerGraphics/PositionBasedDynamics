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

	//allocators to be used in STL collections containing Eigen structures
	using Alloc_Vector2r = Eigen::aligned_allocator<Vector2r>;
	using Alloc_Vector3r = Eigen::aligned_allocator<Vector2r>;
	using Alloc_Vector4r = Eigen::aligned_allocator<Vector4r>;
	using Alloc_Matrix2r = Eigen::aligned_allocator<Matrix2r>;
	using Alloc_Matrix3r = Eigen::aligned_allocator<Matrix3r>;
	using Alloc_Matrix4r = Eigen::aligned_allocator<Matrix4r>;
	using Alloc_AlignedBox2r = Eigen::aligned_allocator<AlignedBox2r>;
	using Alloc_AlignedBox3r = Eigen::aligned_allocator<AlignedBox3r>;
	using Alloc_AngleAxisr = Eigen::aligned_allocator<AngleAxisr>;
	using Alloc_Quaternionr = Eigen::aligned_allocator<Quaternionr>;

#if EIGEN_ALIGN
	#define PDB_MAKE_ALIGNED_OPERATOR_NEW EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#if defined(WIN32) || defined(_WIN32) || defined(WIN64)	   
#ifdef _DEBUG
	// Enable memory leak detection for Eigen new
	#undef PDB_MAKE_ALIGNED_OPERATOR_NEW
	#define PDB_MAKE_ALIGNED_OPERATOR_NEW	EIGEN_MAKE_ALIGNED_OPERATOR_NEW \
		void *operator new(size_t size, int const block_use, char const*  file_name, int const line_number) { \
		\
			return _aligned_malloc_dbg(size, 16, file_name, line_number); \
		} \
		void *operator new[](size_t size, int const block_use, char const*  file_name, int const line_number) { \
			return operator new(size, block_use, file_name, line_number); \
		}\
		void operator delete(void* block, int const block_use, char const*  file_name, int const line_number) noexcept { \
		\
			return _aligned_free_dbg(block); \
		} \
		void operator delete[](void* block, int const block_use, char const*  file_name, int const line_number) noexcept { \
			return operator delete(block, block_use, file_name, line_number); \
		}	

#endif
#endif
#else
	#define PDB_MAKE_ALIGNED_OPERATOR_NEW
#endif
}

#endif
