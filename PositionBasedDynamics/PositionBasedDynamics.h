#ifndef POSITION_BASED_DYNAMICS_H
#define POSITION_BASED_DYNAMICS_H

#include <Eigen/Dense>

// ------------------------------------------------------------------------------------
namespace PBD
{
	class PositionBasedDynamics
	{
	public:
		// -------------- standard PBD -----------------------------------------------------

		static bool solveDistanceConstraint(
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			float restLength,
			float compressionStiffness,
			float stretchStiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1);


		static bool solveDihedralConstraint(
			const Eigen::Vector3f &p0, float invMass0,		// angle on (p2, p3) between triangles (p0, p2, p3) and (p1, p3, p2)
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Vector3f &p3, float invMass3,
			float restAngle,
			float stiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);


		static bool solveVolumeConstraint(
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Vector3f &p3, float invMass3,
			float restVolume,
			float negVolumeStiffness,
			float posVolumeStiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);

		static bool computeShapeMatchingRestInfo(
			const Eigen::Vector3f x0[], const float invMasses[], int numPoints,
			Eigen::Vector3f &restCm, Eigen::Matrix3f &invRestMat);

		static bool shapeMatchingConstraint(
			const Eigen::Vector3f x0[], const Eigen::Vector3f x[], const float invMasses[], int numPoints,
			Eigen::Vector3f &restCm, const Eigen::Matrix3f &invRestMat,
			float stiffness,
			bool allowStretch,		// default false
			Eigen::Vector3f corr[], Eigen::Matrix3f *rot = NULL);

		static bool solveEdgePointDistConstraint(
			const Eigen::Vector3f &p, float invMass,
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			float restDist,
			float compressionStiffness,
			float stretchStiffness,
			Eigen::Vector3f &corr, Eigen::Vector3f &corr0, Eigen::Vector3f &corr1);

		static bool solveTrianglePointDistConstraint(
			const Eigen::Vector3f &p, float invMass,
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			float restDist,
			float compressionStiffness,
			float stretchStiffness,
			Eigen::Vector3f &corr, Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2);

		static bool solveEdgeEdgeDistConstraint(
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Vector3f &p3, float invMass3,
			float restDist,
			float compressionStiffness,
			float stretchStiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);


		// -------------- Isometric bending -----------------------------------------------------

		static float cotTheta(const Eigen::Vector3f &v, const Eigen::Vector3f &w);

		static bool computeQuadraticBendingMat(		// compute only when rest shape changes, angle on (p2, p3) between triangles (p0, p2, p3) and (p1, p3, p2)
			const Eigen::Vector3f &p0,
			const Eigen::Vector3f &p1,
			const Eigen::Vector3f &p2,
			const Eigen::Vector3f &p3,
			Eigen::Matrix4f &Q
			);

		static bool solveIsometricBendingConstraint(
			const Eigen::Vector3f &p0, float invMass0,		// angle on (p2, p3) between triangles (p0, p2, p3) and (p1, p3, p2)
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Vector3f &p3, float invMass3,
			const Eigen::Matrix4f &Q,
			float stiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);

		// -------------- Strain Based Dynamics  -----------------------------------------------------

		static bool computeStrainTriangleInvRestMat(		// compute only when rest shape changes, triangle in the xy plane
			const Eigen::Vector3f &p0,
			const Eigen::Vector3f &p1,
			const Eigen::Vector3f &p2,
			Eigen::Matrix2f &invRestMat
			);

		static bool solveStrainTriangleConstraint(
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Matrix2f &invRestMat,
			float xxStiffness, float yyStiffness, float xyStiffness,
			bool normalizeStretch,		// use false as default
			bool normalizeShear,		// use false as default
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2);


		static bool computeStrainTetraInvRestMat(		// compute only when rest shape changes
			const Eigen::Vector3f &p0,
			const Eigen::Vector3f &p1,
			const Eigen::Vector3f &p2,
			const Eigen::Vector3f &p3,
			Eigen::Matrix3f &invRestMat
			);


		// has no inversion handling. Possible simple solution: if the volume is negative, 
		// scale corrections down and use the volume constraint to fix the volume sign
		static bool solveStrainTetraConstraint(
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Vector3f &p3, float invMass3,
			const Eigen::Matrix3f &invRestMat,
			const Eigen::Vector3f &stretchStiffness,			// xx, yy, zz
			const Eigen::Vector3f &shearStiffness,			// xy, xz, yz
			bool normalizeStretch,		// use false as default
			bool normalizeShear,		// use false as default
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);


		// -------------- FEM Based PBD  -----------------------------------------------------

		static bool computeFEMTriangleInvRestMat(		// compute only when rest shape changes
			const Eigen::Vector3f &p0,
			const Eigen::Vector3f &p1,
			const Eigen::Vector3f &p2,
			float &area,
			Eigen::Matrix2f &invRestMat
			);

		static bool solveFEMTriangleConstraint(
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const float &area,
			const Eigen::Matrix2f &invRestMat,
			float youngsModulusX,
			float youngsModulusY,
			float youngsModulusShear,
			float poissonRatioXY,
			float poissonRatioYX,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2);


		static bool computeFEMTetraInvRestMat(			// compute only when rest shape changes
			const Eigen::Vector3f &p0,
			const Eigen::Vector3f &p1,
			const Eigen::Vector3f &p2,
			const Eigen::Vector3f &p3,
			float &volume,
			Eigen::Matrix3f &invRestMat
			);

		static bool solveFEMTetraConstraint(
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Vector3f &p3, float invMass3,
			const float restVolume,
			const Eigen::Matrix3f &invRestMat,
			float youngsModulus,
			float poissonRatio,
			bool  handleInversion,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);
	};
}

#endif