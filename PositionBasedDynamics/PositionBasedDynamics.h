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
			const float restLength,
			const float compressionStiffness,
			const float stretchStiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1);


		static bool solveDihedralConstraint(
			const Eigen::Vector3f &p0, float invMass0,		// angle on (p2, p3) between triangles (p0, p2, p3) and (p1, p3, p2)
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Vector3f &p3, float invMass3,
			const float restAngle,
			const float stiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);


		static bool solveVolumeConstraint(
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Vector3f &p3, float invMass3,
			const float restVolume,
			const float negVolumeStiffness,
			const float posVolumeStiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);

		static bool solveEdgePointDistConstraint(
			const Eigen::Vector3f &p, float invMass,
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const float restDist,
			const float compressionStiffness,
			const float stretchStiffness,
			Eigen::Vector3f &corr, Eigen::Vector3f &corr0, Eigen::Vector3f &corr1);

		static bool solveTrianglePointDistConstraint(
			const Eigen::Vector3f &p, float invMass,
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const float restDist,
			const float compressionStiffness,
			const float stretchStiffness,
			Eigen::Vector3f &corr, Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2);

		static bool solveEdgeEdgeDistConstraint(
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Vector3f &p3, float invMass3,
			const float restDist,
			const float compressionStiffness,
			const float stretchStiffness,
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
			const float stiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);

		// -------------- Shape Matching  -----------------------------------------------------

		static bool computeShapeMatchingRestInfo(
			const Eigen::Vector3f x0[], const float invMasses[], const int numPoints,
			Eigen::Vector3f &restCm, Eigen::Matrix3f &invRestMat);

		static bool solveShapeMatchingConstraint(
			const Eigen::Vector3f x0[], const Eigen::Vector3f x[], const float invMasses[], const int numPoints,
			const Eigen::Vector3f &restCm, 
			const Eigen::Matrix3f &invRestMat,
			const float stiffness,
			const bool allowStretch,		// default false
			Eigen::Vector3f corr[], Eigen::Matrix3f *rot = NULL);


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
			const float xxStiffness, 
			const float yyStiffness, 
			const float xyStiffness,
			const bool normalizeStretch,		// use false as default
			const bool normalizeShear,		// use false as default
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
			const bool normalizeStretch,		// use false as default
			const bool normalizeShear,		// use false as default
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
			const float youngsModulusX,
			const float youngsModulusY,
			const float youngsModulusShear,
			const float poissonRatioXY,
			const float poissonRatioYX,
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
			const float youngsModulus,
			const float poissonRatio,
			const bool  handleInversion,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);


		// -------------- Position Based Fluids  -----------------------------------------------------
		
		static bool computePBFDensity(
			const unsigned int particleIndex,				// current fluid particle	
			const unsigned int numberOfParticles,			// number of fluid particles 
			const Eigen::Vector3f x[],						// array of all particle positions
			const float mass[],								// array of all particle masses
			const Eigen::Vector3f boundaryX[],				// array of all boundary particles
			const float boundaryPsi[],						// array of all boundary psi values (Akinci2012)
			const unsigned int numNeighbors,				// number of neighbors 
			const unsigned int neighbors[],					// array with indices of all neighbors (indices larger than numberOfParticles are boundary particles)
			const float density0,							// rest density
			const bool boundaryHandling,					// perform boundary handling (Akinci2012)
			float &density_err,								// returns the clamped density error (can be used for enforcing a maximal global density error)
			float &density);								// return the density

		static bool computePBFLagrangeMultiplier(
			const unsigned int particleIndex,				// current fluid particle	
			const unsigned int numberOfParticles,			// number of fluid particles 
			const Eigen::Vector3f x[],						// array of all particle positions
			const float mass[],								// array of all particle masses
			const Eigen::Vector3f boundaryX[],				// array of all boundary particles
			const float boundaryPsi[],						// array of all boundary psi values (Akinci2012)
			const float density,							// density of current fluid particle
			const unsigned int numNeighbors,				// number of neighbors 
			const unsigned int neighbors[],					// array with indices of all neighbors
			const float density0,							// rest density
			const bool boundaryHandling,					// perform boundary handling (Akinci2012)
			float &lambda);									// returns the Lagrange multiplier

		static bool solveDensityConstraint(
			const unsigned int particleIndex,				// current fluid particle	
			const unsigned int numberOfParticles,			// number of fluid particles 
			const Eigen::Vector3f x[],						// array of all particle positions
			const float mass[],								// array of all particle masses
			const Eigen::Vector3f boundaryX[],				// array of all boundary particles
			const float boundaryPsi[],						// array of all boundary psi values (Akinci2012)
			const unsigned int numNeighbors,				// number of neighbors 
			const unsigned int neighbors[],					// array with indices of all neighbors
			const float density0,							// rest density
			const bool boundaryHandling,					// perform boundary handling (Akinci2012)
			const float lambda[],							// Lagrange multiplier
			Eigen::Vector3f &corr);							// returns the position correction for the current fluid particle
	};
}

#endif