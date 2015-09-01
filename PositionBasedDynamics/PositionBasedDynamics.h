#ifndef POSITION_BASED_DYNAMICS_H
#define POSITION_BASED_DYNAMICS_H

#include <Eigen/Dense>

// ------------------------------------------------------------------------------------
namespace PBD
{
	class MathFunctions
	{
		// -------------- required methods -----------------------------------------------------
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
	private:
		static void computeGradCGreen(
			float restVolume, 
			const Eigen::Matrix3f &invRestMat, 
			const Eigen::Matrix3f &sigma, 
			Eigen::Vector3f *J);

		static void computeGreenStrainAndPiolaStress(
			const Eigen::Vector3f &x1, const Eigen::Vector3f &x2, const Eigen::Vector3f &x3, const Eigen::Vector3f &x4,
			const Eigen::Matrix3f &invRestMat,
			const float restVolume,
			const float mu, const float lambda,
			Eigen::Matrix3f &epsilon, Eigen::Matrix3f &sigma, float &energy);

		static void computeGreenStrainAndPiolaStressInversion(
			const Eigen::Vector3f &x1, const Eigen::Vector3f &x2, const Eigen::Vector3f &x3, const Eigen::Vector3f &x4,
			const Eigen::Matrix3f &invRestMat,
			const float restVolume,
			const float mu, const float lambda,
			Eigen::Matrix3f &epsilon, Eigen::Matrix3f &sigma, float &energy);


	public:
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


		// -------------- Position Based Rigid Body Dynamics  -----------------------------------------------------
	private:
		static void computeMatrixK(
			const Eigen::Vector3f &connector,
			const float mass,
			const Eigen::Vector3f &x,
			const Eigen::Matrix3f &inertiaInverseW,
			Eigen::Matrix3f &K);

	public:
		static bool solveRigidBodyBallJoint(
			const Eigen::Vector3f &connector0,				// connector of balljoint in body 0
			const float mass0,								// mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const Eigen::Vector3f &connector1,				// connector of balljoint in body 1
			const float mass1,								// mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1);
	};
}

#endif