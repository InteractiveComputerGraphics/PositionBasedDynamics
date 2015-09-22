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

		/** Determine the position corrections for a distance constraint between two particles:\n\n
		* \f$C(\mathbf{p}_0, \mathbf{p}_1) = \| \mathbf{p}_0 - \mathbf{p}_1\| - l_0 = 0\f$\n\n
		* More information can be found in the following papers: \cite Mueller07, \cite BMOT2013, \cite BMOTM2014, \cite BMM2015, 
		*
		* @param p0 position of first particle 
		* @param invMass0 inverse mass of first particle
		* @param p1 position of second particle
		* @param invMass1 inverse mass of second particle
		* @param restLength rest length of distance constraint
		* @param compressionStiffness stiffness coefficient for compression
		* @param stretchStiffness stiffness coefficient for stretching
		* @param corr0 position correction of first particle
		* @param corr1 position correction of second particle
		*/
		static bool solveDistanceConstraint(
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const float restLength,
			const float compressionStiffness,
			const float stretchStiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1);


		/** Determine the position corrections for a dihedral bending constraint. 
		 * For a pair of adjacent triangles 
		 * \f$(\mathbf{p}_1, \mathbf{p}_3, \mathbf{p}_2)\f$ and 
		 * \f$(\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_4)\f$ 
		 * with the common edge \f$(\mathbf{p}_3, \mathbf{p}_4)\f$ a bilateral bending 
		 * constraint is added by the constraint function
		 * \f{equation*}{
		 * C_{bend}(\mathbf{p}_1, \mathbf{p}_2,\mathbf{p}_3, \mathbf{p}_4) =
		 * \text{acos}\left( \frac{\mathbf{p}_{2,1} \times
		 * \mathbf{p}_{3,1}}{|\mathbf{p}_{2,1} \times
		 * \mathbf{p}_{3,1}|} \cdot
		 * \frac{\mathbf{p}_{2,1} \times
		 * \mathbf{p}_{4,1}}{|\mathbf{p}_{2,1} \times
		 *	\mathbf{p}_{4,1}|}\right)-\varphi_0
		 * \f}
		 * and stiffness \f$k_{bend}\f$. The scalar \f$\varphi_0\f$
		 * is the initial dihedral angle between the two triangles and
		 * \f$k_{bend}\f$ is a global user parameter defining the bending stiffness.\n\n
		 * More information can be found in the following papers: \cite Mueller07, \cite BMOT2013, \cite BMOTM2014, \cite BMM2015,
		 *
		 * @param p0 position of first particle
		 * @param invMass0 inverse mass of first particle
		 * @param p1 position of second particle
		 * @param invMass1 inverse mass of second particle
		 * @param p2 position of third particle
		 * @param invMass2 inverse mass of third particle
		 * @param p3 position of fourth particle
		 * @param invMass3 inverse mass of fourth particle
		 * @param restAngle rest angle \f$\varphi_0\f$ 
		 * @param stiffness stiffness coefficient
		 * @param corr0 position correction of first particle
		 * @param corr1 position correction of second particle
		 * @param corr2 position correction of third particle
		 * @param corr3 position correction of fourth particle
		 */
		static bool solveDihedralConstraint(
			const Eigen::Vector3f &p0, float invMass0,		// angle on (p2, p3) between triangles (p0, p2, p3) and (p1, p3, p2)
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Vector3f &p3, float invMass3,
			const float restAngle,
			const float stiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);


		/** Determine the position corrections for a constraint that conserves the volume
		* of single tetrahedron. Such a constraint has the form
		* \f{equation*}{
		* C(\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3, \mathbf{p}_4) = \frac{1}{6}
		* \left(\mathbf{p}_{2,1} \times \mathbf{p}_{3,1}\right) \cdot \mathbf{p}_{4,1} - V_0,
		* \f}
		* where \f$\mathbf{p}_1\f$, \f$\mathbf{p}_2\f$, \f$\mathbf{p}_3\f$ and \f$\mathbf{p}_4\f$ 
		* are the four corners of the tetrahedron and \f$V_0\f$ is its rest volume.\n\n
		* More information can be found in the following papers: \cite Mueller07, \cite BMOT2013, \cite BMOTM2014, \cite BMM2015,
		*
		* @param p0 position of first particle
		* @param invMass0 inverse mass of first particle
		* @param p1 position of second particle
		* @param invMass1 inverse mass of second particle
		* @param p2 position of third particle
		* @param invMass2 inverse mass of third particle
		* @param p3 position of fourth particle
		* @param invMass3 inverse mass of fourth particle
		* @param restVolume rest angle \f$V_0\f$
		* @param negVolumeStiffness stiffness coefficient for compression
		* @param posVolumeStiffness stiffness coefficient for stretching
		* @param corr0 position correction of first particle
		* @param corr1 position correction of second particle
		* @param corr2 position correction of third particle
		* @param corr3 position correction of fourth particle
		*/
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

		/** Initialize the local stiffness matrix Q. The matrix is 
		 * required by the solver step. It must only be recomputed 
		 * if the rest shape changes. \n\n
		 * Bending is simulated for the angle on (p2, p3) between 
		 * the triangles (p0, p2, p3) and (p1, p3, p2).
		 *
		 * @param  p0 point 0 of stencil
		 * @param  p1 point 1 of stencil
		 * @param  p2 point 2 of stencil
		 * @param  p3 point 3 of stencil 
		 * @param  Q returns the local stiffness matrix which is required by the solver
		 */	
		static bool initQuadraticBendingMat(		
			const Eigen::Vector3f &p0,
			const Eigen::Vector3f &p1,
			const Eigen::Vector3f &p2,
			const Eigen::Vector3f &p3,
			Eigen::Matrix4f &Q
			);

		/** Determine the position corrections for the isometric bending constraint.
		 * This constraint can be used for almost inextensible surface models.\n\n
		 * More information can be found in: \cite BMM2015, \cite Bender2014
		 *
		 * @param p0 position of first particle
		 * @param invMass0 inverse mass of first particle
		 * @param p1 position of second particle
		 * @param invMass1 inverse mass of second particle
		 * @param p2 position of third particle
		 * @param invMass2 inverse mass of third particle
		 * @param p3 position of fourth particle
		 * @param invMass3 inverse mass of fourth particle
		 * @param  Q local stiffness matrix which must be initialized by calling initQuadraticBendingMat() 
		 * @param  stiffness stiffness coefficient for bending
		 * @param corr0 position correction of first particle
		 * @param corr1 position correction of second particle
		 * @param corr2 position correction of third particle
		 * @param corr3 position correction of fourth particle
		 */	
		static bool solveIsometricBendingConstraint(
			const Eigen::Vector3f &p0, float invMass0,		// angle on (p2, p3) between triangles (p0, p2, p3) and (p1, p3, p2)
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Vector3f &p3, float invMass3,
			const Eigen::Matrix4f &Q,
			const float stiffness,
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3);

		// -------------- Shape Matching  -----------------------------------------------------

		/** Initialize rest configuration infos for one shape matching cluster 
		 * which are required by the solver step. It must only be reinitialized
		 * if the rest shape changes. 
		 *
		 * @param  x0 rest configuration of all particles in the cluster
		 * @param  invMasses inverse masses of all particles in the cluster
		 * @param  numPoints number of particles in the cluster
		 * @param  restCm returns the center of mass of the rest configuration
		 * @param  invRestMat returns a matrix required by the solver
		 */	
		static bool initShapeMatchingRestInfo(
			const Eigen::Vector3f x0[], const float invMasses[], const int numPoints,
			Eigen::Vector3f &restCm, Eigen::Matrix3f &invRestMat);

		/** Determine the position corrections for a shape matching constraint.\n\n
		 * More information can be found in: \cite BMM2015, \cite BMOT2013, \cite BMOTM2014, 
		 * \cite Muller2005, \cite Bender2013, \cite Diziol2011
		 *
		 * @param  x0			rest configuration of all particles in the cluster
		 * @param  x			current configuration of all particles in the cluster
		 * @param  invMasses	invMasses inverse masses of all particles in the cluster
		 * @param  numPoints	number of particles in the cluster
		 * @param  restCm		center of mass of the rest configuration
		 * @param  invRestMat	matrix precomputed by initShapeMatchingRestInfo()
		 * @param  stiffness	stiffness coefficient 
		 * @param  allowStretch allow stretching
		 * @param  corr			position corrections for all particles in the cluster
		 * @param  rot			returns determined rotation matrix 
		 */	
		static bool solveShapeMatchingConstraint(
			const Eigen::Vector3f x0[], const Eigen::Vector3f x[], const float invMasses[], const int numPoints,
			const Eigen::Vector3f &restCm, 
			const Eigen::Matrix3f &invRestMat,
			const float stiffness,
			const bool allowStretch,		// default false
			Eigen::Vector3f corr[], Eigen::Matrix3f *rot = NULL);


		// -------------- Strain Based Dynamics  -----------------------------------------------------

		/** Initialize rest configuration infos which are required by the solver step.
		 * Recomputation is only necessary when rest shape changes.\n\n
		 * The triangle is defined in the xy plane.
		 *
		 * @param  p0 point 0 of triangle
		 * @param  p1 point 1 of triangle
		 * @param  p2 point 2 of triangle
		 * @param  invRestMat returns a matrix required by the solver
		 */		
		static bool initStrainTriangleInvRestMat(		
			const Eigen::Vector3f &p0,
			const Eigen::Vector3f &p1,
			const Eigen::Vector3f &p2,
			Eigen::Matrix2f &invRestMat
			);

		/** Solve triangle constraint with strain based dynamics and return position corrections.\n\n
		 * More information can be found in: \cite BMM2015, \cite Mueller2014
		 *
		 * @param p0 position of first particle
		 * @param invMass0 inverse mass of first particle
		 * @param p1 position of second particle
		 * @param invMass1 inverse mass of second particle
		 * @param p2 position of third particle
		 * @param invMass2 inverse mass of third particle
		 * @param  invRestMat precomputed matrix determined by initStrainTriangleInvRestMat()
		 * @param  xxStiffness stiffness coefficient for xx stretching
		 * @param  yyStiffness stiffness coefficient for yy stretching
		 * @param  xyStiffness stiffness coefficient for xy shearing
		 * @param  normalizeStretch	should stretching be normalized
		 * @param  normalizeShear should shearing be normalized
		 * @param  corr0 position correction for point 0
		 * @param  corr1 position correction for point 1
		 * @param  corr2 position correction for point 2
		 */		
		static bool solveStrainTriangleConstraint(
			const Eigen::Vector3f &p0, float invMass0,
			const Eigen::Vector3f &p1, float invMass1,
			const Eigen::Vector3f &p2, float invMass2,
			const Eigen::Matrix2f &invRestMat,
			const float xxStiffness, 
			const float yyStiffness, 
			const float xyStiffness,
			const bool normalizeStretch,	// use false as default
			const bool normalizeShear,		// use false as default
			Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2);

		/** Initialize rest configuration infos which are required by the solver step.
		 * Recomputation is only necessary when rest shape changes.
		 *
		 * @param  p0 point 0 of tet
		 * @param  p1 point 1 of tet
		 * @param  p2 point 2 of tet
		 * @param  p3 point 3 of tet
		 * @param  invRestMat returns a matrix required by the solver
		 */
		static bool initStrainTetraInvRestMat(	
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
		/** Initialize rest configuration infos which are required by the solver step.
		* Recomputation is only necessary when rest shape changes.
		*/
		static bool initFEMTriangleInvRestMat(		
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

		/** Initialize rest configuration infos which are required by the solver step.
		* Recomputation is only necessary when rest shape changes.
		*/
		static bool initFEMTetraInvRestMat(			
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
	};
}

#endif