#ifndef POSITION_BASED_ELASTIC_RODS_H
#define POSITION_BASED_ELASTIC_RODS_H

#include <Eigen/Dense>
#include "Common/Common.h"

// ------------------------------------------------------------------------------------
namespace PBD
{
	// Implementation of "Position And Orientation Based Cosserat Rods" paper
	// (https://animation.rwth-aachen.de/publication/0550/)
	//
	//	Implemented by:
	//
	//	Tassilo Kugelstadt
	//	Computer Animation Group
	//	RWTH Aachen University
	//
	//  kugelstadt[at] cs.rwth-aachen.de
	//
	class PositionBasedCosseratRods
	{
	public:
		/** Determine the position and orientation corrections for the stretch and shear constraint constraint (eq. 37 in the paper). \n\n
		*
		* @param p0 position of first particle
		* @param invMass0 inverse mass of first particle
		* @param p1 position of second particle
		* @param invMass1 inverse mass of second particle
		* @param q0 quaternion at the center of the edge
		* @param invMassq0 inverse mass of the quaternion
		* @param stretchingAndShearingKs stiffness coefficients for stretching and shearing
		* @param restLength rest edge length
		* @param corr0 position correction of first particle
		* @param corr1 position correction of second particle
		* @param corrq0 orientation correction of quaternion
		*/
		static bool solve_StretchShearConstraint(
			const Vector3r& p0, Real invMass0,
			const Vector3r& p1, Real invMass1,
			const Quaternionr& q0, Real invMassq0,
			const Vector3r& stretchingAndShearingKs,
			const Real restLength,
			Vector3r& corr0, Vector3r&  corr1, Quaternionr&  corrq0);

		/** Determine the position corrections for the bending and torsion constraint constraint (eq. 40 in the paper). \n\n
		*
		* @param q0 first quaternion
		* @param invMassq0 inverse mass of the first quaternion
		* @param q1 second quaternion
		* @param invMassq1 inverse Mass of the second quaternion
		* @param bendingAndTwistingKs stiffness coefficients for stretching and shearing
		* @param restDarbouxVector rest Darboux vector
		* @param corrq0 position correction of first particle
		* @param corrq1 position correction of second particle
		*/
		static bool solve_BendTwistConstraint(
			const Quaternionr& q0, Real invMassq0,
			const Quaternionr& q1, Real invMassq1,
			const Vector3r& bendingAndTwistingKs,
			const Quaternionr& restDarbouxVector,
			Quaternionr& corrq0, Quaternionr&  corrq1);
	};

	// Implementation of "Position Based Elastic Rods" paper
	// (http://www.nobuyuki-umetani.com/PositionBasedElasticRod/2014_sca_PositionBasedElasticRod.html)
	//
	//	The code is based on the implementation of
	//
	//	Przemyslaw Korzeniowski
	//	Department of Surgery and Cancer
	//	Imperial College London
	//
	//	http://github.com/korzen/PositionBasedDynamics-ElasticRod
	//  korzenio[at] gmail.com
	//
	class PositionBasedElasticRods
	{
	public:
		/** Determine the position corrections for a perpendicular bisector constraint:\n\n
		* \f$C(\mathbf{p}_0, \mathbf{p}_1, \mathbf{p}_2) = ( \mathbf{p}_1 - 0.5 (\mathbf{p}_0 + \mathbf{p}_1))^T (\mathbf{p}_1 - \mathbf{p}_0) = 0\f$\n\n
		*
		* @param p0 position of first particle
		* @param invMass0 inverse mass of first particle
		* @param p1 position of second particle
		* @param invMass1 inverse mass of second particle
		* @param p2 position of third particle
		* @param invMass2 inverse mass of third particle
		* @param stiffness stiffness coefficient
		* @param corr0 position correction of first particle
		* @param corr1 position correction of second particle
		* @param corr2 position correction of third particle
		*/
		static bool solve_PerpendiculaBisectorConstraint(
			const Vector3r &p0, Real invMass0,
			const Vector3r &p1, Real invMass1,
			const Vector3r &p2, Real invMass2,
			const Real stiffness,
			Vector3r &corr0, Vector3r &corr1, Vector3r &corr2);


		/** Determine the position corrections for a constraint that enforces a rest length between an edge and a ghost point:\n\n
		* \f$C(\mathbf{p}_0, \mathbf{p}_1, \mathbf{p}_2) = \| ( 0.5 (\mathbf{p}_0 + \mathbf{p}_1) - \mathbf{p}_2 \| - L_0 = 0\f$\n\n
		*
		* @param p0 position of first particle
		* @param invMass0 inverse mass of first particle
		* @param p1 position of second particle
		* @param invMass1 inverse mass of second particle
		* @param p2 position of third particle
		* @param invMass2 inverse mass of third particle
		* @param stiffness stiffness coefficient
		* @param ghostEdgeRestLength rest length
		* @param corr0 position correction of first particle
		* @param corr1 position correction of second particle
		* @param corr2 position correction of third particle
		*/
		static bool solve_GhostPointEdgeDistanceConstraint(
			const Vector3r& p0, Real invMass0,
			const Vector3r& p1, Real invMass1,
			const Vector3r& p2, Real invMass2,
			const Real stiffness,
			const Real ghostEdgeRestLength,
			Vector3r& corr0, Vector3r&  corr1, Vector3r&  corr2);


		/** Determine the position corrections for the Darboux vector constraint (eq. 21 in the paper). See the paper appendix for derivation details\n\n
		*
		* @param p0 position of first particle
		* @param invMass0 inverse mass of first particle
		* @param p1 position of second particle
		* @param invMass1 inverse mass of second particle
		* @param p2 position of third particle
		* @param invMass2 inverse mass of third particle
		* @param p3 position of fourth particle
		* @param invMass3 inverse mass of fourth particle
		* @param p4 position of fifth particle
		* @param invMass4 inverse mass of fifth particle
		* @param bendingAndTwistingKs stiffness coefficients for bending and twisting
		* @param midEdgeLength average edge length
		* @param restDarbouxVector Darboux vector in rest state
		* @param corr0 position correction of first particle
		* @param corr1 position correction of second particle
		* @param corr2 position correction of third particle
		* @param corr3 position correction of fourth particle
		* @param corr4 position correction of fifth particle
		*/
		static bool solve_DarbouxVectorConstraint(
			const Vector3r& p0, Real invMass0,
			const Vector3r& p1, Real invMass1,
			const Vector3r& p2, Real invMass2,
			const Vector3r& p3, Real invMass3,
			const Vector3r& p4, Real invMass4,
			const Vector3r& bendingAndTwistingKs,
			const Real midEdgeLength,
			const Vector3r& restDarbouxVector,
			Vector3r& oa, Vector3r&  ob, Vector3r&  oc, Vector3r&  od, Vector3r& oe);
		
		/** Computes the material frame (eq. 3 in the paper)
		*/
		static bool computeMaterialFrame(
			const Vector3r& p0, //1st centreline point id
			const Vector3r& p1, //2nd centreline point id
			const Vector3r& p2, //corresponding ghost point
			Matrix3r& frame);   //resulting material frame

		/** Computes the Darboux Vector (eq. 10 in the paper)
		*/
		static bool computeDarbouxVector(
			const Matrix3r& dA, //1st material frame
			const Matrix3r& dB, //2nd material frame
			const Real mid_edge_length, //
			Vector3r& darboux_vector); //resulting darboux vector
		
		/** Computes the material frame derivatives (eq. 43, 44 and 45 in the appendix)
		*/
		static bool computeMaterialFrameDerivative(
			const Vector3r& p0, const Vector3r& p1, const Vector3r& p2, // points
			const Matrix3r& d, //corresponding material frame
			Matrix3r& d1p0, Matrix3r& d1p1, Matrix3r& d1p2,  //resulting matrices
			Matrix3r& d2p0, Matrix3r& d2p1, Matrix3r& d2p2,  //resulting matrices
			Matrix3r& d3p0, Matrix3r& d3p1, Matrix3r& d3p2); //resulting matrices
		
		/** Compute the Darboux gradient in respect to each point (eq. 49-53 in the appendix)
		*/
		static bool computeDarbouxGradient(
			const Vector3r& darboux_vector, //Darboux vector
			const Real length, //element length
			const Matrix3r& dA, //1st material frame
			const Matrix3r& dB, //2nd material frame
			const Matrix3r[3][3], const Matrix3r[3][3], //material frames derivatives
			//const Vector3r& bendAndTwistKs, //bending (x,y) and twisting (z) stiffness
			Matrix3r& omega_pa, Matrix3r& omega_pb, Matrix3r& omega_pc, Matrix3r& omega_pd, Matrix3r& omega_pe); //resulting matrices


	};
}

#endif
