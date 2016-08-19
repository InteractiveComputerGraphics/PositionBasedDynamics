#ifndef POSITION_BASED_ELASTIC_ROD_H
#define POSITION_BASED_ELASTIC_ROD_H

#include <Eigen/Dense>
#include "Common\Common.h"

// ------------------------------------------------------------------------------------
namespace PBD
{

	//  Implementation of "Position Based Elastic Rods" paper
	//  (http://www.nobuyuki-umetani.com/PositionBasedElasticRod/2014_sca_PositionBasedElasticRod.html)
	//  for Position Based Dynamics library
	//  (http://github.com/janbender/PositionBasedDynamics)
	//
	//	Przemyslaw Korzeniowski
	//	Department of Surgery and Cancer
	//	Imperial College London
	//
	//  https://github.com/korzen/PositionBasedDynamics-ElasticRod
	//	p.korzeniowski [at] imperial.ac.uk
	//	korzenio [at] gmail.com
	//
	//	Bending and twisting constraints ported to Eigen from https ://github.com/serpheroth/rod
	//
	//  IMPORTANT: OpenMP is not supported in this release!
	//  By default, the CMake generated ElasticRodDemo project has OpenMP turned on. Deactivate it in order to run the demo.
	//
	//  v0.1 (initial release)- see /Demos/ElasticRodDemos/ReadMe.txt for more info
	class PositionBasedElasticRod
	{
	public:
		// -------------- Position Based Elastic Rod  -----------------------------------------------------

		/// <summary>
		/// Sequentially project all three edge-ghost points constraints (eq. 18, 19 and 20 in the paper) 
		/// See the paper appendix for derivation details
		/// </summary>
		static bool ProjectEdgeConstraints(
			const Vector3r& pA, const float wA, const Vector3r& pB, const float wB, const Vector3r& pG, const float wG,
			const float edgeKs, const float edgeRestLength, const float ghostEdgeRestLength,
			Vector3r& corr0, Vector3r&  corr1, Vector3r&  corr2);

		/// <summary>
		/// Project bending and twisting constraints (eq. 21 in the paper) 
		/// See the paper appendix for derivation details
		/// </summary>
		static bool  PositionBasedElasticRod::ProjectBendingAndTwistingConstraint(
			const Vector3r& pA, const float wA,
			const Vector3r& pB, const float wB,
			const Vector3r& pC, const float wC,
			const Vector3r& pD, const float wD,
			const Vector3r& pE, const float wE,
			const Vector3r& bendingAndTwistingKs,
			const float midEdgeLength,
			const Vector3r& restDarbouxVector,
			Vector3r& oa, Vector3r&  ob, Vector3r&  oc, Vector3r&  od, Vector3r& oe, 
			bool useGaussianElimination);
		
		/// <summary>
		/// Computes the material frame (eq. 3 in the paper)
		/// </summary>
		static bool ComputeMaterialFrame(
			const Vector3r& pA, //1st centreline point id
			const Vector3r& pB, //2nd centreline point id
			const Vector3r& pG, //corresponding ghost point
			Matrix3r& frame);   //resulting material frame

		/// <summary>
		/// Computes the Darboux Vector (eq. 10 in the paper)
		/// </summary>
		static bool ComputeDarbouxVector(
			const Matrix3r& dA, //1st material frame
			const Matrix3r& dB, //2nd material frame
			const float mid_edge_length, //
			Vector3r& darboux_vector); //resulting darboux vector
		
		/// <summary>
		/// Computes the material frame derivatives (eq. 43, 44 and 45 in the appendix)
		/// </summary>
		static bool ComputeMaterialFrameDerivative(
			const Vector3r& p0, const Vector3r& p1, const Vector3r& p2, // points
			const Matrix3r& d, //corresponding material frame
			Matrix3r& d1p0, Matrix3r& d1p1, Matrix3r& d1p2,  //resulting matrices
			Matrix3r& d2p0, Matrix3r& d2p1, Matrix3r& d2p2,  //resulting matrices
			Matrix3r& d3p0, Matrix3r& d3p1, Matrix3r& d3p2); //resulting matrices
		
		/// <summary>
		/// Compute the Darboux gradient in respect to each point (eq. 49-53 in the appendix)
		/// </summary>
		static bool ComputeDarbouxGradient(
			const Vector3r& darboux_vector, //Darboux vector
			const float length, //element length
			const Matrix3r& dA, //1st material frame
			const Matrix3r& dB, //2nd material frame
			const Matrix3r[3][3], const Matrix3r[3][3], //material frames derivatives
			const Vector3r& bendAndTwistKs, //bending (x,y) and twisting (z) stiffness
			Matrix3r& omega_pa, Matrix3r& omega_pb, Matrix3r& omega_pc, Matrix3r& omega_pd, Matrix3r& omega_pe); //resulting matrices


	};
}

#endif