#ifndef POSITION_BASED_ELASTIC_ROD_H
#define POSITION_BASED_ELASTIC_ROD_H

#include <Eigen/Dense>

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
			const Eigen::Vector3f& pA, const float wA, const Eigen::Vector3f& pB, const float wB, const Eigen::Vector3f& pG, const float wG,
			const float edgeKs, const float edgeRestLength, const float ghostEdgeRestLength,
			Eigen::Vector3f& corr0, Eigen::Vector3f&  corr1, Eigen::Vector3f&  corr2);

		/// <summary>
		/// Project bending and twisting constraints (eq. 21 in the paper) 
		/// See the paper appendix for derivation details
		/// </summary>
		static bool  PositionBasedElasticRod::ProjectBendingAndTwistingConstraint(
			const Eigen::Vector3f& pA, const float wA,
			const Eigen::Vector3f& pB, const float wB,
			const Eigen::Vector3f& pC, const float wC,
			const Eigen::Vector3f& pD, const float wD,
			const Eigen::Vector3f& pE, const float wE,
			const Eigen::Vector3f& bendingAndTwistingKs,
			const float midEdgeLength,
			const Eigen::Vector3f& restDarbouxVector,
			Eigen::Vector3f& oa, Eigen::Vector3f&  ob, Eigen::Vector3f&  oc, Eigen::Vector3f&  od, Eigen::Vector3f& oe, 
			bool useGaussianElimination);
		
		/// <summary>
		/// Computes the material frame (eq. 3 in the paper)
		/// </summary>
		static bool ComputeMaterialFrame(
			const Eigen::Vector3f& pA, //1st centreline point id
			const Eigen::Vector3f& pB, //2nd centreline point id
			const Eigen::Vector3f& pG, //corresponding ghost point
			Eigen::Matrix3f& frame);   //resulting material frame

		/// <summary>
		/// Computes the Darboux Vector (eq. 10 in the paper)
		/// </summary>
		static bool ComputeDarbouxVector(
			const Eigen::Matrix3f& dA, //1st material frame
			const Eigen::Matrix3f& dB, //2nd material frame
			const float mid_edge_length, //
			Eigen::Vector3f& darboux_vector); //resulting darboux vector
		
		/// <summary>
		/// Computes the material frame derivatives (eq. 43, 44 and 45 in the appendix)
		/// </summary>
		static bool ComputeMaterialFrameDerivative(
			const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, // points
			const Eigen::Matrix3f& d, //corresponding material frame
			Eigen::Matrix3f& d1p0, Eigen::Matrix3f& d1p1, Eigen::Matrix3f& d1p2,  //resulting matrices
			Eigen::Matrix3f& d2p0, Eigen::Matrix3f& d2p1, Eigen::Matrix3f& d2p2,  //resulting matrices
			Eigen::Matrix3f& d3p0, Eigen::Matrix3f& d3p1, Eigen::Matrix3f& d3p2); //resulting matrices
		
		/// <summary>
		/// Compute the Darboux gradient in respect to each point (eq. 49-53 in the appendix)
		/// </summary>
		static bool ComputeDarbouxGradient(
			const Eigen::Vector3f& darboux_vector, //Darboux vector
			const float length, //element length
			const Eigen::Matrix3f& dA, //1st material frame
			const Eigen::Matrix3f& dB, //2nd material frame
			const Eigen::Matrix3f[3][3], const Eigen::Matrix3f[3][3], //material frames derivatives
			const Eigen::Vector3f& bendAndTwistKs, //bending (x,y) and twisting (z) stiffness
			Eigen::Matrix3f& omega_pa, Eigen::Matrix3f& omega_pb, Eigen::Matrix3f& omega_pc, Eigen::Matrix3f& omega_pd, Eigen::Matrix3f& omega_pe); //resulting matrices


	};
}

#endif