#include "PositionBasedElasticRod.h"
#include "MathFunctions.h"
#include <cfloat>

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

using namespace PBD;

const float EPSILON = 1e-6f;

const int permutation[3][3] = {
	//0, 1, 2,
	//1, 2, 0,
	//2, 1, 0
	0, 2, 1,
	1, 0, 2,
	2, 1, 0
};

bool  PositionBasedElasticRod::ProjectEdgeConstraints(
	const Eigen::Vector3f& pA, const float wA, 
	const Eigen::Vector3f& pB, const float wB, 
	const Eigen::Vector3f& pG, const float wG,
	const float edgeKs, const float edgeRestLength, const float ghostEdgeRestLength,
	Eigen::Vector3f& corrA, Eigen::Vector3f&  corrB, Eigen::Vector3f&  corrC)
{
	corrA.setZero(); corrB.setZero(); corrC.setZero();

	//Edge distance constraint
	Eigen::Vector3f dir = pA - pB;
	float len = dir.norm();

	float wSum = wA + wB;
	if (len > EPSILON && wSum > EPSILON)
	{
		Eigen::Vector3f dP = (1.0f / wSum) * (len - edgeRestLength) * (dir / len) * edgeKs;

		corrA -= dP * wA;
		corrB += dP * wB;
		corrC = Eigen::Vector3f(0, 0, 0);
	}

	//Bisector constraint
	Eigen::Vector3f pm = 0.5f * (pA + pB);
	Eigen::Vector3f p0p2 = pA - pG;
	Eigen::Vector3f p2p1 = pG - pB;
	Eigen::Vector3f p1p0 = pB - pA;
	Eigen::Vector3f p2pm = pG - pm;
	float lambda;

	wSum = wA * p0p2.squaredNorm() + wB * p2p1.squaredNorm() + wG * p1p0.squaredNorm();
	if (wSum > EPSILON)
	{
		lambda = p2pm.dot(p1p0) / wSum * edgeKs;

		corrA -= p0p2 * lambda * wA;
		corrB -= p2p1 * lambda * wB;
		corrC -= p1p0 * lambda * wG;
	}

	////Ghost-Edge constraint
	wSum = 0.25f * wA + 0.25f * wB + 1.0f * wG;

	if (wSum > EPSILON)
	{
		//need to use updated positions
		pm = 0.5f * (pA + corrA + pB + corrB);
		p2pm = pG + corrC - pm;

		float p2pm_mag = p2pm.norm();
		p2pm *= 1.0f / p2pm_mag;

		lambda = (p2pm_mag - ghostEdgeRestLength) / wSum * edgeKs;

		corrA += 0.5f * wA * lambda * p2pm;
		corrB += 0.5f * wB * lambda * p2pm;
		corrC -= 1.0f * wG * lambda * p2pm;
	}


	return true;
}

bool  PositionBasedElasticRod::ProjectBendingAndTwistingConstraint(
	const Eigen::Vector3f& pA, const float wA,
	const Eigen::Vector3f& pB, const float wB,
	const Eigen::Vector3f& pC, const float wC,
	const Eigen::Vector3f& pD, const float wD,
	const Eigen::Vector3f& pE, const float wE,
	const Eigen::Vector3f& bendingAndTwistingKs,
	const float midEdgeLength,
	const Eigen::Vector3f& restDarbouxVector,
	Eigen::Vector3f& corrA, Eigen::Vector3f&  corrB, Eigen::Vector3f&  corrC, Eigen::Vector3f&  corrD, Eigen::Vector3f& corrE,
	bool useGaussianElimination)
{
	
	Eigen::Vector3f darboux_vector;
	Eigen::Matrix3f dA, dB;

	PositionBasedElasticRod::ComputeMaterialFrame(pA, pB, pD, dA);
	PositionBasedElasticRod::ComputeMaterialFrame(pB, pC, pE, dB);

	PositionBasedElasticRod::ComputeDarbouxVector(dA, dB, midEdgeLength, darboux_vector);

	Eigen::Matrix3f dajpi[3][3];
	ComputeMaterialFrameDerivative(pA, pB, pD, dA,
		dajpi[0][0], dajpi[0][1], dajpi[0][2],
		dajpi[1][0], dajpi[1][1], dajpi[1][2],
		dajpi[2][0], dajpi[2][1], dajpi[2][2]);

	Eigen::Matrix3f  dbjpi[3][3];
	ComputeMaterialFrameDerivative(pB, pC, pE, dB,
		dbjpi[0][0], dbjpi[0][1], dbjpi[0][2],
		dbjpi[1][0], dbjpi[1][1], dbjpi[1][2],
		dbjpi[2][0], dbjpi[2][1], dbjpi[2][2]);

	Eigen::Matrix3f constraint_jacobian[5];
	ComputeDarbouxGradient(
		darboux_vector, midEdgeLength, dA, dB, 
		dajpi, dbjpi, 
		bendingAndTwistingKs,
		constraint_jacobian[0],
		constraint_jacobian[1],
		constraint_jacobian[2],
		constraint_jacobian[3],
		constraint_jacobian[4]);

	Eigen::Vector3f constraint_value(bendingAndTwistingKs[0] * (darboux_vector[0] - restDarbouxVector[0]),
									 bendingAndTwistingKs[1] * (darboux_vector[1] - restDarbouxVector[1]),
									 bendingAndTwistingKs[2] * (darboux_vector[2] - restDarbouxVector[2]));

	Eigen::Matrix3f factor_matrix;
	factor_matrix.setZero();

	Eigen::Matrix3f tmp_mat;
	float invMasses[]{ wA, wB, wC, wD, wE };
	for (int i = 0; i < 5; ++i)
	{
		//dj::MulMatrixRightTransposed3x3<real>((real(*)[3]) &constraint_jacobian[i][0][0],
		//	(real(*)[3]) &constraint_jacobian[i][0][0],
		//	(real(*)[3]) &tmp_mat[0][0]);

		//tmp_mat = constraint_jacobian[i] * constraint_jacobian[i].transpose();
		tmp_mat = constraint_jacobian[i].transpose() * constraint_jacobian[i];
		tmp_mat.col(0) *= invMasses[i];
		tmp_mat.col(1) *= invMasses[i];
		tmp_mat.col(2) *= invMasses[i];

		factor_matrix += tmp_mat;
	}

	Eigen::Vector3f dp[5];
	//if (useGaussianElimination)
	//{
	//	Vec3 tmp(constraint_value[0], constraint_value[1], constraint_value[2]);
	//	dj::GaussianElimination<real, 3>(&factor_matrix[0][0], &tmp[0]);

	//	for (int i = 0; i < 5; ++i)
	//	{
	//		//dp[i][0] = (constraint_jacobian[i][0][0] * tmp[0] + constraint_jacobian[i][1][0] * tmp[1] + constraint_jacobian[i][2][0] * tmp[2]);
	//		//dp[i][1] = (constraint_jacobian[i][0][1] * tmp[0] + constraint_jacobian[i][1][1] * tmp[1] + constraint_jacobian[i][2][1] * tmp[2]);
	//		//dp[i][2] = (constraint_jacobian[i][0][2] * tmp[0] + constraint_jacobian[i][1][2] * tmp[1] + constraint_jacobian[i][2][2] * tmp[2]);

	//		dp[i][0] = real(invMasses[i]) * (constraint_jacobian[i][0][0] * tmp[0] + constraint_jacobian[i][1][0] * tmp[1] + constraint_jacobian[i][2][0] * tmp[2]);
	//		dp[i][1] = real(invMasses[i]) * (constraint_jacobian[i][0][1] * tmp[0] + constraint_jacobian[i][1][1] * tmp[1] + constraint_jacobian[i][2][1] * tmp[2]);
	//		dp[i][2] = real(invMasses[i]) * (constraint_jacobian[i][0][2] * tmp[0] + constraint_jacobian[i][1][2] * tmp[1] + constraint_jacobian[i][2][2] * tmp[2]);

	//	}
	//}
	//else
	{
		//dj::Inverse3<real>((real(*)[3]) &factor_matrix[0][0], (real(*)[3]) &tmp_mat[0][0]);
		tmp_mat = factor_matrix.inverse();

		for (int i = 0; i < 5; ++i)
		{
			constraint_jacobian[i].col(0) *= invMasses[i];
			constraint_jacobian[i].col(1) *= invMasses[i];
			constraint_jacobian[i].col(2) *= invMasses[i];
			dp[i] = -(constraint_jacobian[i]) * (tmp_mat * constraint_value);
		}

		corrA = dp[0];
		corrB = dp[1];
		corrC = dp[2];
		corrD = dp[3];
		corrE = dp[4];
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedElasticRod::ComputeMaterialFrame(
	const Eigen::Vector3f& pA, 
	const Eigen::Vector3f& pB, 
	const Eigen::Vector3f& pG, 
	Eigen::Matrix3f& frame)
{

	frame.col(2) = (pB - pA);
	frame.col(2).normalize();

	frame.col(1) = (frame.col(2).cross(pG - pA));
	frame.col(1).normalize();

	frame.col(0) = frame.col(1).cross(frame.col(2));
//	frame.col(0).normalize();
	return true;
}


bool PositionBasedElasticRod::ComputeDarbouxVector(const Eigen::Matrix3f& dA, const Eigen::Matrix3f& dB, const float mid_edge_length, Eigen::Vector3f& darboux_vector)
{

	float factor = 1.0f + dA.col(0).dot(dB.col(0)) + dA.col(1).dot(dB.col(1)) + dA.col(2).dot(dB.col(2));

	factor = 2.0f / (mid_edge_length * factor);

	for (int c = 0; c < 3; ++c)
	{
		const int i = permutation[c][0];
		const int j = permutation[c][1];
		const int k = permutation[c][2];

		darboux_vector[i] = dA.col(j).dot(dB.col(k)) - dA.col(k).dot(dB.col(j));
	}

	darboux_vector *= factor;
	return true;
}

bool PositionBasedElasticRod::ComputeMaterialFrameDerivative(
	const Eigen::Vector3f& p0, const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Matrix3f& d,
	Eigen::Matrix3f& d1p0, Eigen::Matrix3f& d1p1, Eigen::Matrix3f& d1p2,
	Eigen::Matrix3f& d2p0, Eigen::Matrix3f& d2p1, Eigen::Matrix3f& d2p2,
	Eigen::Matrix3f& d3p0, Eigen::Matrix3f& d3p1, Eigen::Matrix3f& d3p2)
{

	// d3pi
	Eigen::Vector3f p01 = p1 - p0;
	float length_p01 = p01.norm();

	d3p0.col(0) = d.col(2)[0] * d.col(2);
	d3p0.col(1) = d.col(2)[1] * d.col(2);
	d3p0.col(2) = d.col(2)[2] * d.col(2);

	d3p0.col(0)[0] -= 1.0f;
	d3p0.col(1)[1] -= 1.0f;
	d3p0.col(2)[2] -= 1.0f;

	d3p0.col(0) *= (1.0f / length_p01);
	d3p0.col(1) *= (1.0f / length_p01);
	d3p0.col(2) *= (1.0f / length_p01);

	d3p1.col(0) = -d3p0.col(0);
	d3p1.col(1) = -d3p0.col(1);
	d3p1.col(2) = -d3p0.col(2);

	d3p2.col(0).setZero();
	d3p2.col(1).setZero();
	d3p2.col(2).setZero();

	////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	//// d2pi
	Eigen::Vector3f p02 = p2 - p0;
	Eigen::Vector3f p01_cross_p02 = p01.cross(p02);

	float length_cross = p01_cross_p02.norm();

	Eigen::Matrix3f mat;
	mat.col(0) = d.col(1)[0] * d.col(1);
	mat.col(1) = d.col(1)[1] * d.col(1);
	mat.col(2) = d.col(1)[2] * d.col(1);

	mat.col(0)[0] -= 1.0f;
	mat.col(1)[1] -= 1.0f;
	mat.col(2)[2] -= 1.0f;

	mat.col(0) *= (-1.0f / length_cross);
	mat.col(1) *= (-1.0f / length_cross);
	mat.col(2) *= (-1.0f / length_cross);

	Eigen::Matrix3f product_matrix;
	MathFunctions::crossProductMatrix(p2 - p1, product_matrix);
	d2p0 = mat * product_matrix;

	MathFunctions::crossProductMatrix(p0 - p2, product_matrix);
	d2p1 = mat * product_matrix;

	MathFunctions::crossProductMatrix(p1 - p0, product_matrix);
	d2p2 = mat * product_matrix;

	////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	//// d1pi
	Eigen::Matrix3f product_mat_d3;
	Eigen::Matrix3f product_mat_d2;
	Eigen::Matrix3f m1, m2;

	MathFunctions::crossProductMatrix(d.col(2), product_mat_d3);
	MathFunctions::crossProductMatrix(d.col(1), product_mat_d2);

	//dj::MulMatrix3x3<real>(&product_mat_d3[0][0], &d2p0[0][0], &m1[0][0]);
	//dj::MulMatrix3x3<real>(&product_mat_d2[0][0], &d3p0[0][0], &m2[0][0]);
	d1p0 = product_mat_d2 * d3p0 - product_mat_d3 * d2p0;

	//dj::MulMatrix3x3<real>(&product_mat_d3[0][0], &d2p1[0][0], &m1[0][0]);
	//dj::MulMatrix3x3<real>(&product_mat_d2[0][0], &d3p1[0][0], &m2[0][0]);
	d1p1 = product_mat_d2 * d3p1 - product_mat_d3 * d2p1;

	/*dj::MulMatrix3x3<real>(&product_mat_d3[0][0], &d2p2[0][0], &d1p2[0][0]);*/
	d1p2 = product_mat_d3 * d2p2;
	d1p2.col(0) *= -1.0f;
	d1p2.col(1) *= -1.0f;
	d1p2.col(2) *= -1.0f;
	return true;
}


bool PositionBasedElasticRod::ComputeDarbouxGradient(
	const Eigen::Vector3f& darboux_vector, const float length,
	const Eigen::Matrix3f& da, const Eigen::Matrix3f& db,
	//const Eigen::Matrix3f(*dajpi)[3], const Eigen::Matrix3f(*dbjpi)[3], 
	const Eigen::Matrix3f dajpi[3][3], const Eigen::Matrix3f dbjpi[3][3],
	const Eigen::Vector3f& bendAndTwistKs,
	Eigen::Matrix3f& omega_pa, Eigen::Matrix3f& omega_pb, Eigen::Matrix3f& omega_pc, Eigen::Matrix3f& omega_pd, Eigen::Matrix3f& omega_pe
	)
{


	//float x = 1.0f + da[0] * db[0] + da[1] * db[1] + da[2] * db[2];
	float x = 1.0f + da.col(0).dot(db.col(0)) + da.col(1).dot(db.col(1)) + da.col(2).dot(db.col(2));
	x = 2.0f / (length * x);

	for (int c = 0; c < 3; ++c) 
	{
		const int i = permutation[c][0];
		const int j = permutation[c][1];
		const int k = permutation[c][2];
		// pa
		{
			Eigen::Vector3f term1(0,0,0);
			Eigen::Vector3f term2(0,0,0);
			Eigen::Vector3f tmp(0,0,0);
			// first term
			//dj::MulVecMatrix3x3<real>(db[k](), (real(*)[3]) &dajpi[j][0], term1());
			//dj::MulVecMatrix3x3<real>(db[j](), (real(*)[3]) &dajpi[k][0], tmp());

			//DOUBLE CHECK !!!
			term1 = dajpi[j][0].transpose() * db.col(k);
			tmp =   dajpi[k][0].transpose() * db.col(j);
			term1 = term1 - tmp;
			// second term
			for (int n = 0; n < 3; ++n) 
			{
				//dj::MulVecMatrix3x3<real>(db[n](), (real(*)[3]) &dajpi[n][0], tmp());

				//DOUBLE CHECK !!!
				tmp = dajpi[n][0].transpose() * db.col(n);
				term2 = term2 + tmp;
			}
			omega_pa.col(i) = (term1)-(0.5f * darboux_vector[i] * length) * (term2);
			omega_pa.col(i) *= (x * bendAndTwistKs[i]);
		}
		// pb
		{
			Eigen::Vector3f term1(0, 0, 0);
			Eigen::Vector3f term2(0, 0, 0);
			Eigen::Vector3f tmp(0, 0, 0);
			// first term
			//dj::MulVecMatrix3x3<real>(db[k](), (real(*)[3]) &dajpi[j][1], term1());
			//dj::MulVecMatrix3x3<real>(db[j](), (real(*)[3]) &dajpi[k][1], tmp());
			term1 = dajpi[j][1].transpose() * db.col(k);
			tmp =   dajpi[k][1].transpose() * db.col(j);
			term1 = term1 - tmp;
			// third term
			//dj::MulVecMatrix3x3<real>(da[k](), (real(*)[3]) &dbjpi[j][0], tmp());
			tmp = dbjpi[j][0].transpose() * da.col(k);
			term1 = term1 - tmp;
			
			//dj::MulVecMatrix3x3<real>(da[j](), (real(*)[3]) &dbjpi[k][0], tmp());
			tmp = dbjpi[k][0].transpose() * da.col(j);
			term1 = term1 + tmp;

			// second term
			for (int n = 0; n < 3; ++n) 
			{
				//dj::MulVecMatrix3x3<real>(db[n](), (real(*)[3]) &dajpi[n][1], tmp());
				tmp = dajpi[n][1].transpose() * db.col(n);
				term2 = term2 + tmp;
				
				//dj::MulVecMatrix3x3<real>(da[n](), (real(*)[3]) &dbjpi[n][0], tmp());
				tmp = dbjpi[n][0].transpose() * da.col(n);
				term2 = term2 + tmp;
			}
			omega_pb.col(i) = (term1)-(0.5f * darboux_vector[i] * length) * (term2);
			omega_pb.col(i) *= (x * bendAndTwistKs[i]);
		}
		// pc
		{
			Eigen::Vector3f term1(0, 0, 0);
			Eigen::Vector3f term2(0, 0, 0);
			Eigen::Vector3f tmp(0, 0, 0);
			
			// first term
			//dj::MulVecMatrix3x3<real>(da[k](), (real(*)[3]) &dbjpi[j][1], term1());
			//dj::MulVecMatrix3x3<real>(da[j](), (real(*)[3]) &dbjpi[k][1], tmp());
			term1 = dbjpi[j][1].transpose() * da.col(k);
			tmp =   dbjpi[k][1].transpose() * da.col(j);
			term1 = term1 - tmp;

			// second term
			for (int n = 0; n < 3; ++n) 
			{
				//dj::MulVecMatrix3x3<real>(da[n](), (real(*)[3]) &dbjpi[n][1], tmp());
				tmp = dbjpi[n][1].transpose() * da.col(n);
				term2 = term2 + tmp;
			}
			omega_pc.col(i) = (term1)+(0.5f * darboux_vector[i] * length) * (term2);
			omega_pc.col(i) *= (-x * bendAndTwistKs[i]);
		}
		// pd
		{
			Eigen::Vector3f term1(0, 0, 0);
			Eigen::Vector3f term2(0, 0, 0);
			Eigen::Vector3f tmp(0, 0, 0);
			// first term
			//dj::MulVecMatrix3x3<real>(db[k](), (real(*)[3]) &dajpi[j][2], term1());
			//dj::MulVecMatrix3x3<real>(db[j](), (real(*)[3]) &dajpi[k][2], tmp());
			term1 = dajpi[j][2].transpose() * db.col(k);
			tmp =   dajpi[k][2].transpose() * db.col(j);
			term1 = term1 - tmp;
			// second term
			for (int n = 0; n < 3; ++n) {
				//dj::MulVecMatrix3x3<real>(db[n](), (real(*)[3]) &dajpi[n][2], tmp());
				tmp = dajpi[n][2].transpose() * db.col(n);
				term2 = term2 + tmp;
			}
			omega_pd.col(i) = (term1)-(0.5f * darboux_vector[i] * length) * (term2);
			omega_pd.col(i) *= (x * bendAndTwistKs[i]);
		}
		// pe
		{
			Eigen::Vector3f term1(0, 0, 0);
			Eigen::Vector3f term2(0, 0, 0);
			Eigen::Vector3f tmp(0, 0, 0);
			// first term

			//dj::MulVecMatrix3x3<real>(da[k](), (real(*)[3]) &dbjpi[j][2], term1());
			//dj::MulVecMatrix3x3<real>(da[j](), (real(*)[3]) &dbjpi[k][2], tmp());
			term1 = dbjpi[j][2].transpose() * da.col(k);
			tmp = dbjpi[k][2].transpose() * da.col(j);
			term1 -= tmp;
			
			// second term
			for (int n = 0; n < 3; ++n) 
			{	//WARNING!!! &dbjpi[n][2][0][0] ???
				//dj::MulVecMatrix3x3<real>(da[n](), (real(*)[3]) &dbjpi[n][2][0][0], tmp());
				tmp = dbjpi[n][2].transpose() * da.col(n);
				term2 += tmp;
			}

			omega_pe.col(i) = (term1)+(0.5f * darboux_vector[i] * length) * (term2);
			omega_pe.col(i) *= (-x * bendAndTwistKs[i]);
		}
	}
	return true;
}
// ----------------------------------------------------------------------------------------------
