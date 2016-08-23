#include "PositionBasedElasticRods.h"
#include "PositionBasedDynamics/MathFunctions.h"

using namespace PBD;

const Real eps = 1e-6;

const int permutation[3][3] = {
	0, 2, 1,
	1, 0, 2,
	2, 1, 0
};

// ----------------------------------------------------------------------------------------------
bool PositionBasedElasticRods::solve_PerpendiculaBisectorConstraint(
	const Vector3r &p0, Real invMass0,
	const Vector3r &p1, Real invMass1,
	const Vector3r &p2, Real invMass2,
	const Real stiffness,
	Vector3r &corr0, Vector3r &corr1, Vector3r &corr2)
{
	const Vector3r pm = 0.5 * (p0 + p1);
	const Vector3r p0p2 = p0 - p2;
	const Vector3r p2p1 = p2 - p1;
	const Vector3r p1p0 = p1 - p0;
	const Vector3r p2pm = p2 - pm;

	Real wSum = invMass0 * p0p2.squaredNorm() + invMass1 * p2p1.squaredNorm() + invMass2 * p1p0.squaredNorm();
	if (wSum < eps)
		return false;

	const Real lambda = stiffness * p2pm.dot(p1p0) / wSum;

	corr0 = -invMass0 * lambda * p0p2;
	corr1 = -invMass1 * lambda * p2p1;
	corr2 = -invMass2 * lambda * p1p0;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedElasticRods::solve_GhostPointEdgeDistanceConstraint(
	const Vector3r& p0, Real invMass0,
	const Vector3r& p1, Real invMass1,
	const Vector3r& p2, Real invMass2,
	const Real stiffness, 
	const Real ghostEdgeRestLength,
	Vector3r& corr0, Vector3r&  corr1, Vector3r&  corr2)
{
	// Ghost-Edge constraint
	Vector3r pm = 0.5 * (p0 + p1);
	Vector3r p2pm = p2 - pm;
	Real wSum = 0.25 * invMass0 + 0.25 * invMass1 + 1.0 * invMass2;

	if (wSum < eps)
		return false;

	Real p2pm_mag = p2pm.norm();
	p2pm *= 1.0 / p2pm_mag;

	const Real lambda = stiffness * (p2pm_mag - ghostEdgeRestLength) / wSum;

	corr0 = 0.5 * invMass0 * lambda * p2pm;
	corr1 = 0.5 * invMass1 * lambda * p2pm;
	corr2 = -1.0 * invMass2 * lambda * p2pm;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedElasticRods::solve_DarbouxVectorConstraint(
	const Vector3r& p0, Real invMass0,
	const Vector3r& p1, Real invMass1,
	const Vector3r& p2, Real invMass2,
	const Vector3r& p3, Real invMass3,
	const Vector3r& p4, Real invMass4,
	const Vector3r& bendingAndTwistingKs,
	const Real midEdgeLength,
	const Vector3r& restDarbouxVector,
	Vector3r& corr0, Vector3r&  corr1, Vector3r&  corr2, Vector3r&  corr3, Vector3r& corr4)
{
	//  Single rod element:
	//      3   4		//ghost points
	//		|	|
	//  --0---1---2--	// rod points

	Vector3r darboux_vector;
	Matrix3r d0, d1;

	PositionBasedElasticRods::computeMaterialFrame(p0, p1, p3, d0);
	PositionBasedElasticRods::computeMaterialFrame(p1, p2, p4, d1);

	PositionBasedElasticRods::computeDarbouxVector(d0, d1, midEdgeLength, darboux_vector);

	Matrix3r dajpi[3][3];
	computeMaterialFrameDerivative(p0, p1, p3, d0,
		dajpi[0][0], dajpi[0][1], dajpi[0][2],
		dajpi[1][0], dajpi[1][1], dajpi[1][2],
		dajpi[2][0], dajpi[2][1], dajpi[2][2]);

	Matrix3r dbjpi[3][3];
	computeMaterialFrameDerivative(p1, p2, p4, d1,
		dbjpi[0][0], dbjpi[0][1], dbjpi[0][2],
		dbjpi[1][0], dbjpi[1][1], dbjpi[1][2],
		dbjpi[2][0], dbjpi[2][1], dbjpi[2][2]);

	Matrix3r constraint_jacobian[5];
	computeDarbouxGradient(
		darboux_vector, midEdgeLength, d0, d1, 
		dajpi, dbjpi, 
		//bendingAndTwistingKs,
		constraint_jacobian[0],
		constraint_jacobian[1],
		constraint_jacobian[2],
		constraint_jacobian[3],
		constraint_jacobian[4]);

	const Vector3r constraint_value(bendingAndTwistingKs[0] * (darboux_vector[0] - restDarbouxVector[0]),
							 bendingAndTwistingKs[1] * (darboux_vector[1] - restDarbouxVector[1]),
							 bendingAndTwistingKs[2] * (darboux_vector[2] - restDarbouxVector[2]));

	Matrix3r factor_matrix;
	factor_matrix.setZero();

	Matrix3r tmp_mat;
	Real invMasses[]{ invMass0, invMass1, invMass2, invMass3, invMass4 };
	for (int i = 0; i < 5; ++i)
	{
		tmp_mat = constraint_jacobian[i].transpose() * constraint_jacobian[i];
		tmp_mat.col(0) *= invMasses[i];
		tmp_mat.col(1) *= invMasses[i];
		tmp_mat.col(2) *= invMasses[i];

		factor_matrix += tmp_mat;
	}

	Vector3r dp[5];
	tmp_mat = factor_matrix.inverse();

	for (int i = 0; i < 5; ++i)
	{
		constraint_jacobian[i].col(0) *= invMasses[i];
		constraint_jacobian[i].col(1) *= invMasses[i];
		constraint_jacobian[i].col(2) *= invMasses[i];
		dp[i] = -(constraint_jacobian[i]) * (tmp_mat * constraint_value);
	}

	corr0 = dp[0];
	corr1 = dp[1];
	corr2 = dp[2];
	corr3 = dp[3];
	corr4 = dp[4];

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedElasticRods::computeMaterialFrame(
	const Vector3r& p0, 
	const Vector3r& p1, 
	const Vector3r& p2, 
	Matrix3r& frame)
{
	frame.col(2) = (p1 - p0);
	frame.col(2).normalize();

	frame.col(1) = (frame.col(2).cross(p2 - p0));
	frame.col(1).normalize();

	frame.col(0) = frame.col(1).cross(frame.col(2));
	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedElasticRods::computeDarbouxVector(const Matrix3r& dA, const Matrix3r& dB, const Real mid_edge_length, Vector3r& darboux_vector)
{
	Real factor = 1.0 + dA.col(0).dot(dB.col(0)) + dA.col(1).dot(dB.col(1)) + dA.col(2).dot(dB.col(2));

	factor = 2.0 / (mid_edge_length * factor);

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

// ----------------------------------------------------------------------------------------------
bool PositionBasedElasticRods::computeMaterialFrameDerivative(
	const Vector3r& p0, const Vector3r& p1, const Vector3r& p2, const Matrix3r& d,
	Matrix3r& d1p0, Matrix3r& d1p1, Matrix3r& d1p2,
	Matrix3r& d2p0, Matrix3r& d2p1, Matrix3r& d2p2,
	Matrix3r& d3p0, Matrix3r& d3p1, Matrix3r& d3p2)
{
	//////////////////////////////////////////////////////////////////////////
	// d3pi
	//////////////////////////////////////////////////////////////////////////
	const Vector3r p01 = p1 - p0;
	Real length_p01 = p01.norm();

	d3p0.col(0) = d.col(2)[0] * d.col(2);
	d3p0.col(1) = d.col(2)[1] * d.col(2);
	d3p0.col(2) = d.col(2)[2] * d.col(2);

	d3p0.col(0)[0] -= 1.0;
	d3p0.col(1)[1] -= 1.0;
	d3p0.col(2)[2] -= 1.0;

	d3p0.col(0) *= (1.0 / length_p01);
	d3p0.col(1) *= (1.0 / length_p01);
	d3p0.col(2) *= (1.0 / length_p01);

	d3p1.col(0) = -d3p0.col(0);
	d3p1.col(1) = -d3p0.col(1);
	d3p1.col(2) = -d3p0.col(2);

	d3p2.col(0).setZero();
	d3p2.col(1).setZero();
	d3p2.col(2).setZero();

	//////////////////////////////////////////////////////////////////////////
	// d2pi
	//////////////////////////////////////////////////////////////////////////
	const Vector3r p02 = p2 - p0;
	const Vector3r p01_cross_p02 = p01.cross(p02);

	const Real length_cross = p01_cross_p02.norm();

	Matrix3r mat;
	mat.col(0) = d.col(1)[0] * d.col(1);
	mat.col(1) = d.col(1)[1] * d.col(1);
	mat.col(2) = d.col(1)[2] * d.col(1);

	mat.col(0)[0] -= 1.0;
	mat.col(1)[1] -= 1.0;
	mat.col(2)[2] -= 1.0;

	mat.col(0) *= (-1.0 / length_cross);
	mat.col(1) *= (-1.0 / length_cross);
	mat.col(2) *= (-1.0 / length_cross);

	Matrix3r product_matrix;
	MathFunctions::crossProductMatrix(p2 - p1, product_matrix);
	d2p0 = mat * product_matrix;

	MathFunctions::crossProductMatrix(p0 - p2, product_matrix);
	d2p1 = mat * product_matrix;

	MathFunctions::crossProductMatrix(p1 - p0, product_matrix);
	d2p2 = mat * product_matrix;

	//////////////////////////////////////////////////////////////////////////
	// d1pi
	//////////////////////////////////////////////////////////////////////////
	Matrix3r product_mat_d3;
	Matrix3r product_mat_d2;
	MathFunctions::crossProductMatrix(d.col(2), product_mat_d3);
	MathFunctions::crossProductMatrix(d.col(1), product_mat_d2);

	d1p0 = product_mat_d2 * d3p0 - product_mat_d3 * d2p0;
	d1p1 = product_mat_d2 * d3p1 - product_mat_d3 * d2p1;
	d1p2 = -product_mat_d3 * d2p2;
	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedElasticRods::computeDarbouxGradient(
	const Vector3r& darboux_vector, const Real length,
	const Matrix3r& da, const Matrix3r& db,
	const Matrix3r dajpi[3][3], const Matrix3r dbjpi[3][3],
	//const Vector3r& bendAndTwistKs,
	Matrix3r& omega_pa, Matrix3r& omega_pb, Matrix3r& omega_pc, Matrix3r& omega_pd, Matrix3r& omega_pe
	)
{
	Real X = 1.0 + da.col(0).dot(db.col(0)) + da.col(1).dot(db.col(1)) + da.col(2).dot(db.col(2));
	X = 2.0 / (length * X);

	for (int c = 0; c < 3; ++c) 
	{
		const int i = permutation[c][0];
		const int j = permutation[c][1];
		const int k = permutation[c][2];
		// pa
		{
			Vector3r term1(0,0,0);
			Vector3r term2(0,0,0);
			Vector3r tmp(0,0,0);

			// first term
			term1 = dajpi[j][0].transpose() * db.col(k);
			tmp =   dajpi[k][0].transpose() * db.col(j);
			term1 = term1 - tmp;
			// second term
			for (int n = 0; n < 3; ++n) 
			{
				tmp = dajpi[n][0].transpose() * db.col(n);
				term2 = term2 + tmp;
			}
			omega_pa.col(i) = X * (term1-(0.5 * darboux_vector[i] * length) * term2);
			//omega_pa.col(i) *= bendAndTwistKs[i];
		}
		// pb
		{
			Vector3r term1(0, 0, 0);
			Vector3r term2(0, 0, 0);
			Vector3r tmp(0, 0, 0);
			// first term
			term1 = dajpi[j][1].transpose() * db.col(k);
			tmp =   dajpi[k][1].transpose() * db.col(j);
			term1 = term1 - tmp;
			// third term
			tmp = dbjpi[j][0].transpose() * da.col(k);
			term1 = term1 - tmp;
			
			tmp = dbjpi[k][0].transpose() * da.col(j);
			term1 = term1 + tmp;

			// second term
			for (int n = 0; n < 3; ++n) 
			{
				tmp = dajpi[n][1].transpose() * db.col(n);
				term2 = term2 + tmp;
				
				tmp = dbjpi[n][0].transpose() * da.col(n);
				term2 = term2 + tmp;
			}
			omega_pb.col(i) = X * (term1-(0.5 * darboux_vector[i] * length) * term2);
			//omega_pb.col(i) *= bendAndTwistKs[i];
		}
		// pc
		{
			Vector3r term1(0, 0, 0);
			Vector3r term2(0, 0, 0);
			Vector3r tmp(0, 0, 0);
			
			// first term
			term1 = dbjpi[j][1].transpose() * da.col(k);
			tmp =   dbjpi[k][1].transpose() * da.col(j);
			term1 = term1 - tmp;

			// second term
			for (int n = 0; n < 3; ++n) 
			{
				tmp = dbjpi[n][1].transpose() * da.col(n);
				term2 = term2 + tmp;
			}
			omega_pc.col(i) = -X*(term1+(0.5 * darboux_vector[i] * length) * term2);
			//omega_pc.col(i) *= bendAndTwistKs[i];
		}
		// pd
		{
			Vector3r term1(0, 0, 0);
			Vector3r term2(0, 0, 0);
			Vector3r tmp(0, 0, 0);
			// first term
			term1 = dajpi[j][2].transpose() * db.col(k);
			tmp =   dajpi[k][2].transpose() * db.col(j);
			term1 = term1 - tmp;
			// second term
			for (int n = 0; n < 3; ++n) {
				tmp = dajpi[n][2].transpose() * db.col(n);
				term2 = term2 + tmp;
			}
			omega_pd.col(i) = X*(term1-(0.5 * darboux_vector[i] * length) * term2);
			//omega_pd.col(i) *= bendAndTwistKs[i];
		}
		// pe
		{
			Vector3r term1(0, 0, 0);
			Vector3r term2(0, 0, 0);
			Vector3r tmp(0, 0, 0);
			// first term
			term1 = dbjpi[j][2].transpose() * da.col(k);
			tmp = dbjpi[k][2].transpose() * da.col(j);
			term1 -= tmp;
			
			// second term
			for (int n = 0; n < 3; ++n) 
			{	
				tmp = dbjpi[n][2].transpose() * da.col(n);
				term2 += tmp;
			}

			omega_pe.col(i) = -X*(term1+(0.5 * darboux_vector[i] * length) * term2);
			//omega_pe.col(i) *= bendAndTwistKs[i];
		}
	}
	return true;
}
// ----------------------------------------------------------------------------------------------
