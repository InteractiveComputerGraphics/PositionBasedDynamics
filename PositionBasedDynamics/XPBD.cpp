#include "XPBD.h"
#include "PositionBasedDynamics.h"
#include "MathFunctions.h"
#include <cfloat>

using namespace PBD;

const Real eps = static_cast<Real>(1e-6);

//////////////////////////////////////////////////////////////////////////
// XPBD
//////////////////////////////////////////////////////////////////////////

bool XPBD::solve_DistanceConstraint(
	const Vector3r &p0, Real invMass0, 
	const Vector3r &p1, Real invMass1,
	const Real restLength,
	const Real stiffness,
	const Real dt,
	Real& lambda,
	Vector3r &corr0, Vector3r &corr1)
{				
	Real K = invMass0 + invMass1;
	Vector3r n = p0 - p1;
	Real d = n.norm();
	Real C = d - restLength;
	if (d > static_cast<Real>(1e-6))
		n /= d;
	else
	{
		corr0.setZero();
		corr1.setZero();
		return true;
	}

	Real alpha = 0.0;
	if (stiffness != 0.0)
	{
		alpha = static_cast<Real>(1.0) / (stiffness * dt * dt);
		K += alpha;
	}

	Real Kinv = 0.0;
	if (fabs(K) > static_cast<Real>(1e-6))
		Kinv = static_cast<Real>(1.0) / K;
	else
	{
		corr0.setZero();
		corr1.setZero();
		return true;
	}

	const Real delta_lambda = -Kinv * (C + alpha * lambda);
	lambda += delta_lambda;
	const Vector3r pt = n * delta_lambda;

	corr0 =  invMass0 * pt;
	corr1 = -invMass1 * pt;
	return true;
}

// ----------------------------------------------------------------------------------------------
bool XPBD::solve_VolumeConstraint(
	const Vector3r& p0, Real invMass0,
	const Vector3r& p1, Real invMass1,
	const Vector3r& p2, Real invMass2,
	const Vector3r& p3, Real invMass3,
	const Real restVolume,
	const Real stiffness,
	const Real dt,
	Real& lambda,
	Vector3r& corr0, Vector3r& corr1, Vector3r& corr2, Vector3r& corr3)
{
	Real volume = static_cast<Real>(1.0 / 6.0) * (p1 - p0).cross(p2 - p0).dot(p3 - p0);

	corr0.setZero(); corr1.setZero(); corr2.setZero(); corr3.setZero();

	Vector3r grad0 = (p1 - p2).cross(p3 - p2);
	Vector3r grad1 = (p2 - p0).cross(p3 - p0);
	Vector3r grad2 = (p0 - p1).cross(p3 - p1);
	Vector3r grad3 = (p1 - p0).cross(p2 - p0);

	Real K =
		invMass0 * grad0.squaredNorm() +
		invMass1 * grad1.squaredNorm() +
		invMass2 * grad2.squaredNorm() +
		invMass3 * grad3.squaredNorm();

	Real alpha = 0.0;
	if (stiffness != 0.0)
	{
		alpha = static_cast<Real>(1.0) / (stiffness * dt * dt);
		K += alpha;
	}

	if (fabs(K) < eps)
		return false;

	const Real C = volume - restVolume;
	const Real delta_lambda = -(C + alpha * lambda) / K;
	lambda += delta_lambda;

	corr0 = delta_lambda * invMass0 * grad0;
	corr1 = delta_lambda * invMass1 * grad1;
	corr2 = delta_lambda * invMass2 * grad2;
	corr3 = delta_lambda * invMass3 * grad3;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool XPBD::init_IsometricBendingConstraint(
	const Vector3r& p0,
	const Vector3r& p1,
	const Vector3r& p2,
	const Vector3r& p3,
	Matrix4r& Q)
{
	// Compute matrix Q for quadratic bending
	const Vector3r* x[4] = { &p2, &p3, &p0, &p1 };

	const Vector3r e0 = *x[1] - *x[0];
	const Vector3r e1 = *x[2] - *x[0];
	const Vector3r e2 = *x[3] - *x[0];
	const Vector3r e3 = *x[2] - *x[1];
	const Vector3r e4 = *x[3] - *x[1];

	const Real c01 = MathFunctions::cotTheta(e0, e1);
	const Real c02 = MathFunctions::cotTheta(e0, e2);
	const Real c03 = MathFunctions::cotTheta(-e0, e3);
	const Real c04 = MathFunctions::cotTheta(-e0, e4);

	const Real A0 = static_cast<Real>(0.5) * (e0.cross(e1)).norm();
	const Real A1 = static_cast<Real>(0.5) * (e0.cross(e2)).norm();

	const Real coef = -3.f / (2.f * (A0 + A1));
	const Real K[4] = { c03 + c04, c01 + c02, -c01 - c03, -c02 - c04 };
	const Real K2[4] = { coef * K[0], coef * K[1], coef * K[2], coef * K[3] };

	for (unsigned char j = 0; j < 4; j++)
	{
		for (unsigned char k = 0; k < j; k++)
		{
			Q(j, k) = Q(k, j) = K[j] * K2[k];
		}
		Q(j, j) = K[j] * K2[j];
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
bool XPBD::solve_IsometricBendingConstraint(
	const Vector3r& p0, Real invMass0,
	const Vector3r& p1, Real invMass1,
	const Vector3r& p2, Real invMass2,
	const Vector3r& p3, Real invMass3,
	const Matrix4r& Q,
	const Real stiffness,
	const Real dt,
	Real& lambda,
	Vector3r& corr0, Vector3r& corr1, Vector3r& corr2, Vector3r& corr3)
{
	const Vector3r* x[4] = { &p2, &p3, &p0, &p1 };
	Real invMass[4] = { invMass2, invMass3, invMass0, invMass1 };

	Real energy = 0.0;
	for (unsigned char k = 0; k < 4; k++)
		for (unsigned char j = 0; j < 4; j++)
			energy += Q(j, k) * (x[k]->dot(*x[j]));
	energy *= 0.5;

	Vector3r gradC[4];
	gradC[0].setZero();
	gradC[1].setZero();
	gradC[2].setZero();
	gradC[3].setZero();
	for (unsigned char k = 0; k < 4; k++)
		for (unsigned char j = 0; j < 4; j++)
			gradC[j] += Q(j, k) * *x[k];


	Real sum_normGradC = 0.0;
	for (unsigned int j = 0; j < 4; j++)
	{
		// compute sum of squared gradient norms
		if (invMass[j] != 0.0)
			sum_normGradC += invMass[j] * gradC[j].squaredNorm();
	}

	Real alpha = 0.0;
	if (stiffness != 0.0)
	{
		alpha = static_cast<Real>(1.0) / (stiffness * dt * dt);
		sum_normGradC += alpha;
	}

	// exit early if required
	if (fabs(sum_normGradC) > eps)
	{
		// compute impulse-based scaling factor
		const Real delta_lambda = -(energy + alpha * lambda) / sum_normGradC;
		lambda += delta_lambda;

		corr0 = (delta_lambda * invMass[2]) * gradC[2];
		corr1 = (delta_lambda * invMass[3]) * gradC[3];
		corr2 = (delta_lambda * invMass[0]) * gradC[0];
		corr3 = (delta_lambda * invMass[1]) * gradC[1];

		return true;
	}
	return false;
}


// ----------------------------------------------------------------------------------------------
bool XPBD::solve_FEMTetraConstraint(
	const Vector3r& p0, Real invMass0,
	const Vector3r& p1, Real invMass1,
	const Vector3r& p2, Real invMass2,
	const Vector3r& p3, Real invMass3,
	const Real restVolume,
	const Matrix3r& invRestMat,
	const Real youngsModulus,
	const Real poissonRatio,
	const bool  handleInversion,
	const Real dt,
	Real& multiplier,
	Vector3r& corr0, Vector3r& corr1, Vector3r& corr2, Vector3r& corr3)
{
	corr0.setZero();
	corr1.setZero();
	corr2.setZero();
	corr3.setZero();

	if (youngsModulus <= 0.0)
		return true;

	if (poissonRatio < 0.0 || poissonRatio > 0.49)
		return false;


	Vector3r gradU_[4];
	Matrix3r epsilon, sigma;
	Real volume = (p1 - p0).cross(p2 - p0).dot(p3 - p0) / static_cast<Real>(6.0);

	// compute the Lame coefficients mu and lambda divided by Young's modulus E
	// since we use Young's modulus as compliance factor alpha = 1/E.
	Real mu_ = 1.0 / static_cast<Real>(2.0) / (static_cast<Real>(1.0) + poissonRatio);
	Real lambda_ = 1.0 * poissonRatio / (static_cast<Real>(1.0) + poissonRatio) / (static_cast<Real>(1.0) - static_cast<Real>(2.0) * poissonRatio);

	// compute value U' which is the potential energy of the elastic solid U divided by Young's modulus E
	// U = E * U'
	Real U_ = 0.0;
	if (!handleInversion || volume > 0.0)
	{
		PositionBasedDynamics::computeGreenStrainAndPiolaStress(p0, p1, p2, p3, invRestMat, restVolume, mu_, lambda_, epsilon, sigma, U_);
		PositionBasedDynamics::computeGradCGreen(restVolume, invRestMat, sigma, gradU_);
	}
	else
	{
		PositionBasedDynamics::computeGreenStrainAndPiolaStressInversion(p0, p1, p2, p3, invRestMat, restVolume, mu_, lambda_, epsilon, sigma, U_);
		PositionBasedDynamics::computeGradCGreen(restVolume, invRestMat, sigma, gradU_);
	}

	// By choosing the constraint function as sqrt(2 U'), the potential energy used in XPBD: 
	// U = 0.5 * alpha^-1 * C^2
	// gives us exactly the required potential energy of the elastic solid. 
	const Real C = sqrt(2.0 * U_);

	Real sum_normGradU_ =
		invMass0 * gradU_[0].squaredNorm() +
		invMass1 * gradU_[1].squaredNorm() +
		invMass2 * gradU_[2].squaredNorm() +
		invMass3 * gradU_[3].squaredNorm();

	Real alpha = static_cast<Real>(1.0) / (youngsModulus * dt * dt);
	// Note that grad C = 1/C grad U', by multiplying C^2 to alpha, we now can add the factor C^2 to the nominator
	sum_normGradU_ += C * C * alpha;

	if (sum_normGradU_ < eps)
		return false;

	// compute scaling factor
	const Real lambda = -C * (C + alpha * multiplier) / sum_normGradU_;		// since in the next step we use gradU instead of gradC, we only add the factor C instead of C^2
	multiplier += lambda;

	corr0 = lambda * invMass0 * gradU_[0];
	corr1 = lambda * invMass1 * gradU_[1];
	corr2 = lambda * invMass2 * gradU_[2];
	corr3 = lambda * invMass3 * gradU_[3];

	return true;
}
