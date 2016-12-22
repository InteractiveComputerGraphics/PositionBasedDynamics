#include "GenericConstraints.h"
#include "Demos/Simulation/SimulationModel.h"
#include "PositionBasedDynamics/PositionBasedGenericConstraints.h"
#include "PositionBasedDynamics/MathFunctions.h"
#include "Demos/Simulation/IDFactory.h"

using namespace PBD;

int GenericDistanceConstraint::TYPE_ID = IDFactory::getId();
int GenericIsometricBendingConstraint::TYPE_ID = IDFactory::getId();


//////////////////////////////////////////////////////////////////////////
// GenericDistanceConstraint
//////////////////////////////////////////////////////////////////////////

void GenericDistanceConstraint::constraintFct(
	const unsigned int numberOfParticles,
	const Real mass[],
	const Vector3r x[],
	void *userData,
	Eigen::Matrix<Real, 1, 1> &constraintValue)
{
	Real restLength = *(Real*)userData;
	Eigen::Matrix<Real, 1, 1> C;
	C(0, 0) = (x[1] - x[0]).norm() - restLength;
	constraintValue = C;
}

void GenericDistanceConstraint::gradientFct(
	const unsigned int i,
	const unsigned int numberOfParticles,
	const Real mass[],
	const Vector3r x[],
	void *userData,
	Eigen::Matrix<Real, 1, 3> &jacobian)
{
	Vector3r n = x[i] - x[1 - i];
	n.normalize();
	jacobian = n.transpose();
}

bool GenericDistanceConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	ParticleData &pd = model.getParticles();

	const Vector3r &x1_0 = pd.getPosition0(particle1);
	const Vector3r &x2_0 = pd.getPosition0(particle2);

	m_restLength = (x2_0 - x1_0).norm();

	return true;
}

bool GenericDistanceConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];

	Vector3r &x1 = pd.getPosition(i1);
	Vector3r &x2 = pd.getPosition(i2);
	const Real invMass1 = pd.getInvMass(i1);
	const Real invMass2 = pd.getInvMass(i2);

	const Real invMass[2] = { invMass1, invMass2 };
	const Vector3r x[2] = { x1, x2 };

	Vector3r corr[2];
	const bool res = PositionBasedGenericConstraints::solve_GenericConstraint<2, 1>(
		invMass, x, &m_restLength,
		GenericDistanceConstraint::constraintFct,
		GenericDistanceConstraint::gradientFct,
		corr);

	if (res)
	{
		const Real stiffness = model.getClothStiffness();
		if (invMass1 != 0.0)
			x1 += stiffness * corr[0];
		if (invMass2 != 0.0)
			x2 += stiffness * corr[1];
	}
	return res;
}


//////////////////////////////////////////////////////////////////////////
// GenericIsometricBendingConstraint
//////////////////////////////////////////////////////////////////////////

void GenericIsometricBendingConstraint::constraintFct(
	const unsigned int numberOfParticles,
	const Real invMass[],
	const Vector3r x[],
	void *userData,
	Eigen::Matrix<Real, 1, 1> &constraintValue)
{
	Matrix4r *Q = (Matrix4r*)userData;

	double energy = 0.0;
	for (unsigned char k = 0; k < 4; k++)
	for (unsigned char j = 0; j < 4; j++)
		energy += (double) (*Q)(j, k)*(x[k].dot(x[j]));
	energy *= 0.5;

	constraintValue(0, 0) = energy;
}

bool GenericIsometricBendingConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
						const unsigned int particle3, const unsigned int particle4)
{
	m_bodies[0] = particle3;
	m_bodies[1] = particle4;
	m_bodies[2] = particle1;
	m_bodies[3] = particle2;
	ParticleData &pd = model.getParticles();

	const Vector3r &x0 = pd.getPosition0(m_bodies[0]);
	const Vector3r &x1 = pd.getPosition0(m_bodies[1]);
	const Vector3r &x2 = pd.getPosition0(m_bodies[2]);
	const Vector3r &x3 = pd.getPosition0(m_bodies[3]);

	// Compute matrix Q for quadratic bending
	const Vector3r *x[4] = { &x0, &x1, &x2, &x3 };

	const Vector3r e0 = *x[1] - *x[0];
	const Vector3r e1 = *x[2] - *x[0];
	const Vector3r e2 = *x[3] - *x[0];
	const Vector3r e3 = *x[2] - *x[1];
	const Vector3r e4 = *x[3] - *x[1];

	const Real c01 = MathFunctions::cotTheta(e0, e1);
	const Real c02 = MathFunctions::cotTheta(e0, e2);
	const Real c03 = MathFunctions::cotTheta(-e0, e3);
	const Real c04 = MathFunctions::cotTheta(-e0, e4);

	const Real A0 = 0.5 * (e0.cross(e1)).norm();
	const Real A1 = 0.5 * (e0.cross(e2)).norm();

	const Real coef = -3.0 / (2.0*(A0 + A1));
	const Real K[4] = { c03 + c04, c01 + c02, -c01 - c03, -c02 - c04 };
	const Real K2[4] = { coef*K[0], coef*K[1], coef*K[2], coef*K[3] };

	for (unsigned char j = 0; j < 4; j++)
	{
		for (unsigned char k = 0; k < j; k++)
		{
			m_Q(j, k) = m_Q(k, j) = K[j] * K2[k];
		}
		m_Q(j, j) = K[j] * K2[j];
	}

	return true;
}

bool GenericIsometricBendingConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i0 = m_bodies[0];
	const unsigned i1 = m_bodies[1];
	const unsigned i2 = m_bodies[2];
	const unsigned i3 = m_bodies[3];

	Vector3r &x0 = pd.getPosition(i0);
	Vector3r &x1 = pd.getPosition(i1);
	Vector3r &x2 = pd.getPosition(i2);
	Vector3r &x3 = pd.getPosition(i3);
	const Real invMass0 = pd.getInvMass(i0);
	const Real invMass1 = pd.getInvMass(i1);
	const Real invMass2 = pd.getInvMass(i2);
	const Real invMass3 = pd.getInvMass(i3);

	Real invMass[4] = { invMass0, invMass1, invMass2, invMass3 };
	const Vector3r x[4] = { x0, x1, x2, x3 };

	Vector3r corr[4];

	const bool res = PositionBasedGenericConstraints::solve_GenericConstraint<4, 1>(
		invMass, x, &m_Q,
		GenericIsometricBendingConstraint::constraintFct,
		//GenericIsometricBendingConstraint::gradientFct,
		corr);

	if (res)
	{
		const Real stiffness = model.getClothBendingStiffness();
		if (invMass0 != 0.0)
			x0 += stiffness*corr[0];
		if (invMass1 != 0.0)
			x1 += stiffness*corr[1];
		if (invMass2 != 0.0)
			x2 += stiffness*corr[2];
		if (invMass3 != 0.0)
			x3 += stiffness*corr[3];
	}
	return res;
}

