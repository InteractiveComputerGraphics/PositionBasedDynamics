#include "GenericConstraints.h"
#include "Demos/Simulation/SimulationModel.h"
#include "PositionBasedDynamics/PositionBasedGenericConstraints.h"
#include "PositionBasedDynamics/MathFunctions.h"

using namespace PBD;

int GenericDistanceConstraint::TYPE_ID = 1001;
int GenericIsometricBendingConstraint::TYPE_ID = 1002;


//////////////////////////////////////////////////////////////////////////
// GenericDistanceConstraint
//////////////////////////////////////////////////////////////////////////

void GenericDistanceConstraint::constraintFct(
	const unsigned int numberOfParticles,
	const float mass[],
	const Eigen::Vector3f x[],
	void *userData,
	Eigen::Matrix<float, 1, 1> &constraintValue)
{
	float restLength = *(float*)userData;
	Eigen::Matrix<float, 1, 1> C;
	C(0, 0) = (x[1] - x[0]).norm() - restLength;
	constraintValue = C;
}

void GenericDistanceConstraint::gradientFct(
	const unsigned int i,
	const unsigned int numberOfParticles,
	const float mass[],
	const Eigen::Vector3f x[],
	void *userData,
	Eigen::Matrix<float, 1, 3> &jacobian)
{
	Eigen::Vector3f n = x[i] - x[1 - i];
	n.normalize();
	jacobian = n.transpose();
}

bool GenericDistanceConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	ParticleData &pd = model.getParticles();

	const Eigen::Vector3f &x1_0 = pd.getPosition0(particle1);
	const Eigen::Vector3f &x2_0 = pd.getPosition0(particle2);

	m_restLength = (x2_0 - x1_0).norm();

	return true;
}

bool GenericDistanceConstraint::solveConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];

	Eigen::Vector3f &x1 = pd.getPosition(i1);
	Eigen::Vector3f &x2 = pd.getPosition(i2);
	const float invMass1 = pd.getInvMass(i1);
	const float invMass2 = pd.getInvMass(i2);

	const float invMass[2] = { invMass1, invMass2 };
	const Eigen::Vector3f x[2] = { x1, x2 };

	Eigen::Vector3f corr[2];
	const bool res = PositionBasedGenericConstraints::solveGenericConstraint<2, 1>(
		invMass, x, &m_restLength,
		GenericDistanceConstraint::constraintFct,
		GenericDistanceConstraint::gradientFct,
		corr);

	if (res)
	{
		const float stiffness = model.getClothStiffness();
		if (invMass1 != 0.0f)
			x1 += stiffness * corr[0];
		if (invMass2 != 0.0f)
			x2 += stiffness * corr[1];
	}
	return res;
}


//////////////////////////////////////////////////////////////////////////
// GenericIsometricBendingConstraint
//////////////////////////////////////////////////////////////////////////

void GenericIsometricBendingConstraint::constraintFct(
	const unsigned int numberOfParticles,
	const float invMass[],
	const Eigen::Vector3d x[],
	void *userData,
	Eigen::Matrix<double, 1, 1> &constraintValue)
{
	Eigen::Matrix4f *Q = (Eigen::Matrix4f*)userData;

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

	const Eigen::Vector3f &x0 = pd.getPosition0(m_bodies[0]);
	const Eigen::Vector3f &x1 = pd.getPosition0(m_bodies[1]);
	const Eigen::Vector3f &x2 = pd.getPosition0(m_bodies[2]);
	const Eigen::Vector3f &x3 = pd.getPosition0(m_bodies[3]);

	// Compute matrix Q for quadratic bending
	const Eigen::Vector3f *x[4] = { &x0, &x1, &x2, &x3 };

	const Eigen::Vector3f e0 = *x[1] - *x[0];
	const Eigen::Vector3f e1 = *x[2] - *x[0];
	const Eigen::Vector3f e2 = *x[3] - *x[0];
	const Eigen::Vector3f e3 = *x[2] - *x[1];
	const Eigen::Vector3f e4 = *x[3] - *x[1];

	const float c01 = MathFunctions::cotTheta(e0, e1);
	const float c02 = MathFunctions::cotTheta(e0, e2);
	const float c03 = MathFunctions::cotTheta(-e0, e3);
	const float c04 = MathFunctions::cotTheta(-e0, e4);

	const float A0 = 0.5f * (e0.cross(e1)).norm();
	const float A1 = 0.5f * (e0.cross(e2)).norm();

	const float coef = -3.f / (2.f*(A0 + A1));
	const float K[4] = { c03 + c04, c01 + c02, -c01 - c03, -c02 - c04 };
	const float K2[4] = { coef*K[0], coef*K[1], coef*K[2], coef*K[3] };

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

bool GenericIsometricBendingConstraint::solveConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i0 = m_bodies[0];
	const unsigned i1 = m_bodies[1];
	const unsigned i2 = m_bodies[2];
	const unsigned i3 = m_bodies[3];

	Eigen::Vector3f &x0 = pd.getPosition(i0);
	Eigen::Vector3f &x1 = pd.getPosition(i1);
	Eigen::Vector3f &x2 = pd.getPosition(i2);
	Eigen::Vector3f &x3 = pd.getPosition(i3);
	const float invMass0 = pd.getInvMass(i0);
	const float invMass1 = pd.getInvMass(i1);
	const float invMass2 = pd.getInvMass(i2);
	const float invMass3 = pd.getInvMass(i3);

	float invMass[4] = { invMass0, invMass1, invMass2, invMass3 };
	const Eigen::Vector3f x[4] = { x0, x1, x2, x3 };
 
 	Eigen::Vector3f corr[4];
 
	const bool res = PositionBasedGenericConstraints::solveGenericConstraint<4, 1>(
		invMass, x, &m_Q,
		GenericIsometricBendingConstraint::constraintFct,
		//GenericIsometricBendingConstraint::gradientFct,
 		corr);
 
 	if (res)
 	{
		const float stiffness = model.getClothBendingStiffness();
 		if (invMass0 != 0.0f)
			x0 += stiffness*corr[0];
 		if (invMass1 != 0.0f)
			x1 += stiffness*corr[1];
		if (invMass2 != 0.0f)
			x2 += stiffness*corr[2];
		if (invMass3 != 0.0f)
			x3 += stiffness*corr[3];
 	}
	return res;
}

