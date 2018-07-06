#include "GenericConstraints.h"
#include "Simulation/SimulationModel.h"
#include "PositionBasedDynamics/PositionBasedGenericConstraints.h"
#include "PositionBasedDynamics/MathFunctions.h"
#include "Simulation/IDFactory.h"

using namespace PBD;

int GenericDistanceConstraint::TYPE_ID = IDFactory::getId();
int GenericIsometricBendingConstraint::TYPE_ID = IDFactory::getId();
int GenericHingeJoint::TYPE_ID = IDFactory::getId();
int GenericBallJoint::TYPE_ID = IDFactory::getId();
int GenericSliderJoint::TYPE_ID = IDFactory::getId();


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

bool GenericDistanceConstraint::solvePositionConstraint(SimulationModel &model, const unsigned int iter)
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
		const Real stiffness = model.getValue<Real>(SimulationModel::CLOTH_STIFFNESS);
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

	Real energy = 0.0;
	for (unsigned char k = 0; k < 4; k++)
	for (unsigned char j = 0; j < 4; j++)
		energy += (*Q)(j, k)*(x[k].dot(x[j]));
	energy *= static_cast<Real>(0.5);

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

	const Real A0 = static_cast<Real>(0.5) * (e0.cross(e1)).norm();
	const Real A1 = static_cast<Real>(0.5) * (e0.cross(e2)).norm();

	const Real coef = -static_cast<Real>(3.0) / (static_cast<Real>(2.0)*(A0 + A1));
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

bool GenericIsometricBendingConstraint::solvePositionConstraint(SimulationModel &model, const unsigned int iter)
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
		const Real stiffness = model.getValue<Real>(SimulationModel::CLOTH_BENDING_STIFFNESS);
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

//////////////////////////////////////////////////////////////////////////
// GenericHingeJoint
//////////////////////////////////////////////////////////////////////////

void GenericHingeJoint::constraintFct(
	const unsigned int numberOfRigidBodies,
	const Real mass[],
	const Vector3r x[],
	const Matrix3r inertiaInverseW[],
	const Quaternionr q[],
	void *userData,
	Eigen::Matrix<Real, 5, 1> &constraintValue)
{
	Eigen::Matrix<Real, 3, 12> &jointInfo = *(Eigen::Matrix<Real, 3, 12>*)userData;
	
	const Vector3r &c0 = jointInfo.col(6);
	const Vector3r &c1 = jointInfo.col(7);
	const Vector3r &axis1 = jointInfo.col(11);
	const Vector3r &t1 = jointInfo.col(9);
	const Vector3r &t2 = jointInfo.col(10);

	constraintValue.block<3, 1>(0, 0) = c0 - c1;
	constraintValue(3, 0) = t1.dot(axis1);
	constraintValue(4, 0) = t2.dot(axis1);
}

void GenericHingeJoint::gradientFct(
	const unsigned int i,
	const unsigned int numberOfRigidBodies,
	const Real mass[],
	const Vector3r x[],
	const Matrix3r inertiaInverseW[],
	const Quaternionr q[],
	void *userData,
	Eigen::Matrix<Real, 5, 6> &jacobian)
{
	Eigen::Matrix<Real, 3, 12> &jointInfo = *(Eigen::Matrix<Real, 3, 12>*)userData;

	const Vector3r &c0 = jointInfo.col(6);
	const Vector3r &c1 = jointInfo.col(7);
	const Vector3r &axis1 = jointInfo.col(11);
	const Vector3r &t1 = jointInfo.col(9);
	const Vector3r &t2 = jointInfo.col(10);

	const Vector3r r0 = c0 - x[0];
	const Vector3r r1 = c1 - x[1];
	Matrix3r r_star;
	if (i==0)
		MathFunctions::crossProductMatrix(r0, r_star);
	else
		MathFunctions::crossProductMatrix(r1, r_star);

	const Vector3r u1 = t1.cross(axis1);
	const Vector3r u2 = t2.cross(axis1);

	// Jacobian:
	//
	// (I_3   -r*)
	// (0     u1^T)
	// (0     u2^T)

	jacobian.setZero();

	if (i == 0)
	{
		jacobian.block<3, 3>(0, 0) = Matrix3r::Identity();
		jacobian.block<3, 3>(0, 3) = -r_star;
		jacobian.block<1, 3>(3, 3) = u1.transpose();
		jacobian.block<1, 3>(4, 3) = u2.transpose();
	}
	else
	{
		jacobian.block<3, 3>(0, 0) = -Matrix3r::Identity();
		jacobian.block<3, 3>(0, 3) = r_star;
		jacobian.block<1, 3>(3, 3) = -u1.transpose();
		jacobian.block<1, 3>(4, 3) = -u2.transpose();
	}
}

bool GenericHingeJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody *rb0 = rb[rbIndex1];
	RigidBody *rb1 = rb[rbIndex2];

	const Vector3r &x0 = rb0->getPosition();
	const Vector3r &x1 = rb1->getPosition();
	const Quaternionr &q0 = rb0->getRotation();
	const Quaternionr &q1 = rb1->getRotation();

	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)

	// transform in local coordinates
	const Matrix3r rot0T = q0.matrix().transpose();
	const Matrix3r rot1T = q1.matrix().transpose();

	// connector in body 0 (local)
	m_jointInfo.col(0) = rot0T * (pos - x0);
	// connector in body 1 (local)
	m_jointInfo.col(1) = rot1T * (pos - x1);
	// connector in body 0 (global)
	m_jointInfo.col(6) = pos;
	// connector in body 1 (global)
	m_jointInfo.col(7) = pos;

	// determine constraint coordinate system
	// with direction as x-axis
	m_jointInfo.col(8) = axis;
	m_jointInfo.col(8).normalize();

	Vector3r v(1.0, 0.0, 0.0);
	// check if vectors are parallel
	if (fabs(v.dot(m_jointInfo.col(8))) > 0.99)
		v = Vector3r(0.0, 1.0, 0.0);

	m_jointInfo.col(9) = m_jointInfo.col(8).cross(v);
	m_jointInfo.col(10) = m_jointInfo.col(8).cross(m_jointInfo.col(9));
	m_jointInfo.col(9).normalize();
	m_jointInfo.col(10).normalize();

	// joint axis in body 1 (global)
	m_jointInfo.col(11) = m_jointInfo.col(8);

	// coordinate system of body 0 (local)
	m_jointInfo.block<3, 3>(0, 2) = rot0T * m_jointInfo.block<3, 3>(0, 8);

	// joint axis in body 1 (local)
	m_jointInfo.col(5) = rot1T * m_jointInfo.col(11);

	return true;
}

bool GenericHingeJoint::updateConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody *rb0 = rb[m_bodies[0]];
	RigidBody *rb1 = rb[m_bodies[1]];

	const Vector3r &x0 = rb0->getPosition();
	const Vector3r &x1 = rb1->getPosition();
	const Quaternionr &q0 = rb0->getRotation();
	const Quaternionr &q1 = rb1->getRotation();

	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)

	// compute world space positions of connectors
	const Matrix3r rot0 = q0.matrix();
	const Matrix3r rot1 = q1.matrix();
	m_jointInfo.col(6) = rot0 * m_jointInfo.col(0) + x0;
	m_jointInfo.col(7) = rot1 * m_jointInfo.col(1) + x1;

	// transform constraint coordinate system of body 0 to world space
	m_jointInfo.block<3, 3>(0, 8) = rot0 * m_jointInfo.block<3, 3>(0, 2);
	// transform joint axis of body 1 to world space
	m_jointInfo.col(11) = rot1 * m_jointInfo.col(5);

	return true;
}

bool GenericHingeJoint::solvePositionConstraint(SimulationModel &model, const unsigned int iter)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody *rb0 = rb[m_bodies[0]];
	RigidBody *rb1 = rb[m_bodies[1]];

	const Matrix3r &inertiaInverseW0 = rb0->getInertiaTensorInverseW();
	const Matrix3r &inertiaInverseW1 = rb1->getInertiaTensorInverseW();
	const Real invMass0 = rb0->getInvMass();
	const Real invMass1 = rb1->getInvMass();
	Vector3r &x0 = rb0->getPosition();
	Vector3r &x1 = rb1->getPosition();
	Quaternionr &q0 = rb0->getRotation();
	Quaternionr &q1 = rb1->getRotation();

	const Real invMass[2] = { invMass0, invMass1 };
	const Vector3r x[2] = { x0, x1 };
	const Quaternionr q[2] = { q0, q1 };
	const Matrix3r inertiaInverseW[2] = { inertiaInverseW0, inertiaInverseW1 };

	Vector3r corrX[2];
	Quaternionr corrQ[2];
	const bool res = PositionBasedGenericConstraints::solve_GenericConstraint<2, 5>(
		invMass, x, inertiaInverseW, q, &m_jointInfo, 
		GenericHingeJoint::constraintFct,
		GenericHingeJoint::gradientFct,
		corrX, corrQ);

	if (res)
	{
		if (invMass0 != 0.0)
		{
			x0 += corrX[0];
			q0.coeffs() += corrQ[0].coeffs();
			q0.normalize();
			rb0->rotationUpdated();
		}
		if (invMass1 != 0.0)
		{
			x1 += corrX[1];
			q1.coeffs() += corrQ[1].coeffs();
			q1.normalize();
			rb1->rotationUpdated();
		}
	}
	return res;
}


//////////////////////////////////////////////////////////////////////////
// GenericBallJoint
//////////////////////////////////////////////////////////////////////////

void GenericBallJoint::constraintFct(
	const unsigned int numberOfRigidBodies,
	const Real mass[],
	const Vector3r x[],
	const Matrix3r inertiaInverseW[],
	const Quaternionr q[],
	void *userData,
	Eigen::Matrix<Real, 3, 1> &constraintValue)
{
	Eigen::Matrix<Real, 3, 2> &jointInfo = *(Eigen::Matrix<Real, 3, 2>*)userData;

	const Vector3r c0 = q[0].matrix() * jointInfo.col(0) + x[0];
	const Vector3r c1 = q[1].matrix() * jointInfo.col(1) + x[1];

	constraintValue = c0 - c1;
}

bool GenericBallJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody *rb0 = rb[rbIndex1];
	RigidBody *rb1 = rb[rbIndex2];

	const Vector3r &x0 = rb0->getPosition();
	const Vector3r &x1 = rb1->getPosition();
	const Quaternionr &q0 = rb0->getRotation();
	const Quaternionr &q1 = rb1->getRotation();

	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)

	// transform in local coordinates
	const Matrix3r rot0T = q0.matrix().transpose();
	const Matrix3r rot1T = q1.matrix().transpose();

	m_jointInfo.col(0) = rot0T * (pos - x0);
	m_jointInfo.col(1) = rot1T * (pos - x1);

	return true;
}

bool GenericBallJoint::solvePositionConstraint(SimulationModel &model, const unsigned int iter)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody *rb0 = rb[m_bodies[0]];
	RigidBody *rb1 = rb[m_bodies[1]];

	const Matrix3r &inertiaInverseW0 = rb0->getInertiaTensorInverseW();
	const Matrix3r &inertiaInverseW1 = rb1->getInertiaTensorInverseW();
	const Real invMass0 = rb0->getInvMass();
	const Real invMass1 = rb1->getInvMass();
	Vector3r &x0 = rb0->getPosition();
	Vector3r &x1 = rb1->getPosition();
	Quaternionr &q0 = rb0->getRotation();
	Quaternionr &q1 = rb1->getRotation();

	const Real invMass[2] = { invMass0, invMass1 };
	const Vector3r x[2] = { x0, x1 };
	const Quaternionr q[2] = { q0, q1 };
	const Matrix3r inertiaInverseW[2] = { inertiaInverseW0, inertiaInverseW1 };

	Vector3r corrX[2];
	Quaternionr corrQ[2];
	const bool res = PositionBasedGenericConstraints::solve_GenericConstraint<2, 3>(
		invMass, x, inertiaInverseW, q, &m_jointInfo,
		GenericBallJoint::constraintFct,
		corrX, corrQ);

	if (res)
	{
		if (invMass0 != 0.0)
		{
			x0 += corrX[0];
			q0.coeffs() += corrQ[0].coeffs();
			q0.normalize();
			rb0->rotationUpdated();
		}
		if (invMass1 != 0.0)
		{
			x1 += corrX[1];
			q1.coeffs() += corrQ[1].coeffs();
			q1.normalize();
			rb1->rotationUpdated();
		}
	}
	return res;
}

//////////////////////////////////////////////////////////////////////////
// GenericSliderJoint
//////////////////////////////////////////////////////////////////////////

void GenericSliderJoint::constraintFct(
	const unsigned int numberOfRigidBodies,
	const Real mass[],
	const Vector3r x[],
	const Matrix3r inertiaInverseW[],
	const Quaternionr q[],
	void *userData,
	Eigen::Matrix<Real, 5, 1> &constraintValue)
{
	Eigen::Matrix<Real, 3, 7> &jointInfo = *(Eigen::Matrix<Real, 3, 7>*)userData;

	const Vector3r c0 = q[0].matrix() * jointInfo.col(0) + x[0];
	const Vector3r c1 = q[1].matrix() * jointInfo.col(1) + x[1];
	const Vector3r axis0 = q[0].matrix() * jointInfo.col(2);
	const Vector3r axis1 = q[1].matrix() * jointInfo.col(5);
	const Vector3r u = axis0.cross(axis1);
	const Vector3r t1 = q[0].matrix() * jointInfo.col(3);
	const Vector3r t2 = q[0].matrix() * jointInfo.col(4);
	const Vector3r t3 = q[1].matrix() * jointInfo.col(6);

	// projection 	
	Eigen::Matrix<Real, 2, 3> P;
	P.row(0) = t1.transpose();
	P.row(1) = t2.transpose();

	constraintValue.block<2, 1>(0, 0) = P * (c0 - c1);
	constraintValue(2, 0) = t1.dot(axis1);
	constraintValue(3, 0) = t2.dot(axis1);
	constraintValue(4, 0) = t2.dot(t3);
}

bool GenericSliderJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody *rb0 = rb[rbIndex1];
	RigidBody *rb1 = rb[rbIndex2];

	const Vector3r &x0 = rb0->getPosition();
	const Vector3r &x1 = rb1->getPosition();
	const Quaternionr &q0 = rb0->getRotation();
	const Quaternionr &q1 = rb1->getRotation();

	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)	
	// 6:	perpendicular vector on joint axis (normalized) in body 1 (local)

	// transform in local coordinates
	const Matrix3r rot0T = q0.matrix().transpose();
	const Matrix3r rot1T = q1.matrix().transpose();

	// connector in body 0 (local)
	m_jointInfo.col(0) = rot0T * (pos - x0);
	// connector in body 1 (local)
	m_jointInfo.col(1) = rot1T * (pos - x1);

	// determine constraint coordinate system
	// with direction as x-axis
	Matrix3r A;
	A.col(0) = axis;
	A.col(0).normalize();

	Vector3r v(1.0, 0.0, 0.0);
	// check if vectors are parallel
	if (fabs(v.dot(A.col(0))) > 0.99)
		v = Vector3r(0.0, 1.0, 0.0);

	A.col(1) = A.col(0).cross(v);
	A.col(2) = A.col(0).cross(A.col(1));
	A.col(1).normalize();
	A.col(2).normalize();

	// coordinate system of body 0 (local)
	m_jointInfo.block<3, 3>(0, 2) = rot0T * A;

	// joint axis in body 1 (local)
	m_jointInfo.col(5) = rot1T * A.col(0);

	// perpendicular vector on joint axis(normalized) in body 1 (local)
	m_jointInfo.col(6) = rot1T * A.col(1);

	return true;
}


bool GenericSliderJoint::solvePositionConstraint(SimulationModel &model, const unsigned int iter)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody *rb0 = rb[m_bodies[0]];
	RigidBody *rb1 = rb[m_bodies[1]];

	const Matrix3r &inertiaInverseW0 = rb0->getInertiaTensorInverseW();
	const Matrix3r &inertiaInverseW1 = rb1->getInertiaTensorInverseW();
	const Real invMass0 = rb0->getInvMass();
	const Real invMass1 = rb1->getInvMass();
	Vector3r &x0 = rb0->getPosition();
	Vector3r &x1 = rb1->getPosition();
	Quaternionr &q0 = rb0->getRotation();
	Quaternionr &q1 = rb1->getRotation();

	const Real invMass[2] = { invMass0, invMass1 };
	const Vector3r x[2] = { x0, x1 };
	const Quaternionr q[2] = { q0, q1 };
	const Matrix3r inertiaInverseW[2] = { inertiaInverseW0, inertiaInverseW1 };

	Vector3r corrX[2];
	Quaternionr corrQ[2];
	const bool res = PositionBasedGenericConstraints::solve_GenericConstraint<2, 5>(
		invMass, x, inertiaInverseW, q, &m_jointInfo,
		GenericSliderJoint::constraintFct,
		corrX, corrQ);

	if (res)
	{
		if (invMass0 != 0.0)
		{
			x0 += corrX[0];
			q0.coeffs() += corrQ[0].coeffs();
			q0.normalize();
			rb0->rotationUpdated();
		}
		if (invMass1 != 0.0)
		{
			x1 += corrX[1];
			q1.coeffs() += corrQ[1].coeffs();
			q1.normalize();
			rb1->rotationUpdated();
		}
	}
	return res;
}
