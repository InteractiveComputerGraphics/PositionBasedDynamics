#include "RigidBodyModel.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"

using namespace PBD;

RigidBodyModel::RigidBodyModel()
{	
}

RigidBodyModel::~RigidBodyModel(void)
{
	m_rigidBodies.clear();
	m_ballJoints.clear();
}

void RigidBodyModel::reset()
{
	for (size_t i = 0; i < m_rigidBodies.size(); i++)
	{
		m_rigidBodies[i].getPosition() = m_rigidBodies[i].getPosition0();
		m_rigidBodies[i].getOldPosition() = m_rigidBodies[i].getPosition0();
		m_rigidBodies[i].getLastPosition() = m_rigidBodies[i].getPosition0();

		m_rigidBodies[i].getRotation() = m_rigidBodies[i].getRotation0();
		m_rigidBodies[i].getOldRotation() = m_rigidBodies[i].getRotation0();
		m_rigidBodies[i].getLastRotation() = m_rigidBodies[i].getRotation0();

		m_rigidBodies[i].getVelocity().setZero();
		m_rigidBodies[i].getAngularVelocity().setZero();

		m_rigidBodies[i].getAcceleration().setZero();
		m_rigidBodies[i].getTorque().setZero();

		m_rigidBodies[i].rotationUpdated();
	}
	updateBallJoints();
}

RigidBodyModel::RigidBodyVector & RigidBodyModel::getRigidBodies()
{
	return m_rigidBodies;
}

RigidBodyModel::BallJointVector & RigidBodyModel::getBallJoints()
{
	return m_ballJoints;
}


void RigidBodyModel::setBallJoint(const unsigned int i, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos)
{
	m_ballJoints[i].m_index[0] = rbIndex1;
	m_ballJoints[i].m_index[1] = rbIndex2;

	m_ballJoints[i].m_points[0] = pos;
	m_ballJoints[i].m_points[1] = pos;

	// transform in local coordinates
	RigidBody &r1 = m_rigidBodies[rbIndex1];
	RigidBody &r2 = m_rigidBodies[rbIndex2];

	const Eigen::Matrix3f rot1 = r1.getRotation0().matrix();
	const Eigen::Matrix3f rot2 = r2.getRotation0().matrix();

	m_ballJoints[i].m_localPoints[0] = rot1 * (pos - r1.getPosition0());
	m_ballJoints[i].m_localPoints[1] = rot2 * (pos - r2.getPosition0());
}

void RigidBodyModel::updateBallJoints()
{
	for (unsigned int i = 0; i < m_ballJoints.size(); i++)
	{
		updateBallJoint(i);
	}
}

void RigidBodyModel::updateBallJoint(const unsigned int i)
{
	BallJoint &bj = m_ballJoints[i];
	for (unsigned int j = 0; j < 2; j++)
	{
		RigidBody &rb = m_rigidBodies[bj.m_index[j]];
		bj.m_points[j] = rb.getRotationMatrix() * bj.m_localPoints[j] + rb.getPosition();
	}
}
