#include "RigidBodyModel.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"

using namespace PBD;

int RigidBodyModel::BallJoint::TYPE_ID = 1;
int RigidBodyModel::BallOnLineJoint::TYPE_ID = 2;
int RigidBodyModel::HingeJoint::TYPE_ID = 3;
int RigidBodyModel::UniversalJoint::TYPE_ID = 4;

RigidBodyModel::RigidBodyModel()
{	
}

RigidBodyModel::~RigidBodyModel(void)
{
	for (unsigned int i = 0; i < m_rigidBodies.size(); i++)
		delete m_rigidBodies[i];
	m_rigidBodies.clear();
	for (unsigned int i = 0; i < m_joints.size(); i++)
		delete m_joints[i];
	m_joints.clear();
}

void RigidBodyModel::reset()
{
	for (size_t i = 0; i < m_rigidBodies.size(); i++)
	{
		m_rigidBodies[i]->getPosition() = m_rigidBodies[i]->getPosition0();
		m_rigidBodies[i]->getOldPosition() = m_rigidBodies[i]->getPosition0();
		m_rigidBodies[i]->getLastPosition() = m_rigidBodies[i]->getPosition0();

		m_rigidBodies[i]->getRotation() = m_rigidBodies[i]->getRotation0();
		m_rigidBodies[i]->getOldRotation() = m_rigidBodies[i]->getRotation0();
		m_rigidBodies[i]->getLastRotation() = m_rigidBodies[i]->getRotation0();

		m_rigidBodies[i]->getVelocity().setZero();
		m_rigidBodies[i]->getAngularVelocity().setZero();

		m_rigidBodies[i]->getAcceleration().setZero();
		m_rigidBodies[i]->getTorque().setZero();

		m_rigidBodies[i]->rotationUpdated();
	}
	
	updateJoints();
}

RigidBodyModel::RigidBodyVector & RigidBodyModel::getRigidBodies()
{
	return m_rigidBodies;
}

RigidBodyModel::JointVector & RigidBodyModel::getJoints()
{
	return m_joints;
}

void RigidBodyModel::updateJoints()
{
	for (unsigned int i = 0; i < m_joints.size(); i++)
	{
		if (m_joints[i]->getTypeId() == RigidBodyModel::BallJoint::TYPE_ID)
			updateBallJoint(i);
		else if (m_joints[i]->getTypeId() == RigidBodyModel::BallOnLineJoint::TYPE_ID)
			updateBallOnLineJoint(i);
		else if (m_joints[i]->getTypeId() == RigidBodyModel::HingeJoint::TYPE_ID)
			updateHingeJoint(i);
		else if (m_joints[i]->getTypeId() == RigidBodyModel::UniversalJoint::TYPE_ID)
			updateUniversalJoint(i);
	}
}


void RigidBodyModel::addBallJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos)
{
	BallJoint *bj = new BallJoint();
	bj->m_index[0] = rbIndex1;
	bj->m_index[1] = rbIndex2;

	// transform in local coordinates
	RigidBody &rb1 = *m_rigidBodies[rbIndex1];
	RigidBody &rb2 = *m_rigidBodies[rbIndex2];

	PositionBasedRigidBodyDynamics::initRigidBodyBallJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos,
		bj->m_jointInfo);

	m_joints.push_back(bj);
}

void RigidBodyModel::updateBallJoint(const unsigned int i)
{
	BallJoint &bj = *(BallJoint*)m_joints[i];
	RigidBody &rb1 = *m_rigidBodies[bj.m_index[0]];
	RigidBody &rb2 = *m_rigidBodies[bj.m_index[1]];
	PositionBasedRigidBodyDynamics::updateRigidBodyBallJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		bj.m_jointInfo);
}

void RigidBodyModel::addBallOnLineJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &dir)
{
	BallOnLineJoint *bj = new BallOnLineJoint();
	bj->m_index[0] = rbIndex1;
	bj->m_index[1] = rbIndex2;

	// transform in local coordinates
	RigidBody &rb1 = *m_rigidBodies[rbIndex1];
	RigidBody &rb2 = *m_rigidBodies[rbIndex2];

	PositionBasedRigidBodyDynamics::initRigidBodyBallOnLineJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos, dir,
		bj->m_jointInfo);

	m_joints.push_back(bj);
}

void RigidBodyModel::updateBallOnLineJoint(const unsigned int i)
{
	BallOnLineJoint &bj = *(BallOnLineJoint*)m_joints[i];
	RigidBody &rb1 = *m_rigidBodies[bj.m_index[0]];
	RigidBody &rb2 = *m_rigidBodies[bj.m_index[1]];
	PositionBasedRigidBodyDynamics::updateRigidBodyBallOnLineJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		bj.m_jointInfo);
}


void RigidBodyModel::addHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
{
	HingeJoint *hj = new HingeJoint();
	hj->m_index[0] = rbIndex1;
	hj->m_index[1] = rbIndex2;

	// transform in local coordinates
	RigidBody &rb1 = *m_rigidBodies[rbIndex1];
	RigidBody &rb2 = *m_rigidBodies[rbIndex2];

	PositionBasedRigidBodyDynamics::initRigidBodyHingeJointInfo(
		rb1.getPosition0(),
		rb1.getRotation0(),
		rb2.getPosition0(),
		rb2.getRotation0(),
		pos,
		axis,
		hj->m_jointInfo);

	m_joints.push_back(hj);
}

void RigidBodyModel::updateHingeJoint(const unsigned int i)
{
	HingeJoint &hj = *(HingeJoint*) m_joints[i];
	RigidBody &rb1 = *m_rigidBodies[hj.m_index[0]];
	RigidBody &rb2 = *m_rigidBodies[hj.m_index[1]];
	PositionBasedRigidBodyDynamics::updateRigidBodyHingeJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		hj.m_jointInfo);
}

void RigidBodyModel::addUniversalJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2)
{
	UniversalJoint *uj = new UniversalJoint();
	uj->m_index[0] = rbIndex1;
	uj->m_index[1] = rbIndex2;

	// transform in local coordinates
	RigidBody &rb1 = *m_rigidBodies[rbIndex1];
	RigidBody &rb2 = *m_rigidBodies[rbIndex2];

	PositionBasedRigidBodyDynamics::initRigidBodyUniversalJointInfo(
		rb1.getPosition0(),
		rb1.getRotation0(),
		rb2.getPosition0(),
		rb2.getRotation0(),
		pos,
		axis1,
		axis2,
		uj->m_jointInfo);

	m_joints.push_back(uj);
}

void RigidBodyModel::updateUniversalJoint(const unsigned int i)
{
	UniversalJoint &uj = *(UniversalJoint*)m_joints[i];
	RigidBody &rb1 = *m_rigidBodies[uj.m_index[0]];
	RigidBody &rb2 = *m_rigidBodies[uj.m_index[1]];
	PositionBasedRigidBodyDynamics::updateRigidBodyUniversalJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		uj.m_jointInfo);
}
