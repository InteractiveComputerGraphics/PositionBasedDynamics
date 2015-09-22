#include "TimeStepRigidBodyModel.h"
#include "Demos/Utils/TimeManager.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"
#include "PositionBasedDynamics/TimeIntegration.h"
#include <iostream>

using namespace PBD;
using namespace std;

TimeStepRigidBodyModel::TimeStepRigidBodyModel()
{
	m_velocityUpdateMethod = 0;
}

TimeStepRigidBodyModel::~TimeStepRigidBodyModel(void)
{
}

void TimeStepRigidBodyModel::step(RigidBodyModel &model)
{
 	TimeManager *tm = TimeManager::getCurrent ();
 	const float h = tm->getTimeStepSize();
 
 	clearAccelerations(model);
	RigidBodyModel::RigidBodyVector &rb = model.getRigidBodies();
	for (size_t i = 0; i < rb.size(); i++)
 	{ 
		rb[i]->getLastPosition() = rb[i]->getOldPosition();
		rb[i]->getOldPosition() = rb[i]->getPosition();
		TimeIntegration::semiImplicitEuler(h, rb[i]->getMass(), rb[i]->getPosition(), rb[i]->getVelocity(), rb[i]->getAcceleration());
		rb[i]->getLastRotation() = rb[i]->getOldRotation();
		rb[i]->getOldRotation() = rb[i]->getRotation();
		TimeIntegration::semiImplicitEulerRotation(h, rb[i]->getMass(), rb[i]->getInertiaTensorInverseW(), rb[i]->getRotation(), rb[i]->getAngularVelocity(), rb[i]->getTorque());
		rb[i]->rotationUpdated();
 	}
 
 	constraintProjection(model);
 
 	// Update velocities	
	for (size_t i = 0; i < rb.size(); i++)
 	{
		if (m_velocityUpdateMethod == 0)
		{
			TimeIntegration::velocityUpdateFirstOrder(h, rb[i]->getMass(), rb[i]->getPosition(), rb[i]->getOldPosition(), rb[i]->getVelocity());
			TimeIntegration::angularVelocityUpdateFirstOrder(h, rb[i]->getMass(), rb[i]->getRotation(), rb[i]->getOldRotation(), rb[i]->getAngularVelocity());
		}
		else
		{
			TimeIntegration::velocityUpdateSecondOrder(h, rb[i]->getMass(), rb[i]->getPosition(), rb[i]->getOldPosition(), rb[i]->getLastPosition(), rb[i]->getVelocity());
			TimeIntegration::angularVelocityUpdateSecondOrder(h, rb[i]->getMass(), rb[i]->getRotation(), rb[i]->getOldRotation(), rb[i]->getLastRotation(), rb[i]->getAngularVelocity());
		}
 	}

	// compute new time	
	tm->setTime (tm->getTime () + h);
}

void TimeStepRigidBodyModel::clearAccelerations(RigidBodyModel &model)
{
	RigidBodyModel::RigidBodyVector &rb = model.getRigidBodies();
 	const Eigen::Vector3f grav(0.0f, -9.81f, 0.0f);
 	for (size_t i=0; i < rb.size(); i++)
 	{
 		// Clear accelerations of dynamic particles
 		if (rb[i]->getMass() != 0.0)
 		{
			Eigen::Vector3f &a = rb[i]->getAcceleration();
 			a = grav;
 		}
 	}
}

void TimeStepRigidBodyModel::reset(RigidBodyModel &model)
{

}

void TimeStepRigidBodyModel::constraintProjection(RigidBodyModel &model)
{
	const unsigned int maxIter = 50;
	unsigned int iter = 0;

	RigidBodyModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyModel::JointVector &joints = model.getJoints();

 	while (iter < maxIter)
 	{
		for (unsigned int i = 0; i < joints.size(); i++)
		{
			if (joints[i]->getTypeId() == RigidBodyModel::BallJoint::TYPE_ID)
				constraintProjectionBallJoint(model, i);
			else if (joints[i]->getTypeId() == RigidBodyModel::BallOnLineJoint::TYPE_ID)
				constraintProjectionBallOnLineJoint(model, i);
			else if (joints[i]->getTypeId() == RigidBodyModel::HingeJoint::TYPE_ID)
				constraintProjectionHingeJoint(model, i);
			else if (joints[i]->getTypeId() == RigidBodyModel::UniversalJoint::TYPE_ID)
				constraintProjectionUniversalJoint(model, i);
		}
 		iter++;
 	}
}

void TimeStepRigidBodyModel::constraintProjectionBallJoint(RigidBodyModel &model, const unsigned int index)
{
	RigidBodyModel::JointVector &joints = model.getJoints();
	RigidBodyModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyModel::BallJoint &bj = *(RigidBodyModel::BallJoint*) joints[index];	

	model.updateBallJoint(index);

	RigidBody &rb1 = *rb[bj.m_index[0]];
	RigidBody &rb2 = *rb[bj.m_index[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solveRigidBodyBallJoint(
		rb1.getMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		bj.m_jointInfo,
		corr_x1,
		corr_q1,
		corr_x2,
		corr_q2);

	if (res)
	{
		if (rb1.getMass() != 0.0f)
		{
			rb1.getPosition() += corr_x1;
			rb1.getRotation().coeffs() += corr_q1.coeffs();
			rb1.getRotation().normalize();
			rb1.rotationUpdated();
		}
		if (rb2.getMass() != 0.0f)
		{
			rb2.getPosition() += corr_x2;
			rb2.getRotation().coeffs() += corr_q2.coeffs();
			rb2.getRotation().normalize();
			rb2.rotationUpdated();
		}
	}
}

void TimeStepRigidBodyModel::constraintProjectionBallOnLineJoint(RigidBodyModel &model, const unsigned int index)
{
	RigidBodyModel::JointVector &joints = model.getJoints();
	RigidBodyModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyModel::BallOnLineJoint &bj = *(RigidBodyModel::BallOnLineJoint*) joints[index];

	model.updateBallOnLineJoint(index);

	RigidBody &rb1 = *rb[bj.m_index[0]];
	RigidBody &rb2 = *rb[bj.m_index[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solveRigidBodyBallOnLineJoint(
		rb1.getMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		bj.m_jointInfo,
		corr_x1,
		corr_q1,
		corr_x2,
		corr_q2);

	if (res)
	{
		if (rb1.getMass() != 0.0f)
		{
			rb1.getPosition() += corr_x1;
			rb1.getRotation().coeffs() += corr_q1.coeffs();
			rb1.getRotation().normalize();
			rb1.rotationUpdated();
		}
		if (rb2.getMass() != 0.0f)
		{
			rb2.getPosition() += corr_x2;
			rb2.getRotation().coeffs() += corr_q2.coeffs();
			rb2.getRotation().normalize();
			rb2.rotationUpdated();
		}
	}
}

void TimeStepRigidBodyModel::constraintProjectionHingeJoint(RigidBodyModel &model, const unsigned int index)
{
	RigidBodyModel::JointVector &joints = model.getJoints();
	RigidBodyModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyModel::HingeJoint &hj = *(RigidBodyModel::HingeJoint*) joints[index];

	model.updateHingeJoint(index);

	RigidBody &rb1 = *rb[hj.m_index[0]];
	RigidBody &rb2 = *rb[hj.m_index[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solveRigidBodyHingeJoint(
		rb1.getMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		hj.m_jointInfo,
		corr_x1,
		corr_q1,
		corr_x2,
		corr_q2);

	if (res)
	{
		if (rb1.getMass() != 0.0f)
		{
			rb1.getPosition() += corr_x1;
			rb1.getRotation().coeffs() += corr_q1.coeffs();
			rb1.getRotation().normalize();
			rb1.rotationUpdated();
		}
		if (rb2.getMass() != 0.0f)
		{
			rb2.getPosition() += corr_x2;
			rb2.getRotation().coeffs() += corr_q2.coeffs();
			rb2.getRotation().normalize();
			rb2.rotationUpdated();
		}
	}
}

void TimeStepRigidBodyModel::constraintProjectionUniversalJoint(RigidBodyModel &model, const unsigned int index)
{
	RigidBodyModel::JointVector &joints = model.getJoints();
	RigidBodyModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyModel::UniversalJoint &uj = *(RigidBodyModel::UniversalJoint*) joints[index];

	model.updateUniversalJoint(index);

	RigidBody &rb1 = *rb[uj.m_index[0]];
	RigidBody &rb2 = *rb[uj.m_index[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solveRigidBodyUniversalJoint(
		rb1.getMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		uj.m_jointInfo,
		corr_x1,
		corr_q1,
		corr_x2,
		corr_q2);

	if (res)
	{
		if (rb1.getMass() != 0.0f)
		{
			rb1.getPosition() += corr_x1;
			rb1.getRotation().coeffs() += corr_q1.coeffs();
			rb1.getRotation().normalize();
			rb1.rotationUpdated();
		}
		if (rb2.getMass() != 0.0f)
		{
			rb2.getPosition() += corr_x2;
			rb2.getRotation().coeffs() += corr_q2.coeffs();
			rb2.getRotation().normalize();
			rb2.rotationUpdated();
		}
	}
}
