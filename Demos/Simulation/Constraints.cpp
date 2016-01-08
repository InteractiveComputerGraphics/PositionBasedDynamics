#include "Constraints.h"
#include "SimulationModel.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"
#include "PositionBasedDynamics/PositionBasedElasticRod.h"
#include "TimeManager.h"

using namespace PBD;


int BallJoint::TYPE_ID = 1;
int BallOnLineJoint::TYPE_ID = 2;
int HingeJoint::TYPE_ID = 3;
int UniversalJoint::TYPE_ID = 4;
int RigidBodyParticleBallJoint::TYPE_ID = 5;
int DistanceConstraint::TYPE_ID = 6;
int DihedralConstraint::TYPE_ID = 7;
int IsometricBendingConstraint::TYPE_ID = 8;
int FEMTriangleConstraint::TYPE_ID = 9;
int StrainTriangleConstraint::TYPE_ID = 10;
int VolumeConstraint::TYPE_ID = 11;
int FEMTetConstraint::TYPE_ID = 12;
int StrainTetConstraint::TYPE_ID = 13;
int ShapeMatchingConstraint::TYPE_ID = 14;
int TargetAngleMotorHingeJoint::TYPE_ID = 15;
int TargetVelocityMotorHingeJoint::TYPE_ID = 16;
int SliderJoint::TYPE_ID = 17;
int TargetPositionMotorSliderJoint::TYPE_ID = 18;
int TargetVelocityMotorSliderJoint::TYPE_ID = 19;
int ElasticRodEdgeConstraint::TYPE_ID = 20;
int ElasticRodBendAndTwistConstraint::TYPE_ID = 21;

//////////////////////////////////////////////////////////////////////////
// BallJoint
//////////////////////////////////////////////////////////////////////////
bool BallJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::init_BallJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos,
		m_jointInfo);
}

bool BallJoint::updateConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::update_BallJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool BallJoint::solvePositionConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solve_BallJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getInvMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		m_jointInfo,
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
	return res;
}


//////////////////////////////////////////////////////////////////////////
// BallOnLineJoint
//////////////////////////////////////////////////////////////////////////
bool BallOnLineJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &dir)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::init_BallOnLineJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos, dir,
		m_jointInfo);
}

bool BallOnLineJoint::updateConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::update_BallOnLineJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool BallOnLineJoint::solvePositionConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solve_BallOnLineJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getInvMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		m_jointInfo,
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
	return res;
}


//////////////////////////////////////////////////////////////////////////
// HingeJoint
//////////////////////////////////////////////////////////////////////////
bool HingeJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::init_HingeJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos, axis,
		m_jointInfo);
}

bool HingeJoint::updateConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::update_HingeJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool HingeJoint::solvePositionConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solve_HingeJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getInvMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		m_jointInfo,
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
	return res;
}


//////////////////////////////////////////////////////////////////////////
// UniversalJoint
//////////////////////////////////////////////////////////////////////////
bool UniversalJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::init_UniversalJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos,
		axis1,
		axis2,
		m_jointInfo);
}

bool UniversalJoint::updateConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::update_UniversalJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool UniversalJoint::solvePositionConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solve_UniversalJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getInvMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		m_jointInfo,
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
	return res;
}


//////////////////////////////////////////////////////////////////////////
// SliderJoint
//////////////////////////////////////////////////////////////////////////
bool SliderJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::init_SliderJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos, axis,
		m_jointInfo);
}

bool SliderJoint::updateConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::update_SliderJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool SliderJoint::solvePositionConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solve_SliderJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getInvMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		m_jointInfo,
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
	return res;
}


//////////////////////////////////////////////////////////////////////////
// TargetPositionMotorSliderJoint
//////////////////////////////////////////////////////////////////////////
bool TargetPositionMotorSliderJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::init_TargetPositionMotorSliderJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos, axis,
		m_jointInfo);
}

bool TargetPositionMotorSliderJoint::updateConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::update_TargetPositionMotorSliderJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool TargetPositionMotorSliderJoint::solvePositionConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solve_TargetPositionMotorSliderJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getInvMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		m_targetPosition,
		m_jointInfo,
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
	return res;
}



//////////////////////////////////////////////////////////////////////////
// TargetVelocityMotorSliderJoint
//////////////////////////////////////////////////////////////////////////
bool TargetVelocityMotorSliderJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::init_TargetVelocityMotorSliderJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos, axis,
		m_jointInfo);
}

bool TargetVelocityMotorSliderJoint::updateConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::update_TargetVelocityMotorSliderJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool TargetVelocityMotorSliderJoint::solvePositionConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solve_TargetVelocityMotorSliderJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getInvMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		m_jointInfo,
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
	return res;
}


bool TargetVelocityMotorSliderJoint::solveVelocityConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_v1, corr_v2;
	Eigen::Vector3f corr_omega1, corr_omega2;
	const bool res = PositionBasedRigidBodyDynamics::velocitySolve_TargetVelocityMotorSliderJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getVelocity(),
		rb1.getInertiaTensorInverseW(),
		rb1.getAngularVelocity(),
		rb2.getInvMass(),
		rb2.getPosition(),
		rb2.getVelocity(),
		rb2.getInertiaTensorInverseW(),
		rb2.getAngularVelocity(),
		m_targetVelocity,
		m_jointInfo,
		corr_v1,
		corr_omega1,
		corr_v2,
		corr_omega2);

	if (res)
	{
		if (rb1.getMass() != 0.0f)
		{
			rb1.getVelocity() += corr_v1;
			rb1.getAngularVelocity() += corr_omega1;
		}
		if (rb2.getMass() != 0.0f)
		{
			rb2.getVelocity() += corr_v2;
			rb2.getAngularVelocity() += corr_omega2;
		}
	}
	return res;
}

//////////////////////////////////////////////////////////////////////////
// TargetAngleMotorHingeJoint
//////////////////////////////////////////////////////////////////////////
bool TargetAngleMotorHingeJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::init_TargetAngleMotorHingeJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos, axis,
		m_jointInfo);
}

bool TargetAngleMotorHingeJoint::updateConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::update_TargetAngleMotorHingeJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool TargetAngleMotorHingeJoint::solvePositionConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solve_TargetAngleMotorHingeJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getInvMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		m_targetAngle,
		m_jointInfo,
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
	return res;
}

//////////////////////////////////////////////////////////////////////////
// TargetVelocityMotorHingeJoint
//////////////////////////////////////////////////////////////////////////
bool TargetVelocityMotorHingeJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
{
	m_bodies[0] = rbIndex1;
	m_bodies[1] = rbIndex2;
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::init_TargetVelocityMotorHingeJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos, axis,
		m_jointInfo);
}

bool TargetVelocityMotorHingeJoint::updateConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];
	return PositionBasedRigidBodyDynamics::update_TargetVelocityMotorHingeJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool TargetVelocityMotorHingeJoint::solvePositionConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solve_TargetVelocityMotorHingeJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		rb2.getInvMass(),
		rb2.getPosition(),
		rb2.getInertiaTensorInverseW(),
		rb2.getRotation(),
		m_jointInfo,
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
	return res;
}

bool TargetVelocityMotorHingeJoint::solveVelocityConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_v1, corr_v2;
	Eigen::Vector3f corr_omega1, corr_omega2;
	const bool res = PositionBasedRigidBodyDynamics::velocitySolve_TargetVelocityMotorHingeJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getVelocity(),
		rb1.getInertiaTensorInverseW(),
		rb1.getAngularVelocity(),
		rb2.getInvMass(),
		rb2.getPosition(),
		rb2.getVelocity(),
		rb2.getInertiaTensorInverseW(),
		rb2.getAngularVelocity(),
		m_targetAngularVelocity, 
		m_jointInfo,
		corr_v1,
		corr_omega1,
		corr_v2,
		corr_omega2);

	if (res)
	{
		if (rb1.getMass() != 0.0f)
		{
			rb1.getVelocity() += corr_v1;
			rb1.getAngularVelocity() += corr_omega1;
		}
		if (rb2.getMass() != 0.0f)
		{
			rb2.getVelocity() += corr_v2;
			rb2.getAngularVelocity() += corr_omega2;
		}
	}
	return res;
}

//////////////////////////////////////////////////////////////////////////
// RigidBodyParticleBallJoint
//////////////////////////////////////////////////////////////////////////
bool RigidBodyParticleBallJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex, const unsigned int particleIndex)
{
	m_bodies[0] = rbIndex;
	m_bodies[1] = particleIndex;
	SimulationModel::RigidBodyVector &rbs = model.getRigidBodies();
	ParticleData &pd = model.getParticles();
	RigidBody &rb = *rbs[m_bodies[0]];
	return PositionBasedRigidBodyDynamics::init_RigidBodyParticleBallJoint(
		rb.getPosition(),
		rb.getRotation(),
		pd.getPosition(particleIndex),
		m_jointInfo);
}

bool RigidBodyParticleBallJoint::updateConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	ParticleData &pd = model.getParticles();
	RigidBody &rb1 = *rb[m_bodies[0]];
	return PositionBasedRigidBodyDynamics::update_RigidBodyParticleBallJoint(
		rb1.getPosition(),
		rb1.getRotation(),
		pd.getPosition(m_bodies[1]),
		m_jointInfo);
}

bool RigidBodyParticleBallJoint::solvePositionConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	ParticleData &pd = model.getParticles();

	RigidBody &rb1 = *rb[m_bodies[0]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1;
	const bool res = PositionBasedRigidBodyDynamics::solve_RigidBodyParticleBallJoint(
		rb1.getInvMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		pd.getInvMass(m_bodies[1]),
		pd.getPosition(m_bodies[1]),
		m_jointInfo,
		corr_x1,
		corr_q1,
		corr_x2);

	if (res)
	{
		if (rb1.getMass() != 0.0f)
		{
			rb1.getPosition() += corr_x1;
			rb1.getRotation().coeffs() += corr_q1.coeffs();
			rb1.getRotation().normalize();
			rb1.rotationUpdated();
		}
		if (pd.getMass(m_bodies[1]) != 0.0f)
		{
			pd.getPosition(m_bodies[1]) += corr_x2;
		}
	}
	return res;
}

//////////////////////////////////////////////////////////////////////////
// DistanceConstraint
//////////////////////////////////////////////////////////////////////////
bool DistanceConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	ParticleData &pd = model.getParticles();

	const Eigen::Vector3f &x1_0 = pd.getPosition0(particle1);
	const Eigen::Vector3f &x2_0 = pd.getPosition0(particle2);

	m_restLength = (x2_0 - x1_0).norm();

	return true;
}

bool DistanceConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];

	Eigen::Vector3f &x1 = pd.getPosition(i1);
	Eigen::Vector3f &x2 = pd.getPosition(i2);
	const float invMass1 = pd.getInvMass(i1);
	const float invMass2 = pd.getInvMass(i2);

	Eigen::Vector3f corr1, corr2;
	const bool res = PositionBasedDynamics::solve_DistanceConstraint(
		x1, invMass1, x2, invMass2,
		m_restLength, model.getClothStiffness(), model.getClothStiffness(), corr1, corr2);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr1;
		if (invMass2 != 0.0f)
			x2 += corr2;
	}
	return res;
}


//////////////////////////////////////////////////////////////////////////
// DihedralConstraint
//////////////////////////////////////////////////////////////////////////

bool DihedralConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
										const unsigned int particle3, const unsigned int particle4)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	m_bodies[2] = particle3;
	m_bodies[3] = particle4;
	ParticleData &pd = model.getParticles();

	const Eigen::Vector3f &p0 = pd.getPosition0(particle1);
	const Eigen::Vector3f &p1 = pd.getPosition0(particle2);
	const Eigen::Vector3f &p2 = pd.getPosition0(particle3);
	const Eigen::Vector3f &p3 = pd.getPosition0(particle4);

	Eigen::Vector3f e = p3 - p2;
	float  elen = e.norm();
	if (elen < 1e-6f)
		return false;

	float invElen = 1.0f / elen;

	Eigen::Vector3f n1 = (p2 - p0).cross(p3 - p0); n1 /= n1.squaredNorm();
	Eigen::Vector3f n2 = (p3 - p1).cross(p2 - p1); n2 /= n2.squaredNorm();

	n1.normalize();
	n2.normalize();
	float dot = n1.dot(n2);

	if (dot < -1.0f) dot = -1.0f;
	if (dot > 1.0f) dot = 1.0f;

	m_restAngle = acosf(dot);

	return true;
}

bool DihedralConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];
	const unsigned i3 = m_bodies[2];
	const unsigned i4 = m_bodies[3];

	Eigen::Vector3f &x1 = pd.getPosition(i1);
	Eigen::Vector3f &x2 = pd.getPosition(i2);
	Eigen::Vector3f &x3 = pd.getPosition(i3);
	Eigen::Vector3f &x4 = pd.getPosition(i4);

	const float invMass1 = pd.getInvMass(i1);
	const float invMass2 = pd.getInvMass(i2);
	const float invMass3 = pd.getInvMass(i3);
	const float invMass4 = pd.getInvMass(i4);

	Eigen::Vector3f corr1, corr2, corr3, corr4;
	const bool res = PositionBasedDynamics::solve_DihedralConstraint(
		x1, invMass1, x2, invMass2, x3, invMass3, x4, invMass4,
		m_restAngle,
		model.getClothBendingStiffness(),
		corr1, corr2, corr3, corr4);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr1;
		if (invMass2 != 0.0f)
			x2 += corr2;
		if (invMass3 != 0.0f)
			x3 += corr3;
		if (invMass4 != 0.0f)
			x4 += corr4;
	}
	return res;
}


//////////////////////////////////////////////////////////////////////////
// IsometricBendingConstraint
//////////////////////////////////////////////////////////////////////////
bool IsometricBendingConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
												const unsigned int particle3, const unsigned int particle4)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	m_bodies[2] = particle3;
	m_bodies[3] = particle4;

	ParticleData &pd = model.getParticles();

	const Eigen::Vector3f &x1 = pd.getPosition0(particle1);
	const Eigen::Vector3f &x2 = pd.getPosition0(particle2);
	const Eigen::Vector3f &x3 = pd.getPosition0(particle3);
	const Eigen::Vector3f &x4 = pd.getPosition0(particle4);

	return PositionBasedDynamics::init_IsometricBendingConstraint(x1, x2, x3, x4, m_Q);
}

bool IsometricBendingConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];
	const unsigned i3 = m_bodies[2];
	const unsigned i4 = m_bodies[3];

	Eigen::Vector3f &x1 = pd.getPosition(i1);
	Eigen::Vector3f &x2 = pd.getPosition(i2);
	Eigen::Vector3f &x3 = pd.getPosition(i3);
	Eigen::Vector3f &x4 = pd.getPosition(i4);

	const float invMass1 = pd.getInvMass(i1);
	const float invMass2 = pd.getInvMass(i2);
	const float invMass3 = pd.getInvMass(i3);
	const float invMass4 = pd.getInvMass(i4);

	Eigen::Vector3f corr1, corr2, corr3, corr4;
	const bool res = PositionBasedDynamics::solve_IsometricBendingConstraint(
		x1, invMass1, x2, invMass2, x3, invMass3, x4, invMass4,
		m_Q,
		model.getClothBendingStiffness(),
		corr1, corr2, corr3, corr4);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr1;
		if (invMass2 != 0.0f)
			x2 += corr2;
		if (invMass3 != 0.0f)
			x3 += corr3;
		if (invMass4 != 0.0f)
			x4 += corr4;
	}
	return res;
}

//////////////////////////////////////////////////////////////////////////
// FEMTriangleConstraint
//////////////////////////////////////////////////////////////////////////
bool FEMTriangleConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
	const unsigned int particle3)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	m_bodies[2] = particle3;
	
	ParticleData &pd = model.getParticles();

	Eigen::Vector3f &x1 = pd.getPosition0(particle1);
	Eigen::Vector3f &x2 = pd.getPosition0(particle2);
	Eigen::Vector3f &x3 = pd.getPosition0(particle3);

	return PositionBasedDynamics::init_FEMTriangleConstraint(x1, x2, x3, m_area, m_invRestMat);
}

bool FEMTriangleConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];
	const unsigned i3 = m_bodies[2];

	Eigen::Vector3f &x1 = pd.getPosition(i1);
	Eigen::Vector3f &x2 = pd.getPosition(i2);
	Eigen::Vector3f &x3 = pd.getPosition(i3);

	const float invMass1 = pd.getInvMass(i1);
	const float invMass2 = pd.getInvMass(i2);
	const float invMass3 = pd.getInvMass(i3);
	
	Eigen::Vector3f corr1, corr2, corr3;
	const bool res = PositionBasedDynamics::solve_FEMTriangleConstraint(
		x1, invMass1,
		x2, invMass2,
		x3, invMass3,
		m_area,
		m_invRestMat,
		model.getClothXXStiffness(),
		model.getClothYYStiffness(),
		model.getClothXYStiffness(),
		model.getClothXYPoissonRatio(),
		model.getClothYXPoissonRatio(),
		corr1, corr2, corr3);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr1;
		if (invMass2 != 0.0f)
			x2 += corr2;
		if (invMass3 != 0.0f)
			x3 += corr3;
	}
	return res;
}


//////////////////////////////////////////////////////////////////////////
// StrainTriangleConstraint
//////////////////////////////////////////////////////////////////////////
bool StrainTriangleConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
	const unsigned int particle3)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	m_bodies[2] = particle3;

	ParticleData &pd = model.getParticles();

	Eigen::Vector3f &x1 = pd.getPosition0(particle1);
	Eigen::Vector3f &x2 = pd.getPosition0(particle2);
	Eigen::Vector3f &x3 = pd.getPosition0(particle3);

	// Bring triangles to xy plane
	const Eigen::Vector3f y1(x1[0], x1[2], 0.0);
	const Eigen::Vector3f y2(x2[0], x2[2], 0.0);
	const Eigen::Vector3f y3(x3[0], x3[2], 0.0);

	return PositionBasedDynamics::init_StrainTriangleConstraint(y1, y2, y3, m_invRestMat);
}

bool StrainTriangleConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];
	const unsigned i3 = m_bodies[2];

	Eigen::Vector3f &x1 = pd.getPosition(i1);
	Eigen::Vector3f &x2 = pd.getPosition(i2);
	Eigen::Vector3f &x3 = pd.getPosition(i3);

	const float invMass1 = pd.getInvMass(i1);
	const float invMass2 = pd.getInvMass(i2);
	const float invMass3 = pd.getInvMass(i3);

	Eigen::Vector3f corr1, corr2, corr3;
	const bool res = PositionBasedDynamics::solve_StrainTriangleConstraint(
		x1, invMass1,
		x2, invMass2,
		x3, invMass3,
		m_invRestMat,
		model.getClothXXStiffness(),
		model.getClothYYStiffness(),
		model.getClothXYStiffness(),
		model.getClothNormalizeStretch(),
		model.getClothNormalizeShear(),
		corr1, corr2, corr3);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr1;
		if (invMass2 != 0.0f)
			x2 += corr2;
		if (invMass3 != 0.0f)
			x3 += corr3;
	}
	return res;
}


//////////////////////////////////////////////////////////////////////////
// VolumeConstraint
//////////////////////////////////////////////////////////////////////////

bool VolumeConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
	const unsigned int particle3, const unsigned int particle4)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	m_bodies[2] = particle3;
	m_bodies[3] = particle4;
	ParticleData &pd = model.getParticles();

	const Eigen::Vector3f &p0 = pd.getPosition0(particle1);
	const Eigen::Vector3f &p1 = pd.getPosition0(particle2);
	const Eigen::Vector3f &p2 = pd.getPosition0(particle3);
	const Eigen::Vector3f &p3 = pd.getPosition0(particle4);

	m_restVolume = fabs((1.0f / 6.0f) * (p3 - p0).dot((p2 - p0).cross(p1 - p0)));

	return true;
}

bool VolumeConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];
	const unsigned i3 = m_bodies[2];
	const unsigned i4 = m_bodies[3];

	Eigen::Vector3f &x1 = pd.getPosition(i1);
	Eigen::Vector3f &x2 = pd.getPosition(i2);
	Eigen::Vector3f &x3 = pd.getPosition(i3);
	Eigen::Vector3f &x4 = pd.getPosition(i4);

	const float invMass1 = pd.getInvMass(i1);
	const float invMass2 = pd.getInvMass(i2);
	const float invMass3 = pd.getInvMass(i3);
	const float invMass4 = pd.getInvMass(i4);

	Eigen::Vector3f corr1, corr2, corr3, corr4;
	const bool res = PositionBasedDynamics::solve_VolumeConstraint(x1, invMass1,
		x2, invMass2,
		x3, invMass3,
		x4, invMass4,
		m_restVolume,
		model.getSolidStiffness(),
		model.getSolidStiffness(),
		corr1, corr2, corr3, corr4);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr1;
		if (invMass2 != 0.0f)
			x2 += corr2;
		if (invMass3 != 0.0f)
			x3 += corr3;
		if (invMass4 != 0.0f)
			x4 += corr4;
	}
	return res;
}


//////////////////////////////////////////////////////////////////////////
// FEMTetConstraint
//////////////////////////////////////////////////////////////////////////
bool FEMTetConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
									const unsigned int particle3, const unsigned int particle4)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	m_bodies[2] = particle3;
	m_bodies[3] = particle4;

	ParticleData &pd = model.getParticles();

	Eigen::Vector3f &x1 = pd.getPosition0(particle1);
	Eigen::Vector3f &x2 = pd.getPosition0(particle2);
	Eigen::Vector3f &x3 = pd.getPosition0(particle3);
	Eigen::Vector3f &x4 = pd.getPosition0(particle4);

	return PositionBasedDynamics::init_FEMTetraConstraint(x1, x2, x3, x4, m_volume, m_invRestMat);
}

bool FEMTetConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];
	const unsigned i3 = m_bodies[2];
	const unsigned i4 = m_bodies[3];

	Eigen::Vector3f &x1 = pd.getPosition(i1);
	Eigen::Vector3f &x2 = pd.getPosition(i2);
	Eigen::Vector3f &x3 = pd.getPosition(i3);
	Eigen::Vector3f &x4 = pd.getPosition(i4);

	const float invMass1 = pd.getInvMass(i1);
	const float invMass2 = pd.getInvMass(i2);
	const float invMass3 = pd.getInvMass(i3);
	const float invMass4 = pd.getInvMass(i4);

	float currentVolume = -(1.0f / 6.0f) * (x4 - x1).dot((x3 - x1).cross(x2 - x1));
	bool handleInversion = false;
	if (currentVolume / m_volume < 0.2)		// Only 20% of initial volume left
		handleInversion = true;


	Eigen::Vector3f corr1, corr2, corr3, corr4;
	const bool res = PositionBasedDynamics::solve_FEMTetraConstraint(
		x1, invMass1,
		x2, invMass2,
		x3, invMass3,
		x4, invMass4,
		m_volume,
		m_invRestMat,
		model.getSolidStiffness(),
		model.getSolidPoissonRatio(), handleInversion,
		corr1, corr2, corr3, corr4);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr1;
		if (invMass2 != 0.0f)
			x2 += corr2;
		if (invMass3 != 0.0f)
			x3 += corr3;
		if (invMass4 != 0.0f)
			x4 += corr4;
	}
	return res;
}


//////////////////////////////////////////////////////////////////////////
// StrainTetConstraint
//////////////////////////////////////////////////////////////////////////
bool StrainTetConstraint::initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
	const unsigned int particle3, const unsigned int particle4)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	m_bodies[2] = particle3;
	m_bodies[3] = particle4;

	ParticleData &pd = model.getParticles();

	Eigen::Vector3f &x1 = pd.getPosition0(particle1);
	Eigen::Vector3f &x2 = pd.getPosition0(particle2);
	Eigen::Vector3f &x3 = pd.getPosition0(particle3);
	Eigen::Vector3f &x4 = pd.getPosition0(particle4);

	return PositionBasedDynamics::init_StrainTetraConstraint(x1, x2, x3, x4, m_invRestMat);
}

bool StrainTetConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];
	const unsigned i3 = m_bodies[2];
	const unsigned i4 = m_bodies[3];

	Eigen::Vector3f &x1 = pd.getPosition(i1);
	Eigen::Vector3f &x2 = pd.getPosition(i2);
	Eigen::Vector3f &x3 = pd.getPosition(i3);
	Eigen::Vector3f &x4 = pd.getPosition(i4);

	const float invMass1 = pd.getInvMass(i1);
	const float invMass2 = pd.getInvMass(i2);
	const float invMass3 = pd.getInvMass(i3);
	const float invMass4 = pd.getInvMass(i4);

	Eigen::Vector3f stiffness(model.getSolidStiffness(), model.getSolidStiffness(), model.getSolidStiffness());

	Eigen::Vector3f corr1, corr2, corr3, corr4;
	const bool res = PositionBasedDynamics::solve_StrainTetraConstraint(
		x1, invMass1,
		x2, invMass2,
		x3, invMass3,
		x4, invMass4,
		m_invRestMat,
		stiffness,
		stiffness,
		model.getSolidNormalizeStretch(),
		model.getSolidNormalizeShear(),
		corr1, corr2, corr3, corr4);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr1;
		if (invMass2 != 0.0f)
			x2 += corr2;
		if (invMass3 != 0.0f)
			x3 += corr3;
		if (invMass4 != 0.0f)
			x4 += corr4;
	}
	return res;
}

//////////////////////////////////////////////////////////////////////////
// ShapeMatchingConstraint
//////////////////////////////////////////////////////////////////////////
bool ShapeMatchingConstraint::initConstraint(SimulationModel &model, 
			const unsigned int particleIndices[], const unsigned int numClusters[])
{
	ParticleData &pd = model.getParticles();
	for (unsigned int i = 0; i < m_numberOfBodies; i++)
	{
		m_bodies[i] = particleIndices[i];
		m_x0[i] = pd.getPosition0(m_bodies[i]);
		m_w[i] = pd.getInvMass(m_bodies[i]);
		m_numClusters[i] = numClusters[i];
	}

	const bool res = PositionBasedDynamics::init_ShapeMatchingConstraint(m_x0, m_w, m_numberOfBodies, m_restCm, m_invRestMat);
	return res;
}

bool ShapeMatchingConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();
	for (unsigned int i = 0; i < m_numberOfBodies; i++)
	{
		m_x[i] = pd.getPosition(m_bodies[i]);
	}

	const bool res = PositionBasedDynamics::solve_ShapeMatchingConstraint(
		m_x0, m_x, m_w, m_numberOfBodies,
		m_restCm, m_invRestMat,
		model.getSolidStiffness(), false,
		m_corr);

	if (res)
	{
		for (unsigned int i = 0; i < m_numberOfBodies; i++)
		{
			// Important: Divide position correction by the number of clusters 
			// which contain the vertex. 
			if (m_w[i] != 0.0f)
				pd.getPosition(m_bodies[i]) += (1.0f / m_numClusters[i]) * m_corr[i];
		}
	}
	return res;
}

//////////////////////////////////////////////////////////////////////////
// ElasticRodEdgeConstraint
//////////////////////////////////////////////////////////////////////////
bool ElasticRodEdgeConstraint::initConstraint(SimulationModel &model, const unsigned int pA, const unsigned int pB, const unsigned int pG)
{
	m_bodies[0] = pA;
	m_bodies[1] = pB;
	m_bodies[2] = pG;

	ParticleData &pd = model.getParticles();
	ParticleData &pg = model.getGhostParticles();

	const Eigen::Vector3f &xA_0 = pd.getPosition0(pA);
	const Eigen::Vector3f &xB_0 = pd.getPosition0(pB);
	const Eigen::Vector3f &xG_0 = pg.getPosition0(pG);

	m_restLength = 1.0f;

	return true;
}

bool ElasticRodEdgeConstraint::solvePositionConstraint(SimulationModel &model)
{
	//ElasticRodSimulationModel simModel = dynamic_cast<ElasticRodSimulationModel&>(model);

	ParticleData &pd = model.getParticles();
	ParticleData &pg = model.getGhostParticles();

	const unsigned iA = m_bodies[0];
	const unsigned iB = m_bodies[1];
	const unsigned iG = m_bodies[2];

	Eigen::Vector3f &xA = pd.getPosition(iA);
	Eigen::Vector3f &xB = pd.getPosition(iB);
	Eigen::Vector3f &xG = pg.getPosition(iG);

	float wA = pd.getInvMass(iA);
	float wB = pd.getInvMass(iB);
	float wG = pg.getInvMass(iG);
	
	Eigen::Vector3f corr[3];
	//const bool res = PositionBasedDynamics::solve_DistanceConstraint(xA, wA, xB, wB, m_restLength, 1.0f, 1.0f, corr[0], corr[1]);
	const bool res = PositionBasedElasticRod::ProjectEdgeConstraints(xA, wA, xB, wB, xG, wG, 1.0f, m_restLength, m_restLength, corr[0], corr[1], corr[2]);

	if (res)
	{
		if (wA != 0.0f)
			xA += corr[0];
		
		if (wB != 0.0f)
			xB += corr[1];

		if (wG != 0.0f)
			xG += corr[2];
	}

	return res;
}
//////////////////////////////////////////////////////////////////////////
// ElasticRodBendAndTwistConstraint
//////////////////////////////////////////////////////////////////////////
bool ElasticRodBendAndTwistConstraint::initConstraint(SimulationModel &model, const unsigned int pA, const unsigned int pB,
	const unsigned int pC, const unsigned int pD, const unsigned int pE)
{
	m_bodies[0] = pA;
	m_bodies[1] = pB;
	m_bodies[2] = pC;
	m_bodies[3] = pD; //ghost point id
	m_bodies[4] = pE; //ghost point id

	ParticleData &pd = model.getParticles();
	ParticleData &pg = model.getGhostParticles();

	const Eigen::Vector3f &xA = pd.getPosition0(m_bodies[0]);
	const Eigen::Vector3f &xB = pd.getPosition0(m_bodies[1]);
	const Eigen::Vector3f &xC = pd.getPosition0(m_bodies[2]);
	const Eigen::Vector3f &xD = pg.getPosition0(m_bodies[3]);
	const Eigen::Vector3f &xE = pg.getPosition0(m_bodies[4]);

	PositionBasedElasticRod::ComputeMaterialFrame(xA, xB, xD, m_dA);
	PositionBasedElasticRod::ComputeMaterialFrame(xB, xC, xE, m_dB);

	PositionBasedElasticRod::ComputeDarbouxVector(m_dA, m_dB, 1.0f, m_restDarbouxVector);
	m_bendAndTwistKs.setOnes();

	return true;
}

bool ElasticRodBendAndTwistConstraint::solvePositionConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();
	ParticleData &pg = model.getGhostParticles();

	Eigen::Vector3f &xA = pd.getPosition(m_bodies[0]);
	Eigen::Vector3f &xB = pd.getPosition(m_bodies[1]);
	Eigen::Vector3f &xC = pd.getPosition(m_bodies[2]);
	Eigen::Vector3f &xD = pg.getPosition(m_bodies[3]);
	Eigen::Vector3f &xE = pg.getPosition(m_bodies[4]);

	const float wA = pd.getInvMass(m_bodies[0]);
	const float wB = pd.getInvMass(m_bodies[1]);
	const float wC = pd.getInvMass(m_bodies[2]);
	const float wD = pg.getInvMass(m_bodies[3]);
	const float wE = pg.getInvMass(m_bodies[4]);

	Eigen::Vector3f corr[5];

	bool res = PositionBasedElasticRod::ProjectBendingAndTwistingConstraint(
		xA, wA, xB, wB, xC, wC, xD, wD, xE, wE, 
		m_bendAndTwistKs, 1.0f, m_restDarbouxVector, 
		corr[0], corr[1], corr[2], corr[3], corr[4], true);

	if (res)
	{
		const float stiffness = model.getElasticRodBendAndTwistStiffness();
		if (wA != 0.0f)
			xA += stiffness*corr[0];
		
		if (wB != 0.0f)
			xB += stiffness*corr[1];
		
		if (wC != 0.0f)
			xC += stiffness*corr[2];
		
		if (wD != 0.0f)
			xD += stiffness*corr[3];

		if (wE != 0.0f)
			xE += stiffness*corr[4];
	}
	return res;
}
