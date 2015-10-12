#include "Constraints.h"
#include "SimulationModel.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"

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
	return PositionBasedRigidBodyDynamics::initRigidBodyBallJointInfo(
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
	return PositionBasedRigidBodyDynamics::updateRigidBodyBallJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool BallJoint::solveConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	updateConstraint(model);

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solveRigidBodyBallJoint(
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
	return PositionBasedRigidBodyDynamics::initRigidBodyBallOnLineJointInfo(
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
	return PositionBasedRigidBodyDynamics::updateRigidBodyBallOnLineJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool BallOnLineJoint::solveConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	updateConstraint(model);

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solveRigidBodyBallOnLineJoint(
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
	return PositionBasedRigidBodyDynamics::initRigidBodyHingeJointInfo(
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
	return PositionBasedRigidBodyDynamics::updateRigidBodyHingeJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool HingeJoint::solveConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	updateConstraint(model);

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solveRigidBodyHingeJoint(
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
	return PositionBasedRigidBodyDynamics::initRigidBodyUniversalJointInfo(
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
	return PositionBasedRigidBodyDynamics::updateRigidBodyUniversalJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		m_jointInfo);
}

bool UniversalJoint::solveConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	updateConstraint(model);

	RigidBody &rb1 = *rb[m_bodies[0]];
	RigidBody &rb2 = *rb[m_bodies[1]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solveRigidBodyUniversalJoint(
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
// RigidBodyParticleBallJoint
//////////////////////////////////////////////////////////////////////////
bool RigidBodyParticleBallJoint::initConstraint(SimulationModel &model, const unsigned int rbIndex, const unsigned int particleIndex)
{
	m_bodies[0] = rbIndex;
	m_bodies[1] = particleIndex;
	SimulationModel::RigidBodyVector &rbs = model.getRigidBodies();
	ParticleData &pd = model.getParticles();
	RigidBody &rb = *rbs[m_bodies[0]];
	return PositionBasedRigidBodyDynamics::initRigidBodyParticleBallJointInfo(
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
	return PositionBasedRigidBodyDynamics::updateRigidBodyParticleBallJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		pd.getPosition(m_bodies[1]),
		m_jointInfo);
}

bool RigidBodyParticleBallJoint::solveConstraint(SimulationModel &model)
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	ParticleData &pd = model.getParticles();

	updateConstraint(model);

	RigidBody &rb1 = *rb[m_bodies[0]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1;
	const bool res = PositionBasedRigidBodyDynamics::solveRigidBodyParticleBallJoint(
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

bool DistanceConstraint::solveConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];

	Eigen::Vector3f &x1 = pd.getPosition(i1);
	Eigen::Vector3f &x2 = pd.getPosition(i2);
	const float invMass1 = pd.getInvMass(i1);
	const float invMass2 = pd.getInvMass(i2);

	Eigen::Vector3f corr1, corr2;
	const bool res = PositionBasedDynamics::solveDistanceConstraint(
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

bool DihedralConstraint::solveConstraint(SimulationModel &model)
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
	const bool res = PositionBasedDynamics::solveDihedralConstraint(
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

	return PositionBasedDynamics::initQuadraticBendingMat(x1, x2, x3, x4, m_Q);
}

bool IsometricBendingConstraint::solveConstraint(SimulationModel &model)
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
	const bool res = PositionBasedDynamics::solveIsometricBendingConstraint(
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

	return PositionBasedDynamics::initFEMTriangleInvRestMat(x1, x2, x3, m_area, m_invRestMat);
}

bool FEMTriangleConstraint::solveConstraint(SimulationModel &model)
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
	const bool res = PositionBasedDynamics::solveFEMTriangleConstraint(
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

	return PositionBasedDynamics::initStrainTriangleInvRestMat(y1, y2, y3, m_invRestMat);
}

bool StrainTriangleConstraint::solveConstraint(SimulationModel &model)
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
	const bool res = PositionBasedDynamics::solveStrainTriangleConstraint(
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

bool VolumeConstraint::solveConstraint(SimulationModel &model)
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
	const bool res = PositionBasedDynamics::solveVolumeConstraint(x1, invMass1,
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

	return PositionBasedDynamics::initFEMTetraInvRestMat(x1, x2, x3, x4, m_volume, m_invRestMat);
}

bool FEMTetConstraint::solveConstraint(SimulationModel &model)
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
	const bool res = PositionBasedDynamics::solveFEMTetraConstraint(
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

	return PositionBasedDynamics::initStrainTetraInvRestMat(x1, x2, x3, x4, m_invRestMat);
}

bool StrainTetConstraint::solveConstraint(SimulationModel &model)
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
	const bool res = PositionBasedDynamics::solveStrainTetraConstraint(
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

	const bool res = PositionBasedDynamics::initShapeMatchingRestInfo(m_x0, m_w, m_numberOfBodies, m_restCm, m_invRestMat);
	return res;
}

bool ShapeMatchingConstraint::solveConstraint(SimulationModel &model)
{
	ParticleData &pd = model.getParticles();
	for (unsigned int i = 0; i < m_numberOfBodies; i++)
	{
		m_x[i] = pd.getPosition(m_bodies[i]);
	}

	const bool res = PositionBasedDynamics::solveShapeMatchingConstraint(
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
