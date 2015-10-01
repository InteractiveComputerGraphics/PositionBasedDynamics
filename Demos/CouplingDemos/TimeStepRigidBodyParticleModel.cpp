#include "TimeStepRigidBodyParticleModel.h"
#include "Demos/Utils/TimeManager.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"
#include "PositionBasedDynamics/TimeIntegration.h"
#include <iostream>
#include "PositionBasedDynamics/PositionBasedDynamics.h"

using namespace PBD;
using namespace std;

TimeStepRigidBodyParticleModel::TimeStepRigidBodyParticleModel()
{
	m_velocityUpdateMethod = 0;
	m_simulationMethod = 2;
	m_bendingMethod = 2;
}

TimeStepRigidBodyParticleModel::~TimeStepRigidBodyParticleModel(void)
{
}

void TimeStepRigidBodyParticleModel::step(RigidBodyParticleModel &model)
{
 	TimeManager *tm = TimeManager::getCurrent ();
 	const float h = tm->getTimeStepSize();
 
	//////////////////////////////////////////////////////////////////////////
	// rigid body model
	//////////////////////////////////////////////////////////////////////////
 	clearAccelerations(model);
	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
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

	//////////////////////////////////////////////////////////////////////////
	// particle model
	//////////////////////////////////////////////////////////////////////////
	ParticleData &pd = model.getParticleMesh().getVertexData();

	clearAccelerations(model);
	for (unsigned int i = 0; i < pd.size(); i++)
	{
		pd.getLastPosition(i) = pd.getOldPosition(i);
		pd.getOldPosition(i) = pd.getPosition(i);
		TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i), pd.getVelocity(i), pd.getAcceleration(i));
	}

	constraintProjection(model);

	// Update velocities	
	for (unsigned int i = 0; i < pd.size(); i++)
	{
		if (m_velocityUpdateMethod == 0)
			TimeIntegration::velocityUpdateFirstOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getVelocity(i));
		else
			TimeIntegration::velocityUpdateSecondOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getLastPosition(i), pd.getVelocity(i));
	}

	// compute new time	
	tm->setTime (tm->getTime () + h);
}

void TimeStepRigidBodyParticleModel::clearAccelerations(RigidBodyParticleModel &model)
{
	//////////////////////////////////////////////////////////////////////////
	// rigid body model
	//////////////////////////////////////////////////////////////////////////

	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
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

	//////////////////////////////////////////////////////////////////////////
	// particle model
	//////////////////////////////////////////////////////////////////////////

	ParticleData &pd = model.getParticleMesh().getVertexData();
	const unsigned int count = pd.size();
	for (unsigned int i = 0; i < count; i++)
	{
		// Clear accelerations of dynamic particles
		if (pd.getMass(i) != 0.0)
		{
			Eigen::Vector3f &a = pd.getAcceleration(i);
			a = grav;
		}
	}
}

void TimeStepRigidBodyParticleModel::reset()
{

}

void TimeStepRigidBodyParticleModel::constraintProjection(RigidBodyParticleModel &model)
{
	const unsigned int maxIter = 5;
	unsigned int iter = 0;

	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyParticleModel::JointVector &joints = model.getJoints();

 	while (iter < maxIter)
 	{
		constraintProjectionParticleModel(model);

		for (unsigned int i = 0; i < joints.size(); i++)
		{
			if (joints[i]->getTypeId() == RigidBodyParticleModel::BallJoint::TYPE_ID)
				constraintProjectionBallJoint(model, i);
			else if (joints[i]->getTypeId() == RigidBodyParticleModel::BallOnLineJoint::TYPE_ID)
				constraintProjectionBallOnLineJoint(model, i);
			else if (joints[i]->getTypeId() == RigidBodyParticleModel::HingeJoint::TYPE_ID)
				constraintProjectionHingeJoint(model, i);
			else if (joints[i]->getTypeId() == RigidBodyParticleModel::UniversalJoint::TYPE_ID)
				constraintProjectionUniversalJoint(model, i);
			else if (joints[i]->getTypeId() == RigidBodyParticleModel::RigidBodyParticleBallJoint::TYPE_ID)
				constraintProjectionRigidBodyParticleBallJoint(model, i);
		}
 		iter++;
 	}
}

void TimeStepRigidBodyParticleModel::constraintProjectionBallJoint(RigidBodyParticleModel &model, const unsigned int index)
{
	RigidBodyParticleModel::JointVector &joints = model.getJoints();
	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyParticleModel::BallJoint &bj = *(RigidBodyParticleModel::BallJoint*) joints[index];	

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

void TimeStepRigidBodyParticleModel::constraintProjectionBallOnLineJoint(RigidBodyParticleModel &model, const unsigned int index)
{
	RigidBodyParticleModel::JointVector &joints = model.getJoints();
	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyParticleModel::BallOnLineJoint &bj = *(RigidBodyParticleModel::BallOnLineJoint*) joints[index];

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

void TimeStepRigidBodyParticleModel::constraintProjectionHingeJoint(RigidBodyParticleModel &model, const unsigned int index)
{
	RigidBodyParticleModel::JointVector &joints = model.getJoints();
	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyParticleModel::HingeJoint &hj = *(RigidBodyParticleModel::HingeJoint*) joints[index];

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

void TimeStepRigidBodyParticleModel::constraintProjectionUniversalJoint(RigidBodyParticleModel &model, const unsigned int index)
{
	RigidBodyParticleModel::JointVector &joints = model.getJoints();
	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyParticleModel::UniversalJoint &uj = *(RigidBodyParticleModel::UniversalJoint*) joints[index];

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

void TimeStepRigidBodyParticleModel::constraintProjectionRigidBodyParticleBallJoint(RigidBodyParticleModel &model, const unsigned int index)
{
	ParticleData &pd = model.getParticleMesh().getVertexData();
	RigidBodyParticleModel::JointVector &joints = model.getJoints();
	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyParticleModel::RigidBodyParticleBallJoint &bj = *(RigidBodyParticleModel::RigidBodyParticleBallJoint*) joints[index];

	model.updateRigidBodyParticleBallJoint(index);

	RigidBody &rb1 = *rb[bj.m_index[0]];

	Eigen::Vector3f corr_x1, corr_x2;
	Eigen::Quaternionf corr_q1, corr_q2;
	const bool res = PositionBasedRigidBodyDynamics::solveRigidBodyParticleBallJoint(
		rb1.getMass(),
		rb1.getPosition(),
		rb1.getInertiaTensorInverseW(),
		rb1.getRotation(),
		pd.getMass(bj.m_index[1]),
		pd.getPosition(bj.m_index[1]),
		bj.m_jointInfo,
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
		if (pd.getMass(bj.m_index[1]) != 0.0f)
		{
			pd.getPosition(bj.m_index[1]) += corr_x2;
		}
	}
}


void TimeStepRigidBodyParticleModel::constraintProjectionParticleModel(RigidBodyParticleModel &model)
{
	const unsigned int maxIter = 5;
	unsigned int iter = 0;


	ParticleData &pd = model.getParticleMesh().getVertexData();

	while (iter < maxIter)
	{
		if (m_simulationMethod == 1)		// Distance constraints
		{
			const unsigned int nEdges = model.getParticleMesh().numEdges();
			const IndexedFaceMesh<ParticleData>::Edge *edges = model.getParticleMesh().getEdges().data();
			for (unsigned int i = 0; i < nEdges; i++)
			{
				const unsigned int v1 = edges[i].m_vert[0];
				const unsigned int v2 = edges[i].m_vert[1];

				const Eigen::Vector3f &x1_0 = pd.getPosition0(v1);
				const Eigen::Vector3f &x2_0 = pd.getPosition0(v2);

				Eigen::Vector3f &x1 = pd.getPosition(v1);
				Eigen::Vector3f &x2 = pd.getPosition(v2);

				const float invMass1 = pd.getInvMass(v1);
				const float invMass2 = pd.getInvMass(v2);

				float restLength = (x2_0 - x1_0).norm();

				Eigen::Vector3f corr1, corr2;
				const bool res = PositionBasedDynamics::solveDistanceConstraint(x1, invMass1, x2, invMass2, restLength, model.getStiffness(), model.getStiffness(), corr1, corr2);

				if (res)
				{
					if (invMass1 != 0.0f)
						x1 += corr1;
					if (invMass2 != 0.0f)
						x2 += corr2;
				}
			}
		}
		else if (m_simulationMethod == 2)		// strain energy constraint
		{
			RigidBodyParticleModel::TriangleConstraintVector &triangleConstraints = model.getTriangleConstraints();
			const unsigned int *tris = model.getParticleMesh().getFaces().data();
			const unsigned int nFaces = model.getParticleMesh().numFaces();
			for (unsigned int i = 0; i < nFaces; i++)
			{
				Eigen::Vector3f &x1 = pd.getPosition(tris[3 * i]);
				Eigen::Vector3f &x2 = pd.getPosition(tris[3 * i + 1]);
				Eigen::Vector3f &x3 = pd.getPosition(tris[3 * i + 2]);

				const float invMass1 = pd.getInvMass(tris[3 * i]);
				const float invMass2 = pd.getInvMass(tris[3 * i + 1]);
				const float invMass3 = pd.getInvMass(tris[3 * i + 2]);

				Eigen::Vector3f corr1, corr2, corr3;
				const bool res = PositionBasedDynamics::solveFEMTriangleConstraint(
					x1, invMass1,
					x2, invMass2,
					x3, invMass3,
					triangleConstraints[i].triangleArea,
					triangleConstraints[i].invRestMat_FEM,
					model.getXXStiffness(),
					model.getYYStiffness(),
					model.getXYStiffness(),
					model.getXYPoissonRatio(),
					model.getYXPoissonRatio(),
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
			}
		}
		else if (m_simulationMethod == 3)		// strain based dynamics
		{
			RigidBodyParticleModel::TriangleConstraintVector &triangleConstraints = model.getTriangleConstraints();
			const unsigned int *tris = model.getParticleMesh().getFaces().data();
			const unsigned int nFaces = model.getParticleMesh().numFaces();
			for (unsigned int i = 0; i < nFaces; i++)
			{
				Eigen::Vector3f &x1 = pd.getPosition(tris[3 * i]);
				Eigen::Vector3f &x2 = pd.getPosition(tris[3 * i + 1]);
				Eigen::Vector3f &x3 = pd.getPosition(tris[3 * i + 2]);

				const float invMass1 = pd.getInvMass(tris[3 * i]);
				const float invMass2 = pd.getInvMass(tris[3 * i + 1]);
				const float invMass3 = pd.getInvMass(tris[3 * i + 2]);

				Eigen::Vector3f corr1, corr2, corr3;
				const bool res = PositionBasedDynamics::solveStrainTriangleConstraint(
					x1, invMass1,
					x2, invMass2,
					x3, invMass3,
					triangleConstraints[i].invRestMat_SBD,
					model.getXXStiffness(),
					model.getYYStiffness(),
					model.getXYStiffness(),
					model.getNormalizeStretch(),
					model.getNormalizeShear(),
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
			}
		}

		if (m_bendingMethod != 0)
		{
			RigidBodyParticleModel::BendingConstraintVector &bendingConstraints = model.getBendingConstraints();

			for (unsigned int i = 0; i < (unsigned int)bendingConstraints.size(); i++)
			{
				Eigen::Vector3f &x1 = pd.getPosition(bendingConstraints[i].vertex1);
				Eigen::Vector3f &x2 = pd.getPosition(bendingConstraints[i].vertex2);
				Eigen::Vector3f &x3 = pd.getPosition(bendingConstraints[i].vertex3);
				Eigen::Vector3f &x4 = pd.getPosition(bendingConstraints[i].vertex4);

				const float invMass1 = pd.getInvMass(bendingConstraints[i].vertex1);
				const float invMass2 = pd.getInvMass(bendingConstraints[i].vertex2);
				const float invMass3 = pd.getInvMass(bendingConstraints[i].vertex3);
				const float invMass4 = pd.getInvMass(bendingConstraints[i].vertex4);

				Eigen::Vector3f corr1, corr2, corr3, corr4;
				bool res;
				if (m_bendingMethod == 1)
					res = PositionBasedDynamics::solveDihedralConstraint(x1, invMass1, x2, invMass2, x3, invMass3, x4, invMass4, bendingConstraints[i].restAngle, model.getBendingStiffness(), corr1, corr2, corr3, corr4);
				else if (m_bendingMethod == 2)
					res = PositionBasedDynamics::solveIsometricBendingConstraint(x1, invMass1, x2, invMass2, x3, invMass3, x4, invMass4, bendingConstraints[i].Q, model.getBendingStiffness(), corr1, corr2, corr3, corr4);

				if (res)
				{
					x1 += corr1;
					x2 += corr2;
					x3 += corr3;
					x4 += corr4;
				}
			}
		}

		iter++;
	}
}

