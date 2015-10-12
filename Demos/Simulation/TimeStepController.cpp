#include "TimeStepController.h"
#include "Demos/Simulation/TimeManager.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"
#include "PositionBasedDynamics/TimeIntegration.h"
#include <iostream>
#include "PositionBasedDynamics/PositionBasedDynamics.h"

using namespace PBD;
using namespace std;

TimeStepController::TimeStepController()
{
	m_velocityUpdateMethod = 0;
	m_simulationMethod = 2;
	m_bendingMethod = 2;
}

TimeStepController::~TimeStepController(void)
{
}

void TimeStepController::step(SimulationModel &model)
{
 	TimeManager *tm = TimeManager::getCurrent ();
 	const float h = tm->getTimeStepSize();
 
	//////////////////////////////////////////////////////////////////////////
	// rigid body model
	//////////////////////////////////////////////////////////////////////////
 	clearAccelerations(model);
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
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

	//////////////////////////////////////////////////////////////////////////
	// particle model
	//////////////////////////////////////////////////////////////////////////
	ParticleData &pd = model.getParticles();
	for (unsigned int i = 0; i < pd.size(); i++)
	{
		pd.getLastPosition(i) = pd.getOldPosition(i);
		pd.getOldPosition(i) = pd.getPosition(i);
		TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i), pd.getVelocity(i), pd.getAcceleration(i));
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

void TimeStepController::clearAccelerations(SimulationModel &model)
{
	//////////////////////////////////////////////////////////////////////////
	// rigid body model
	//////////////////////////////////////////////////////////////////////////

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
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

	ParticleData &pd = model.getParticles();
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

void TimeStepController::reset()
{

}

void TimeStepController::constraintProjection(SimulationModel &model)
{
	const unsigned int maxIter = 5;
	unsigned int iter = 0;

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();

 	while (iter < maxIter)
 	{
		for (unsigned int i = 0; i < constraints.size(); i++)
		{
			constraints[i]->updateConstraint(model);
			constraints[i]->solveConstraint(model);
		}
 		iter++;
 	}
}


