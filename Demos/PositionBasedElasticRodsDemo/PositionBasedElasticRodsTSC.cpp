#include "PositionBasedElasticRodsTSC.h"
#include "PositionBasedElasticRodsModel.h"
#include "Simulation/TimeManager.h"
#include "PositionBasedDynamics/TimeIntegration.h"
#include "Simulation/Simulation.h"


using namespace PBD;
using namespace std;

PositionBasedElasticRodsTSC::PositionBasedElasticRodsTSC() : 
	TimeStepController()
{
	m_damping = 0.0;
}

PositionBasedElasticRodsTSC::~PositionBasedElasticRodsTSC()
{
}

void PositionBasedElasticRodsTSC::step(SimulationModel &model)
{
 	TimeManager *tm = TimeManager::getCurrent ();
 	const Real h = tm->getTimeStepSize();
	PositionBasedElasticRodsModel &ermodel = (PositionBasedElasticRodsModel&)model;
 
	//////////////////////////////////////////////////////////////////////////
	// rigid body model
	//////////////////////////////////////////////////////////////////////////
 	clearAccelerations(model);
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	ParticleData &pd = model.getParticles();
	ParticleData &pg = ermodel.getGhostParticles();

	const int numBodies = (int)rb.size();
	#pragma omp parallel if(numBodies > MIN_PARALLEL_SIZE) default(shared)
	{
		#pragma omp for schedule(static) nowait
		for (int i = 0; i < numBodies; i++)
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
		#pragma omp for schedule(static) 
		for (int i = 0; i < (int) pd.size(); i++)
		{
			pd.getLastPosition(i) = pd.getOldPosition(i);
			pd.getOldPosition(i) = pd.getPosition(i);
			pd.getVelocity(i) *= (1.0f - m_damping);

			TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i), pd.getVelocity(i), pd.getAcceleration(i));
		}
			
		for (int i = 0; i < (int)pg.size(); i++)
		{
			pg.getLastPosition(i) = pg.getOldPosition(i);
			pg.getOldPosition(i) = pg.getPosition(i);

			pg.getVelocity(i) *= (1.0f - m_damping);

			TimeIntegration::semiImplicitEuler(h, pg.getMass(i), pg.getPosition(i), pg.getVelocity(i), pg.getAcceleration(i));
		}
	}

	positionConstraintProjection(model);
 
	#pragma omp parallel if(numBodies > MIN_PARALLEL_SIZE) default(shared)
	{
 		// Update velocities	
		#pragma omp for schedule(static) nowait
		for (int i = 0; i < numBodies; i++)
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
			// update geometry
			rb[i]->getGeometry().updateMeshTransformation(rb[i]->getPosition(), rb[i]->getRotationMatrix());
 		}

		// Update velocities	
		#pragma omp for schedule(static) 
		for (int i = 0; i < (int) pd.size(); i++)
		{
			if (m_velocityUpdateMethod == 0)
				TimeIntegration::velocityUpdateFirstOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getVelocity(i));
			else
				TimeIntegration::velocityUpdateSecondOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getLastPosition(i), pd.getVelocity(i));
		}
	
		#pragma omp for schedule(static) 
		for (int i = 0; i < (int)pg.size(); i++)
		{
			if (m_velocityUpdateMethod == 0)
				TimeIntegration::velocityUpdateFirstOrder(h, pg.getMass(i), pg.getPosition(i), pg.getOldPosition(i), pg.getVelocity(i));
			else
				TimeIntegration::velocityUpdateSecondOrder(h, pg.getMass(i), pg.getPosition(i), pg.getOldPosition(i), pg.getLastPosition(i), pg.getVelocity(i));
		}

}

	if (m_collisionDetection)
		m_collisionDetection->collisionDetection(model);

	velocityConstraintProjection(model);
	
	// compute new time	
	tm->setTime (tm->getTime () + h);
}

void PositionBasedElasticRodsTSC::clearAccelerations(SimulationModel &model)
{
	//////////////////////////////////////////////////////////////////////////
	// rigid body model
	//////////////////////////////////////////////////////////////////////////

	PositionBasedElasticRodsModel &ermodel = (PositionBasedElasticRodsModel&)model;
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	Simulation *sim = Simulation::getCurrent();
	const Vector3r grav(sim->getVecValue<Real>(Simulation::GRAVITATION));
 	for (size_t i=0; i < rb.size(); i++)
 	{
 		// Clear accelerations of dynamic particles
 		if (rb[i]->getMass() != 0.0)
 		{
			Vector3r &a = rb[i]->getAcceleration();
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
			Vector3r &a = pd.getAcceleration(i);
			a = grav;
		}
	}

	ParticleData &pg = ermodel.getGhostParticles();
	for (unsigned int i = 0; i <  pg.size(); i++)
	{
		// Clear accelerations of dynamic particles
		if (pg.getMass(i) != 0.0)
		{
			Vector3r &a = pg.getAcceleration(i);
			a.setZero();
		}
	}
}


