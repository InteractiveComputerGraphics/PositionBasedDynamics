#include "PositionBasedElasticRodsConstraints.h"
#include "Simulation/SimulationModel.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"
#include "PositionBasedDynamics/PositionBasedElasticRods.h"
#include "Simulation/IDFactory.h"

using namespace PBD;

int PerpendiculaBisectorConstraint::TYPE_ID = IDFactory::getId();
int GhostPointEdgeDistanceConstraint::TYPE_ID = IDFactory::getId();
int DarbouxVectorConstraint::TYPE_ID = IDFactory::getId();


//////////////////////////////////////////////////////////////////////////
// PerpendiculaBisectorConstraint
//////////////////////////////////////////////////////////////////////////
bool PerpendiculaBisectorConstraint::initConstraint(SimulationModel &model,
	const unsigned int particle1, const unsigned int particle2, const unsigned int particle3)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	m_bodies[2] = particle3;

	return true;
}

bool PerpendiculaBisectorConstraint::solvePositionConstraint(SimulationModel &model, const unsigned int iter)
{
	PositionBasedElasticRodsModel &simModel = static_cast<PositionBasedElasticRodsModel&>(model);

	ParticleData &pd = model.getParticles();
	ParticleData &pg = simModel.getGhostParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];
	const unsigned i3 = m_bodies[2];

	Vector3r &x1 = pd.getPosition(i1);
	Vector3r &x2 = pd.getPosition(i2);
	Vector3r &x3 = pg.getPosition(i3);

	const Real invMass1 = pd.getInvMass(i1);
	const Real invMass2 = pd.getInvMass(i2);
	const Real invMass3 = pg.getInvMass(i3);

	Vector3r corr[3];
	const bool res = PositionBasedElasticRods::solve_PerpendiculaBisectorConstraint(
		x1, invMass1, 
		x2, invMass2, 
		x3, invMass3, 
		1.0, 
		corr[0], corr[1], corr[2]);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr[0];

		if (invMass2 != 0.0f)
			x2 += corr[1];

		if (invMass2 != 0.0f)
			x3 += corr[2];
	}

	return res;
}

//////////////////////////////////////////////////////////////////////////
// GhostPointEdgeDistanceConstraint
//////////////////////////////////////////////////////////////////////////
bool GhostPointEdgeDistanceConstraint::initConstraint(PositionBasedElasticRodsModel &model, const unsigned int particle1, const unsigned int particle2, const unsigned int particle3)
{
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	m_bodies[2] = particle3;

	ParticleData &pd = model.getParticles();
	ParticleData &pg = model.getGhostParticles();

	Vector3r &x1 = pd.getPosition( particle1);
	Vector3r &x2 = pd.getPosition( particle2);
	Vector3r &x3 = pg.getPosition( particle3);

	Vector3r xm = 0.5 * (x1 + x2);

	m_restLength = (x3 - xm).norm();

	return true;
}

bool GhostPointEdgeDistanceConstraint::solvePositionConstraint(SimulationModel &model, const unsigned int iter)
{
	PositionBasedElasticRodsModel &simModel = static_cast<PositionBasedElasticRodsModel&>(model);

	ParticleData &pd = model.getParticles();
	ParticleData &pg = simModel.getGhostParticles();

	const unsigned i1 = m_bodies[0];
	const unsigned i2 = m_bodies[1];
	const unsigned i3 = m_bodies[2];

	Vector3r &x1 = pd.getPosition(i1);
	Vector3r &x2 = pd.getPosition(i2);
	Vector3r &x3 = pg.getPosition(i3);

	const Real invMass1 = pd.getInvMass(i1);
	const Real invMass2 = pd.getInvMass(i2);
	const Real invMass3 = pg.getInvMass(i3);
	
	Vector3r corr[3];
	const bool res = PositionBasedElasticRods::solve_GhostPointEdgeDistanceConstraint(x1, invMass1, x2, invMass2, x3, invMass3, 1.0, m_restLength, corr[0], corr[1], corr[2]);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr[0];
		
		if (invMass2 != 0.0f)
			x2 += corr[1];

		if (invMass3 != 0.0f)
			x3 += corr[2];
	}

	return res;
}

//////////////////////////////////////////////////////////////////////////
// DarbouxVectorConstraint
//////////////////////////////////////////////////////////////////////////
bool DarbouxVectorConstraint::initConstraint(PositionBasedElasticRodsModel &model, const unsigned int particle1, const unsigned int particle2,
	const unsigned int particle3, const unsigned int particle4, const unsigned int particle5)
{	
	m_bodies[0] = particle1;
	m_bodies[1] = particle2;
	m_bodies[2] = particle3;
	m_bodies[3] = particle4; //ghost point id
	m_bodies[4] = particle5; //ghost point id

	ParticleData &pd = model.getParticles();
	ParticleData &pg = model.getGhostParticles();

	const Vector3r &x1 = pd.getPosition0(m_bodies[0]);
	const Vector3r &x2 = pd.getPosition0(m_bodies[1]);
	const Vector3r &x3 = pd.getPosition0(m_bodies[2]);
	const Vector3r &x4 = pg.getPosition0(m_bodies[3]);
	const Vector3r &x5 = pg.getPosition0(m_bodies[4]);

	PositionBasedElasticRods::computeMaterialFrame(x1, x2, x4, m_dA);
	PositionBasedElasticRods::computeMaterialFrame(x2, x3, x5, m_dB);
	Vector3r restDarbouxVector;
	PositionBasedElasticRods::computeDarbouxVector(m_dA, m_dB, 1.0, restDarbouxVector);
	model.setRestDarbouxVector(restDarbouxVector);

	return true;
}

bool DarbouxVectorConstraint::solvePositionConstraint(SimulationModel &model, const unsigned int iter)
{
	PositionBasedElasticRodsModel &simModel = static_cast<PositionBasedElasticRodsModel&>(model);

	ParticleData &pd = model.getParticles();
	ParticleData &pg = simModel.getGhostParticles();

	Vector3r &x1 = pd.getPosition(m_bodies[0]);
	Vector3r &x2 = pd.getPosition(m_bodies[1]);
	Vector3r &x3 = pd.getPosition(m_bodies[2]);
	Vector3r &x4 = pg.getPosition(m_bodies[3]);
	Vector3r &x5 = pg.getPosition(m_bodies[4]);

	const Real invMass1 = pd.getInvMass(m_bodies[0]);
	const Real invMass2 = pd.getInvMass(m_bodies[1]);
	const Real invMass3 = pd.getInvMass(m_bodies[2]);
	const Real invMass4 = pg.getInvMass(m_bodies[3]);
	const Real invMass5 = pg.getInvMass(m_bodies[4]);

	Vector3r corr[5];

	bool res = PositionBasedElasticRods::solve_DarbouxVectorConstraint(
		x1, invMass1, x2, invMass2, x3, invMass3, x4, invMass4, x5, invMass5,
		simModel.getBendingAndTwistingStiffness(), 1.0, simModel.getRestDarbouxVector(), 
		corr[0], corr[1], corr[2], corr[3], corr[4]);

	if (res)
	{
		if (invMass1 != 0.0f)
			x1 += corr[0];
		
		if (invMass2 != 0.0f)
			x2 += corr[1];
		
		if (invMass3 != 0.0f)
			x3 += corr[2];
		
		if (invMass4 != 0.0f)
			x4 += corr[3];

		if (invMass5 != 0.0f)
			x5 += corr[4];
	}
	return res;
}

