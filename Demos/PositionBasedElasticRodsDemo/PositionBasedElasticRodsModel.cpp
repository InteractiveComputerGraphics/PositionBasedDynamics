#include "PositionBasedElasticRodsModel.h"
#include "PositionBasedDynamics/PositionBasedElasticRods.h"
#include "PositionBasedElasticRodsConstraints.h"

using namespace PBD;

PositionBasedElasticRodsModel::PositionBasedElasticRodsModel() :
	SimulationModel()
{
	m_stiffness.setOnes();
}

PositionBasedElasticRodsModel::~PositionBasedElasticRodsModel(void)
{
}

void PositionBasedElasticRodsModel::cleanup()
{
	SimulationModel::cleanup();
	m_ghostParticles.release();
}

void PositionBasedElasticRodsModel::reset()
{
	// ghost particles
	for (unsigned int i = 0; i < m_ghostParticles.size(); i++)
	{
		const Vector3r & x0 = m_ghostParticles.getPosition0(i);
		m_ghostParticles.getPosition(i) = x0;
		m_ghostParticles.getLastPosition(i) = m_ghostParticles.getPosition(i);
		m_ghostParticles.getOldPosition(i) = m_ghostParticles.getPosition(i);
		m_ghostParticles.getVelocity(i).setZero();
		m_ghostParticles.getAcceleration(i).setZero();
	}
	SimulationModel::reset();
}

ParticleData & PositionBasedElasticRodsModel::getGhostParticles()
{
	return m_ghostParticles;
}

bool PositionBasedElasticRodsModel::addPerpendiculaBisectorConstraint(const unsigned int p0, const unsigned int p1, const unsigned int p2)
{
	PerpendiculaBisectorConstraint *c = new PerpendiculaBisectorConstraint();
	const bool res = c->initConstraint(*this, p0, p1, p2);
	if (res)
		m_constraints.push_back(c);
	return res;
}

bool PositionBasedElasticRodsModel::addGhostPointEdgeDistanceConstraint(const unsigned int pA, const unsigned int pB, const unsigned int pG)
{
	GhostPointEdgeDistanceConstraint *c = new GhostPointEdgeDistanceConstraint();
	const bool res = c->initConstraint(*this, pA, pB, pG);
	if (res)
		m_constraints.push_back(c);
	return res;
}

bool PositionBasedElasticRodsModel::addDarbouxVectorConstraint(const unsigned int pA, const unsigned int pB,
	const unsigned int pC, const unsigned int pD, const unsigned int pE)
{
	DarbouxVectorConstraint *c = new DarbouxVectorConstraint();
	const bool res = c->initConstraint(*this, pA, pB, pC, pD, pE);
	if (res)
		m_constraints.push_back(c);
	return res;
}

void PBD::PositionBasedElasticRodsModel::addElasticRodModel(
	const unsigned int nPoints, 
	Vector3r * points)
{


}

