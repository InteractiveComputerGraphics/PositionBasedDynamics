#include "GenericConstraintsModel.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"
#include "GenericConstraints.h"

using namespace PBD;


GenericConstraintsModel::GenericConstraintsModel() :
	SimulationModel()	
{	
}

GenericConstraintsModel::~GenericConstraintsModel(void)
{
}

bool GenericConstraintsModel::addGenericDistanceConstraint(const unsigned int particle1, const unsigned int particle2)
{
	GenericDistanceConstraint *c = new GenericDistanceConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2);
	if (res)
		m_constraints.push_back(c);
	return res;
}

bool GenericConstraintsModel::addGenericIsometricBendingConstraint(const unsigned int particle1, const unsigned int particle2,
				const unsigned int particle3, const unsigned int particle4)
{
	GenericIsometricBendingConstraint *c = new GenericIsometricBendingConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2, particle3, particle4);
	if (res)
		m_constraints.push_back(c);
	return res;
}