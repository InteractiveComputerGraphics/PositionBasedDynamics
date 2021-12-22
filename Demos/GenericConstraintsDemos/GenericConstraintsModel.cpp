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

bool GenericConstraintsModel::addGenericDistanceConstraint(const unsigned int particle1, const unsigned int particle2, const Real stiffness)
{
	GenericDistanceConstraint *c = new GenericDistanceConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2, stiffness);
	if (res)
		m_constraints.push_back(c);
	return res;
}

bool GenericConstraintsModel::addGenericIsometricBendingConstraint(const unsigned int particle1, const unsigned int particle2,
				const unsigned int particle3, const unsigned int particle4, const Real stiffness)
{
	GenericIsometricBendingConstraint *c = new GenericIsometricBendingConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2, particle3, particle4, stiffness);
	if (res)
		m_constraints.push_back(c);
	return res;
}

bool GenericConstraintsModel::addGenericHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis)
{
	GenericHingeJoint *c = new GenericHingeJoint();
	const bool res = c->initConstraint(*this, rbIndex1, rbIndex2, pos, axis);
	if (res)
		m_constraints.push_back(c);
	return res;
}

bool GenericConstraintsModel::addGenericSliderJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis)
{
	GenericSliderJoint *c = new GenericSliderJoint();
	const bool res = c->initConstraint(*this, rbIndex1, rbIndex2, pos, axis);
	if (res)
		m_constraints.push_back(c);
	return res;
}

bool GenericConstraintsModel::addGenericBallJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos)
{
	GenericBallJoint *c = new GenericBallJoint();
	const bool res = c->initConstraint(*this, rbIndex1, rbIndex2, pos);
	if (res)
		m_constraints.push_back(c);
	return res;
}