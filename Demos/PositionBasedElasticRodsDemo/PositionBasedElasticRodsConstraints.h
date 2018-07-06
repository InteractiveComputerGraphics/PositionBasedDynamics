#ifndef _POSITIONBASEDELASTICRODSCONSTRAINTS_H
#define _POSITIONBASEDELASTICRODSCONSTRAINTS_H

#include <Eigen/Dense>
#include "Simulation/Constraints.h"
#include "PositionBasedElasticRodsModel.h"

namespace PBD
{
	class SimulationModel;

	class GhostPointEdgeDistanceConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Real m_restLength;

		GhostPointEdgeDistanceConstraint() : Constraint(3) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(PositionBasedElasticRodsModel &model, const unsigned int particle1, const unsigned int particle2, const unsigned int particle3);
		virtual bool solvePositionConstraint(SimulationModel &model, const unsigned int iter);
	};

	class PerpendiculaBisectorConstraint : public Constraint
	{
	public:
		static int TYPE_ID;

		PerpendiculaBisectorConstraint() : Constraint(3) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2, const unsigned int particle3);
		virtual bool solvePositionConstraint(SimulationModel &model, const unsigned int iter);
	};

	class DarbouxVectorConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Matrix3r m_dA; //material frame A
		Matrix3r m_dB; //material frame B

		DarbouxVectorConstraint() : Constraint(5) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(PositionBasedElasticRodsModel &model, const unsigned int particle1, const unsigned int particle2,
			const unsigned int particle3, const unsigned int particle4, const unsigned int particle5);

		virtual bool solvePositionConstraint(SimulationModel &model, const unsigned int iter);
	};

}

#endif
