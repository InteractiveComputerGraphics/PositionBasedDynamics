#ifndef _GENERICCONSTRAINTS_H
#define _GENERICCONSTRAINTS_H

#include <Eigen/Dense>
#include "Demos/Simulation/Constraints.h"

namespace PBD
{
	class SimulationModel;

	class GenericDistanceConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Real m_restLength;

		static void constraintFct(
			const unsigned int numberOfParticles,
			const Real invMass[],
			const Vector3r x[],
			void *userData,
			Eigen::Matrix<Real, 1, 1> &constraintValue);

		static void gradientFct(
			const unsigned int i,
			const unsigned int numberOfParticles,
			const Real invMass[],
			const Vector3r x[],
			void *userData,
			Eigen::Matrix<Real, 1, 3> &jacobian);

		GenericDistanceConstraint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class GenericIsometricBendingConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Matrix4r m_Q;

		static void constraintFct(
			const unsigned int numberOfParticles,
			const Real invMass[],
			const Vector3r x[],
			void *userData,
			Eigen::Matrix<Real, 1, 1> &constraintValue);

		GenericIsometricBendingConstraint() : Constraint(4) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
								const unsigned int particle3, const unsigned int particle4);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};
}

#endif
