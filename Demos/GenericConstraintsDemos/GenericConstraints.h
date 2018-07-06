#ifndef _GENERICCONSTRAINTS_H
#define _GENERICCONSTRAINTS_H

#include <Eigen/Dense>
#include "Simulation/Constraints.h"

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
		virtual bool solvePositionConstraint(SimulationModel &model, const unsigned int iter);
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
		virtual bool solvePositionConstraint(SimulationModel &model, const unsigned int iter);
	};

	class GenericHingeJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 12> m_jointInfo;

		static void constraintFct(
			const unsigned int numberOfRigidBodies,
			const Real invMass[],					// inverse mass is zero if body is static
			const Vector3r x[],						// positions of bodies
			const Matrix3r inertiaInverseW[],		// inverse inertia tensor (world space) of bodies
			const Quaternionr q[],
			void *userData,
			Eigen::Matrix<Real, 5, 1> &constraintValue);

		static void gradientFct(
			const unsigned int i,
			const unsigned int numberOfRigidBodies,
			const Real invMass[],
			const Vector3r x[],
			const Matrix3r inertiaInverseW[],
			const Quaternionr q[],
			void *userData,
			Eigen::Matrix<Real, 5, 6> &jacobian);

		GenericHingeJoint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model, const unsigned int iter);
	};

	class GenericBallJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 2> m_jointInfo;

		static void constraintFct(
			const unsigned int numberOfRigidBodies,
			const Real invMass[],					// inverse mass is zero if body is static
			const Vector3r x[],						// positions of bodies
			const Matrix3r inertiaInverseW[],		// inverse inertia tensor (world space) of bodies
			const Quaternionr q[],
			void *userData,
			Eigen::Matrix<Real, 3, 1> &constraintValue);

		GenericBallJoint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos);
		virtual bool solvePositionConstraint(SimulationModel &model, const unsigned int iter);
	};

	class GenericSliderJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 7> m_jointInfo;

		static void constraintFct(
			const unsigned int numberOfRigidBodies,
			const Real invMass[],					// inverse mass is zero if body is static
			const Vector3r x[],						// positions of bodies
			const Matrix3r inertiaInverseW[],		// inverse inertia tensor (world space) of bodies
			const Quaternionr q[],
			void *userData,
			Eigen::Matrix<Real, 5, 1> &constraintValue);

		GenericSliderJoint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
		virtual bool solvePositionConstraint(SimulationModel &model, const unsigned int iter);
	};
}

#endif
