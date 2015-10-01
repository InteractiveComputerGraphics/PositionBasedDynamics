#ifndef __TimeStepRigidBodyParticleModel_h__
#define __TimeStepRigidBodyParticleModel_h__

#include "Demos/Utils/Config.h"
#include "RigidBodyParticleModel.h"

namespace PBD
{
	class TimeStepRigidBodyParticleModel 
	{
	protected:
		unsigned int m_velocityUpdateMethod;
		unsigned int m_simulationMethod;
		unsigned int m_bendingMethod;

		/** Clear accelerations and add gravitation.
		*/
		void clearAccelerations(RigidBodyParticleModel &model);
		void constraintProjection(RigidBodyParticleModel &model);

		void constraintProjectionBallJoint(RigidBodyParticleModel &model, const unsigned int index);
		void constraintProjectionBallOnLineJoint(RigidBodyParticleModel &model, const unsigned int index);
		void constraintProjectionHingeJoint(RigidBodyParticleModel &model, const unsigned int index);
		void constraintProjectionUniversalJoint(RigidBodyParticleModel &model, const unsigned int index);
		void constraintProjectionRigidBodyParticleBallJoint(RigidBodyParticleModel &model, const unsigned int index);

		void constraintProjectionParticleModel(RigidBodyParticleModel &model);

	public:
		TimeStepRigidBodyParticleModel();
		virtual ~TimeStepRigidBodyParticleModel(void);

		void step(RigidBodyParticleModel &model);
		void reset();

		unsigned int getVelocityUpdateMethod() const { return m_velocityUpdateMethod; }
		void setVelocityUpdateMethod(unsigned int val) { m_velocityUpdateMethod = val; }
		unsigned int getSimulationMethod() const { return m_simulationMethod; }
		void setSimulationMethod(unsigned int val) { m_simulationMethod = val; }
		unsigned int getBendingMethod() const { return m_bendingMethod; }
		void setBendingMethod(unsigned int val) { m_bendingMethod = val; }
	};
}

#endif
