#ifndef __TimeStepRigidBodyModel_h__
#define __TimeStepRigidBodyModel_h__

#include "Demos/Utils/Config.h"
#include "RigidBodyModel.h"

namespace PBD
{
	class TimeStepRigidBodyModel 
	{
	protected:
		unsigned int m_velocityUpdateMethod;

		/** Clear accelerations and add gravitation.
		*/
		void clearAccelerations(RigidBodyModel &model);
		void constraintProjection(RigidBodyModel &model);
		void constraintProjectionBallJoint(RigidBodyModel &model, const unsigned int index);
		void constraintProjectionBallOnLineJoint(RigidBodyModel &model, const unsigned int index);
		void constraintProjectionHingeJoint(RigidBodyModel &model, const unsigned int index);
		void constraintProjectionUniversalJoint(RigidBodyModel &model, const unsigned int index);

	public:
		TimeStepRigidBodyModel();
		virtual ~TimeStepRigidBodyModel(void);

		void step(RigidBodyModel &model);
		void reset(RigidBodyModel &model);

		unsigned int getVelocityUpdateMethod() const { return m_velocityUpdateMethod; }
		void setVelocityUpdateMethod(unsigned int val) { m_velocityUpdateMethod = val; }
	};
}

#endif
