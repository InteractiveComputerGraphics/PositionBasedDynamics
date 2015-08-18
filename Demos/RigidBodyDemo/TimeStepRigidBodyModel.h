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

		void clearAccelerations(RigidBodyModel &model);
		void constraintProjection(RigidBodyModel &model);

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
