#ifndef __TimeStepFluidModel_h__
#define __TimeStepFluidModel_h__

#include "Demos/Utils/Config.h"
#include "FluidModel.h"

namespace PBD
{
	class TimeStepFluidModel 
	{
	protected:
		unsigned int m_velocityUpdateMethod;

		void clearAccelerations(FluidModel &model);
		void computeViscosityAccels(FluidModel &model);
		void computeDensities(FluidModel &model);
		void updateTimeStepSizeCFL(FluidModel &model, const float minTimeStepSize, const float maxTimeStepSize);
		void constraintProjection(FluidModel &model);

	public:
		TimeStepFluidModel();
		virtual ~TimeStepFluidModel(void);

		void step(FluidModel &model);
		void reset();

		unsigned int getVelocityUpdateMethod() const { return m_velocityUpdateMethod; }
		void setVelocityUpdateMethod(unsigned int val) { m_velocityUpdateMethod = val; }
	};
}

#endif
