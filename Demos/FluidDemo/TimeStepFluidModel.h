#ifndef __TimeStepFluidModel_h__
#define __TimeStepFluidModel_h__

#include "FluidModel.h"

namespace PBD
{
	class TimeStepFluidModel 
	{
	protected:
		unsigned int m_velocityUpdateMethod;

		void clearAccelerations(FluidModel &model);
		void computeXSPHViscosity(FluidModel &model);
		void computeDensities(FluidModel &model);
		void updateTimeStepSizeCFL(FluidModel &model, const Real minTimeStepSize, const Real maxTimeStepSize);
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
