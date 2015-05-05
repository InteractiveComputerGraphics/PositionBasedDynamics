#ifndef __TimeStepTetModel_h__
#define __TimeStepTetModel_h__

#include "Demos/Utils/Config.h"
#include "TetModel.h"

namespace PBD
{
	class TimeStepTetModel 
	{
	protected:
		unsigned int m_simulationMethod;

		void clearAccelerations(TetModel &model);
		void semiImplicitEulerStep(TetModel &model, const float h);
		void constraintProjection(TetModel &model);

	public:
		TimeStepTetModel();
		virtual ~TimeStepTetModel(void);

		void step(TetModel &model);
		void reset(TetModel &model);	

		unsigned int getSimulationMethod() const { return m_simulationMethod; }
		void setSimulationMethod(unsigned int val) { m_simulationMethod = val; }
	};
}

#endif
