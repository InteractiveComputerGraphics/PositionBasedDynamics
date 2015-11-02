#ifndef __TIMESTEPCONTROLLER_h__
#define __TIMESTEPCONTROLLER_h__

#include "Demos/Utils/Config.h"
#include "SimulationModel.h"

namespace PBD
{
	class TimeStepController 
	{
	protected:
		unsigned int m_velocityUpdateMethod;
		unsigned int m_simulationMethod;
		unsigned int m_bendingMethod;
		unsigned int m_maxIter;
		
		/** Clear accelerations and add gravitation.
		*/
		void clearAccelerations(SimulationModel &model);
		void positionConstraintProjection(SimulationModel &model);
		void velocityConstraintProjection(SimulationModel &model);

	public:
		TimeStepController();
		virtual ~TimeStepController(void);

		void step(SimulationModel &model);
		void reset();

		unsigned int getVelocityUpdateMethod() const { return m_velocityUpdateMethod; }
		void setVelocityUpdateMethod(unsigned int val) { m_velocityUpdateMethod = val; }
		unsigned int getSimulationMethod() const { return m_simulationMethod; }
		void setSimulationMethod(unsigned int val) { m_simulationMethod = val; }
		unsigned int getBendingMethod() const { return m_bendingMethod; }
		void setBendingMethod(unsigned int val) { m_bendingMethod = val; }
		unsigned int getMaxIterations() const { return m_maxIter; }
		void setMaxIterations(unsigned int val) { m_maxIter = val; }
	};
}

#endif
