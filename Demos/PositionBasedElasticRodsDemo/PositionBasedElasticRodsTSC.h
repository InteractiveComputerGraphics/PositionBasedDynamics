#ifndef __POSITIONBASEDELASTICRODSTSC_H__
#define __POSITIONBASEDELASTICRODSTSC_H__

#include "Simulation/TimeStepController.h"
#include "PositionBasedElasticRodsModel.h"

namespace PBD
{
	class PositionBasedElasticRodsTSC : public TimeStepController
	{
	protected:
		Real m_damping;
		virtual void clearAccelerations(SimulationModel &model);

	public:
		PositionBasedElasticRodsTSC();
		virtual ~PositionBasedElasticRodsTSC(void);

		virtual void step(SimulationModel &model);

		Real getDamping() const { return m_damping; }
		void setDamping(Real val) { m_damping = val; }
	};
}

#endif
