#ifndef __POSITIONBASEDELASTICRODSTSC_H__
#define __POSITIONBASEDELASTICRODSTSC_H__

#include "Demos/Simulation/TimeStepController.h"
#include "PositionBasedElasticRodsModel.h"

namespace PBD
{
	class PositionBasedElasticRodsTSC : public TimeStepController
	{
	protected:
		Real m_damping;
		virtual void clearAccelerations(PositionBasedElasticRodsModel &model);

	public:
		PositionBasedElasticRodsTSC();
		virtual ~PositionBasedElasticRodsTSC(void);

		virtual void step(PositionBasedElasticRodsModel &model);

		Real getDamping() const { return m_damping; }
		void setDamping(Real val) { m_damping = val; }
	};
}

#endif
