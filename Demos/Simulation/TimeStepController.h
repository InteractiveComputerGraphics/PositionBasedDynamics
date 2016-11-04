#ifndef __TIMESTEPCONTROLLER_h__
#define __TIMESTEPCONTROLLER_h__

#include "Common/Common.h"
#include "SimulationModel.h"
#include "CollisionDetection.h"

namespace PBD
{
	class TimeStepController 
	{
	protected:
		CollisionDetection *m_collisionDetection;
		unsigned int m_velocityUpdateMethod;
		unsigned int m_maxIter;
		unsigned int m_maxIterVel;
		Vector3r m_gravity;		

		/** Clear accelerations and add gravitation.
		*/
		void clearAccelerations(SimulationModel &model);
		void positionConstraintProjection(SimulationModel &model);
		void velocityConstraintProjection(SimulationModel &model);

		static void contactCallbackFunction(const unsigned int contactType, 
			const unsigned int bodyIndex1, const unsigned int bodyIndex2,
			const Vector3r &cp1, const Vector3r &cp2,
			const Vector3r &normal, const Real dist, 
			const Real restitutionCoeff, const Real frictionCoeff, void *userData);


	public:
		TimeStepController();
		virtual ~TimeStepController(void);

		void step(SimulationModel &model);
		void reset();

		void setCollisionDetection(SimulationModel &model, CollisionDetection *cd);
		CollisionDetection *getCollisionDetection();

		unsigned int getVelocityUpdateMethod() const { return m_velocityUpdateMethod; }
		void setVelocityUpdateMethod(unsigned int val) { m_velocityUpdateMethod = val; }
		unsigned int getMaxIterations() const { return m_maxIter; }
		void setMaxIterations(unsigned int val) { m_maxIter = val; }
		unsigned int getMaxIterationsV() const { return m_maxIterVel; }
		void setMaxIterationsV(unsigned int val) { m_maxIterVel = val; }
		const Vector3r& getGravity() const { return m_gravity; }
		void setGravity(const Vector3r& val) { m_gravity = val; }
	};
}

#endif
