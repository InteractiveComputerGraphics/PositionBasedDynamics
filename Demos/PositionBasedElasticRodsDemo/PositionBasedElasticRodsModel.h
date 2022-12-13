#ifndef __POSITIONBASEDELASTICRODSMODEL_H__
#define __POSITIONBASEDELASTICRODSMODEL_H__

#include "Simulation/ParticleData.h"
#include "Simulation/SimulationModel.h"
#include <vector>

namespace PBD 
{	
	class Constraint;

	class PositionBasedElasticRodsModel : public SimulationModel
	{
		public:
			static int REST_DARBOUX_VECTOR;

			PositionBasedElasticRodsModel();
			virtual ~PositionBasedElasticRodsModel();

		protected:
			ParticleData m_ghostParticles;
			Vector3r m_restDarbouxVector;
			Vector3r m_stiffness;

		public:
			virtual void reset();
			virtual void cleanup();
			virtual void initParameters();

			ParticleData &getGhostParticles();
			void addElasticRodModel(
				const unsigned int nPoints,
				Vector3r *points);

			bool addPerpendiculaBisectorConstraint(const unsigned int p0, const unsigned int p1, const unsigned int p2);
			bool addGhostPointEdgeDistanceConstraint(const unsigned int pA, const unsigned int pB, const unsigned int pG);
			bool addDarbouxVectorConstraint(const unsigned int pA, const unsigned int pB,
											const unsigned int pC, const unsigned int pD, const unsigned int pE);


			void setRestDarbouxVector(const Vector3r &val) { m_restDarbouxVector = val; }
			Vector3r &getRestDarbouxVector() { return m_restDarbouxVector; }
			void setBendingAndTwistingStiffness(const Vector3r &val) { m_stiffness = val; }
			Vector3r &getBendingAndTwistingStiffness() { return m_stiffness; }

			virtual Real getRodBendingStiffnessX() { return m_stiffness[0]; }
			virtual void setRodBendingStiffnessX(Real val);
			virtual Real getRodBendingStiffnessY() { return m_stiffness[1]; }
			virtual void setRodBendingStiffnessY(Real val);
			virtual Real getRodTwistingStiffness() { return m_stiffness[2]; }
			virtual void setRodTwistingStiffness(Real val);

	};
}

#endif