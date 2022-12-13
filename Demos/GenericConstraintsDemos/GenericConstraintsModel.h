#ifndef __GENERICCONSTRAINTSMODEL_H__
#define __GENERICCONSTRAINTSMODEL_H__

#include "Utils/IndexedFaceMesh.h"
#include "Simulation/ParticleData.h"
#include "Simulation/SimulationModel.h"
#include <vector>

namespace PBD 
{	
	class GenericConstraintsModel : public SimulationModel
	{
		public:
			GenericConstraintsModel();
			virtual ~GenericConstraintsModel();

			virtual void initParameters();

			bool addGenericDistanceConstraint(const unsigned int particle1, const unsigned int particle2, const Real stiffness);
			bool addGenericIsometricBendingConstraint(const unsigned int particle1, const unsigned int particle2,
									const unsigned int particle3, const unsigned int particle4, const Real stiffness);
			bool addGenericHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
			bool addGenericSliderJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
			bool addGenericBallJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos);

			virtual void setClothStiffness(Real val);
			virtual void setClothBendingStiffness(Real val);
	};
}

#endif