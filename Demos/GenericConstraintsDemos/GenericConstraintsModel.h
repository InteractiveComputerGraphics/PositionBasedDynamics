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

			bool addGenericDistanceConstraint(const unsigned int particle1, const unsigned int particle2);
			bool addGenericIsometricBendingConstraint(const unsigned int particle1, const unsigned int particle2,
									const unsigned int particle3, const unsigned int particle4);
			bool addGenericHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
			bool addGenericSliderJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
			bool addGenericBallJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos);
	};
}

#endif