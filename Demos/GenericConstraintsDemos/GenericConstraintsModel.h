#ifndef __GENERICCONSTRAINTSMODEL_H__
#define __GENERICCONSTRAINTSMODEL_H__

#include "Demos/Common/Config.h"
#include "Demos/Utils/IndexedFaceMesh.h"
#include "Demos/Simulation/ParticleData.h"
#include "Demos/Simulation/SimulationModel.h"
#include <vector>
#include <Eigen/StdVector>

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
	};
}

#endif