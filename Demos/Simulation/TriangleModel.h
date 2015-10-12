#ifndef __TRIANGLEMODEL_H__
#define __TRIANGLEMODEL_H__

#include "Demos/Utils/Config.h"
#include <vector>
#include "Demos/Simulation/RigidBody.h"
#include "Demos/Utils/IndexedFaceMesh.h"
#include "Demos/Simulation/ParticleData.h"
#include <Eigen/StdVector>
#include "Constraints.h"

namespace PBD 
{	
	class TriangleModel
	{
		public:
			TriangleModel();
			virtual ~TriangleModel();

			typedef IndexedFaceMesh ParticleMesh;

		protected:
			/** offset which must be added to get the correct index in the particles array */
			unsigned int m_indexOffset;
			/** Face mesh of particles which represents the simulation model */
			ParticleMesh m_particleMesh;

		public:
			void updateConstraints();

			ParticleMesh &getParticleMesh();
			void cleanupModel();

			unsigned int getIndexOffset() const; 

			void initMesh(const unsigned int nPoints, const unsigned int nFaces, const unsigned int indexOffset, unsigned int* indices, const ParticleMesh::UVIndices& uvIndices, const ParticleMesh::UVs& uvs);
			void updateMeshNormals(const ParticleData &pd);
	};
}

#endif