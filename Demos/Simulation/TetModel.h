#ifndef __TETMODEL_H__
#define __TETMODEL_H__

#include "Demos/Utils/Config.h"
#include "Demos/Utils/IndexedFaceMesh.h"
#include "Demos/Utils/IndexedTetMesh.h"
#include "Demos/Simulation/ParticleData.h"
#include <vector>

namespace PBD 
{	
	class TetModel 
	{
		public:
			TetModel();
			virtual ~TetModel();

			typedef IndexedFaceMesh VisMesh;
			typedef IndexedTetMesh ParticleMesh;

		protected:
			/** offset which must be added to get the correct index in the particles array */
			unsigned int m_indexOffset;
			/** Tet mesh of particles which represents the simulation model */
			ParticleMesh m_particleMesh;
			VisMesh m_visMesh;
			
			void createVisMesh();

		public:
			void updateConstraints();

			VisMesh &getVisMesh();
			ParticleMesh &getParticleMesh();
			void cleanupModel();

			unsigned int getIndexOffset() const;

			void initMesh(const unsigned int nPoints, const unsigned int nTets, const unsigned int indexOffset, unsigned int* indices);
			void updateMeshNormals(const ParticleData &pd);
			
	};
}

#endif