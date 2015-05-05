#ifndef __TETMODEL_H__
#define __TETMODEL_H__

#include "Demos/Utils/Config.h"
#include "Demos/Utils/IndexedFaceMesh.h"
#include "Demos/Utils/IndexedTetMesh.h"
#include "Demos/Utils/ParticleData.h"
#include <vector>

namespace PBD 
{	
	class TetModel 
	{
		public:
			struct TetConstraint
			{
				float tetVolume;
				Eigen::Matrix3f invRestMat_SBD;
				Eigen::Matrix3f invRestMat_FEM;
			};

			TetModel();
			virtual ~TetModel();

			typedef IndexedFaceMesh<VertexData> VisMesh;
			typedef IndexedTetMesh<ParticleData> ParticleMesh;

		protected:
			/** Tet mesh of particles which represents the simulation model */
			ParticleMesh m_particleMesh;
			VisMesh m_visMesh;
			std::vector<TetConstraint> m_tetConstraints;
			float m_stiffness;			
			float m_poissonRatio;
			bool m_normalizeStretch;
			bool m_normalizeShear;

			void initConstraints();	
			void initTetConstraints();
			void createVisMesh();

		public:
			VisMesh &getVisMesh();
			ParticleMesh &getParticleMesh();
			void cleanupModel();

			float getStiffness() const { return m_stiffness; }
			void setStiffness(float val) { m_stiffness = val; }
			float getPoissonRatio() { return m_poissonRatio; }
			void setPoissonRatio(const float val) { m_poissonRatio = val; }
			bool getNormalizeStretch() const { return m_normalizeStretch; }
			void setNormalizeStretch(bool val) { m_normalizeStretch = val; }
			bool getNormalizeShear() const { return m_normalizeShear; }
			void setNormalizeShear(bool val) { m_normalizeShear = val; }

			void setGeometry(const unsigned int nPoints, Eigen::Vector3f* coords, const unsigned int nTets, unsigned int* indices);
			virtual void reset();			

			std::vector<TetConstraint> &getTetConstraints();
			
	};
}

#endif