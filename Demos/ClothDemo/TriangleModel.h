#ifndef __TRIANGLEMODEL_H__
#define __TRIANGLEMODEL_H__

#include "Demos/Utils/Config.h"
#include "Demos/Utils/IndexedFaceMesh.h"
#include "Demos/Utils/ParticleData.h"
#include <vector>

namespace PBD 
{	
	class TriangleModel 
	{
		public:
			struct TriangleConstraint
			{
				float triangleArea;
				Eigen::Matrix2f invRestMat_SBD;
				Eigen::Matrix2f invRestMat_FEM;
			};

			struct BendingConstraint
			{
				unsigned int vertex1;
				unsigned int vertex2;
				unsigned int vertex3;
				unsigned int vertex4;
				float restAngle;
				Eigen::Matrix4f Q;
			};

			TriangleModel();
			virtual ~TriangleModel();

			typedef IndexedFaceMesh<ParticleData> ParticleMesh;

		protected:
			/** Face mesh of particles which represents the simulation model */
			ParticleMesh m_particleMesh;
			std::vector<BendingConstraint> m_bendingConstraints;
			std::vector<TriangleConstraint> m_triangleConstraints;
			float m_stiffness;			
			float m_bendingStiffness;
			float m_xxStiffness;			
			float m_yyStiffness;			
			float m_xyStiffness;			
			float m_xyPoissonRatio;
			float m_yxPoissonRatio;
			bool m_normalizeStretch;
			bool m_normalizeShear;
			
			void initBendingConstraints();
			void initTriangleConstraints();

		public:
			ParticleMesh &getParticleMesh();
			void cleanupModel();

			void initConstraints();

			float getStiffness() const { return m_stiffness; }
			void setStiffness(float val) { m_stiffness = val; }
			float getBendingStiffness() const { return m_bendingStiffness; }
			void setBendingStiffness(float val) { m_bendingStiffness = val; }
			float getXXStiffness() const { return m_xxStiffness; }
			void setXXStiffness(float val) { m_xxStiffness = val; }
			float getYYStiffness() const { return m_yyStiffness; }
			void setYYStiffness(float val) { m_yyStiffness = val; }
			float getXYStiffness() const { return m_xyStiffness; }
			void setXYStiffness(float val) { m_xyStiffness = val; }
			bool getNormalizeStretch() const { return m_normalizeStretch; }
			void setNormalizeStretch(bool val) { m_normalizeStretch = val; }
			bool getNormalizeShear() const { return m_normalizeShear; }
			void setNormalizeShear(bool val) { m_normalizeShear = val; }
			float getXYPoissonRatio() const { return m_xyPoissonRatio; }
			void setXYPoissonRatio(float val) { m_xyPoissonRatio = val; }
			float getYXPoissonRatio() const { return m_yxPoissonRatio; }
			void setYXPoissonRatio(float val) { m_yxPoissonRatio = val; }


			void setGeometry(const unsigned int nPoints, Eigen::Vector3f* coords, const unsigned int nFaces, unsigned int* indices, const ParticleMesh::UVIndices& uvIndices, const ParticleMesh::UVs& uvs);
			virtual void reset();
			void updateMeshNormals();

			std::vector<BendingConstraint> &getBendingConstraints();
			std::vector<TriangleConstraint> &getTriangleConstraints();
			
	};
}

#endif