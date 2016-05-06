#ifndef __TETMODEL_H__
#define __TETMODEL_H__

#include "Demos/Common/Config.h"
#include "Common/Common.h"
#include "Demos/Utils/IndexedFaceMesh.h"
#include "Demos/Utils/IndexedTetMesh.h"
#include "Demos/Simulation/ParticleData.h"
#include <vector>
#include "Demos/Utils/ObjectArray.h"

namespace PBD 
{	
	class TetModel 
	{
		public:
			TetModel();
			virtual ~TetModel();

			typedef IndexedFaceMesh SurfaceMesh;
			typedef IndexedTetMesh ParticleMesh;

			struct Attachment
			{
				unsigned int m_index;
				unsigned int m_triIndex;
				Real m_bary[3];
				Real m_dist;
				Real m_minError;
			};

		protected:
			/** offset which must be added to get the correct index in the particles array */
			unsigned int m_indexOffset;
			/** Tet mesh of particles which represents the simulation model */
			ParticleMesh m_particleMesh;			
			SurfaceMesh m_surfaceMesh;
			VertexData m_visVertices;
			SurfaceMesh m_visMesh;
			Real m_restitutionCoeff;
			Real m_frictionCoeff;
			ObjectArray<Attachment> m_attachments;
			
			void createSurfaceMesh();
 			void solveQuadraticForZero(const Vector3r& F, const Vector3r& Fu, 
 				const Vector3r& Fv, const Vector3r& Fuu,
 				const Vector3r&Fuv, const Vector3r& Fvv, 
 				Real& u, Real& v);
 			bool pointInTriangle(const Vector3r& p0, const Vector3r& p1, const Vector3r& p2, 
 				const Vector3r& p, Vector3r& inter, Vector3r &bary);


		public:
			void updateConstraints();

			SurfaceMesh &getSurfaceMesh();
			VertexData &getVisVertices();
			SurfaceMesh &getVisMesh();
			ParticleMesh &getParticleMesh();
			void cleanupModel();

			unsigned int getIndexOffset() const;

			void initMesh(const unsigned int nPoints, const unsigned int nTets, const unsigned int indexOffset, unsigned int* indices);
			void updateMeshNormals(const ParticleData &pd);

			/** Attach a visualization mesh to the surface of the body.
			 * Important: The vertex normals have to be updated before 
			 * calling this function by calling updateMeshNormals(). 
			 */
 			void attachVisMesh(const ParticleData &pd);

			/** Update the visualization mesh of the body.
			* Important: The vertex normals have to be updated before
			* calling this function by calling updateMeshNormals().
			*/
			void updateVisMesh(const ParticleData &pd);

			FORCE_INLINE Real getRestitutionCoeff() const
			{
				return m_restitutionCoeff;
			}

			FORCE_INLINE void setRestitutionCoeff(Real val)
			{
				m_restitutionCoeff = val;
			}

			FORCE_INLINE Real getFrictionCoeff() const
			{
				return m_frictionCoeff;
			}

			FORCE_INLINE void setFrictionCoeff(Real val)
			{
				m_frictionCoeff = val;
			}
			
	};
}

#endif