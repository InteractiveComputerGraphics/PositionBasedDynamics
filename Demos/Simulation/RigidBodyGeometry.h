#ifndef __RIGIDBODYGEOMETRY_H__
#define __RIGIDBODYGEOMETRY_H__

#include "Demos/Utils/Config.h"
#include "Demos/Utils/IndexedFaceMesh.h"
#include "Demos/Simulation/ParticleData.h"
#include <vector>

namespace PBD 
{	
	class RigidBodyGeometry
	{
		public:
			RigidBodyGeometry();
			virtual ~RigidBodyGeometry();

			typedef IndexedFaceMesh Mesh;

		protected:
			Mesh m_mesh;
			VertexData m_vertexData_local;
			VertexData m_vertexData;

		public:
			Mesh &getMesh();
			VertexData &getVertexData();
			const VertexData &getVertexData() const;
			VertexData &getVertexDataLocal();
			const VertexData &getVertexDataLocal() const;

			void initMesh(const unsigned int nVertices, const unsigned int nFaces, const Eigen::Vector3f *vertices, const unsigned int* indices, const Mesh::UVIndices& uvIndices, const Mesh::UVs& uvs);
			void updateMeshTransformation(const Eigen::Vector3f &x, const Eigen::Matrix3f &R);
			void updateMeshNormals(const VertexData &vd);
			
	};
}

#endif