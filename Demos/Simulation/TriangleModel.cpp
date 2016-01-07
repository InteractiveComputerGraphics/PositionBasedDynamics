#include "TriangleModel.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"

using namespace PBD;

TriangleModel::TriangleModel() :
	m_particleMesh()
{
}

TriangleModel::~TriangleModel(void)
{
	cleanupModel();
}

void TriangleModel::cleanupModel()
{
	m_particleMesh.release();
}

void TriangleModel::updateMeshNormals(const ParticleData &pd)
{
	m_particleMesh.updateNormals(pd);
	m_particleMesh.updateVertexNormals(pd);
}

TriangleModel::ParticleMesh &TriangleModel::getParticleMesh()
{
	return m_particleMesh;
}

void TriangleModel::initMesh(const unsigned int nPoints, const unsigned int nFaces, const unsigned int indexOffset, unsigned int* indices, const ParticleMesh::UVIndices& uvIndices, const ParticleMesh::UVs& uvs)
{
	m_indexOffset = indexOffset;
	m_particleMesh.release();

	m_particleMesh.initMesh(nPoints, nFaces * 2, nFaces);

	for (unsigned int i = 0; i < nFaces; i++)
	{
		m_particleMesh.addFace(&indices[3 * i]);
	}
	m_particleMesh.copyUVs(uvIndices, uvs);
	m_particleMesh.buildNeighbors();
}

unsigned int TriangleModel::getIndexOffset() const
{
	return m_indexOffset;
}
