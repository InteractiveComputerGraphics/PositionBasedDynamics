#include "TetModel.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"

using namespace PBD;

TetModel::TetModel() :
	m_visMesh(),
	m_particleMesh()	
{	
}

TetModel::~TetModel(void)
{
	cleanupModel();
}

void TetModel::cleanupModel()
{
	m_particleMesh.release();
}

TetModel::ParticleMesh &TetModel::getParticleMesh() 
{
	return m_particleMesh;
}

TetModel::VisMesh &TetModel::getVisMesh()
{
	return m_visMesh;
}

void TetModel::initMesh(const unsigned int nPoints, const unsigned int nTets, const unsigned int indexOffset, unsigned int* indices)
{
	m_indexOffset = indexOffset;
	m_particleMesh.release();
	m_particleMesh.initMesh(nPoints, nTets * 6, nTets * 4, nTets);

	for (unsigned int i = 0; i < nTets; i++)
	{
		m_particleMesh.addTet(&indices[4 * i]);
	}
	m_particleMesh.buildNeighbors();

	createVisMesh();
}

unsigned int TetModel::getIndexOffset() const
{
	return m_indexOffset;
}

void TetModel::createVisMesh()
{
	const unsigned int nVerts = m_particleMesh.numVertices();

	m_visMesh.initMesh(nVerts, m_particleMesh.numEdges(), m_particleMesh.numFaces());

	// Search for all border faces of the tet mesh
	const IndexedTetMesh::Face *faceData = m_particleMesh.getFaceData().data();
	const unsigned int *faces = m_particleMesh.getFaces().data();
	for (unsigned int i = 0; i < m_particleMesh.numFaces(); i++)
	{
		const IndexedTetMesh::Face &face = faceData[i];
		// Found border face
		if ((face.m_tets[1] == 0xffffffff) || (face.m_tets[0] == 0xffffffff))
		{
			m_visMesh.addFace(&faces[3 * i]);
		}
	}
	m_visMesh.buildNeighbors();
}

void TetModel::updateMeshNormals(const ParticleData &pd)
{
	m_visMesh.updateNormals(pd);
	m_visMesh.updateVertexNormals(pd);
}