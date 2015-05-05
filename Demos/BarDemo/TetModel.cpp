#include "TetModel.h"
#include "PositionBasedDynamics\PositionBasedDynamics.h"

using namespace PBD;

TetModel::TetModel() :
	m_visMesh(),
	m_particleMesh()	
{	
	m_stiffness = 1.0f;
	m_poissonRatio = 0.3f;
	m_normalizeShear = false;
	m_normalizeStretch = false;
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

void TetModel::setGeometry(const unsigned int nPoints, Eigen::Vector3f* coords, const unsigned int nTets, unsigned int* indices)
{
	m_particleMesh.release();
	m_particleMesh.initMemory(nPoints, nTets * 6, nTets * 4, nTets);

	ParticleData& pd = m_particleMesh.getVertexData();
	for (unsigned int i = 0; i < nPoints; i++)
	{
		m_particleMesh.addVertex(coords[i]);
	}
	for (unsigned int i = 0; i < nTets; i++)
	{
		m_particleMesh.addTet(&indices[4 * i]);
	}
	m_particleMesh.buildNeighbors();

	createVisMesh();

	initConstraints();
}

void TetModel::reset()
{
	const unsigned int nPoints = m_particleMesh.numVertices();
	ParticleData &pd = m_particleMesh.getVertexData();

	for (unsigned int i = 0; i < nPoints; i++)
	{
		const Eigen::Vector3f& x0 = pd.getPosition0(i);
		pd.getPosition(i) = x0;
		pd.getLastPosition(i) = pd.getPosition(i);
		pd.getVelocity(i).setZero();
		pd.getAcceleration(i).setZero();
	}
}

void TetModel::initConstraints()
{
	initTetConstraints();
}

void TetModel::initTetConstraints()
{
 	m_tetConstraints.clear();
 	unsigned int nTets = getParticleMesh().numTets();
	const unsigned int *tets = getParticleMesh().getTets().data();
 	const ParticleData &pd = getParticleMesh().getVertexData();
	for (unsigned int i = 0; i < nTets; i++)
	{
		const Eigen::Vector3f &x1 = pd.getPosition(tets[4 * i]);
		const Eigen::Vector3f &x2 = pd.getPosition(tets[4 * i + 1]);
		const Eigen::Vector3f &x3 = pd.getPosition(tets[4 * i + 2]);
		const Eigen::Vector3f &x4 = pd.getPosition(tets[4 * i + 3]);
 
 		TetConstraint tc;
 		PositionBasedDynamics::computeStrainTetraInvRestMat(x1, x2, x3, x4, tc.invRestMat_SBD);
 		PositionBasedDynamics::computeFEMTetraInvRestMat(x1, x2, x3, x4, tc.tetVolume, tc.invRestMat_FEM);
 		m_tetConstraints.push_back(tc);
 	}
}

std::vector<TetModel::TetConstraint> & TetModel::getTetConstraints()
{
	return m_tetConstraints;
}

void TetModel::createVisMesh()
{
	const unsigned int nVerts = m_particleMesh.numVertices();
	const ParticleData &pd = getParticleMesh().getVertexData();

	m_visMesh.initMemory(nVerts, m_particleMesh.numEdges(), m_particleMesh.numFaces());

	// Add points
	for (unsigned int i = 0; i < nVerts; i++)
	{
		m_visMesh.addVertex(pd.getPosition0(i));
	}

	// Search for all border faces of the tet mesh
	const IndexedTetMesh<ParticleData>::Face *faceData = m_particleMesh.getFaceData().data();
	const unsigned int *faces = m_particleMesh.getFaces().data();
	for (unsigned int i = 0; i < m_particleMesh.numFaces(); i++)
	{
		const IndexedTetMesh<ParticleData>::Face &face = faceData[i];
		// Found border face
		if ((face.m_tets[1] == 0xffffffff) || (face.m_tets[0] == 0xffffffff))
		{
			m_visMesh.addFace(&faces[3 * i]);
		}
	}
	m_visMesh.buildNeighbors();

	m_visMesh.updateNormals();
 	m_visMesh.updateVertexNormals(); 	
}

