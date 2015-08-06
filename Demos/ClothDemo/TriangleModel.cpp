#include "TriangleModel.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"

using namespace PBD;

TriangleModel::TriangleModel() :
	m_particleMesh()	
{	
	m_stiffness = 1.0f;
	m_bendingStiffness = 0.01f;
	m_xxStiffness = 1.0f;
	m_yyStiffness = 1.0f;
	m_xyStiffness = 1.0f;
	m_xyPoissonRatio = 0.3f;
	m_yxPoissonRatio = 0.3f;
	m_normalizeShear = false;
	m_normalizeStretch = false;
}

TriangleModel::~TriangleModel(void)
{
	cleanupModel();
}

void TriangleModel::cleanupModel()
{
	m_particleMesh.release();
}

void TriangleModel::updateMeshNormals()
{
	m_particleMesh.updateNormals();
	m_particleMesh.updateVertexNormals();
}

TriangleModel::ParticleMesh &TriangleModel::getParticleMesh() 
{
	return m_particleMesh;
}

void TriangleModel::setGeometry(const unsigned int nPoints, Eigen::Vector3f* coords, const unsigned int nFaces, unsigned int* indices, const ParticleMesh::UVIndices& uvIndices, const ParticleMesh::UVs& uvs)
{
	m_particleMesh.release();

	m_particleMesh.initMemory(nPoints, nFaces * 2, nFaces);
	
	ParticleData& pd = m_particleMesh.getVertexData();
	for(unsigned int i=0;i<nPoints;i++)
	{
		m_particleMesh.addVertex(coords[i]);
	}
	for(unsigned int i=0;i<nFaces;i++)
	{
		m_particleMesh.addFace(&indices[3 * i]);
	}
	m_particleMesh.copyUVs(uvIndices, uvs);
	m_particleMesh.buildNeighbors();

	// Update normals
	m_particleMesh.updateNormals();
	m_particleMesh.updateVertexNormals();
}


void TriangleModel::reset()
{
	const unsigned int nPoints = m_particleMesh.numVertices();
	ParticleData &pd = m_particleMesh.getVertexData();
	
	for(unsigned int i=0; i < nPoints; i++)
	{
		const Eigen::Vector3f& x0 = pd.getPosition0(i);
		pd.getPosition(i) = x0;
		pd.getLastPosition(i) = pd.getPosition(i);
		pd.getOldPosition(i) = pd.getPosition(i);
		pd.getVelocity(i).setZero();
		pd.getAcceleration(i).setZero();
	}
}

void TriangleModel::initConstraints()
{
	initTriangleConstraints();
	initBendingConstraints();
}

void TriangleModel::initTriangleConstraints()
{
	m_triangleConstraints.clear();
	unsigned int nFaces = getParticleMesh().numFaces();
	const unsigned int *tris = getParticleMesh().getFaces().data();
	const ParticleData &pd = getParticleMesh().getVertexData();
	m_triangleConstraints.reserve(nFaces);
	for (unsigned int i = 0; i < nFaces; i++)
	{
		const Eigen::Vector3f &x1 = pd.getPosition0(tris[3 * i]);
		const Eigen::Vector3f &x2 = pd.getPosition0(tris[3 * i + 1]);
		const Eigen::Vector3f &x3 = pd.getPosition0(tris[3 * i + 2]);

		// Bring triangles to xy plane
		const Eigen::Vector3f y1(x1[0], x1[2], 0.0);
		const Eigen::Vector3f y2(x2[0], x2[2], 0.0);
		const Eigen::Vector3f y3(x3[0], x3[2], 0.0);

		TriangleConstraint tc;
		PositionBasedDynamics::computeStrainTriangleInvRestMat(y1, y2, y3, tc.invRestMat_SBD);
		PositionBasedDynamics::computeFEMTriangleInvRestMat(x1, x2, x3, tc.triangleArea, tc.invRestMat_FEM);
		m_triangleConstraints.push_back(tc);
	}
}

void TriangleModel::initBendingConstraints()
{
	m_bendingConstraints.clear();
	unsigned int nEdges = getParticleMesh().numEdges();
	const TriangleModel::ParticleMesh::Edge *edges = getParticleMesh().getEdges().data();
	const unsigned int *tris = getParticleMesh().getFaces().data();
	const ParticleData &pd = getParticleMesh().getVertexData();
	for (unsigned int i = 0; i < nEdges; i++)
	{
		const int tri1 = edges[i].m_face[0];
		const int tri2 = edges[i].m_face[1];
		if ((tri1 != 0xffffffff) && (tri2 != 0xffffffff))
		{
			// Find the triangle points which do not lie on the axis
			const int axisPoint1 = edges[i].m_vert[0];
			const int axisPoint2 = edges[i].m_vert[1];
			int point1 = -1;
			int point2 = -1;
			for (int j = 0; j < 3; j++)
			{
				if ((tris[3 * tri1 + j] != axisPoint1) && (tris[3 * tri1 + j] != axisPoint2))
				{
					point1 = tris[3 * tri1 + j];
					break;
				}
			}
			for (int j = 0; j < 3; j++)
			{
				if ((tris[3 * tri2 + j] != axisPoint1) && (tris[3 * tri2 + j] != axisPoint2))
				{
					point2 = tris[3 * tri2 + j];
					break;
				}
			}
			if ((point1 != -1) && (point2 != -1))
			{
				BendingConstraint bc; 
				bc.vertex1 = point1;
				bc.vertex2 = point2;
				bc.vertex3 = edges[i].m_vert[0];
				bc.vertex4 = edges[i].m_vert[1];				
				bc.restAngle = 0.0f;
				PositionBasedDynamics::computeQuadraticBendingMat(pd.getPosition0(bc.vertex1), pd.getPosition0(bc.vertex2), pd.getPosition0(bc.vertex3), pd.getPosition0(bc.vertex4), bc.Q);
				m_bendingConstraints.push_back(bc);
			}
		}
	}
}

TriangleModel::BendingConstraintVector & TriangleModel::getBendingConstraints()
{
	return m_bendingConstraints;
}

TriangleModel::TriangleConstraintVector & TriangleModel::getTriangleConstraints()
{
	return m_triangleConstraints;
}


