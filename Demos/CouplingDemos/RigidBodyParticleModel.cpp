#include "RigidBodyParticleModel.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"

using namespace PBD;

int RigidBodyParticleModel::BallJoint::TYPE_ID = 1;
int RigidBodyParticleModel::BallOnLineJoint::TYPE_ID = 2;
int RigidBodyParticleModel::HingeJoint::TYPE_ID = 3;
int RigidBodyParticleModel::UniversalJoint::TYPE_ID = 4;
int RigidBodyParticleModel::RigidBodyParticleBallJoint::TYPE_ID = 5;

RigidBodyParticleModel::RigidBodyParticleModel() :
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

RigidBodyParticleModel::~RigidBodyParticleModel(void)
{
	for (unsigned int i = 0; i < m_rigidBodies.size(); i++)
		delete m_rigidBodies[i];
	m_rigidBodies.clear();
	for (unsigned int i = 0; i < m_joints.size(); i++)
		delete m_joints[i];
	m_joints.clear();

	cleanupModel();
}

void RigidBodyParticleModel::reset()
{
	// rigid body model
	for (size_t i = 0; i < m_rigidBodies.size(); i++)
	{
		m_rigidBodies[i]->getPosition() = m_rigidBodies[i]->getPosition0();
		m_rigidBodies[i]->getOldPosition() = m_rigidBodies[i]->getPosition0();
		m_rigidBodies[i]->getLastPosition() = m_rigidBodies[i]->getPosition0();

		m_rigidBodies[i]->getRotation() = m_rigidBodies[i]->getRotation0();
		m_rigidBodies[i]->getOldRotation() = m_rigidBodies[i]->getRotation0();
		m_rigidBodies[i]->getLastRotation() = m_rigidBodies[i]->getRotation0();

		m_rigidBodies[i]->getVelocity().setZero();
		m_rigidBodies[i]->getAngularVelocity().setZero();

		m_rigidBodies[i]->getAcceleration().setZero();
		m_rigidBodies[i]->getTorque().setZero();

		m_rigidBodies[i]->rotationUpdated();
	}
	
	updateJoints();

	// particle model 
	const unsigned int nPoints = m_particleMesh.numVertices();
	ParticleData &pd = m_particleMesh.getVertexData();

	for (unsigned int i = 0; i < nPoints; i++)
	{
		const Eigen::Vector3f& x0 = pd.getPosition0(i);
		pd.getPosition(i) = x0;
		pd.getLastPosition(i) = pd.getPosition(i);
		pd.getOldPosition(i) = pd.getPosition(i);
		pd.getVelocity(i).setZero();
		pd.getAcceleration(i).setZero();
	}
}

RigidBodyParticleModel::RigidBodyVector & RigidBodyParticleModel::getRigidBodies()
{
	return m_rigidBodies;
}

RigidBodyParticleModel::JointVector & RigidBodyParticleModel::getJoints()
{
	return m_joints;
}

void RigidBodyParticleModel::updateJoints()
{
	for (unsigned int i = 0; i < m_joints.size(); i++)
	{
		if (m_joints[i]->getTypeId() == RigidBodyParticleModel::BallJoint::TYPE_ID)
			updateBallJoint(i);
		else if (m_joints[i]->getTypeId() == RigidBodyParticleModel::BallOnLineJoint::TYPE_ID)
			updateBallOnLineJoint(i);
		else if (m_joints[i]->getTypeId() == RigidBodyParticleModel::HingeJoint::TYPE_ID)
			updateHingeJoint(i);
		else if (m_joints[i]->getTypeId() == RigidBodyParticleModel::UniversalJoint::TYPE_ID)
			updateUniversalJoint(i);
	}
}


void RigidBodyParticleModel::addBallJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos)
{
	BallJoint *bj = new BallJoint();
	bj->m_index[0] = rbIndex1;
	bj->m_index[1] = rbIndex2;

	// transform in local coordinates
	RigidBody &rb1 = *m_rigidBodies[rbIndex1];
	RigidBody &rb2 = *m_rigidBodies[rbIndex2];

	PositionBasedRigidBodyDynamics::initRigidBodyBallJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos,
		bj->m_jointInfo);

	m_joints.push_back(bj);
}

void RigidBodyParticleModel::updateBallJoint(const unsigned int i)
{
	BallJoint &bj = *(BallJoint*)m_joints[i];
	RigidBody &rb1 = *m_rigidBodies[bj.m_index[0]];
	RigidBody &rb2 = *m_rigidBodies[bj.m_index[1]];
	PositionBasedRigidBodyDynamics::updateRigidBodyBallJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		bj.m_jointInfo);
}

void RigidBodyParticleModel::addBallOnLineJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &dir)
{
	BallOnLineJoint *bj = new BallOnLineJoint();
	bj->m_index[0] = rbIndex1;
	bj->m_index[1] = rbIndex2;

	// transform in local coordinates
	RigidBody &rb1 = *m_rigidBodies[rbIndex1];
	RigidBody &rb2 = *m_rigidBodies[rbIndex2];

	PositionBasedRigidBodyDynamics::initRigidBodyBallOnLineJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		pos, dir,
		bj->m_jointInfo);

	m_joints.push_back(bj);
}

void RigidBodyParticleModel::updateBallOnLineJoint(const unsigned int i)
{
	BallOnLineJoint &bj = *(BallOnLineJoint*)m_joints[i];
	RigidBody &rb1 = *m_rigidBodies[bj.m_index[0]];
	RigidBody &rb2 = *m_rigidBodies[bj.m_index[1]];
	PositionBasedRigidBodyDynamics::updateRigidBodyBallOnLineJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		bj.m_jointInfo);
}


void RigidBodyParticleModel::addHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
{
	HingeJoint *hj = new HingeJoint();
	hj->m_index[0] = rbIndex1;
	hj->m_index[1] = rbIndex2;

	// transform in local coordinates
	RigidBody &rb1 = *m_rigidBodies[rbIndex1];
	RigidBody &rb2 = *m_rigidBodies[rbIndex2];

	PositionBasedRigidBodyDynamics::initRigidBodyHingeJointInfo(
		rb1.getPosition0(),
		rb1.getRotation0(),
		rb2.getPosition0(),
		rb2.getRotation0(),
		pos,
		axis,
		hj->m_jointInfo);

	m_joints.push_back(hj);
}

void RigidBodyParticleModel::updateHingeJoint(const unsigned int i)
{
	HingeJoint &hj = *(HingeJoint*) m_joints[i];
	RigidBody &rb1 = *m_rigidBodies[hj.m_index[0]];
	RigidBody &rb2 = *m_rigidBodies[hj.m_index[1]];
	PositionBasedRigidBodyDynamics::updateRigidBodyHingeJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		hj.m_jointInfo);
}

void RigidBodyParticleModel::addUniversalJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2)
{
	UniversalJoint *uj = new UniversalJoint();
	uj->m_index[0] = rbIndex1;
	uj->m_index[1] = rbIndex2;

	// transform in local coordinates
	RigidBody &rb1 = *m_rigidBodies[rbIndex1];
	RigidBody &rb2 = *m_rigidBodies[rbIndex2];

	PositionBasedRigidBodyDynamics::initRigidBodyUniversalJointInfo(
		rb1.getPosition0(),
		rb1.getRotation0(),
		rb2.getPosition0(),
		rb2.getRotation0(),
		pos,
		axis1,
		axis2,
		uj->m_jointInfo);

	m_joints.push_back(uj);
}

void RigidBodyParticleModel::updateUniversalJoint(const unsigned int i)
{
	UniversalJoint &uj = *(UniversalJoint*)m_joints[i];
	RigidBody &rb1 = *m_rigidBodies[uj.m_index[0]];
	RigidBody &rb2 = *m_rigidBodies[uj.m_index[1]];
	PositionBasedRigidBodyDynamics::updateRigidBodyUniversalJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		rb2.getPosition(),
		rb2.getRotation(),
		uj.m_jointInfo);
}

void RigidBodyParticleModel::addRigidBodyParticleBallJoint(const unsigned int rbIndex, const unsigned int particleIndex)
{
	ParticleData &pd = m_particleMesh.getVertexData();
	RigidBodyParticleBallJoint *bj = new RigidBodyParticleBallJoint();
	bj->m_index[0] = rbIndex;
	bj->m_index[1] = particleIndex;

	// transform in local coordinates
	RigidBody &rb = *m_rigidBodies[rbIndex];

	PositionBasedRigidBodyDynamics::initRigidBodyParticleBallJointInfo(
		rb.getPosition(),
		rb.getRotation(),
		pd.getPosition(particleIndex),
		bj->m_jointInfo);

	m_joints.push_back(bj);
}

void RigidBodyParticleModel::updateRigidBodyParticleBallJoint(const unsigned int i)
{
	ParticleData &pd = m_particleMesh.getVertexData();
	RigidBodyParticleBallJoint &bj = *(RigidBodyParticleBallJoint*)m_joints[i];
	RigidBody &rb1 = *m_rigidBodies[bj.m_index[0]];
	
	PositionBasedRigidBodyDynamics::updateRigidBodyParticleBallJointInfo(
		rb1.getPosition(),
		rb1.getRotation(),
		pd.getPosition(bj.m_index[1]),
		bj.m_jointInfo);
}

void RigidBodyParticleModel::cleanupModel()
{
	m_particleMesh.release();
}

void RigidBodyParticleModel::updateMeshNormals()
{
	m_particleMesh.updateNormals();
	m_particleMesh.updateVertexNormals();
}

RigidBodyParticleModel::ParticleMesh &RigidBodyParticleModel::getParticleMesh()
{
	return m_particleMesh;
}

void RigidBodyParticleModel::setGeometry(const unsigned int nPoints, Eigen::Vector3f* coords, const unsigned int nFaces, unsigned int* indices, const ParticleMesh::UVIndices& uvIndices, const ParticleMesh::UVs& uvs)
{
	m_particleMesh.release();

	m_particleMesh.initMemory(nPoints, nFaces * 2, nFaces);

	ParticleData& pd = m_particleMesh.getVertexData();
	for (unsigned int i = 0; i < nPoints; i++)
	{
		m_particleMesh.addVertex(coords[i]);
	}
	for (unsigned int i = 0; i < nFaces; i++)
	{
		m_particleMesh.addFace(&indices[3 * i]);
	}
	m_particleMesh.copyUVs(uvIndices, uvs);
	m_particleMesh.buildNeighbors();

	// Update normals
	m_particleMesh.updateNormals();
	m_particleMesh.updateVertexNormals();
}


void RigidBodyParticleModel::initConstraints()
{
	initTriangleConstraints();
	initBendingConstraints();
}

void RigidBodyParticleModel::initTriangleConstraints()
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
		PositionBasedDynamics::initStrainTriangleInvRestMat(y1, y2, y3, tc.invRestMat_SBD);
		PositionBasedDynamics::initFEMTriangleInvRestMat(x1, x2, x3, tc.triangleArea, tc.invRestMat_FEM);
		m_triangleConstraints.push_back(tc);
	}
}

void RigidBodyParticleModel::initBendingConstraints()
{
	m_bendingConstraints.clear();
	unsigned int nEdges = getParticleMesh().numEdges();
	const RigidBodyParticleModel::ParticleMesh::Edge *edges = getParticleMesh().getEdges().data();
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
				PositionBasedDynamics::initQuadraticBendingMat(pd.getPosition0(bc.vertex1), pd.getPosition0(bc.vertex2), pd.getPosition0(bc.vertex3), pd.getPosition0(bc.vertex4), bc.Q);
				m_bendingConstraints.push_back(bc);
			}
		}
	}
}

RigidBodyParticleModel::BendingConstraintVector & RigidBodyParticleModel::getBendingConstraints()
{
	return m_bendingConstraints;
}

RigidBodyParticleModel::TriangleConstraintVector & RigidBodyParticleModel::getTriangleConstraints()
{
	return m_triangleConstraints;
}