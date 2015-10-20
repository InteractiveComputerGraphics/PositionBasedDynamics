#include "SimulationModel.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"
#include "Constraints.h"

using namespace PBD;

SimulationModel::SimulationModel()
{
	m_cloth_stiffness = 1.0f;
	m_cloth_bendingStiffness = 0.01f;
	m_cloth_xxStiffness = 1.0f;
	m_cloth_yyStiffness = 1.0f;
	m_cloth_xyStiffness = 1.0f;
	m_cloth_xyPoissonRatio = 0.3f;
	m_cloth_yxPoissonRatio = 0.3f;
	m_cloth_normalizeShear = false;
	m_cloth_normalizeStretch = false;

	m_solid_stiffness = 1.0f;
	m_solid_poissonRatio = 0.3f;
	m_solid_normalizeShear = false;
	m_solid_normalizeStretch = false;

	m_groupsInitialized = false;
}

SimulationModel::~SimulationModel(void)
{
	cleanup();
}

void SimulationModel::cleanup()
{
	for (unsigned int i = 0; i < m_rigidBodies.size(); i++)
		delete m_rigidBodies[i];
	m_rigidBodies.clear();
	for (unsigned int i = 0; i < m_triangleModels.size(); i++)
		delete m_triangleModels[i];
	m_triangleModels.clear();
	for (unsigned int i = 0; i < m_constraints.size(); i++)
		delete m_constraints[i];
	m_constraints.clear();
	m_particles.release();
}

void SimulationModel::reset()
{
	// rigid bodies
	for (size_t i = 0; i < m_rigidBodies.size(); i++)
		m_rigidBodies[i]->reset();

	// particles
	for (unsigned int i = 0; i < m_particles.size(); i++)
	{
		const Eigen::Vector3f& x0 = m_particles.getPosition0(i);
		m_particles.getPosition(i) = x0;
		m_particles.getLastPosition(i) = m_particles.getPosition(i);
		m_particles.getOldPosition(i) = m_particles.getPosition(i);
		m_particles.getVelocity(i).setZero();
		m_particles.getAcceleration(i).setZero();
	}

	updateConstraints();
}

SimulationModel::RigidBodyVector & SimulationModel::getRigidBodies()
{
	return m_rigidBodies;
}

ParticleData & SimulationModel::getParticles()
{
	return m_particles;
}

SimulationModel::TriangleModelVector & SimulationModel::getTriangleModels()
{
	return m_triangleModels;
}

SimulationModel::TetModelVector & SimulationModel::getTetModels()
{
	return m_tetModels;
}

SimulationModel::ConstraintVector & SimulationModel::getConstraints()
{
	return m_constraints;
}

SimulationModel::ConstraintGroupVector & SimulationModel::getConstraintGroups()
{
	return m_constraintGroups;
}

void SimulationModel::updateConstraints()
{
	for (unsigned int i = 0; i < m_constraints.size(); i++)
		m_constraints[i]->updateConstraint(*this);
}


bool SimulationModel::addBallJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos)
{
	BallJoint *bj = new BallJoint();
	const bool res = bj->initConstraint(*this, rbIndex1, rbIndex2, pos);
	if (res)
	{
		m_constraints.push_back(bj);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addBallOnLineJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &dir)
{
	BallOnLineJoint *bj = new BallOnLineJoint();
	const bool res = bj->initConstraint(*this, rbIndex1, rbIndex2, pos, dir);
	if (res)
	{
		m_constraints.push_back(bj);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis)
{
	HingeJoint *hj = new HingeJoint();
	const bool res = hj->initConstraint(*this, rbIndex1, rbIndex2, pos, axis);
	if (res)
	{
		m_constraints.push_back(hj);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addUniversalJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2)
{
	UniversalJoint *uj = new UniversalJoint();
	const bool res = uj->initConstraint(*this, rbIndex1, rbIndex2, pos, axis1, axis2);
	if (res)
	{
		m_constraints.push_back(uj);
		m_groupsInitialized = false;
	}
	return res;
}


bool SimulationModel::addRigidBodyParticleBallJoint(const unsigned int rbIndex, const unsigned int particleIndex)
{
	RigidBodyParticleBallJoint *bj = new RigidBodyParticleBallJoint();
	const bool res = bj->initConstraint(*this, rbIndex, particleIndex);
	if (res)
	{
		m_constraints.push_back(bj);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addDistanceConstraint(const unsigned int particle1, const unsigned int particle2)
{
	DistanceConstraint *c = new DistanceConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2);
	if (res)
	{
		m_constraints.push_back(c);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addDihedralConstraint(const unsigned int particle1, const unsigned int particle2, 
											const unsigned int particle3, const unsigned int particle4)
{
	DihedralConstraint *c = new DihedralConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2, particle3, particle4);
	if (res)
	{
		m_constraints.push_back(c);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addIsometricBendingConstraint(const unsigned int particle1, const unsigned int particle2,
													const unsigned int particle3, const unsigned int particle4)
{
	IsometricBendingConstraint *c = new IsometricBendingConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2, particle3, particle4);
	if (res)
	{
		m_constraints.push_back(c);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addFEMTriangleConstraint(const unsigned int particle1, const unsigned int particle2,
			const unsigned int particle3)
{
	FEMTriangleConstraint *c = new FEMTriangleConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2, particle3);
	if (res)
	{
		m_constraints.push_back(c);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addStrainTriangleConstraint(const unsigned int particle1, const unsigned int particle2,
	const unsigned int particle3)
{
	StrainTriangleConstraint *c = new StrainTriangleConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2, particle3);
	if (res)
	{
		m_constraints.push_back(c);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addVolumeConstraint(const unsigned int particle1, const unsigned int particle2,
										const unsigned int particle3, const unsigned int particle4)
{
	VolumeConstraint *c = new VolumeConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2, particle3, particle4);
	if (res)
	{
		m_constraints.push_back(c);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addFEMTetConstraint(const unsigned int particle1, const unsigned int particle2,
										const unsigned int particle3, const unsigned int particle4)
{
	FEMTetConstraint *c = new FEMTetConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2, particle3, particle4);
	if (res)
	{
		m_constraints.push_back(c);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addStrainTetConstraint(const unsigned int particle1, const unsigned int particle2,
										const unsigned int particle3, const unsigned int particle4)
{
	StrainTetConstraint *c = new StrainTetConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2, particle3, particle4);
	if (res)
	{
		m_constraints.push_back(c);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addShapeMatchingConstraint(const unsigned int numberOfParticles, const unsigned int particleIndices[], const unsigned int numClusters[])
{
	ShapeMatchingConstraint *c = new ShapeMatchingConstraint(numberOfParticles);
	const bool res = c->initConstraint(*this, particleIndices, numClusters);
	if (res)
	{
		m_constraints.push_back(c);
		m_groupsInitialized = false;
	}
	return res;
}


void SimulationModel::addTriangleModel(
	const unsigned int nPoints, 
	const unsigned int nFaces, 
	Eigen::Vector3f *points,
	unsigned int* indices, 
	const TriangleModel::ParticleMesh::UVIndices& uvIndices, 
	const TriangleModel::ParticleMesh::UVs& uvs)
{
	TriangleModel *triModel = new TriangleModel();
	m_triangleModels.push_back(triModel);

	unsigned int startIndex = m_particles.size();
	m_particles.reserve(startIndex + nPoints);

	for (unsigned int i = 0; i < nPoints; i++)
		m_particles.addVertex(points[i]);

	triModel->initMesh(nPoints, nFaces, startIndex, indices, uvIndices, uvs);

	// Update normals
	triModel->updateMeshNormals(m_particles);
}

void SimulationModel::addTetModel(
	const unsigned int nPoints,
	const unsigned int nTets,
	Eigen::Vector3f *points,
	unsigned int* indices)
{
	TetModel *tetModel = new TetModel();
	m_tetModels.push_back(tetModel);

	unsigned int startIndex = m_particles.size();
	m_particles.reserve(startIndex + nPoints);

	for (unsigned int i = 0; i < nPoints; i++)
		m_particles.addVertex(points[i]);

	tetModel->initMesh(nPoints, nTets, startIndex, indices);
}



void SimulationModel::initConstraintGroups()
{
	if (m_groupsInitialized)
		return;

	const unsigned int numConstraints = (unsigned int) m_constraints.size();
	const unsigned int numParticles = (unsigned int) m_particles.size();
	const unsigned int numRigidBodies = (unsigned int) m_rigidBodies.size();
	const unsigned int numBodies = numParticles + numRigidBodies;
	m_constraintGroups.clear();

	// Maps in which group a particle is or 0 if not yet mapped
	std::vector<unsigned char*> mapping;

	for (unsigned int i = 0; i < numConstraints; i++)
	{
		Constraint *constraint = m_constraints[i];

		bool addToNewGroup = true;
		for (unsigned int j = 0; j < m_constraintGroups.size(); j++)
		{
			bool addToThisGroup = true;

			for (unsigned int k = 0; k < constraint->m_numberOfBodies; k++)
			{
				if (mapping[j][constraint->m_bodies[k]] != 0)
				{
					addToThisGroup = false;
					break;
				}
			}

			if (addToThisGroup)
			{
				m_constraintGroups[j].push_back(i);

				for (unsigned int k = 0; k < constraint->m_numberOfBodies; k++)
					mapping[j][constraint->m_bodies[k]] = 1;

				addToNewGroup = false;
				break;
			}
		}
		if (addToNewGroup)
		{
			mapping.push_back(new unsigned char[numBodies]);
			memset(mapping[mapping.size() - 1], 0, sizeof(unsigned char)*numBodies);
			m_constraintGroups.resize(m_constraintGroups.size() + 1);
			m_constraintGroups[m_constraintGroups.size()-1].push_back(i);
			for (unsigned int k = 0; k < constraint->m_numberOfBodies; k++)
				mapping[m_constraintGroups.size() - 1][constraint->m_bodies[k]] = 1;
		}
	}

	for (unsigned int i = 0; i < mapping.size(); i++)
	{
		delete[] mapping[i];
	}
	mapping.clear();

	m_groupsInitialized = true;
}