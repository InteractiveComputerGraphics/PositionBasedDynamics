#include "SimulationModel.h"
#include "PositionBasedDynamics/PositionBasedRigidBodyDynamics.h"
#include "Constraints.h"

using namespace PBD;

SimulationModel::SimulationModel()
{
	m_cloth_stiffness = 1.0;
	m_cloth_bendingStiffness = 0.01;
	m_cloth_xxStiffness = 1.0;
	m_cloth_yyStiffness = 1.0;
	m_cloth_xyStiffness = 1.0;
	m_cloth_xyPoissonRatio = 0.3;
	m_cloth_yxPoissonRatio = 0.3;
	m_cloth_normalizeShear = false;
	m_cloth_normalizeStretch = false;

	m_solid_stiffness = 1.0;
	m_solid_poissonRatio = 0.3;
	m_solid_normalizeShear = false;
	m_solid_normalizeStretch = false;

	m_contactStiffnessRigidBody = 1.0;
	m_contactStiffnessParticleRigidBody = 100.0;

	m_rod_shearingStiffness1 = 1.0;
	m_rod_shearingStiffness2 = 1.0;
	m_rod_stretchingStiffness = 1.0;
	m_rod_bendingStiffness1 = 0.5;
	m_rod_bendingStiffness2 = 0.5;
	m_rod_twistingStiffness = 0.5;

	m_groupsInitialized = false;

	m_rigidBodyContactConstraints.reserve(10000);
	m_particleRigidBodyContactConstraints.reserve(10000);
}

SimulationModel::~SimulationModel(void)
{
	cleanup();
}

void SimulationModel::cleanup()
{
	resetContacts();
	for (unsigned int i = 0; i < m_rigidBodies.size(); i++)
		delete m_rigidBodies[i];
	m_rigidBodies.clear();
	for (unsigned int i = 0; i < m_triangleModels.size(); i++)
		delete m_triangleModels[i];
	m_triangleModels.clear();
	for (unsigned int i = 0; i < m_tetModels.size(); i++)
		delete m_tetModels[i];
	m_tetModels.clear();
	for (unsigned int i = 0; i < m_lineModels.size(); i++)
		delete m_lineModels[i];
	m_lineModels.clear();
	for (unsigned int i = 0; i < m_constraints.size(); i++)
		delete m_constraints[i];
	m_constraints.clear();
	m_particles.release();
	m_orientations.release();
	m_groupsInitialized = false;
}

void SimulationModel::reset()
{
	resetContacts();

	// rigid bodies
	for (size_t i = 0; i < m_rigidBodies.size(); i++)
	{
		m_rigidBodies[i]->reset();
		m_rigidBodies[i]->getGeometry().updateMeshTransformation(m_rigidBodies[i]->getPosition(), m_rigidBodies[i]->getRotationMatrix());
	}

	// particles
	for (unsigned int i = 0; i < m_particles.size(); i++)
	{
		const Vector3r& x0 = m_particles.getPosition0(i);
		m_particles.getPosition(i) = x0;
		m_particles.getLastPosition(i) = m_particles.getPosition(i);
		m_particles.getOldPosition(i) = m_particles.getPosition(i);
		m_particles.getVelocity(i).setZero();
		m_particles.getAcceleration(i).setZero();
	}

	// orientations
	for(unsigned int i = 0; i < m_orientations.size(); i++)
	{
		const Quaternionr& q0 = m_orientations.getQuaternion0(i);
		m_orientations.getQuaternion(i) = q0;
		m_orientations.getLastQuaternion(i) = q0;
		m_orientations.getOldQuaternion(i) = q0;
		m_orientations.getVelocity(i).setZero();
		m_orientations.getAcceleration(i).setZero();
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

OrientationData & SimulationModel::getOrientations()
{
	return m_orientations;
}

SimulationModel::TriangleModelVector & SimulationModel::getTriangleModels()
{
	return m_triangleModels;
}

SimulationModel::TetModelVector & SimulationModel::getTetModels()
{
	return m_tetModels;
}

SimulationModel::LineModelVector & SimulationModel::getLineModels()
{
	return m_lineModels;
}

SimulationModel::ConstraintVector & SimulationModel::getConstraints()
{
	return m_constraints;
}

SimulationModel::RigidBodyContactConstraintVector & SimulationModel::getRigidBodyContactConstraints()
{
	return m_rigidBodyContactConstraints;
}

SimulationModel::ParticleRigidBodyContactConstraintVector & SimulationModel::getParticleRigidBodyContactConstraints()
{
	return m_particleRigidBodyContactConstraints;
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


bool SimulationModel::addBallJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos)
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

bool SimulationModel::addBallOnLineJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &dir)
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

bool SimulationModel::addHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis)
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

bool SimulationModel::addUniversalJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis1, const Vector3r &axis2)
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

bool SimulationModel::addSliderJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis)
{
	SliderJoint *joint = new SliderJoint();
	const bool res = joint->initConstraint(*this, rbIndex1, rbIndex2, pos, axis);
	if (res)
	{
		m_constraints.push_back(joint);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addTargetPositionMotorSliderJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis)
{
	TargetPositionMotorSliderJoint *joint = new TargetPositionMotorSliderJoint();
	const bool res = joint->initConstraint(*this, rbIndex1, rbIndex2, pos, axis);
	if (res)
	{
		m_constraints.push_back(joint);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addTargetVelocityMotorSliderJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis)
{
	TargetVelocityMotorSliderJoint *joint = new TargetVelocityMotorSliderJoint();
	const bool res = joint->initConstraint(*this, rbIndex1, rbIndex2, pos, axis);
	if (res)
	{
		m_constraints.push_back(joint);
		m_groupsInitialized = false;
	}
	return res;
}


bool SimulationModel::addTargetAngleMotorHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis)
{
	TargetAngleMotorHingeJoint *hj = new TargetAngleMotorHingeJoint();
	const bool res = hj->initConstraint(*this, rbIndex1, rbIndex2, pos, axis);
	if (res)
	{
		m_constraints.push_back(hj);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addTargetVelocityMotorHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis)
{
	TargetVelocityMotorHingeJoint *hj = new TargetVelocityMotorHingeJoint();
	const bool res = hj->initConstraint(*this, rbIndex1, rbIndex2, pos, axis);
	if (res)
	{
		m_constraints.push_back(hj);
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

bool SimulationModel::addRigidBodyContactConstraint(const unsigned int rbIndex1, const unsigned int rbIndex2, 
	const Vector3r &cp1, const Vector3r &cp2, 
	const Vector3r &normal, const Real dist,
	const Real restitutionCoeff, const Real frictionCoeff)
{
	RigidBodyContactConstraint &cc = m_rigidBodyContactConstraints.create();
	const bool res = cc.initConstraint(*this, rbIndex1, rbIndex2, cp1, cp2, normal, dist, restitutionCoeff, m_contactStiffnessRigidBody, frictionCoeff);
	if (!res)
		m_rigidBodyContactConstraints.pop_back();
	return res;
}

 bool SimulationModel::addParticleRigidBodyContactConstraint(const unsigned int particleIndex, const unsigned int rbIndex, 
 	const Vector3r &cp1, const Vector3r &cp2, 
 	const Vector3r &normal, const Real dist,
 	const Real restitutionCoeff, const Real frictionCoeff)
{
 	ParticleRigidBodyContactConstraint &cc = m_particleRigidBodyContactConstraints.create();
 	const bool res = cc.initConstraint(*this, particleIndex, rbIndex, cp1, cp2, normal, dist, restitutionCoeff, m_contactStiffnessParticleRigidBody, frictionCoeff);
 	if (!res)
 		m_particleRigidBodyContactConstraints.pop_back();
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

bool SimulationModel::addStretchShearConstraint(const unsigned int particle1, const unsigned int particle2, const unsigned int quaternion1)
{
	StretchShearConstraint *c = new StretchShearConstraint();
	const bool res = c->initConstraint(*this, particle1, particle2, quaternion1);
	if (res)
	{
		m_constraints.push_back(c);
		m_groupsInitialized = false;
	}
	return res;
}

bool SimulationModel::addBendTwistConstraint(const unsigned int quaternion1, const unsigned int quaternion2)
{
	BendTwistConstraint *c = new BendTwistConstraint();
	const bool res = c->initConstraint(*this, quaternion1, quaternion2);
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
	Vector3r *points,
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
	Vector3r *points,
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

void SimulationModel::addLineModel(
	const unsigned int nPoints,
	const unsigned int nQuaternions,
	Vector3r *points,
	Quaternionr *quaternions,
	unsigned int *indices,
	unsigned int *indicesQuaternions)
{
	LineModel *lineModel = new LineModel();
	m_lineModels.push_back(lineModel);

	unsigned int startIndex = m_particles.size();
	m_particles.reserve(startIndex + nPoints);

	for (unsigned int i = 0; i < nPoints; i++)
		m_particles.addVertex(points[i]);

	unsigned int startIndexOrientations = m_orientations.size();
	m_orientations.reserve(startIndexOrientations + nQuaternions);

	for (unsigned int i = 0; i < nQuaternions; i++)
		m_orientations.addQuaternion(quaternions[i]);

	lineModel->initMesh(nPoints, nQuaternions, startIndex, startIndexOrientations, indices, indicesQuaternions);
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

void SimulationModel::resetContacts()
{
	m_rigidBodyContactConstraints.reset();
	m_particleRigidBodyContactConstraints.reset();
}

