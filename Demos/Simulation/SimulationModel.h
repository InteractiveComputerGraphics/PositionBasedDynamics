#ifndef __SIMULATIONMODEL_H__
#define __SIMULATIONMODEL_H__

#include "Demos/Utils/Config.h"
#include <vector>
#include "Demos/Simulation/RigidBody.h"
#include "Demos/Simulation/ParticleData.h"
#include <Eigen/StdVector>
#include "TriangleModel.h"
#include "TetModel.h"

namespace PBD 
{	
	class Constraint;

	class SimulationModel
	{
		public:
			SimulationModel();
			virtual ~SimulationModel();

			typedef std::vector<Constraint*, Eigen::aligned_allocator<Constraint*> > ConstraintVector;
			typedef std::vector<RigidBody*, Eigen::aligned_allocator<RigidBody*> > RigidBodyVector;
			typedef std::vector<TriangleModel*, Eigen::aligned_allocator<TriangleModel*> > TriangleModelVector;
			typedef std::vector<TetModel*, Eigen::aligned_allocator<TetModel*> > TetModelVector;
			typedef std::vector<unsigned int> ConstraintGroup;
			typedef std::vector<ConstraintGroup> ConstraintGroupVector;


		protected:
			RigidBodyVector m_rigidBodies;
			TriangleModelVector m_triangleModels;
			TetModelVector m_tetModels;
			ParticleData m_particles;
			ConstraintVector m_constraints;
			ConstraintGroupVector m_constraintGroups;

			float m_cloth_stiffness;
			float m_cloth_bendingStiffness;
			float m_cloth_xxStiffness;
			float m_cloth_yyStiffness;
			float m_cloth_xyStiffness;
			float m_cloth_xyPoissonRatio;
			float m_cloth_yxPoissonRatio;
			bool  m_cloth_normalizeStretch;
			bool  m_cloth_normalizeShear;

			float m_solid_stiffness;
			float m_solid_poissonRatio;
			bool m_solid_normalizeStretch;
			bool m_solid_normalizeShear;


		public:
			void reset();
			void cleanup();

			RigidBodyVector &getRigidBodies();
			ParticleData &getParticles();
			TriangleModelVector &getTriangleModels();
			TetModelVector &getTetModels();
			ConstraintVector &getConstraints();
			ConstraintGroupVector &getConstraintGroups();
			bool m_groupsInitialized;

			void addTriangleModel(
				const unsigned int nPoints,
				const unsigned int nFaces,
				Eigen::Vector3f *points,
				unsigned int* indices,
				const TriangleModel::ParticleMesh::UVIndices& uvIndices,
				const TriangleModel::ParticleMesh::UVs& uvs);

			void addTetModel(
				const unsigned int nPoints, 
				const unsigned int nTets, 
				Eigen::Vector3f *points,
				unsigned int* indices);

			void updateConstraints();
			void initConstraintGroups();

			bool addBallJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos);
			bool addBallOnLineJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &dir);
			bool addHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
			bool addTargetAngleMotorHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
			bool addTargetVelocityMotorHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
			bool addUniversalJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2);
			bool addRigidBodyParticleBallJoint(const unsigned int rbIndex, const unsigned int particleIndex);

			bool addDistanceConstraint(const unsigned int particle1, const unsigned int particle2);
			bool addDihedralConstraint(	const unsigned int particle1, const unsigned int particle2,
										const unsigned int particle3, const unsigned int particle4);
			bool addIsometricBendingConstraint(const unsigned int particle1, const unsigned int particle2,
										const unsigned int particle3, const unsigned int particle4);
			bool addFEMTriangleConstraint(const unsigned int particle1, const unsigned int particle2, const unsigned int particle3);
			bool addStrainTriangleConstraint(const unsigned int particle1, const unsigned int particle2, const unsigned int particle3);
			bool addVolumeConstraint(const unsigned int particle1, const unsigned int particle2,
									const unsigned int particle3, const unsigned int particle4);
			bool addFEMTetConstraint(const unsigned int particle1, const unsigned int particle2,
									const unsigned int particle3, const unsigned int particle4);
			bool addStrainTetConstraint(const unsigned int particle1, const unsigned int particle2,
									const unsigned int particle3, const unsigned int particle4);
			bool addShapeMatchingConstraint(const unsigned int numberOfParticles, const unsigned int particleIndices[], const unsigned int numClusters[]);


			float getClothStiffness() const { return m_cloth_stiffness; }
			void setClothStiffness(float val) { m_cloth_stiffness = val; }
			float getClothBendingStiffness() const { return m_cloth_bendingStiffness; }
			void setClothBendingStiffness(float val) { m_cloth_bendingStiffness = val; }
			float getClothXXStiffness() const { return m_cloth_xxStiffness; }
			void setClothXXStiffness(float val) { m_cloth_xxStiffness = val; }
			float getClothYYStiffness() const { return m_cloth_yyStiffness; }
			void setClothYYStiffness(float val) { m_cloth_yyStiffness = val; }
			float getClothXYStiffness() const { return m_cloth_xyStiffness; }
			void setClothXYStiffness(float val) { m_cloth_xyStiffness = val; }
			bool getClothNormalizeStretch() const { return m_cloth_normalizeStretch; }
			void setClothNormalizeStretch(bool val) { m_cloth_normalizeStretch = val; }
			bool getClothNormalizeShear() const { return m_cloth_normalizeShear; }
			void setClothNormalizeShear(bool val) { m_cloth_normalizeShear = val; }
			float getClothXYPoissonRatio() const { return m_cloth_xyPoissonRatio; }
			void setClothXYPoissonRatio(float val) { m_cloth_xyPoissonRatio = val; }
			float getClothYXPoissonRatio() const { return m_cloth_yxPoissonRatio; }
			void setClothYXPoissonRatio(float val) { m_cloth_yxPoissonRatio = val; }

			float getSolidStiffness() const { return m_solid_stiffness; }
			void setSolidStiffness(float val) { m_solid_stiffness = val; }
			float getSolidPoissonRatio() { return m_solid_poissonRatio; }
			void setSolidPoissonRatio(const float val) { m_solid_poissonRatio = val; }
			bool getSolidNormalizeStretch() const { return m_solid_normalizeStretch; }
			void setSolidNormalizeStretch(bool val) { m_solid_normalizeStretch = val; }
			bool getSolidNormalizeShear() const { return m_solid_normalizeShear; }
			void setSolidNormalizeShear(bool val) { m_solid_normalizeShear = val; }

	};
}

#endif