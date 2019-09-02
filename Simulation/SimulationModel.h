#ifndef __SIMULATIONMODEL_H__
#define __SIMULATIONMODEL_H__

#include "Common/Common.h"
#include <vector>
#include "Simulation/RigidBody.h"
#include "Simulation/ParticleData.h"
#include "TriangleModel.h"
#include "TetModel.h"
#include "LineModel.h"
#include "ParameterObject.h"

namespace PBD 
{	
	class Constraint;

	class SimulationModel : public GenParam::ParameterObject
	{
		public:
			static int CLOTH_STIFFNESS;
			static int CLOTH_BENDING_STIFFNESS;
			static int CLOTH_STIFFNESS_XX;
			static int CLOTH_STIFFNESS_YY;
			static int CLOTH_STIFFNESS_XY;
			static int CLOTH_POISSON_RATIO_XY;
			static int CLOTH_POISSON_RATIO_YX;
			static int CLOTH_NORMALIZE_STRETCH;
			static int CLOTH_NORMALIZE_SHEAR;

			static int SOLID_STIFFNESS;
			static int SOLID_POISSON_RATIO;
			static int SOLID_NORMALIZE_STRETCH;
			static int SOLID_NORMALIZE_SHEAR;

		public:
			SimulationModel();
			virtual ~SimulationModel();

			void init();

			typedef std::vector<Constraint*> ConstraintVector;
			typedef std::vector<RigidBodyContactConstraint> RigidBodyContactConstraintVector;
			typedef std::vector<ParticleRigidBodyContactConstraint> ParticleRigidBodyContactConstraintVector;
			typedef std::vector<ParticleTetContactConstraint> ParticleSolidContactConstraintVector;
			typedef std::vector<RigidBody*> RigidBodyVector;
			typedef std::vector<TriangleModel*> TriangleModelVector;
			typedef std::vector<TetModel*> TetModelVector;
			typedef std::vector<LineModel*> LineModelVector;
			typedef std::vector<unsigned int> ConstraintGroup;
			typedef std::vector<ConstraintGroup> ConstraintGroupVector;


		protected:
			RigidBodyVector m_rigidBodies;
			TriangleModelVector m_triangleModels;
			TetModelVector m_tetModels;
			LineModelVector m_lineModels;
			ParticleData m_particles;
			OrientationData m_orientations;
			ConstraintVector m_constraints;
			RigidBodyContactConstraintVector m_rigidBodyContactConstraints;
			ParticleRigidBodyContactConstraintVector m_particleRigidBodyContactConstraints;
			ParticleSolidContactConstraintVector m_particleSolidContactConstraints;
			ConstraintGroupVector m_constraintGroups;

			Real m_cloth_stiffness;
			Real m_cloth_bendingStiffness;
			Real m_cloth_xxStiffness;
			Real m_cloth_yyStiffness;
			Real m_cloth_xyStiffness;
			Real m_cloth_xyPoissonRatio;
			Real m_cloth_yxPoissonRatio;
			bool  m_cloth_normalizeStretch;
			bool  m_cloth_normalizeShear;

			Real m_solid_stiffness;
			Real m_solid_poissonRatio;
			bool m_solid_normalizeStretch;
			bool m_solid_normalizeShear;

			Real m_contactStiffnessRigidBody;
			Real m_contactStiffnessParticleRigidBody;

			Real m_rod_stretchingStiffness;
			Real m_rod_shearingStiffness1;
			Real m_rod_shearingStiffness2;
			Real m_rod_bendingStiffness1;
			Real m_rod_bendingStiffness2;
			Real m_rod_twistingStiffness;

			virtual void initParameters();

	public:
			void reset();			
			void cleanup();

			RigidBodyVector &getRigidBodies();
			ParticleData &getParticles();
			OrientationData &getOrientations();
			TriangleModelVector &getTriangleModels();
			TetModelVector &getTetModels();
			LineModelVector &getLineModels();
			ConstraintVector &getConstraints();
			RigidBodyContactConstraintVector &getRigidBodyContactConstraints();
			ParticleRigidBodyContactConstraintVector &getParticleRigidBodyContactConstraints();
			ParticleSolidContactConstraintVector &getParticleSolidContactConstraints();
			ConstraintGroupVector &getConstraintGroups();
			bool m_groupsInitialized;

			void resetContacts();

			void addTriangleModel(
				const unsigned int nPoints,
				const unsigned int nFaces,
				Vector3r *points,
				unsigned int* indices,
				const TriangleModel::ParticleMesh::UVIndices& uvIndices,
				const TriangleModel::ParticleMesh::UVs& uvs);

			void addTetModel(
				const unsigned int nPoints, 
				const unsigned int nTets, 
				Vector3r *points,
				unsigned int* indices);

			void addLineModel(
				const unsigned int nPoints,
				const unsigned int nQuaternions,
				Vector3r *points,
				Quaternionr *quaternions,
				unsigned int *indices,
				unsigned int *indicesQuaternions);

			void updateConstraints();
			void initConstraintGroups();

			bool addBallJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos);
			bool addBallOnLineJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &dir);
			bool addHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
			bool addTargetAngleMotorHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
			bool addTargetVelocityMotorHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
			bool addUniversalJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis1, const Vector3r &axis2);
			bool addSliderJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &axis);
			bool addTargetPositionMotorSliderJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &axis);
			bool addTargetVelocityMotorSliderJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &axis);
			bool addRigidBodyParticleBallJoint(const unsigned int rbIndex, const unsigned int particleIndex);
			bool addRigidBodySpring(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos1, const Vector3r &pos2, const Real stiffness);
			bool addDistanceJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos1, const Vector3r &pos2);
			bool addDamperJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &axis, const Real stiffness);
			bool addRigidBodyContactConstraint(const unsigned int rbIndex1, const unsigned int rbIndex2, 
					const Vector3r &cp1, const Vector3r &cp2,	
					const Vector3r &normal, const Real dist, 
					const Real restitutionCoeff, const Real frictionCoeff);
			bool addParticleRigidBodyContactConstraint(const unsigned int particleIndex, const unsigned int rbIndex, 
					const Vector3r &cp1, const Vector3r &cp2, 
					const Vector3r &normal, const Real dist,
					const Real restitutionCoeff, const Real frictionCoeff);

			bool addParticleSolidContactConstraint(const unsigned int particleIndex, const unsigned int solidIndex,
				const unsigned int tetIndex, const Vector3r &bary,
				const Vector3r &cp1, const Vector3r &cp2,
				const Vector3r &normal, const Real dist,
				const Real restitutionCoeff, const Real frictionCoeff);

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
			bool addStretchShearConstraint(const unsigned int particle1, const unsigned int particle2, const unsigned int quaternion1);
			bool addBendTwistConstraint(const unsigned int quaternion1, const unsigned int quaternion2);
			bool addStretchBendingTwistingConstraint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Real averageRadius, const Real averageSegmentLength, const Real youngsModulus, const Real torsionModulus);
			bool addDirectPositionBasedSolverForStiffRodsConstraint(const std::vector<std::pair<unsigned int, unsigned int>> & jointSegmentIndices, const std::vector<Vector3r> &jointPositions, const std::vector<Real> &averageRadii, const std::vector<Real> &averageSegmentLengths, const std::vector<Real> &youngsModuli, const std::vector<Real> &torsionModuli);

			Real getContactStiffnessRigidBody() const { return m_contactStiffnessRigidBody; }
			void setContactStiffnessRigidBody(Real val) { m_contactStiffnessRigidBody = val; }
			Real getContactStiffnessParticleRigidBody() const { return m_contactStiffnessParticleRigidBody; }
			void setContactStiffnessParticleRigidBody(Real val) { m_contactStiffnessParticleRigidBody = val; }
		
		    Real getRodStretchingStiffness() const { return m_rod_stretchingStiffness;  }
			void setRodStretchingStiffness(Real val) { m_rod_stretchingStiffness = val; }
			Real getRodShearingStiffness1() const { return m_rod_shearingStiffness1; }
			void setRodShearingStiffness1(Real val) { m_rod_shearingStiffness1 = val; }
			Real getRodShearingStiffness2() const { return m_rod_shearingStiffness2; }
			void setRodShearingStiffness2(Real val) { m_rod_shearingStiffness2 = val; }
			Real getRodBendingStiffness1() const { return m_rod_bendingStiffness1; }
			void setRodBendingStiffness1(Real val) { m_rod_bendingStiffness1 = val; }
			Real getRodBendingStiffness2() const { return m_rod_bendingStiffness2; }
			void setRodBendingStiffness2(Real val) { m_rod_bendingStiffness2 = val; }
			Real getRodTwistingStiffness() const { return m_rod_twistingStiffness; }
			void setRodTwistingStiffness(Real val) { m_rod_twistingStiffness = val; }
	};
}

#endif