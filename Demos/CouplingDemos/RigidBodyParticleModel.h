#ifndef __RIGIDBODYPARTICLEMODEL_H__
#define __RIGIDBODYPARTICLEMODEL_H__

#include "Demos/Utils/Config.h"
#include <vector>
#include "Demos/Utils/RigidBody.h"
#include "Demos/Utils/IndexedFaceMesh.h"
#include "Demos/Utils/ParticleData.h"
#include <Eigen/StdVector>

namespace PBD 
{	
	class RigidBodyParticleModel
	{
		public:
			struct Joint
			{
				/** indices of the linked rigid bodies */
				unsigned int m_index[2];

				virtual int &getTypeId() const = 0;
			};

			struct BallJoint : public Joint
			{
				Eigen::Matrix<float, 3, 4> m_jointInfo;

				static int TYPE_ID;
				virtual int &getTypeId() const { return TYPE_ID; }
			};

			struct BallOnLineJoint : public Joint
			{
				Eigen::Matrix<float, 3, 10> m_jointInfo;

				static int TYPE_ID;
				virtual int &getTypeId() const { return TYPE_ID; }
			};

			struct HingeJoint : public Joint
			{
				Eigen::Matrix<float, 3, 12> m_jointInfo;

				static int TYPE_ID;
				virtual int &getTypeId() const { return TYPE_ID; }
			};

			struct UniversalJoint : public Joint
			{
				Eigen::Matrix<float, 3, 8> m_jointInfo;

				static int TYPE_ID;
				virtual int &getTypeId() const { return TYPE_ID; }
			};

			struct RigidBodyParticleBallJoint : public Joint
			{
				Eigen::Matrix<float, 3, 2> m_jointInfo;

				static int TYPE_ID;
				virtual int &getTypeId() const { return TYPE_ID; }
			};

			struct TriangleConstraint
			{
				float triangleArea;
				Eigen::Matrix2f invRestMat_SBD;
				Eigen::Matrix2f invRestMat_FEM;
			};

			struct BendingConstraint
			{
				unsigned int vertex1;
				unsigned int vertex2;
				unsigned int vertex3;
				unsigned int vertex4;
				float restAngle;
				Eigen::Matrix4f Q;
			};


			RigidBodyParticleModel();
			virtual ~RigidBodyParticleModel();

			typedef std::vector<Joint*, Eigen::aligned_allocator<Joint*> > JointVector;
			typedef std::vector<RigidBody*, Eigen::aligned_allocator<RigidBody*> > RigidBodyVector;
			typedef IndexedFaceMesh<ParticleData> ParticleMesh;
			typedef std::vector<BendingConstraint, Eigen::aligned_allocator<BendingConstraint> > BendingConstraintVector;
			typedef std::vector<TriangleConstraint, Eigen::aligned_allocator<TriangleConstraint> > TriangleConstraintVector;


		protected:
			RigidBodyVector m_rigidBodies;
			JointVector m_joints;

			/** Face mesh of particles which represents the simulation model */
			ParticleMesh m_particleMesh;
			BendingConstraintVector m_bendingConstraints;
			TriangleConstraintVector m_triangleConstraints;
			float m_stiffness;
			float m_bendingStiffness;
			float m_xxStiffness;
			float m_yyStiffness;
			float m_xyStiffness;
			float m_xyPoissonRatio;
			float m_yxPoissonRatio;
			bool m_normalizeStretch;
			bool m_normalizeShear;

			void initBendingConstraints();
			void initTriangleConstraints();

	public:
			virtual void reset();

			RigidBodyVector &getRigidBodies();
			JointVector &getJoints();

			void updateJoints();

			void addBallJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos);
			void updateBallJoint(const unsigned int i);

			void addBallOnLineJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &dir);
			void updateBallOnLineJoint(const unsigned int i);

			void addHingeJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
			void updateHingeJoint(const unsigned int i);

			void addUniversalJoint(const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2);
			void updateUniversalJoint(const unsigned int i);

			void addRigidBodyParticleBallJoint(const unsigned int rbIndex, const unsigned int particleIndex);
			void updateRigidBodyParticleBallJoint(const unsigned int i);

			ParticleMesh &getParticleMesh();
			void cleanupModel();

			void initConstraints();

			float getStiffness() const { return m_stiffness; }
			void setStiffness(float val) { m_stiffness = val; }
			float getBendingStiffness() const { return m_bendingStiffness; }
			void setBendingStiffness(float val) { m_bendingStiffness = val; }
			float getXXStiffness() const { return m_xxStiffness; }
			void setXXStiffness(float val) { m_xxStiffness = val; }
			float getYYStiffness() const { return m_yyStiffness; }
			void setYYStiffness(float val) { m_yyStiffness = val; }
			float getXYStiffness() const { return m_xyStiffness; }
			void setXYStiffness(float val) { m_xyStiffness = val; }
			bool getNormalizeStretch() const { return m_normalizeStretch; }
			void setNormalizeStretch(bool val) { m_normalizeStretch = val; }
			bool getNormalizeShear() const { return m_normalizeShear; }
			void setNormalizeShear(bool val) { m_normalizeShear = val; }
			float getXYPoissonRatio() const { return m_xyPoissonRatio; }
			void setXYPoissonRatio(float val) { m_xyPoissonRatio = val; }
			float getYXPoissonRatio() const { return m_yxPoissonRatio; }
			void setYXPoissonRatio(float val) { m_yxPoissonRatio = val; }


			void setGeometry(const unsigned int nPoints, Eigen::Vector3f* coords, const unsigned int nFaces, unsigned int* indices, const ParticleMesh::UVIndices& uvIndices, const ParticleMesh::UVs& uvs);
			void updateMeshNormals();

			BendingConstraintVector &getBendingConstraints();
			TriangleConstraintVector &getTriangleConstraints();
	};
}

#endif