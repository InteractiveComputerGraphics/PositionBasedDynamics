#ifndef __RIGIDBODYMODEL_H__
#define __RIGIDBODYMODEL_H__

#include "Demos/Utils/Config.h"
#include <vector>
#include "Demos/Utils/RigidBody.h"
#include <Eigen/StdVector>

namespace PBD 
{	
	class RigidBodyModel
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
				/** joint points in local coordinates */
				Eigen::Matrix<float, 3, 4> m_jointInfo;

				static int TYPE_ID;
				virtual int &getTypeId() const { return TYPE_ID; }
			};

			struct BallOnLineJoint : public Joint
			{
				/** joint points in local coordinates */
				Eigen::Matrix<float, 3, 10> m_jointInfo;

				static int TYPE_ID;
				virtual int &getTypeId() const { return TYPE_ID; }
			};

			struct HingeJoint : public Joint
			{
				/** joint points in local coordinates */
				Eigen::Matrix<float, 3, 12> m_jointInfo;

				static int TYPE_ID;
				virtual int &getTypeId() const { return TYPE_ID; }
			};

			struct UniversalJoint : public Joint
			{
				/** joint points in local coordinates */
				Eigen::Matrix<float, 3, 8> m_jointInfo;

				static int TYPE_ID;
				virtual int &getTypeId() const { return TYPE_ID; }
			};


			RigidBodyModel();
			virtual ~RigidBodyModel();

			typedef std::vector<Joint*, Eigen::aligned_allocator<Joint*> > JointVector;
			typedef std::vector<RigidBody*, Eigen::aligned_allocator<RigidBody*> > RigidBodyVector;

		protected:
			RigidBodyVector m_rigidBodies;
			JointVector m_joints;

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
	};
}

#endif