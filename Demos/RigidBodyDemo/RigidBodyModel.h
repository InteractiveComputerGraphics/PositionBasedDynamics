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
			struct BallJoint
			{
				/** indices of the linked rigid bodies */
				unsigned int m_index[2];
				/** joint points in world space */
				Eigen::Vector3f m_points[2];
				/** joint points in local coordinates */
				Eigen::Vector3f m_localPoints[2];
			};

			RigidBodyModel();
			virtual ~RigidBodyModel();

			typedef std::vector<BallJoint, Eigen::aligned_allocator<BallJoint> > BallJointVector;
			typedef std::vector<RigidBody, Eigen::aligned_allocator<RigidBody> > RigidBodyVector;

		protected:
			RigidBodyVector m_rigidBodies;
			BallJointVector m_ballJoints;

	public:
			virtual void reset();

			RigidBodyVector &getRigidBodies();
			BallJointVector &getBallJoints();
			
			void setBallJoint(const unsigned int i, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos);
			void updateBallJoints();
			void updateBallJoint(const unsigned int i);
	};
}

#endif