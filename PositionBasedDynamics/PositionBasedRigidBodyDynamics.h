#ifndef POSITION_BASED_RIGID_BODY_DYNAMICS_H
#define POSITION_BASED_RIGID_BODY_DYNAMICS_H

#include <Eigen/Dense>

// ------------------------------------------------------------------------------------
namespace PBD
{
	class PositionBasedRigidBodyDynamics
	{
	// -------------- Position Based Rigid Body Dynamics  -----------------------------------------------------
	private:
		static void computeMatrixK(
			const Eigen::Vector3f &connector,
			const float mass,
			const Eigen::Vector3f &x,
			const Eigen::Matrix3f &inertiaInverseW,
			Eigen::Matrix3f &K);

		static void computeMatrixK(
			const Eigen::Vector3f &connector0,
			const Eigen::Vector3f &connector1,
			const float mass,
			const Eigen::Vector3f &x,
			const Eigen::Matrix3f &inertiaInverseW,
			Eigen::Matrix3f &K);

	public:
		/** Initialize ball joint and return info which is required by the solver step.
		* 
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param ballJointPosition position of ball joint
		* @param ballJointInfo Stores the local and global positions of the connector points. 
		* The first two columns store the local connectors in body 0 and 1, respectively, while
		* the last two columns contain the global connector positions which have to be
		* updated in each simulation step by calling updateRigidBodyBallJointInfo().
		*/
		static bool initRigidBodyBallJointInfo(			
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Vector3f &ballJointPosition,		// position of balljoint
			Eigen::Matrix<float, 3, 4> &ballJointInfo
			);

		/** Update ball joint info which is required by the solver step.
		* The ball joint info must be generated in the initialization process of the model
		* by calling the function initRigidBodyBallJointInfo().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param ballJointInfo ball joint information which should be updated
		*/
		static bool updateRigidBodyBallJointInfo(
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 4> &ballJointInfo
			);

		/** Perform a solver step for a ball joint which links two rigid bodies.
		* A ball joint removes three translational degrees of freedom between the bodies.
		* The ball joint info must be generated in the initialization process of the model
		* by calling the function initRigidBodyBallJointInfo() and updated each time the bodies 
		* change their state by updateRigidBodyBallJointInfo().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html balljoint.jpg "ball joint"
		* \image latex balljoint.jpg "ball joint" width=0.5\textwidth
		*
		* @param mass0 mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param mass1 mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param ballJointInfo Ball joint information which is required by the solver. This 
		* information must be generated in the beginning by calling initRigidBodyBallJointInfo()
		* and updated each time the bodies change their state by updateRigidBodyBallJointInfo().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solveRigidBodyBallJoint(
			const float mass0,								// mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float mass1,								// mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Matrix<float,3,4> &ballJointInfo,	// precomputed ball joint info
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1);

		/** Initialize ball-on-line-joint and return information which is required by the solver step.
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param position position of joint
		* @param direction direction of line
		* @param jointInfo Stores the local and global positions of the connector points.
		* The first two columns store the local connectors in body 0 and 1, respectively.
		* The next three columns contain a coordinate system for the constraint correction
		* in local coordinates. The last five columns contain the global connector positions 
		* and the constraint coordinate system in world space which have to be
		* updated in each simulation step by calling updateRigidBodyBallOnLineJointInfo().
		*/
		static bool initRigidBodyBallOnLineJointInfo(
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Vector3f &position,				// position of joint
			const Eigen::Vector3f &direction,				// direction of line
			Eigen::Matrix<float, 3, 10> &jointInfo
			);

		/** Update ball-on-line-joint information which is required by the solver step.
		* The ball-on-line-joint info must be generated in the initialization process of the model
		* by calling the function initRigidBodyBallOnLineJointInfo().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param jointInfo ball-on-line-joint information which should be updated
		*/
		static bool updateRigidBodyBallOnLineJointInfo(
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 10> &jointInfo
			);

		/** Perform a solver step for a ball-on-line-joint which links two rigid bodies.
		* A ball-on-line-joint removes two translational degrees of freedom between the bodies.
		* The joint info must be generated in the initialization process of the model
		* by calling the function initRigidBodyBallOnLineJointInfo() and updated each time the bodies
		* change their state by updateRigidBodyBallOnLineJointInfo().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html ballonlinejoint.jpg "ball-on-line joint"
		* \image latex ballonlinejoint.jpg "ball-on-line joint" width=0.5\textwidth
		*
		* @param mass0 mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param mass1 mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param jointInfo Ball joint information which is required by the solver. This
		* information must be generated in the beginning by calling initRigidBodyBallJointInfo()
		* and updated each time the bodies change their state by updateRigidBodyBallJointInfo().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solveRigidBodyBallOnLineJoint(
			const float mass0,								// mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float mass1,								// mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Matrix<float, 3, 10> &jointInfo,	// precomputed joint info
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1);
			
		/** Initialize hinge joint and return info which is required by the solver step.
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param hingeJointPosition position of hinge joint
		* @param hingeJointAxis axis of hinge joint
		* @param hingeJointInfo Stores the local and global positions of the connector points.
		* The hinge joint is a combination of a ball joint and a ball-on-line joint. 
		* The joint info stores first the info of the ball joint and then the info of
		* the ball-on-line joint. The info must be updated in each simulation step 
		* by calling updateRigidBodyHingeJointInfo().
		*/
		static bool initRigidBodyHingeJointInfo(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Vector3f &hingeJointPosition,		// position of hinge joint
			const Eigen::Vector3f &hingeJointAxis,			// axis of hinge joint
			Eigen::Matrix<float, 3, 14> &hingeJointInfo
			);

		/** Update hinge joint info which is required by the solver step.
		* The ball joint info must be generated in the initialization process of the model
		* by calling the function initRigidBodyHingeJointInfo().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param hingeJointInfo hinge joint information which should be updated
		*/
		static bool updateRigidBodyHingeJointInfo(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 14> &hingeJointInfo
			);

		/** Perform a solver step for a hinge joint which links two rigid bodies.
		* A hinge joint removes three translational and two rotational  degrees of freedom between the bodies.
		* The hinge joint info must be generated in the initialization process of the model
		* by calling the function initRigidBodyHingeJointInfo() and updated each time the bodies
		* change their state by updateRigidBodyHingeJointInfo().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html hingejoint.jpg "hinge joint"
		* \image latex hingejoint.jpg "hinge joint" width=0.5\textwidth
		*
		* @param mass0 mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param mass1 mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param hingeJointInfo Hinge joint information which is required by the solver. This
		* information must be generated in the beginning by calling initRigidBodyHingeJointInfo()
		* and updated each time the bodies change their state by updateRigidBodyHingeJointInfo().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solveRigidBodyHingeJoint(
			const float mass0,								// mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float mass1,								// mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Matrix<float, 3, 14> &hingeJointInfo,	// precomputed hinge joint info
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1);
	};
}

#endif