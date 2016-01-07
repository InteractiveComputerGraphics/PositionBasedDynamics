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
			const float invMass,
			const Eigen::Vector3f &x,
			const Eigen::Matrix3f &inertiaInverseW,
			Eigen::Matrix3f &K);

		static void computeMatrixK(
			const Eigen::Vector3f &connector0,
			const Eigen::Vector3f &connector1,
			const float invMass,
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
		* @param jointPosition position of ball joint
		* @param jointInfo Stores the local and global positions of the connector points. 
		* The first two columns store the local connectors in body 0 and 1, respectively, while
		* the last two columns contain the global connector positions which have to be
		* updated in each simulation step by calling update_BallJoint().\n
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (local)\n
		* 1:	connector in body 1 (local)\n
		* 2:	connector in body 0 (global)\n
		* 3:	connector in body 1 (global)		
		*/
		static bool init_BallJoint(			
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Vector3f &jointPosition,			// position of balljoint
			Eigen::Matrix<float, 3, 4> &jointInfo
			);

		/** Update ball joint info which is required by the solver step.
		* The ball joint info must be generated in the initialization process of the model
		* by calling the function init_BallJoint().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param ballJointInfo ball joint information which should be updated
		*/
		static bool update_BallJoint(
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 4> &ballJointInfo
			);

		/** Perform a solver step for a ball joint which links two rigid bodies.
		* A ball joint removes three translational degrees of freedom between the bodies.
		* The ball joint info must be generated in the initialization process of the model
		* by calling the function init_BallJoint() and updated each time the bodies 
		* change their state by update_BallJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html balljoint.jpg "ball joint"
		* \image latex balljoint.jpg "ball joint" width=0.5\textwidth
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param ballJointInfo Ball joint information which is required by the solver. This 
		* information must be generated in the beginning by calling init_BallJoint()
		* and updated each time the bodies change their state by update_BallJoint().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solve_BallJoint(
			const float invMass0,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float invMass1,							// inverse mass is zero if body is static
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
		* The next three columns contain a coordinate system for the constraint correction (a
		* full coordinate system where the x-axis is the slider axis)
		* in local coordinates. The last five columns contain the global connector positions
		* and the constraint coordinate system in world space which have to be
		* updated in each simulation step by calling update_BallOnLineJoint().\n
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (local)\n
		* 1:	connector in body 1 (local)\n
		* 2-4:	coordinate system of body 0 (local)\n
		* 5:	connector in body 0 (global)\n
		* 6:	connector in body 1 (global)\n
		* 7-9:	coordinate system of body 0 (global)\n
		*/
		static bool init_BallOnLineJoint(
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
		* by calling the function init_BallOnLineJoint().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param jointInfo ball-on-line-joint information which should be updated
		*/
		static bool update_BallOnLineJoint(
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 10> &jointInfo
			);

		/** Perform a solver step for a ball-on-line-joint which links two rigid bodies.
		* A ball-on-line-joint removes two translational degrees of freedom between the bodies.
		* The joint info must be generated in the initialization process of the model
		* by calling the function init_BallOnLineJoint() and updated each time the bodies
		* change their state by update_BallOnLineJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html ballonlinejoint.jpg "ball-on-line joint"
		* \image latex ballonlinejoint.jpg "ball-on-line joint" width=0.5\textwidth
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param jointInfo Ball joint information which is required by the solver. This
		* information must be generated in the beginning by calling init_BallJoint()
		* and updated each time the bodies change their state by update_BallJoint().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solve_BallOnLineJoint(
			const float invMass0,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float invMass1,							// inverse mass is zero if body is static
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
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (local)\n
		* 1:	connector in body 1 (local)\n
		* 2-4:	coordinate system of body 0 (local)\n		
		* 5:	joint axis in body 1 (local)\n
		* 6:	connector in body 0 (global)\n
		* 7:	connector in body 1 (global)\n
		* 8-10:	coordinate system of body 0 (global)\n		
		* 11:	joint axis in body 1 (global)\n\n
		* The joint info stores first the info of the first body (the connector point and a 
		* full coordinate system where the x-axis is the hinge axis) and then the info of
		* the second body (the connector point and the hinge axis). 
		* The info must be updated in each simulation step 
		* by calling update_HingeJoint().
		*/
		static bool init_HingeJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Vector3f &hingeJointPosition,		// position of hinge joint
			const Eigen::Vector3f &hingeJointAxis,			// axis of hinge joint
			Eigen::Matrix<float, 3, 12> &hingeJointInfo
			);

		/** Update hinge joint info which is required by the solver step.
		* The joint info must be generated in the initialization process of the model
		* by calling the function init_HingeJoint().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param hingeJointInfo hinge joint information which should be updated
		*/
		static bool update_HingeJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 12> &hingeJointInfo
			);

		/** Perform a solver step for a hinge joint which links two rigid bodies.
		* A hinge joint removes three translational and two rotational  degrees of freedom between the bodies.
		* The hinge joint info must be generated in the initialization process of the model
		* by calling the function init_HingeJoint() and updated each time the bodies
		* change their state by update_HingeJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html hingejoint.jpg "hinge joint"
		* \image latex hingejoint.jpg "hinge joint" width=0.5\textwidth
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param hingeJointInfo Hinge joint information which is required by the solver. This
		* information must be generated in the beginning by calling init_HingeJoint()
		* and updated each time the bodies change their state by update_HingeJoint().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solve_HingeJoint(
			const float invMass0,							//inverse  mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float invMass1,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Matrix<float, 3, 12> &hingeJointInfo,	// precomputed hinge joint info
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1);


		/** Initialize universal joint and return info which is required by the solver step.
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param jointPosition position of universal joint
		* @param jointAxis0 first axis of universal joint
		* @param jointAxis1 second axis of universal joint
		* @param jointInfo Stores the local and global positions of the connector points.
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (local)\n
		* 1:	connector in body 1 (local)\n
		* 2:	constraint axis 0 in body 0 (local)\n
		* 3:	constraint axis 1 in body 1 (local)\n
		* 4:	connector in body 0 (global)\n
		* 5:	connector in body 1 (global)\n
		* 6:	constraint axis 0 in body 0 (global)\n
		* 7:	constraint axis 1 in body 1 (global)\n\n
		* The info must be updated in each simulation step
		* by calling update_UniversalJoint().
		*/
		static bool init_UniversalJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Vector3f &jointPosition,			// position of universal joint
			const Eigen::Vector3f &jointAxis0,				// first axis of universal joint
			const Eigen::Vector3f &jointAxis1,				// second axis of universal joint
			Eigen::Matrix<float, 3, 8> &jointInfo
			);

		/** Update universal joint info which is required by the solver step.
		* The joint info must be generated in the initializatgion process of the model
		* by calling the function init_UniversalJoint().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param jointInfo universal joint information which should be updated
		*/
		static bool update_UniversalJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 8> &jointInfo
			);

		/** Perform a solver step for a universal joint which links two rigid bodies.
		* A universal joint removes three translational and one rotational degree of freedom between the bodies.
		* The universal joint info must be generated in the initialization process of the model
		* by calling the function init_UniversalJoint() and updated each time the bodies
		* change their state by update_UniversalJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html universaljoint.jpg "universal joint"
		* \image latex universaljoint.jpg "universal joint" width=0.5\textwidth
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param jointInfo Universal joint information which is required by the solver. This
		* information must be generated in the beginning by calling init_UniversalJoint()
		* and updated each time the bodies change their state by update_UniversalJoint().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solve_UniversalJoint(
			const float invMass0,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float invMass1,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Matrix<float, 3, 8> &jointInfo,	// precomputed universal joint info
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1);


		/** Initialize slider joint and return info which is required by the solver step.
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param sliderJointPosition position of slider joint
		* @param sliderJointAxis axis of slider joint
		* @param jointInfo Stores the local and global positions of the connector points.
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (local)\n
		* 1:	connector in body 1 (local)\n
		* 2-4:	coordinate system of body 0 (local)\n
		* 5:	joint axis in body 1 (local)\n
		* 6:	connector in body 0 (global)\n
		* 7:	connector in body 1 (global)\n
		* 8-10:coordinate system of body 0 (global)\n
		* 11:	joint axis in body 1 (global)\n
		* 12:	perpendicular vector on joint axis (normalized) in body 1 (local)\n
		* 13:	perpendicular vector on joint axis (normalized) in body 1 (global)\n\n
		* The info must be updated in each simulation step
		* by calling update_SliderJoint().
		*/
		static bool init_SliderJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Vector3f &sliderJointPosition,		// position of slider joint
			const Eigen::Vector3f &sliderJointAxis,			// axis of slider joint
			Eigen::Matrix<float, 3, 14> &jointInfo
			);

		/** Update slider joint info which is required by the solver step.
		* The joint info must be generated in the initialization process of the model
		* by calling the function init_SliderJoint().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param jointInfo slider joint information which should be updated
		*/
		static bool update_SliderJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 14> &jointInfo
			);

		/** Perform a solver step for a slider joint which links two rigid bodies.
		* A slider joint removes two translational and three rotational degrees of freedom between the bodies.
		* The slider joint info must be generated in the initialization process of the model
		* by calling the function init_SliderJoint() and updated each time the bodies
		* change their state by update_SliderJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html sliderjoint.jpg "slider joint"
		* \image latex sliderjoint.jpg "slider joint" width=0.5\textwidth
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param jointInfo Slider joint information which is required by the solver. This
		* information must be generated in the beginning by calling init_SliderJoint()
		* and updated each time the bodies change their state by update_SliderJoint().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solve_SliderJoint(
			const float invMass0,							//inverse  mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float invMass1,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Matrix<float, 3, 14> &jointInfo,	// precomputed slider joint info
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1);


		/** Initialize a motor slider joint which is able to enforce
		* a target position and return info which is required by the solver step.
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param sliderJointPosition position of slider joint
		* @param sliderJointAxis axis of slider joint
		* @param jointInfo Stores the local and global positions of the connector points.
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (local)\n
		* 1:	connector in body 1 (local)\n
		* 2-4:	coordinate system of body 0 (local)\n
		* 5:	joint axis in body 1 (local)\n
		* 6:	connector in body 0 (global)\n
		* 7:	connector in body 1 (global)\n
		* 8-10:coordinate system of body 0 (global)\n
		* 11:	joint axis in body 1 (global)\n
		* 12:	perpendicular vector on joint axis (normalized) in body 1 (local)\n
		* 13:	perpendicular vector on joint axis (normalized) in body 1 (global)\n\n
		* The info must be updated in each simulation step
		* by calling update_TargetPositionMotorSliderJoint().
		*/
		static bool init_TargetPositionMotorSliderJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Vector3f &sliderJointPosition,		// position of slider joint
			const Eigen::Vector3f &sliderJointAxis,			// axis of slider joint
			Eigen::Matrix<float, 3, 14> &jointInfo
			);

		/** Update motor slider joint info which is required by the solver step.
		* The joint info must be generated in the initialization process of the model
		* by calling the function init_TargetPositionMotorSliderJoint().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param jointInfo slider joint information which should be updated
		*/
		static bool update_TargetPositionMotorSliderJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 14> &jointInfo
			);

		/** Perform a solver step for a motor slider joint which links two rigid bodies.
		* A motor slider joint removes two translational and three rotational degrees of freedom between the bodies.
		* Moreover, a target position can be enforced on the remaining translation axis.
		* The motor slider joint info must be generated in the initialization process of the model
		* by calling the function init_TargetPositionMotorSliderJoint() and updated each time the bodies
		* change their state by update_TargetPositionMotorSliderJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html motorsliderjoint.jpg "motor slider joint"
		* \image latex motorsliderjoint.jpg "motor slider joint" width=0.5\textwidth
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param targetPosition target position of the servo motor
		* @param jointInfo Motor slider joint information which is required by the solver. This
		* information must be generated in the beginning by calling init_TargetPositionMotorSliderJoint()
		* and updated each time the bodies change their state by update_TargetPositionMotorSliderJoint().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solve_TargetPositionMotorSliderJoint(
			const float invMass0,							//inverse  mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float invMass1,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const float targetPosition,						// target position of the servo motor
			const Eigen::Matrix<float, 3, 14> &jointInfo,	// precomputed slider joint info
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1);


		/** Initialize a motor slider joint which is able to enforce
		* a target velocity and return info which is required by the solver step.
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param sliderJointPosition position of slider joint
		* @param sliderJointAxis axis of slider joint
		* @param jointInfo Stores the local and global positions of the connector points.
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (local)\n
		* 1:	connector in body 1 (local)\n
		* 2-4:	coordinate system of body 0 (local)\n
		* 5:	joint axis in body 1 (local)\n
		* 6:	connector in body 0 (global)\n
		* 7:	connector in body 1 (global)\n
		* 8-10:coordinate system of body 0 (global)\n
		* 11:	joint axis in body 1 (global)\n
		* 12:	perpendicular vector on joint axis (normalized) in body 1 (local)\n
		* 13:	perpendicular vector on joint axis (normalized) in body 1 (global)\n\n
		* The info must be updated in each simulation step
		* by calling update_TargetVelocityMotorSliderJoint().
		*/
		static bool init_TargetVelocityMotorSliderJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Vector3f &sliderJointPosition,		// position of slider joint
			const Eigen::Vector3f &sliderJointAxis,			// axis of slider joint
			Eigen::Matrix<float, 3, 14> &jointInfo
			);

		/** Update motor slider joint info which is required by the solver step.
		* The joint info must be generated in the initialization process of the model
		* by calling the function init_TargetVelocityMotorSliderJoint().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param jointInfo slider joint information which should be updated
		*/
		static bool update_TargetVelocityMotorSliderJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 14> &jointInfo
			);

		/** Perform a solver step for a motor slider joint which links two rigid bodies.
		* A motor slider joint removes two translational and three rotational degrees of freedom between the bodies.
		* Moreover, a target velocity can be enforced on the remaining translation axis.
		* The motor slider joint info must be generated in the initialization process of the model
		* by calling the function init_TargetVelocityMotorSliderJoint() and updated each time the bodies
		* change their state by update_TargetVelocityMotorSliderJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html motorsliderjoint.jpg "motor slider joint"
		* \image latex motorsliderjoint.jpg "motor slider joint" width=0.5\textwidth
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param jointInfo Motor slider joint information which is required by the solver. This
		* information must be generated in the beginning by calling init_TargetVelocityMotorSliderJoint()
		* and updated each time the bodies change their state by update_TargetVelocityMotorSliderJoint().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solve_TargetVelocityMotorSliderJoint(
			const float invMass0,							//inverse  mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float invMass1,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Matrix<float, 3, 14> &jointInfo,	// precomputed slider joint info
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1);

		/** Perform a velocity solver step for a motor slider joint which links two rigid bodies.
		* A motor slider joint removes two translational and three rotational degrees of freedom between the bodies.
		* Moreover, a target velocity can be enforced on the remaining translation axis.
		* The motor slider joint info must be generated in the initialization process of the model
		* by calling the function init_TargetVelocityMotorSliderJoint() and updated each time the bodies
		* change their state by update_TargetVelocityMotorSliderJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html motorsliderjoint.jpg "motor slider joint"
		* \image latex motorsliderjoint.jpg "motor slider joint" width=0.5\textwidth
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param v0 velocity of body 0
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param omega0 angular velocity of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param v1 velocity of body 1
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param omega1 angular velocity of second body
		* @param targetVelocity target velocity of the motor
		* @param jointInfo Motor slider joint information which is required by the solver. This
		* information must be generated in the beginning by calling init_TargetVelocityMotorSliderJoint()
		* and updated each time the bodies change their state by update_TargetVelocityMotorSliderJoint().
		* @param corr_v0 velocity correction of first body
		* @param corr_omega0 angular velocity correction of first body
		* @param corr_v1 velocity correction of second body
		* @param corr_omega1 angular velocity correction of second body
		*/
		static bool velocitySolve_TargetVelocityMotorSliderJoint(
			const float invMass0,							//inverse  mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Vector3f &v0,						// velocity of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Vector3f &omega0,
			const float invMass1,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Vector3f &v1,
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Vector3f &omega1,
			const float targetVelocity,						// target velocity of the servo motor
			const Eigen::Matrix<float, 3, 14> &jointInfo,	// precomputed joint info
			Eigen::Vector3f &corr_v0, Eigen::Vector3f &corr_omega0,
			Eigen::Vector3f &corr_v1, Eigen::Vector3f &corr_omega1);


		/** Initialize a motor hinge joint which is able to enforce
		* a target angle and return info which is required by the solver step.
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param hingeJointPosition position of hinge joint
		* @param hingeJointAxis axis of hinge joint
		* @param jointInfo Stores the local and global positions of the connector points.
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (local)\n
		* 1:	connector in body 1 (local)\n
		* 2-4:	coordinate system of body 0 (local)\n		
		* 5:	joint axis in body 1 (local)\n
		* 6:	connector in body 0 (global)\n
		* 7:	connector in body 1 (global)\n
		* 8-10:coordinate system of body 0 (global)\n
		* 11:	joint axis in body 1 (global)\n
		* 12:	perpendicular vector on joint axis (normalized) in body 1 (local)\n
		* 13:	perpendicular vector on joint axis (normalized) in body 1 (global)\n\n		
		* The info must be updated in each simulation step 
		* by calling update_TargetAngleMotorHingeJoint().
		*/
		static bool init_TargetAngleMotorHingeJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Vector3f &hingeJointPosition,		// position of hinge joint
			const Eigen::Vector3f &hingeJointAxis,			// axis of hinge joint
			Eigen::Matrix<float, 3, 14> &jointInfo
			);

		/** Update motor hinge joint info which is required by the solver step.
		* The joint info must be generated in the initialization process of the model
		* by calling the function init_TargetAngleMotorHingeJoint().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param jointInfo motor hinge joint information which should be updated
		*/
		static bool update_TargetAngleMotorHingeJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 14> &jointInfo
			);

		/** Perform a solver step for a motor hinge joint which links two rigid bodies.
		* A motor hinge joint removes three translational and two rotational degrees of freedom between the bodies.
		* Moreover, a target angle can be enforced on the remaining rotation axis.
		* The motor hinge joint info must be generated in the initialization process of the model
		* by calling the function init_TargetAngleMotorHingeJoint() and updated each time the bodies
		* change their state by update_TargetAngleMotorHingeJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html motorhingejoint.jpg "motor hinge joint"
		* \image latex motorhingejoint.jpg "motor hinge joint" width=0.5\textwidth
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param targetAngle target angle of the servo motor
		* @param jointInfo Motor hinge joint information which is required by the solver. This
		* information must be generated in the beginning by calling init_TargetAngleMotorHingeJoint()
		* and updated each time the bodies change their state by update_TargetAngleMotorHingeJoint().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solve_TargetAngleMotorHingeJoint(
			const float invMass0,							//inverse  mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float invMass1,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const float targetAngle,						// target angle of the servo motor
			const Eigen::Matrix<float, 3, 14> &jointInfo,	// precomputed hinge joint info
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1);

		/** Initialize a motor hinge joint which is able to enforce
		* a target velocity and return info which is required by the solver step.
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param hingeJointPosition position of hinge joint
		* @param hingeJointAxis axis of hinge joint
		* @param jointInfo Stores the local and global positions of the connector points.
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (local)\n
		* 1:	connector in body 1 (local)\n
		* 2-4:	coordinate system of body 0 (local)\n
		* 5:	joint axis in body 1 (local)\n
		* 6:	perpendicular vector on joint axis (normalized) in body 1 (local)\n
		* 7:	connector in body 0 (global)\n
		* 8:	connector in body 1 (global)\n
		* 9-11:coordinate system of body 0 (global)\n
		* 12:	joint axis in body 1 (global)\n
		* 13:	perpendicular vector on joint axis (normalized) in body 1 (global)\n\n		
		* The info must be updated in each simulation step
		* by calling update_TargetVelocityMotorHingeJoint().
		*/
		static bool init_TargetVelocityMotorHingeJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Vector3f &hingeJointPosition,		// position of hinge joint
			const Eigen::Vector3f &hingeJointAxis,			// axis of hinge joint
			Eigen::Matrix<float, 3, 14> &jointInfo
			);

		/** Update motor hinge joint info which is required by the solver step.
		* The joint info must be generated in the initialization process of the model
		* by calling the function init_TargetVelocityMotorHingeJoint().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param jointInfo motor hinge joint information which should be updated
		*/
		static bool update_TargetVelocityMotorHingeJoint(
			const Eigen::Vector3f &x0,						// center of mass of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0	
			const Eigen::Vector3f &x1,						// center of mass of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			Eigen::Matrix<float, 3, 14> &jointInfo
			);

		/** Perform a solver step for a motor hinge joint which links two rigid bodies.
		* A motor hinge joint removes three translational and two rotational degrees of freedom between the bodies.
		* Moreover, a target velocity can be enforced on the remaining rotation axis by
		* calling the function velocitySolve_TargetVelocityMotorHingeJoint().
		* The motor hinge joint info must be generated in the initialization process of the model
		* by calling the function init_TargetVelocityMotorHingeJoint() and updated each time the bodies
		* change their state by update_TargetVelocityMotorHingeJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html motorhingejoint.jpg "motor hinge joint"
		* \image latex motorhingejoint.jpg "motor hinge joint" width=0.5\textwidth
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param jointInfo Motor hinge joint information which is required by the solver. This
		* information must be generated in the beginning by calling init_TargetVelocityMotorHingeJoint()
		* and updated each time the bodies change their state by update_TargetVelocityMotorHingeJoint().
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solve_TargetVelocityMotorHingeJoint(
			const float invMass0,							// inverse  mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float invMass1,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Quaternionf &q1,					// rotation of body 1
			const Eigen::Matrix<float, 3, 14> &jointInfo,	// precomputed hinge joint info
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1);

		/** Perform a velocity solver step for a motor hinge joint which links two rigid bodies.
		* A motor hinge joint removes three translational and two rotational degrees of freedom between the bodies.
		* Moreover, a target velocity can be enforced on the remaining rotation axis.
		* The motor hinge joint info must be generated in the initialization process of the model
		* by calling the function init_TargetVelocityMotorHingeJoint() and updated each time the bodies
		* change their state by update_TargetVelocityMotorHingeJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* \image html motorhingejoint.jpg "motor hinge joint"
		* \image latex motorhingejoint.jpg "motor hinge joint" width=0.5\textwidth
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param v0 velocity of body 0
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param omega0 angular velocity of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param v1 velocity of body 1
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param omega1 angular velocity of second body
		* @param targetAngularVelocity target angular velocity of the motor
		* @param jointInfo Motor hinge joint information which is required by the solver. This
		* information must be generated in the beginning by calling init_TargetVelocityMotorHingeJoint()
		* and updated each time the bodies change their state by update_TargetVelocityMotorHingeJoint().
		* @param corr_v0 velocity correction of first body
		* @param corr_omega0 angular velocity correction of first body
		* @param corr_v1 velocity correction of second body
		* @param corr_omega1 angular velocity correction of second body
		*/
		static bool velocitySolve_TargetVelocityMotorHingeJoint(
			const float invMass0,							//inverse  mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Vector3f &v0,						// velocity of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Vector3f &omega0,					
			const float invMass1,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x1, 						// center of mass of body 1
			const Eigen::Vector3f &v1,
			const Eigen::Matrix3f &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Eigen::Vector3f &omega1,					
			const float targetAngularVelocity,				// target angular velocity of the servo motor
			const Eigen::Matrix<float, 3, 14> &jointInfo,	// precomputed joint info
			Eigen::Vector3f &corr_v0, Eigen::Vector3f &corr_omega0,
			Eigen::Vector3f &corr_v1, Eigen::Vector3f &corr_omega1);


		/** Initialize ball joint between a rigid body and a particle 
		* at the position of the particle and return info which is required by the solver step.
		*
		* @param x0 center of mass of the rigid body
		* @param q0 rotation of the rigid body body
		* @param x1 position of the particle
		* @param jointInfo Stores the local and global position of the connector point in the rigid body.
		* The info must be updated in each simulation step by calling update_RigidBodyParticleBallJoint().\n
		* The joint info contains the following columns:\n
		* 0:	connector in rigid body (local)\n
		* 1:	connector in rigid body (global)\n
		*/
		static bool init_RigidBodyParticleBallJoint(
			const Eigen::Vector3f &x0, 						// center of mass of the rigid body
			const Eigen::Quaternionf &q0,					// rotation of the rigid body body
			const Eigen::Vector3f &x1, 						// position of the particle
			Eigen::Matrix<float, 3, 2> &jointInfo
			);

		/** Update joint info which is required by the solver step.
		* The joint info must be generated in the initialization process of the model
		* by calling the function init_RigidBodyParticleBallJoint().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of the rigid body
		* @param q0 rotation of the rigid body body
		* @param x1 position of the particle
		* @param jointInfo ball joint information which should be updated
		*/
		static bool update_RigidBodyParticleBallJoint(
			const Eigen::Vector3f &x0, 						// center of mass of the rigid body
			const Eigen::Quaternionf &q0,					// rotation of the rigid body body
			const Eigen::Vector3f &x1, 						// position of the particle
			Eigen::Matrix<float, 3, 2> &jointInfo
			);

		/** Perform a solver step for a ball joint which links a rigid body and a particle.
		* The joint info must be generated in the initialization process of the model
		* by calling the function init_RigidBodyParticleBallJoint() and updated each time the rigid body 
		* changes its state by update_RigidBodyParticleBallJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* @param invMass0 inverse mass of rigid body
		* @param x0 center of mass of rigid body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of rigid body
		* @param q0 rotation of rigid body
		* @param invMass1 inverse mass of particle
		* @param x1 position of particle
		* @param jointInfo Joint information which is required by the solver. This
		* information must be generated in the beginning by calling init_RigidBodyParticleBallJoint()
		* and updated each time the rigid body changes its state by update_RigidBodyParticleBallJoint().
		* @param corr_x0 position correction of the center of mass of the rigid body
		* @param corr_q0 rotation correction of the rigid body
		* @param corr_x1 position correction of the particle
		*/
		static bool solve_RigidBodyParticleBallJoint(
			const float invMass0,							// inverse mass is zero if body is static
			const Eigen::Vector3f &x0, 						// center of mass of body 0
			const Eigen::Matrix3f &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Eigen::Quaternionf &q0,					// rotation of body 0			
			const float invMass1,							// inverse mass is zero if particle is static
			const Eigen::Vector3f &x1, 						// position of particle
			const Eigen::Matrix<float, 3, 2> &jointInfo,	// precomputed joint info
			Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
			Eigen::Vector3f &corr_x1);
	};
}

#endif