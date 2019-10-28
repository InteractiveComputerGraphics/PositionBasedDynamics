#ifndef POSITION_BASED_RIGID_BODY_DYNAMICS_H
#define POSITION_BASED_RIGID_BODY_DYNAMICS_H

#include "Common/Common.h"

// ------------------------------------------------------------------------------------
namespace PBD
{
	class PositionBasedRigidBodyDynamics
	{
	// -------------- Position Based Rigid Body Dynamics  -----------------------------------------------------
	private:
		static void computeMatrixK(
			const Vector3r &connector,
			const Real invMass,
			const Vector3r &x,
			const Matrix3r &inertiaInverseW,
			Matrix3r &K);

		static void computeMatrixK(
			const Vector3r &connector0,
			const Vector3r &connector1,
			const Real invMass,
			const Vector3r &x,
			const Matrix3r &inertiaInverseW,
			Matrix3r &K);

		/** Compute matrix that is required to transform quaternion in
		* a 3D representation. */
		static void computeMatrixG(const Quaternionr &q, Eigen::Matrix<Real, 4, 3, Eigen::DontAlign> &G);
		static void computeMatrixQ(const Quaternionr &q, Eigen::Matrix<Real, 4, 4, Eigen::DontAlign> &Q);
		static void computeMatrixQHat(const Quaternionr &q, Eigen::Matrix<Real, 4, 4, Eigen::DontAlign> &Q);

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
			const Vector3r &x0, 						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1, 						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &jointPosition,			// position of balljoint
			Eigen::Matrix<Real, 3, 4, Eigen::DontAlign> &jointInfo
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
			const Vector3r &x0, 						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1, 						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			Eigen::Matrix<Real, 3, 4, Eigen::DontAlign> &ballJointInfo
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
			const Real invMass0,							// inverse mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Eigen::Matrix<Real,3,4, Eigen::DontAlign> &ballJointInfo,	// precomputed ball joint info
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1, Quaternionr &corr_q1);

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
			const Vector3r &x0, 						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1, 						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &position,				// position of joint
			const Vector3r &direction,				// direction of line
			Eigen::Matrix<Real, 3, 10, Eigen::DontAlign> &jointInfo
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
			const Vector3r &x0, 						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1, 						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			Eigen::Matrix<Real, 3, 10, Eigen::DontAlign> &jointInfo
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
			const Real invMass0,							// inverse mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Eigen::Matrix<Real, 3, 10, Eigen::DontAlign> &jointInfo,	// precomputed joint info
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1, Quaternionr &corr_q1);
			
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
		* 0-1:	projection matrix Pr for the rotational part\n
		* 2:	connector in body 0 (local)\n
		* 3:	connector in body 1 (local)\n
		* 4:	connector in body 0 (global)\n
		* 5:	connector in body 1 (global)\n
		* 6:	hinge axis in body 0 (local) used for rendering\n\n
		* The joint info stores first the info of the first body (the connector point and a 
		* full coordinate system where the x-axis is the hinge axis) and then the info of
		* the second body (the connector point and the hinge axis). 
		* The info must be updated in each simulation step 
		* by calling update_HingeJoint().
		*/
		static bool init_HingeJoint(
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &hingeJointPosition,		// position of hinge joint
			const Vector3r &hingeJointAxis,			// axis of hinge joint
			Eigen::Matrix<Real, 4, 7, Eigen::DontAlign> &hingeJointInfo
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
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			Eigen::Matrix<Real, 4, 7, Eigen::DontAlign> &hingeJointInfo
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
			const Real invMass0,							//inverse  mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Eigen::Matrix<Real, 4, 7, Eigen::DontAlign> &hingeJointInfo,	// precomputed hinge joint info
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1, Quaternionr &corr_q1);


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
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &jointPosition,			// position of universal joint
			const Vector3r &jointAxis0,				// first axis of universal joint
			const Vector3r &jointAxis1,				// second axis of universal joint
			Eigen::Matrix<Real, 3, 8, Eigen::DontAlign> &jointInfo
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
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			Eigen::Matrix<Real, 3, 8, Eigen::DontAlign> &jointInfo
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
			const Real invMass0,							// inverse mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Eigen::Matrix<Real, 3, 8, Eigen::DontAlign> &jointInfo,	// precomputed universal joint info
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1, Quaternionr &corr_q1);


		/** Initialize slider joint and return info which is required by the solver step.
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param sliderJointAxis axis of slider joint
		* @param jointInfo Stores the local and global positions of the connector points.
		* The joint info contains the following columns:\n
		* jointInfo contains\n
		* 0:   coordinate system in body 0, where the x-axis is the slider axis (local)\n
		* 1:   coordinate system in body 0, where the x-axis is the slider axis (global)\n
		* 2:   2D vector d = P * (x0 - x1), where P projects the vector onto a plane perpendicular to the slider axis\n
		* 3-5: projection matrix Pr for the rotational part\n\n
		* The info must be updated in each simulation step
		* by calling update_SliderJoint().
		*/
		static bool init_SliderJoint(
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &sliderJointAxis,			// axis of slider joint
			Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo
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
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo
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
			const Real invMass0,							//inverse  mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo,	// precomputed slider joint info
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1, Quaternionr &corr_q1);


		/** Initialize a motor slider joint which is able to enforce
		* a target position and return info which is required by the solver step.
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param sliderJointAxis axis of slider joint
		* @param jointInfo Stores the local and global positions of the connector points.
		* The joint info contains the following columns:\n
		* 0:	slider axis in body 0 (local)\n
		* 1:	slider axis in body 0 (global)\n
		* 2:   distance vector d = (x0 - x1)\n
		* 3-5:	projection matrix Pr for the rotational part\n\n
		* The info must be updated in each simulation step
		* by calling update_TargetPositionMotorSliderJoint().
		*/
		static bool init_TargetPositionMotorSliderJoint(
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &sliderJointAxis,			// axis of slider joint
			Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo
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
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo
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
			const Real invMass0,							//inverse  mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Real targetPosition,						// target position of the servo motor
			const Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo,	// precomputed slider joint info
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1, Quaternionr &corr_q1);


		/** Initialize a motor slider joint which is able to enforce
		* a target velocity and return info which is required by the solver step.
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param sliderJointAxis axis of slider joint
		* @param jointInfo Stores the local and global positions of the connector points.
		* The joint info contains the following columns:\n
		* 0:   coordinate system in body 0, where the x-axis is the slider axis (local)\n
		* 1:   coordinate system in body 0, where the x-axis is the slider axis (global)\n
		* 2:   2D vector d = P * (x0 - x1), where P projects the vector onto a plane perpendicular to the slider axis\n
		* 3-5: projection matrix Pr for the rotational part\n\n
		* The info must be updated in each simulation step
		* by calling update_TargetVelocityMotorSliderJoint().
		*/
		static bool init_TargetVelocityMotorSliderJoint(
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &sliderJointAxis,			// axis of slider joint
			Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo
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
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo
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
			const Real invMass0,							//inverse  mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo,	// precomputed slider joint info
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1, Quaternionr &corr_q1);

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
		* @param q0 rotation of first body
		* @param omega0 angular velocity of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param v1 velocity of body 1
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
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
			const Real invMass0,							//inverse  mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Vector3r &v0,						// velocity of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Vector3r &omega0,
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Vector3r &v1,
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &omega1,
			const Real targetVelocity,						// target velocity of the servo motor
			const Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo,	// precomputed joint info
			Vector3r &corr_v0, Vector3r &corr_omega0,
			Vector3r &corr_v1, Vector3r &corr_omega1);


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
		* 0-2:	projection matrix Pr for the rotational part\n
		* 3:	connector in body 0 (local)\n
		* 4:	connector in body 1 (local)\n
		* 5:	connector in body 0 (global)\n
		* 6:	connector in body 1 (global)\n
		* 7:	hinge axis in body 0 (local) used for rendering \n\n
		* The info must be updated in each simulation step 
		* by calling update_TargetAngleMotorHingeJoint().
		*/
		static bool init_TargetAngleMotorHingeJoint(
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &hingeJointPosition,		// position of hinge joint
			const Vector3r &hingeJointAxis,			// axis of hinge joint
			Eigen::Matrix<Real, 4, 8, Eigen::DontAlign> &jointInfo
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
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			Eigen::Matrix<Real, 4, 8, Eigen::DontAlign> &jointInfo
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
			const Real invMass0,							//inverse  mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Real targetAngle,						// target angle of the servo motor
			const Eigen::Matrix<Real, 4, 8, Eigen::DontAlign> &jointInfo,	// precomputed hinge joint info
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1, Quaternionr &corr_q1);

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
		* 0-1:	projection matrix Pr for the rotational part\n
		* 2:	connector in body 0 (local)\n
		* 3:	connector in body 1 (local)\n
		* 4:	connector in body 0 (global)\n
		* 5:	connector in body 1 (global)\n
		* 6:	hinge axis in body 0 (local)\n
		* 7:   hinge axis in body 0 (global)\n\n
		* The info must be updated in each simulation step
		* by calling update_TargetVelocityMotorHingeJoint().
		*/
		static bool init_TargetVelocityMotorHingeJoint(
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &hingeJointPosition,		// position of hinge joint
			const Vector3r &hingeJointAxis,			// axis of hinge joint
			Eigen::Matrix<Real, 4, 8, Eigen::DontAlign> &jointInfo
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
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			Eigen::Matrix<Real, 4, 8, Eigen::DontAlign> &jointInfo
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
			const Real invMass0,							// inverse  mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Eigen::Matrix<Real, 4, 8, Eigen::DontAlign> &jointInfo,	// precomputed hinge joint info
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1, Quaternionr &corr_q1);

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
			const Real invMass0,							//inverse  mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Vector3r &v0,						// velocity of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Vector3r &omega0,					
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Vector3r &v1,
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Vector3r &omega1,					
			const Real targetAngularVelocity,				// target angular velocity of the servo motor
			const Eigen::Matrix<Real, 4, 8, Eigen::DontAlign> &jointInfo,	// precomputed joint info
			Vector3r &corr_v0, Vector3r &corr_omega0,
			Vector3r &corr_v1, Vector3r &corr_omega1);


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
			const Vector3r &x0, 						// center of mass of the rigid body
			const Quaternionr &q0,					// rotation of the rigid body body
			const Vector3r &x1, 						// position of the particle
			Eigen::Matrix<Real, 3, 2, Eigen::DontAlign> &jointInfo
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
			const Vector3r &x0, 						// center of mass of the rigid body
			const Quaternionr &q0,					// rotation of the rigid body body
			const Vector3r &x1, 						// position of the particle
			Eigen::Matrix<Real, 3, 2, Eigen::DontAlign> &jointInfo
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
			const Real invMass0,							// inverse mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if particle is static
			const Vector3r &x1, 						// position of particle
			const Eigen::Matrix<Real, 3, 2, Eigen::DontAlign> &jointInfo,	// precomputed joint info
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1);

		/** Initialize contact between two rigid bodies and return 
		* info which is required by the solver step.
		*
		* @param invMass0 inverse mass of rigid body
		* @param x0 center of mass of first body
		* @param v0 velocity of body 0
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param omega0 angular velocity of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param v1 velocity of body 1
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param omega1 angular velocity of second body
		* @param cp0 contact point of body 0
		* @param cp1 contact point of body 1
		* @param normal contact normal in body 1
		* @param restitutionCoeff coefficient of restitution
		* @param constraintInfo Stores the local and global position of the contact points and
		* the contact normal. \n
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (global)\n
		* 1:	connector in body 1 (global)\n
		* 2:	contact normal in body 1 (global)\n
		* 3:	contact tangent (global)\n
		* 0,4:   1.0 / normal^T * K * normal\n
		* 1,4:  maximal impulse in tangent direction\n
		* 2,4:  goal velocity in normal direction after collision
		*/
		static bool init_RigidBodyContactConstraint(
			const Real invMass0,							// inverse mass is zero if body is static
			const Vector3r &x0,						// center of mass of body 0
			const Vector3r &v0,						// velocity of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &omega0,					// angular velocity of body 0
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1,						// center of mass of body 1
			const Vector3r &v1,						// velocity of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &omega1,					// angular velocity of body 1
			const Vector3r &cp0,						// contact point of body 0
			const Vector3r &cp1,						// contact point of body 1
			const Vector3r &normal,					// contact normal in body 1
			const Real restitutionCoeff,					// coefficient of restitution
			Eigen::Matrix<Real, 3, 5, Eigen::DontAlign> &constraintInfo);


		/** Perform a solver step for a contact constraint between two rigid bodies.
		* A contact constraint handles collisions and resting contacts between the bodies.
		* The contact info must be generated in each time step.
		*
		* @param invMass0 inverse mass of rigid body
		* @param x0 center of mass of first body
		* @param v0 velocity of body 0
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param omega0 angular velocity of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param v1 velocity of body 1
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param omega1 angular velocity of second body
		* @param stiffness stiffness parameter of penalty impulse
		* @param frictionCoeff friction coefficient
		* @param sum_impulses sum of all correction impulses in normal direction
		* @param constraintInfo information which is required by the solver. This
		* information must be generated in the beginning by calling init_RigidBodyContactConstraint().
		* @param corr_v0 velocity correction of first body
		* @param corr_omega0 angular velocity correction of first body
		* @param corr_v1 velocity correction of second body
		* @param corr_omega1 angular velocity correction of second body
		*/
		static bool velocitySolve_RigidBodyContactConstraint(
			const Real invMass0,							// inverse mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Vector3r &v0,						// velocity of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Vector3r &omega0,					// angular velocity of body 0
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Vector3r &v1,						// velocity of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Vector3r &omega1,					// angular velocity of body 1
			const Real stiffness,							// stiffness parameter of penalty impulse
			const Real frictionCoeff,						// friction coefficient
			Real &sum_impulses,							// sum of all impulses
			Eigen::Matrix<Real, 3, 5, Eigen::DontAlign> &constraintInfo,		// precomputed contact info
			Vector3r &corr_v0, Vector3r &corr_omega0,
			Vector3r &corr_v1, Vector3r &corr_omega1);


		/** Initialize contact between a rigid body and a particle and return
		* info which is required by the solver step.
		*
		* @param invMass0 inverse mass of rigid body
		* @param x0 center of mass of first body
		* @param v0 velocity of body 0
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param v1 velocity of body 1
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param omega1 angular velocity of second body
		* @param cp0 contact point of body 0
		* @param cp1 contact point of body 1
		* @param normal contact normal in body 1
		* @param restitutionCoeff coefficient of restitution
		* @param constraintInfo Stores the local and global position of the contact points and
		* the contact normal. \n
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (global)\n
		* 1:	connector in body 1 (global)\n
		* 2:	contact normal in body 1 (global)\n
		* 3:	contact tangent (global)\n
		* 0,4:   1.0 / normal^T * K * normal\n
		* 1,4:  maximal impulse in tangent direction\n
		* 2,4:  goal velocity in normal direction after collision
		*/
		static bool init_ParticleRigidBodyContactConstraint(
			const Real invMass0,							// inverse mass is zero if body is static
			const Vector3r &x0,						// center of mass of body 0
			const Vector3r &v0,						// velocity of body 0
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1,						// center of mass of body 1
			const Vector3r &v1,						// velocity of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1	
			const Vector3r &omega1,					// angular velocity of body 1
			const Vector3r &cp0,						// contact point of body 0
			const Vector3r &cp1,						// contact point of body 1
			const Vector3r &normal,					// contact normal in body 1
			const Real restitutionCoeff,					// coefficient of restitution
			Eigen::Matrix<Real, 3, 5, Eigen::DontAlign> &constraintInfo);


		/** Perform a solver step for a contact constraint between a rigid body and a particle.
		* A contact constraint handles collisions and resting contacts between the bodies.
		* The contact info must be generated in each time step.
		*
		* @param invMass0 inverse mass of rigid body
		* @param x0 center of mass of first body
		* @param v0 velocity of body 0
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param v1 velocity of body 1
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param omega1 angular velocity of second body
		* @param stiffness stiffness parameter of penalty impulse
		* @param frictionCoeff friction coefficient
		* @param sum_impulses sum of all correction impulses in normal direction
		* @param constraintInfo information which is required by the solver. This
		* information must be generated in the beginning by calling init_RigidBodyContactConstraint().
		* @param corr_v0 velocity correction of first body
		* @param corr_v1 velocity correction of second body
		* @param corr_omega1 angular velocity correction of second body
		*/
		static bool velocitySolve_ParticleRigidBodyContactConstraint(
			const Real invMass0,							// inverse mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Vector3r &v0,						// velocity of body 0
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Vector3r &v1,						// velocity of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Vector3r &omega1,					// angular velocity of body 1
			const Real stiffness,							// stiffness parameter of penalty impulse
			const Real frictionCoeff,						// friction coefficient
			Real &sum_impulses,							// sum of all impulses
			Eigen::Matrix<Real, 3, 5, Eigen::DontAlign> &constraintInfo,		// precomputed contact info
			Vector3r &corr_v0,
			Vector3r &corr_v1, Vector3r &corr_omega1);


		/** Initialize distance joint and return info which is required by the solver step.
		* 
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param jointPosition position of distance joint
		* @param jointInfo Stores the local and global positions of the connector points. 
		* The first two columns store the local connectors in body 0 and 1, respectively, while
		* the last two columns contain the global connector positions which have to be
		* updated in each simulation step by calling update_DistanceJoint().\n
		* The joint info contains the following columns:\n
		* 0:	connector in body 0 (local)\n
		* 1:	connector in body 1 (local)\n
		* 2:	connector in body 0 (global)\n
		* 3:	connector in body 1 (global)		
		*/
		static bool init_DistanceJoint(			
			const Vector3r &x0, 						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1, 						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &pos0,
			const Vector3r &pos1,
			Eigen::Matrix<Real, 3, 4, Eigen::DontAlign> &jointInfo
			);

		/** Update distance joint info which is required by the solver step.
		* The distance joint info must be generated in the initialization process of the model
		* by calling the function init_DistanceJoint().
		* This method should be called once per simulation step before executing the solver.\n\n
		*
		* @param x0 center of mass of first body
		* @param q0 rotation of first body
		* @param x1 center of mass of second body
		* @param q1 rotation of second body
		* @param jointInfo joint information which should be updated
		*/
		static bool update_DistanceJoint(
			const Vector3r &x0, 						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1, 						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			Eigen::Matrix<Real, 3, 4, Eigen::DontAlign> &jointInfo
			);

		/** Perform a solver step for a distance joint which links two rigid bodies.
		* A distance joint removes one translational degrees of freedom between the bodies.
		* When setting a stiffness value which is not zero, we get an implicit spring.
		* The distance joint info must be generated in the initialization process of the model
		* by calling the function init_DistanceJoint() and updated each time the bodies 
		* change their state by update_DistanceJoint().\n\n
		* More information can be found in: \cite Deul2014
		*
		* @param invMass0 inverse mass of first body
		* @param x0 center of mass of first body
		* @param inertiaInverseW0 inverse inertia tensor in world coordinates of first body
		* @param q0 rotation of first body
		* @param invMass1 inverse mass of second body
		* @param x1 center of mass of second body
		* @param inertiaInverseW1 inverse inertia tensor in world coordinates of second body
		* @param q1 rotation of second body
		* @param stiffness Stiffness of the constraint. 0 means it is totally stiff and we get a distance joint otherwise we get an implicit spring.
		* @param restLength Rest length of the joint.
		* @param dt Time step size (required for XPBD when simulating a spring)
		* @param jointInfo Joint information which is required by the solver. This 
		* information must be generated in the beginning by calling init_DistanceJoint()
		* and updated each time the bodies change their state by update_DistanceJoint().
		* @param lambda Lagrange multiplier (required for XPBD). Must be 0 in the first iteration.
		* @param corr_x0 position correction of center of mass of first body
		* @param corr_q0 rotation correction of first body
		* @param corr_x1 position correction of center of mass of second body
		* @param corr_q1 rotation correction of second body
		*/
		static bool solve_DistanceJoint(
			const Real invMass0,							// inverse mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Real stiffness,
			const Real restLength,
			const Real dt,
			const Eigen::Matrix<Real,3,4, Eigen::DontAlign> &jointInfo,	// precomputed joint info
			Real &lambda,
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1, Quaternionr &corr_q1);

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
		* 0:	coordinate system in body 0, where the x-axis is the slider axis (local)\n
		* 1:	coordinate system in body 0, where the x-axis is the slider axis (global)\n
		* 2:    3D vector d = R^T * (x0 - x1), where R is a rotation matrix with the slider axis as first column\n
		* 3-5:	projection matrix Pr for the rotational part\n\n
		* The info must be updated in each simulation step
		* by calling update_TargetPositionMotorSliderJoint().
		*/
		static bool init_DamperJoint(
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Vector3r &direction,
			Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo
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
		static bool update_DamperJoint(
			const Vector3r &x0,						// center of mass of body 0
			const Quaternionr &q0,					// rotation of body 0	
			const Vector3r &x1,						// center of mass of body 1
			const Quaternionr &q1,					// rotation of body 1
			Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo
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
		static bool solve_DamperJoint(
			const Real invMass0,							//inverse  mass is zero if body is static
			const Vector3r &x0, 						// center of mass of body 0
			const Matrix3r &inertiaInverseW0,		// inverse inertia tensor (world space) of body 0
			const Quaternionr &q0,					// rotation of body 0			
			const Real invMass1,							// inverse mass is zero if body is static
			const Vector3r &x1, 						// center of mass of body 1
			const Matrix3r &inertiaInverseW1,		// inverse inertia tensor (world space) of body 1
			const Quaternionr &q1,					// rotation of body 1
			const Real stiffness,
			const Real dt,
			const Eigen::Matrix<Real, 4, 6, Eigen::DontAlign> &jointInfo,	// precomputed slider joint info
			Real &lambda,
			Vector3r &corr_x0, Quaternionr &corr_q0,
			Vector3r &corr_x1, Quaternionr &corr_q1);
	};
}

#endif
