#include "PositionBasedRigidBodyDynamics.h"
#include "MathFunctions.h"
#include <cfloat>
#include <iostream>

using namespace PBD;

// ----------------------------------------------------------------------------------------------
void PositionBasedRigidBodyDynamics::computeMatrixK(
	const Eigen::Vector3f &connector,
	const float invMass,
	const Eigen::Vector3f &x,
	const Eigen::Matrix3f &inertiaInverseW,
	Eigen::Matrix3f &K)
{
	if (invMass != 0.0f)
	{
		const Eigen::Vector3f v = connector - x;
		const float a = v[0];
		const float b = v[1];
		const float c = v[2];

		// J is symmetric
		const float j11 = inertiaInverseW(0,0);
		const float j12 = inertiaInverseW(0,1);
		const float j13 = inertiaInverseW(0,2);
		const float j22 = inertiaInverseW(1,1);
		const float j23 = inertiaInverseW(1,2);
		const float j33 = inertiaInverseW(2,2);

		K(0,0) = c*c*j22 - b*c*(j23 + j23) + b*b*j33 + invMass;
		K(0,1) = -(c*c*j12) + a*c*j23 + b*c*j13 - a*b*j33;
		K(0,2) = b*c*j12 - a*c*j22 - b*b*j13 + a*b*j23;
		K(1,0) = K(0,1);
		K(1,1) = c*c*j11 - a*c*(j13 + j13) + a*a*j33 + invMass;
		K(1,2) = -(b*c*j11) + a*c*j12 + a*b*j13 - a*a*j23;
		K(2,0) = K(0,2);
		K(2,1) = K(1,2);
		K(2,2) = b*b*j11 - a*b*(j12 + j12) + a*a*j22 + invMass;
	}
	else
		K.setZero();
}

// ----------------------------------------------------------------------------------------------
void PositionBasedRigidBodyDynamics::computeMatrixK(
	const Eigen::Vector3f &connector0,
	const Eigen::Vector3f &connector1,
	const float invMass,
	const Eigen::Vector3f &x,
	const Eigen::Matrix3f &inertiaInverseW,
	Eigen::Matrix3f &K)
{
	if (invMass != 0.0f)
	{
		const Eigen::Vector3f v0 = connector0 - x;
		const float a = v0[0];
		const float b = v0[1];
		const float c = v0[2];

		const Eigen::Vector3f v1 = connector1 - x;
		const float d = v1[0];
		const float e = v1[1];
		const float f = v1[2];

		// J is symmetric
		const float j11 = inertiaInverseW(0, 0);
		const float j12 = inertiaInverseW(0, 1);
		const float j13 = inertiaInverseW(0, 2);
		const float j22 = inertiaInverseW(1, 1);
		const float j23 = inertiaInverseW(1, 2);
		const float j33 = inertiaInverseW(2, 2);

		K(0, 0) = c*f*j22 - c*e*j23 - b*f*j23 + b*e*j33 + invMass;
		K(0, 1) = -(c*f*j12) + c*d*j23 + b*f*j13 - b*d*j33;
		K(0, 2) = c*e*j12 - c*d*j22 - b*e*j13 + b*d*j23;
		K(1, 0) = -(c*f*j12) + c*e*j13 + a*f*j23 - a*e*j33;
		K(1, 1) = c*f*j11 - c*d*j13 - a*f*j13 + a*d*j33 + invMass;
		K(1, 2) = -(c*e*j11) + c*d*j12 + a*e*j13 - a*d*j23;
		K(2, 0) = b*f*j12 - b*e*j13 - a*f*j22 + a*e*j23;
		K(2, 1) = -(b*f*j11) + b*d*j13 + a*f*j12 - a*d*j23;
		K(2, 2) = b*e*j11 - b*d*j12 - a*e*j12 + a*d*j22 + invMass;
	}
	else
		K.setZero();
}


// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::init_BallJoint(
	const Eigen::Vector3f &x0, 						
	const Eigen::Quaternionf &q0,					
	const Eigen::Vector3f &x1, 						
	const Eigen::Quaternionf &q1,					
	const Eigen::Vector3f &ballJointPosition,		
	Eigen::Matrix<float, 3, 4> &ballJointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2:	connector in body 0 (global)
	// 3:	connector in body 1 (global)

	// transform in local coordinates
	const Eigen::Matrix3f rot0T = q0.matrix().transpose();
	const Eigen::Matrix3f rot1T = q1.matrix().transpose();

	ballJointInfo.col(0) = rot0T * (ballJointPosition - x0);
	ballJointInfo.col(1) = rot1T * (ballJointPosition - x1);
	ballJointInfo.col(2) = ballJointPosition;
	ballJointInfo.col(3) = ballJointPosition;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::update_BallJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	Eigen::Matrix<float, 3, 4> &ballJointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2:	connector in body 0 (global)
	// 3:	connector in body 1 (global)

	// compute world space positions of connectors
	const Eigen::Matrix3f rot0 = q0.matrix();
	const Eigen::Matrix3f rot1 = q1.matrix();
	ballJointInfo.col(2) = rot0 * ballJointInfo.col(0) + x0;
	ballJointInfo.col(3) = rot1 * ballJointInfo.col(1) + x1;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::solve_BallJoint(
	const float invMass0,
	const Eigen::Vector3f &x0, 						
	const Eigen::Matrix3f &inertiaInverseW0,		
	const Eigen::Quaternionf &q0,					
	const float invMass1,
	const Eigen::Vector3f &x1, 						
	const Eigen::Matrix3f &inertiaInverseW1,		
	const Eigen::Quaternionf &q1,	
	const Eigen::Matrix<float, 3, 4> &ballJointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2:	connector in body 0 (global)
	// 3:	connector in body 1 (global)

	const Eigen::Vector3f &connector0 = ballJointInfo.col(2);
	const Eigen::Vector3f &connector1 = ballJointInfo.col(3);

	// Compute Kinv
	Eigen::Matrix3f K1, K2;
	computeMatrixK(connector0, invMass0, x0, inertiaInverseW0, K1);
	computeMatrixK(connector1, invMass1, x1, inertiaInverseW1, K2);
	const Eigen::Matrix3f Kinv = (K1 + K2).inverse();

	const Eigen::Vector3f pt = Kinv * (connector1 - connector0);

	if (invMass0 != 0.0f)
	{
		const Eigen::Vector3f r0 = connector0 - x0;
		corr_x0 = invMass0*pt;

		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (invMass1 != 0.0f)
	{
		const Eigen::Vector3f r1 = connector1 - x1;
		corr_x1 = -invMass1*pt;

		const Eigen::Vector3f ot = (inertiaInverseW1 * (r1.cross(-pt)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q1.coeffs() = 0.5f *(otQ*q1).coeffs();
	}

	return true;
}


// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::init_BallOnLineJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	const Eigen::Vector3f &position,
	const Eigen::Vector3f &direction,
	Eigen::Matrix<float, 3, 10> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	connector in body 0 (global)
	// 6:	connector in body 1 (global)
	// 7-9:	coordinate system of body 0 (global)

	// transform in local coordinates
	const Eigen::Matrix3f rot0T = q0.matrix().transpose();
	const Eigen::Matrix3f rot1T = q1.matrix().transpose();

	jointInfo.col(0) = rot0T * (position - x0);
	jointInfo.col(1) = rot1T * (position - x1);
	jointInfo.col(5) = position;
	jointInfo.col(6) = position;

	// determine constraint coordinate system
	// with direction as x-axis
	jointInfo.col(7) = direction;
	jointInfo.col(7).normalize();

	Eigen::Vector3f v(1.0f, 0.0f, 0.0f);
	// check if vectors are parallel
	if (fabs(v.dot(jointInfo.col(7))) > 0.99f)
		v = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

	jointInfo.col(8) = jointInfo.col(7).cross(v);
	jointInfo.col(9) = jointInfo.col(7).cross(jointInfo.col(8));
	jointInfo.col(8).normalize();
	jointInfo.col(9).normalize();

	jointInfo.block<3, 3>(0, 2) = rot0T * jointInfo.block<3, 3>(0, 7);

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::update_BallOnLineJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	Eigen::Matrix<float, 3, 10> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	connector in body 0 (global)
	// 6:	connector in body 1 (global)
	// 7-9:	coordinate system of body 0 (global)

	// compute world space positions of connectors
	const Eigen::Matrix3f rot0 = q0.matrix();
	const Eigen::Matrix3f rot1 = q1.matrix();
	jointInfo.col(5) = rot0 * jointInfo.col(0) + x0;
	jointInfo.col(6) = rot1 * jointInfo.col(1) + x1;

	// transform constraint coordinate system to world space
	jointInfo.block<3, 3>(0, 7) = rot0 * jointInfo.block<3, 3>(0, 2);

	const Eigen::Vector3f dir = jointInfo.col(7);
	const Eigen::Vector3f p = jointInfo.col(5);
	const Eigen::Vector3f s = jointInfo.col(6);
	// move the joint point of body 0 to the closest point on the line to joint point 1
	jointInfo.col(5) = p + (dir * (((s - p).dot(dir)) / dir.squaredNorm()));

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::solve_BallOnLineJoint(
	const float invMass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float invMass1,
	const Eigen::Vector3f &x1,
	const Eigen::Matrix3f &inertiaInverseW1,
	const Eigen::Quaternionf &q1,
	const Eigen::Matrix<float, 3, 10> &jointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	connector in body 0 (global)
	// 6:	connector in body 1 (global)
	// 7-9:	coordinate system of body 0 (global)

	const Eigen::Vector3f &connector0 = jointInfo.col(5);
	const Eigen::Vector3f &connector1 = jointInfo.col(6);

	// Compute Kinv
	Eigen::Matrix3f K1, K2;
	computeMatrixK(connector0, invMass0, x0, inertiaInverseW0, K1);
	computeMatrixK(connector1, invMass1, x1, inertiaInverseW1, K2);

	// projection 
	const Eigen::Matrix<float, 3, 2> PT = jointInfo.block<3, 2>(0, 8);
	const Eigen::Matrix<float, 2, 3> P = PT.transpose();

	const Eigen::Matrix2f K = P * (K1 + K2) * PT;
	const Eigen::Matrix2f Kinv = K.inverse();

	const Eigen::Vector2f pt2D = Kinv * (P * (connector1 - connector0));
	const Eigen::Vector3f pt = PT * pt2D;

	if (invMass0 != 0.0f)
	{
		const Eigen::Vector3f r0 = connector0 - x0;
		corr_x0 = invMass0*pt;

		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (invMass1 != 0.0f)
	{
		const Eigen::Vector3f r1 = connector1 - x1;
		corr_x1 = -invMass1*pt;

		const Eigen::Vector3f ot = (inertiaInverseW1 * (r1.cross(-pt)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q1.coeffs() = 0.5f *(otQ*q1).coeffs();
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::init_HingeJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	const Eigen::Vector3f &position,
	const Eigen::Vector3f &direction,
	Eigen::Matrix<float, 3, 12> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)

	// transform in local coordinates
	const Eigen::Matrix3f rot0T = q0.matrix().transpose();
	const Eigen::Matrix3f rot1T = q1.matrix().transpose();

	// connector in body 0 (local)
	jointInfo.col(0) = rot0T * (position - x0);
	// connector in body 1 (local)
	jointInfo.col(1) = rot1T * (position - x1);
	// connector in body 0 (global)
	jointInfo.col(6) = position;
	// connector in body 1 (global)
	jointInfo.col(7) = position;

	// determine constraint coordinate system
	// with direction as x-axis
	jointInfo.col(8) = direction;
	jointInfo.col(8).normalize();

	Eigen::Vector3f v(1.0f, 0.0f, 0.0f);
	// check if vectors are parallel
	if (fabs(v.dot(jointInfo.col(8))) > 0.99f)
		v = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

	jointInfo.col(9) = jointInfo.col(8).cross(v);
	jointInfo.col(10) = jointInfo.col(8).cross(jointInfo.col(9));
	jointInfo.col(9).normalize();
	jointInfo.col(10).normalize();

	// joint axis in body 1 (global)
	jointInfo.col(11) = jointInfo.col(8);

	// coordinate system of body 0 (local)
	jointInfo.block<3, 3>(0, 2) = rot0T * jointInfo.block<3, 3>(0, 8);

	// joint axis in body 1 (local)
	jointInfo.col(5) = rot1T * jointInfo.col(11);

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::update_HingeJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	Eigen::Matrix<float, 3, 12> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)


	// compute world space positions of connectors
	const Eigen::Matrix3f rot0 = q0.matrix();
	const Eigen::Matrix3f rot1 = q1.matrix();
	jointInfo.col(6) = rot0 * jointInfo.col(0) + x0;
	jointInfo.col(7) = rot1 * jointInfo.col(1) + x1;

	// transform constraint coordinate system of body 0 to world space
	jointInfo.block<3, 3>(0, 8) = rot0 * jointInfo.block<3, 3>(0, 2);
	// transform joint axis of body 1 to world space
	jointInfo.col(11) = rot1 * jointInfo.col(5);

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::solve_HingeJoint(
	const float invMass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float invMass1,
	const Eigen::Vector3f &x1,
	const Eigen::Matrix3f &inertiaInverseW1,
	const Eigen::Quaternionf &q1,
	const Eigen::Matrix<float, 3, 12> &jointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)

	const Eigen::Vector3f &c0 = jointInfo.col(6);
	const Eigen::Vector3f &c1 = jointInfo.col(7);
	const Eigen::Vector3f &axis0 = jointInfo.col(8);
	const Eigen::Vector3f &axis1 = jointInfo.col(11);
	const Eigen::Vector3f u = axis0.cross(axis1);
	const Eigen::Vector3f &t1 = jointInfo.col(9);
	const Eigen::Vector3f &t2 = jointInfo.col(10);
	const Eigen::Vector3f r0 = c0 - x0;
	const Eigen::Vector3f r1 = c1 - x1;
	Eigen::Matrix3f r0_star, r1_star;
	MathFunctions::crossProductMatrix(r0, r0_star);
	MathFunctions::crossProductMatrix(r1, r1_star);


	Eigen::Matrix<float, 5, 1> b;
	b.block<3, 1>(0, 0) = c1 - c0;
	b(3, 0) = t1.dot(u);
	b(4, 0) = t2.dot(u);

	Eigen::Matrix<float, 5, 5> K;
	K.setZero();
	if (invMass0 != 0.0f)
	{
		// Jacobian for body 0 is
		//
		// (I_3   -r0*)
		// (0     t1^T)
		// (0     t2^T)
		//
		// where I_3 is the identity matrix and r0* is the cross product matrix of r0
		//
		// J M^-1 J^T =
		// ( 1/m I_3-r0 * J0^-1 * r0*    -r0 * J0^-1 * t1    -r0 * J0^-1 * t2 )
		// ( (-r0* J0^-1 * t1)^T         t1^T * J0^-1 t1     t1^T * J0^-1 t2  )
		// ( (-r0* J0^-1 * t2)^T         t2^T * J0^-1 t1     t2^T * J0^-1 t2  )

		Eigen::Matrix3f K00;
		computeMatrixK(c0, invMass0, x0, inertiaInverseW0, K00);

		K.block<3, 3>(0, 0) = K00;
		K.block<3, 1>(0, 3) = -r0_star * inertiaInverseW0 * t1;
		K.block<3, 1>(0, 4) = -r0_star * inertiaInverseW0 * t2;
		K.block<1, 3>(3, 0) = K.block<3, 1>(0, 3).transpose();
		K.block<1, 3>(4, 0) = K.block<3, 1>(0, 4).transpose();
		K(3, 3) = t1.transpose() * inertiaInverseW0 * t1;
		K(3, 4) = t1.transpose() * inertiaInverseW0 * t2;
		K(4, 3) = K(3, 4);
		K(4, 4) = t2.transpose() * inertiaInverseW0 * t2;
	}
	if (invMass1 != 0.0f)
	{
		// Jacobian for body 1 is
		//
		// ( -I_3   r1*  )
		// ( 0     -t1^T )
		// ( 0     -t2^T )
		//
		// where I_3 is the identity matrix and r1* is the cross product matrix of r1
		//
		// J M^-1 J^T =
		// ( 1/m I_3-r1 * J1^-1 * r1*    -r1 * J1^-1 * t1    -r1 * J1^-1 * t2 )
		// ( (-r1* J1^-1 * t1)^T         t1^T * J1^-1 t1     t1^T * J1^-1 t2  )
		// ( (-r1* J1^-1 * t2)^T         t2^T * J1^-1 t1     t2^T * J1^-1 t2  )

		Eigen::Matrix3f K11;
		computeMatrixK(c1, invMass1, x1, inertiaInverseW1, K11);

		K.block<3, 3>(0, 0) += K11;
		const Eigen::Vector3f K_03 = -r1_star * inertiaInverseW1 * t1;
		const Eigen::Vector3f K_04 = -r1_star * inertiaInverseW1 * t2;
		K.block<3, 1>(0, 3) += K_03;
		K.block<3, 1>(0, 4) += K_04;
		K.block<1, 3>(3, 0) += K_03.transpose();
		K.block<1, 3>(4, 0) += K_04.transpose();
		K(3, 3) += t1.transpose() * inertiaInverseW1 * t1;
		const float K_34 = t1.transpose() * inertiaInverseW1 * t2;
		K(3, 4) += K_34;
		K(4, 3) += K_34;
		K(4, 4) += t2.transpose() * inertiaInverseW1 * t2;
	}

	const Eigen::Matrix<float, 5, 5> Kinv = K.inverse();

	const Eigen::Matrix<float, 5, 1> lambda = Kinv * b;
	const Eigen::Vector3f pt = lambda.block<3, 1>(0, 0);

	if (invMass0 != 0.0f)
	{
		corr_x0 = invMass0*pt;
		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt) + t1*lambda(3, 0) + t2*lambda(4, 0)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (invMass1 != 0.0f)
	{
		corr_x1 = -invMass1*pt;
		const Eigen::Vector3f ot = (inertiaInverseW1 * (r1.cross(-pt) - t1*lambda(3, 0) - t2*lambda(4, 0)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q1.coeffs() = 0.5f *(otQ*q1).coeffs();
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::init_UniversalJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	const Eigen::Vector3f &position,
	const Eigen::Vector3f &jointAxis0,
	const Eigen::Vector3f &jointAxis1,
	Eigen::Matrix<float, 3, 8> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2:	constraint axis 0 in body 0 (local)
	// 3:	constraint axis 1 in body 1 (local)
	// 4:	connector in body 0 (global)
	// 5:	connector in body 1 (global)
	// 6:	constraint axis 0 in body 0 (global)	
	// 7:	constraint axis 1 in body 1 (global)

	// transform in local coordinates
	const Eigen::Matrix3f rot0T = q0.matrix().transpose();
	const Eigen::Matrix3f rot1T = q1.matrix().transpose();

	// connector in body 0 (local)
	jointInfo.col(0) = rot0T * (position - x0);
	// connector in body 1 (local)
	jointInfo.col(1) = rot1T * (position - x1);
	// connector in body 0 (global)
	jointInfo.col(4) = position;
	// connector in body 1 (global)
	jointInfo.col(5) = position;

	// determine constraint coordinate system
	Eigen::Vector3f constraintAxis = jointAxis0.cross(jointAxis1);
	if (constraintAxis.norm() < 1.0e-3)
		return false;

	// joint axis in body 0 (global)
	jointInfo.col(6) = jointAxis0;
	jointInfo.col(6).normalize();

	// joint axis in body 1 (global)
	jointInfo.col(7) = jointAxis1;
	jointInfo.col(7).normalize();

	// correction axis in body 0 (local)
	jointInfo.col(2) = rot0T * jointInfo.col(6);

	// correction axis in body 1 (local)
	jointInfo.col(3) = rot1T * jointInfo.col(7);

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::update_UniversalJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	Eigen::Matrix<float, 3, 8> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2:	constraint axis 0 in body 0 (local)
	// 3:	constraint axis 1 in body 1 (local)
	// 4:	connector in body 0 (global)
	// 5:	connector in body 1 (global)
	// 6:	constraint axis 0 in body 0 (global)	
	// 7:	constraint axis 1 in body 1 (global)

	// compute world space positions of connectors
	const Eigen::Matrix3f rot0 = q0.matrix();
	const Eigen::Matrix3f rot1 = q1.matrix();
	jointInfo.col(4) = rot0 * jointInfo.col(0) + x0;
	jointInfo.col(5) = rot1 * jointInfo.col(1) + x1;

	// transform joint axis of body 0 to world space
	jointInfo.col(6) = rot0 * jointInfo.col(2);
	// transform joint axis of body 1 to world space
	jointInfo.col(7) = rot1 * jointInfo.col(3);

	return true;
}


// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::solve_UniversalJoint(
	const float invMass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float invMass1,
	const Eigen::Vector3f &x1,
	const Eigen::Matrix3f &inertiaInverseW1,
	const Eigen::Quaternionf &q1,
	const Eigen::Matrix<float, 3, 8> &jointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2:	constraint axis 0 in body 0 (local)
	// 3:	constraint axis 1 in body 1 (local)
	// 4:	connector in body 0 (global)
	// 5:	connector in body 1 (global)
	// 6:	constraint axis 0 in body 0 (global)	
	// 7:	constraint axis 1 in body 1 (global)

	const Eigen::Vector3f &c0 = jointInfo.col(4);
	const Eigen::Vector3f &c1 = jointInfo.col(5);
	const Eigen::Vector3f &axis0 = jointInfo.col(6);
	const Eigen::Vector3f &axis1 = jointInfo.col(7);
	const Eigen::Vector3f u = axis0.cross(axis1);
	const Eigen::Vector3f r0 = c0 - x0;
	const Eigen::Vector3f r1 = c1 - x1;
	Eigen::Matrix3f r0_star, r1_star;
	MathFunctions::crossProductMatrix(r0, r0_star);
	MathFunctions::crossProductMatrix(r1, r1_star);

	Eigen::Matrix<float, 4, 1> b;
	b.block<3, 1>(0, 0) = c1 - c0;
	b(3, 0) = -axis0.dot(axis1);

	Eigen::Matrix<float, 4, 4> K;
	K.setZero();
	Eigen::Matrix<float, 4, 6> J0, J1;
	if (invMass0 != 0.0f)
	{
		// Jacobian for body 0 is
		//
		// (I_3   -r0*)
		// (0     u^T)
		//
		// where I_3 is the identity matrix and r0* is the cross product matrix of r0
		//
		// J M^-1 J^T =
		// ( 1/m I_3-r0 * J0^-1 * r0*    -r0 * J0^-1 * u )
		// ( (-r0 * J0^-1 * u)^T         u^T * J0^-1 * u )

		Eigen::Matrix3f K00;
		computeMatrixK(c0, invMass0, x0, inertiaInverseW0, K00);

		K.block<3, 3>(0, 0) = K00;
		K.block<3, 1>(0, 3) = -r0_star * inertiaInverseW0 * u;
		K.block<1, 3>(3, 0) = K.block<3, 1>(0, 3).transpose();
		K(3, 3) = u.transpose() * inertiaInverseW0 * u;
	}
	if (invMass1 != 0.0f)
	{
		// Jacobian for body 1 is
		//
		// ( -I_3   r1*  )
		// ( 0     -u^T )
		//
		// where I_3 is the identity matrix and r1* is the cross product matrix of r1
		//
		// J M^-1 J^T =
		// ( 1/m I_3-r1 * J1^-1 * r1*    -r1 * J1^-1 * u )
		// ( (-r1 * J1^-1 * u)^T         u^T * J1^-1 * u )

		Eigen::Matrix3f K11;
		computeMatrixK(c1, invMass1, x1, inertiaInverseW1, K11);

		K.block<3, 3>(0, 0) += K11;
		const Eigen::Vector3f K_03 = -r1_star * inertiaInverseW1 * u;
		K.block<3, 1>(0, 3) += K_03;
		K.block<1, 3>(3, 0) += K_03.transpose();
		K(3, 3) += u.transpose() * inertiaInverseW1 * u;
	}

	const Eigen::Matrix<float, 4, 4> Kinv = K.inverse();

	const Eigen::Matrix<float, 4, 1> lambda = Kinv * b;
	const Eigen::Vector3f pt = lambda.block<3, 1>(0, 0);

	if (invMass0 != 0.0f)
	{
		corr_x0 = invMass0*pt;
		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt) + u*lambda(3, 0)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (invMass1 != 0.0f)
	{
		corr_x1 = -invMass1*pt;
		const Eigen::Vector3f ot = (inertiaInverseW1 * (r1.cross(-pt) - u*lambda(3, 0)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q1.coeffs() = 0.5f *(otQ*q1).coeffs();
	}

	return true;
}


// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::init_SliderJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	const Eigen::Vector3f &position,
	const Eigen::Vector3f &direction,
	Eigen::Matrix<float, 3, 14> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)	
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)
	// 12:	perpendicular vector on joint axis (normalized) in body 1 (local)
	// 13:	perpendicular vector on joint axis (normalized) in body 1 (global)

	// transform in local coordinates
	const Eigen::Matrix3f rot0T = q0.matrix().transpose();
	const Eigen::Matrix3f rot1T = q1.matrix().transpose();

	// connector in body 0 (local)
	jointInfo.col(0) = rot0T * (position - x0);
	// connector in body 1 (local)
	jointInfo.col(1) = rot1T * (position - x1);
	// connector in body 0 (global)
	jointInfo.col(6) = position;
	// connector in body 1 (global)
	jointInfo.col(7) = position;

	// determine constraint coordinate system
	// with direction as x-axis
	jointInfo.col(8) = direction;
	jointInfo.col(8).normalize();

	Eigen::Vector3f v(1.0f, 0.0f, 0.0f);
	// check if vectors are parallel
	if (fabs(v.dot(jointInfo.col(8))) > 0.99f)
		v = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

	jointInfo.col(9) = jointInfo.col(8).cross(v);
	jointInfo.col(10) = jointInfo.col(8).cross(jointInfo.col(9));
	jointInfo.col(9).normalize();
	jointInfo.col(10).normalize();

	// joint axis in body 1 (global)
	jointInfo.col(11) = jointInfo.col(8);

	// coordinate system of body 0 (local)
	jointInfo.block<3, 3>(0, 2) = rot0T * jointInfo.block<3, 3>(0, 8);

	// joint axis in body 1 (local)
	jointInfo.col(5) = rot1T * jointInfo.col(11);

	// perpendicular vector on joint axis (normalized) in body 1 (global)
	jointInfo.col(13) = jointInfo.col(9);

	// perpendicular vector on joint axis(normalized) in body 1 (local)
	jointInfo.col(12) = rot1T * jointInfo.col(13);

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::update_SliderJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	Eigen::Matrix<float, 3, 14> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)	
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)
	// 12:	perpendicular vector on joint axis (normalized) in body 1 (local)
	// 13:	perpendicular vector on joint axis (normalized) in body 1 (global)


	// compute world space positions of connectors
	const Eigen::Matrix3f rot0 = q0.matrix();
	const Eigen::Matrix3f rot1 = q1.matrix();
	jointInfo.col(6) = rot0 * jointInfo.col(0) + x0;
	jointInfo.col(7) = rot1 * jointInfo.col(1) + x1;

	// transform constraint coordinate system of body 0 to world space
	jointInfo.block<3, 3>(0, 8) = rot0 * jointInfo.block<3, 3>(0, 2);
	// transform joint axis of body 1 to world space
	jointInfo.col(11) = rot1 * jointInfo.col(5);
	// transform perpendicular vector on joint axis of body 1 to world space 
	jointInfo.col(13) = rot1 * jointInfo.col(12);

  	const Eigen::Vector3f dir = jointInfo.col(8);
  	const Eigen::Vector3f p = jointInfo.col(6);
  	const Eigen::Vector3f s = jointInfo.col(7);
  	// move the joint point of body 0 to the closest point on the line to joint point 1
  	jointInfo.col(6) = p + (dir * (((s - p).dot(dir)) / dir.squaredNorm()));

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::solve_SliderJoint(
	const float invMass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float invMass1,
	const Eigen::Vector3f &x1,
	const Eigen::Matrix3f &inertiaInverseW1,
	const Eigen::Quaternionf &q1,
	const Eigen::Matrix<float, 3, 14> &jointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)	
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)
	// 12:	perpendicular vector on joint axis (normalized) in body 1 (local)
	// 13:	perpendicular vector on joint axis (normalized) in body 1 (global)

	const Eigen::Vector3f &c0 = jointInfo.col(6);
	const Eigen::Vector3f &c1 = jointInfo.col(7);
	const Eigen::Vector3f &axis0 = jointInfo.col(8);
	const Eigen::Vector3f &axis1 = jointInfo.col(11);
	const Eigen::Vector3f axis = 0.5 * (axis0 + axis1);
	const Eigen::Vector3f u = axis0.cross(axis1);
	const Eigen::Vector3f &t1 = jointInfo.col(9);
	const Eigen::Vector3f &t2 = jointInfo.col(10);
	const Eigen::Vector3f &t3 = jointInfo.col(13);
	const Eigen::Vector3f r0 = c0 - x0;
	const Eigen::Vector3f r1 = c1 - x1;
	Eigen::Matrix3f r0_star, r1_star;
	MathFunctions::crossProductMatrix(r0, r0_star);
	MathFunctions::crossProductMatrix(r1, r1_star);

	// projection 
	const Eigen::Matrix<float, 3, 2> PT = jointInfo.block<3, 2>(0, 9);
	const Eigen::Matrix<float, 2, 3> P = PT.transpose();


	Eigen::Matrix<float, 5, 1> b;
	b.block<2, 1>(0, 0) = P * (c1 - c0);
	b(2, 0) = t1.dot(u);
	b(3, 0) = t2.dot(u);

	// determine correction angle (slider axis)
	float delta = 0.0f;
	float c = t1.dot(t3);
	c = std::min(1.0f, c);
	c = std::max(-1.0f, c);
	if ((t1.cross(t3)).dot(axis) > 0.0f)
		delta -= acos(c);
	else
		delta += acos(c);

	const float pi = (float)M_PI;
	if (delta < -pi)
		delta += 2.0f * pi;
	if (delta > pi)
		delta -= 2.0f * pi;

	b(4, 0) = -delta;


	Eigen::Matrix<float, 5, 5> K;
	K.setZero();
	if (invMass0 != 0.0f)
	{
		// Jacobian for body 0 is
		//
		// (P     -P r0*)
		// (0     t1^T)
		// (0     t2^T)
		// (0     axis^T)
		//
		// where I_3 is the identity matrix and r0* is the cross product matrix of r0
		//
		// J M^-1 J^T =
		// ( P (1/m I_3-r0 * J0^-1 * r0*) P^T    P (-r0 * J0^-1 * t1)    P (-r0 * J0^-1 * t2)		P (-r0 * J0^-1 * axis) )
		// ( (-r0 * J0^-1 * t1)^T P^T            t1^T * J0^-1 t1         t1^T * J0^-1 * t2		    t1^T * J0^-1 * axis    )
		// ( (-r0 * J0^-1 * t2)^T P^T            t2^T * J0^-1 t1         t2^T * J0^-1 * t2		    t2^T * J0^-1 axis      )
		// ( (-r0 * J0^-1 * axis)^T P^T          axis^T * J0^-1 t1       axis^T * J0^-1 t2		    axis^T * J0^-1 axis    )

		Eigen::Matrix3f K00;
		computeMatrixK(c0, invMass0, x0, inertiaInverseW0, K00);
		const Eigen::Matrix<float, 2, 3> neg_P_r0_star_Jinv = -P * r0_star * inertiaInverseW0;
		const Eigen::Vector3f Jinv_axis = inertiaInverseW0 * axis;
		const Eigen::Vector3f Jinv_t1 = inertiaInverseW0 * t1;
		const Eigen::Vector3f Jinv_t2 = inertiaInverseW0 * t2;

		K.block<2, 2>(0, 0) = P * K00 * PT;
		K.block<2, 1>(0, 2) = neg_P_r0_star_Jinv * t1;
		K.block<2, 1>(0, 3) = neg_P_r0_star_Jinv * t2;
		K.block<2, 1>(0, 4) = neg_P_r0_star_Jinv * axis;
		K.block<1, 2>(2, 0) = K.block<2, 1>(0, 2).transpose();
		K.block<1, 2>(3, 0) = K.block<2, 1>(0, 3).transpose();
		K.block<1, 2>(4, 0) = K.block<2, 1>(0, 4).transpose();
		K(2, 2) = t1.transpose() * Jinv_t1;
		K(2, 3) = t1.transpose() * Jinv_t2;
		K(2, 4) = t1.transpose() * Jinv_axis;
		K(3, 2) = K(2, 3);
		K(4, 2) = K(2, 4);
		K(3, 3) = t2.transpose() * Jinv_t2;
		K(3, 4) = t2.transpose() * Jinv_axis;
		K(4, 3) = K(3, 4);
		K(4, 4) = axis.transpose() * Jinv_axis;
	}
	if (invMass1 != 0.0f)
	{
		// Jacobian for body 1 is
		//
		// ( -P    P r1*  )
		// ( 0     -t1^T )
		// ( 0     -t2^T )
		// ( 0     axis^T)
		//
		// where I_3 is the identity matrix and r1* is the cross product matrix of r1
		//
		// J M^-1 J^T =
		// ( P (1/m I_3-r1 * J1^-1 * r1*) P^T    P (-r1 * J1^-1 * t1)    P (-r1 * J1^-1 * t2)		P (-r1 * J1^-1 * axis) )
		// ( (-r1 * J1^-1 * t1)^T P^T            t1^T * J1^-1 t1         t1^T * J1^-1 * t2		    t1^T * J1^-1 * axis    )
		// ( (-r1 * J1^-1 * t2)^T P^T            t2^T * J1^-1 t1         t2^T * J1^-1 * t2		    t2^T * J1^-1 axis      )
		// ( (-r1 * J1^-1 * axis)^T P^T          axis^T * J1^-1 t1       axis^T * J1^-1 t2		    axis^T * J1^-1 axis    )

		Eigen::Matrix3f K11;
		computeMatrixK(c1, invMass1, x1, inertiaInverseW1, K11);
		const Eigen::Matrix<float, 2, 3> neg_P_r1_star_Jinv = -P * r1_star * inertiaInverseW1;
		const Eigen::Vector3f Jinv_axis = inertiaInverseW1 * axis;
		const Eigen::Vector3f Jinv_t1 = inertiaInverseW1 * t1;
		const Eigen::Vector3f Jinv_t2 = inertiaInverseW1 * t2;

		K.block<2, 2>(0, 0) += P * K11 * PT;
		const Eigen::Vector2f K_02 = neg_P_r1_star_Jinv * t1;
		const Eigen::Vector2f K_03 = neg_P_r1_star_Jinv * t2;
		const Eigen::Vector2f K_04 = neg_P_r1_star_Jinv * axis;
		K.block<2, 1>(0, 2) += K_02;
		K.block<2, 1>(0, 3) += K_03;
		K.block<2, 1>(0, 4) += K_04;
		K.block<1, 2>(2, 0) += K_02.transpose();
		K.block<1, 2>(3, 0) += K_03.transpose();
		K.block<1, 2>(4, 0) += K_04.transpose();
		K(2, 2) += t1.transpose() * Jinv_t1;
		const float K_23 = t1.transpose() * Jinv_t2;
		K(2, 3) += K_23;
		const float K_24 = t1.transpose() * Jinv_axis;
		K(2, 4) += K_24;
		K(3, 2) += K_23;
		K(3, 3) += t2.transpose() * Jinv_t2;
		const float K_34 = t2.transpose() * Jinv_axis;
		K(3, 4) += K_34;
		K(4, 2) += K_24;
		K(4, 3) += K_34;
		K(4, 4) += axis.transpose() * Jinv_axis;
	}

	const Eigen::Matrix<float, 5, 5> Kinv = K.inverse();
	const Eigen::Matrix<float, 5, 1> lambda = Kinv * b;
	const Eigen::Vector3f pt = PT * lambda.block<2, 1>(0, 0);

	const Eigen::Vector3f angMomentum = t1*lambda(2, 0) + t2*lambda(3, 0) + axis*lambda(4, 0);

	if (invMass0 != 0.0f)
	{
		corr_x0 = invMass0*pt;
		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt) + angMomentum));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (invMass1 != 0.0f)
	{
		corr_x1 = -invMass1*pt;
		const Eigen::Vector3f ot = (inertiaInverseW1 * (r1.cross(-pt) - angMomentum));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q1.coeffs() = 0.5f *(otQ*q1).coeffs();
	}

	return true;
}


// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::init_TargetPositionMotorSliderJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	const Eigen::Vector3f &position,
	const Eigen::Vector3f &direction,
	Eigen::Matrix<float, 3, 14> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)	
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)
	// 12:	perpendicular vector on joint axis (normalized) in body 1 (local)
	// 13:	perpendicular vector on joint axis (normalized) in body 1 (global)

	// transform in local coordinates
	const Eigen::Matrix3f rot0T = q0.matrix().transpose();
	const Eigen::Matrix3f rot1T = q1.matrix().transpose();

	// connector in body 0 (local)
	jointInfo.col(0) = rot0T * (position - x0);
	// connector in body 1 (local)
	jointInfo.col(1) = rot1T * (position - x1);
	// connector in body 0 (global)
	jointInfo.col(6) = position;
	// connector in body 1 (global)
	jointInfo.col(7) = position;

	// determine constraint coordinate system
	// with direction as x-axis
	jointInfo.col(8) = direction;
	jointInfo.col(8).normalize();

	Eigen::Vector3f v(1.0f, 0.0f, 0.0f);
	// check if vectors are parallel
	if (fabs(v.dot(jointInfo.col(8))) > 0.99f)
		v = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

	jointInfo.col(9) = jointInfo.col(8).cross(v);
	jointInfo.col(10) = jointInfo.col(8).cross(jointInfo.col(9));
	jointInfo.col(9).normalize();
	jointInfo.col(10).normalize();

	// joint axis in body 1 (global)
	jointInfo.col(11) = jointInfo.col(8);

	// coordinate system of body 0 (local)
	jointInfo.block<3, 3>(0, 2) = rot0T * jointInfo.block<3, 3>(0, 8);

	// joint axis in body 1 (local)
	jointInfo.col(5) = rot1T * jointInfo.col(11);

	// perpendicular vector on joint axis (normalized) in body 1 (global)
	jointInfo.col(13) = jointInfo.col(9);

	// perpendicular vector on joint axis(normalized) in body 1 (local)
	jointInfo.col(12) = rot1T * jointInfo.col(13);

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::update_TargetPositionMotorSliderJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	Eigen::Matrix<float, 3, 14> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)	
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)
	// 12:	perpendicular vector on joint axis (normalized) in body 1 (local)
	// 13:	perpendicular vector on joint axis (normalized) in body 1 (global)


	// compute world space positions of connectors
	const Eigen::Matrix3f rot0 = q0.matrix();
	const Eigen::Matrix3f rot1 = q1.matrix();
	jointInfo.col(6) = rot0 * jointInfo.col(0) + x0;
	jointInfo.col(7) = rot1 * jointInfo.col(1) + x1;

	// transform constraint coordinate system of body 0 to world space
	jointInfo.block<3, 3>(0, 8) = rot0 * jointInfo.block<3, 3>(0, 2);
	// transform joint axis of body 1 to world space
	jointInfo.col(11) = rot1 * jointInfo.col(5);
	// transform perpendicular vector on joint axis of body 1 to world space 
	jointInfo.col(13) = rot1 * jointInfo.col(12);

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::solve_TargetPositionMotorSliderJoint(
	const float invMass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float invMass1,
	const Eigen::Vector3f &x1,
	const Eigen::Matrix3f &inertiaInverseW1,
	const Eigen::Quaternionf &q1,
	const float targetPosition,	
	const Eigen::Matrix<float, 3, 14> &jointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)	
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)
	// 12:	perpendicular vector on joint axis (normalized) in body 1 (local)
	// 13:	perpendicular vector on joint axis (normalized) in body 1 (global)

	const Eigen::Vector3f &c0 = jointInfo.col(6);
	const Eigen::Vector3f &c1 = jointInfo.col(7);
	const Eigen::Vector3f &axis0 = jointInfo.col(8);
	const Eigen::Vector3f &axis1 = jointInfo.col(11);
	const Eigen::Vector3f axis = 0.5 * (axis0 + axis1);
	const Eigen::Vector3f u = axis0.cross(axis1);
	const Eigen::Vector3f &t1 = jointInfo.col(9);
	const Eigen::Vector3f &t2 = jointInfo.col(10);
	const Eigen::Vector3f &t3 = jointInfo.col(13);
	const Eigen::Vector3f r0 = c0 - x0;
	const Eigen::Vector3f r1 = c1 - x1;
	Eigen::Matrix3f r0_star, r1_star;
	MathFunctions::crossProductMatrix(r0, r0_star);
	MathFunctions::crossProductMatrix(r1, r1_star);

	// projection 
	const Eigen::Matrix<float, 3, 3> PT = jointInfo.block<3, 3>(0, 8);
	const Eigen::Matrix<float, 3, 3> P = PT.transpose();

	Eigen::Matrix<float, 6, 1> b;
	b.block<3, 1>(0, 0) = P * (c1 - c0);
	b(0, 0) = -targetPosition + b(0, 0);
	b(3, 0) = t1.dot(u);
	b(4, 0) = t2.dot(u);

	// determine correction angle (slider axis)
	float delta = 0.0f;
	float c = t1.dot(t3);
	c = std::min(1.0f, c);
	c = std::max(-1.0f, c);
	if ((t1.cross(t3)).dot(axis) > 0.0f)
		delta -= acos(c);
	else
		delta += acos(c);

	const float pi = (float)M_PI;
	if (delta < -pi)
		delta += 2.0f * pi;
	if (delta > pi)
		delta -= 2.0f * pi;

	b(5, 0) = -delta;


	Eigen::Matrix<float, 6, 6> K;
	K.setZero();
	if (invMass0 != 0.0f)
	{
		// Jacobian for body 0 is
		//
		// (P     -P r0*)
		// (0     t1^T)
		// (0     t2^T)
		// (0     axis^T)
		//
		// where I_3 is the identity matrix and r0* is the cross product matrix of r0
		//
		// J M^-1 J^T =
		// ( P (1/m I_3-r0 * J0^-1 * r0*) P^T    P (-r0 * J0^-1 * t1)    P (-r0 * J0^-1 * t2)		P (-r0 * J0^-1 * axis) )
		// ( (-r0 * J0^-1 * t1)^T P^T            t1^T * J0^-1 t1         t1^T * J0^-1 * t2		    t1^T * J0^-1 * axis    )
		// ( (-r0 * J0^-1 * t2)^T P^T            t2^T * J0^-1 t1         t2^T * J0^-1 * t2		    t2^T * J0^-1 axis      )
		// ( (-r0 * J0^-1 * axis)^T P^T          axis^T * J0^-1 t1       axis^T * J0^-1 t2		    axis^T * J0^-1 axis    )

		Eigen::Matrix3f K00;
		computeMatrixK(c0, invMass0, x0, inertiaInverseW0, K00);
		const Eigen::Matrix3f neg_P_r0_star_Jinv = -P * r0_star * inertiaInverseW0;
		const Eigen::Vector3f Jinv_axis = inertiaInverseW0 * axis;
		const Eigen::Vector3f Jinv_t1 = inertiaInverseW0 * t1;
		const Eigen::Vector3f Jinv_t2 = inertiaInverseW0 * t2;

		K.block<3, 3>(0, 0) = P * K00 * PT;
		K.block<3, 1>(0, 3) = neg_P_r0_star_Jinv * t1;
		K.block<3, 1>(0, 4) = neg_P_r0_star_Jinv * t2;
		K.block<3, 1>(0, 5) = neg_P_r0_star_Jinv * axis;
		K.block<1, 3>(3, 0) = K.block<3, 1>(0, 3).transpose();
		K.block<1, 3>(4, 0) = K.block<3, 1>(0, 4).transpose();
		K.block<1, 3>(5, 0) = K.block<3, 1>(0, 5).transpose();
		K(3, 3) = t1.transpose() * Jinv_t1;
		K(3, 4) = t1.transpose() * Jinv_t2;
		K(3, 5) = t1.transpose() * Jinv_axis;
		K(4, 3) = K(3, 4);
		K(5, 3) = K(3, 5);
		K(4, 4) = t2.transpose() * Jinv_t2;
		K(4, 5) = t2.transpose() * Jinv_axis;
		K(5, 4) = K(4, 5);
		K(5, 5) = axis.transpose() * Jinv_axis;
	}
	if (invMass1 != 0.0f)
	{
		// Jacobian for body 1 is
		//
		// ( -P    P r1*  )
		// ( 0     -t1^T )
		// ( 0     -t2^T )
		// ( 0     axis^T)
		//
		// where I_3 is the identity matrix and r1* is the cross product matrix of r1
		//
		// J M^-1 J^T =
		// ( P (1/m I_3-r1 * J1^-1 * r1*) P^T    P (-r1 * J1^-1 * t1)    P (-r1 * J1^-1 * t2)		P (-r1 * J1^-1 * axis) )
		// ( (-r1 * J1^-1 * t1)^T P^T            t1^T * J1^-1 t1         t1^T * J1^-1 * t2		    t1^T * J1^-1 * axis    )
		// ( (-r1 * J1^-1 * t2)^T P^T            t2^T * J1^-1 t1         t2^T * J1^-1 * t2		    t2^T * J1^-1 axis      )
		// ( (-r1 * J1^-1 * axis)^T P^T          axis^T * J1^-1 t1       axis^T * J1^-1 t2		    axis^T * J1^-1 axis    )

		Eigen::Matrix3f K11;
		computeMatrixK(c1, invMass1, x1, inertiaInverseW1, K11);
		const Eigen::Matrix3f neg_P_r1_star_Jinv = -P * r1_star * inertiaInverseW1;
		const Eigen::Vector3f Jinv_axis = inertiaInverseW1 * axis;
		const Eigen::Vector3f Jinv_t1 = inertiaInverseW1 * t1;
		const Eigen::Vector3f Jinv_t2 = inertiaInverseW1 * t2;

		K.block<3, 3>(0, 0) += P * K11 * PT;
		const Eigen::Vector3f K_03 = neg_P_r1_star_Jinv * t1;
		const Eigen::Vector3f K_04 = neg_P_r1_star_Jinv * t2;
		const Eigen::Vector3f K_05 = neg_P_r1_star_Jinv * axis;
		K.block<3, 1>(0, 3) += K_03;
		K.block<3, 1>(0, 4) += K_04;
		K.block<3, 1>(0, 5) += K_05;
		K.block<1, 3>(3, 0) += K_03.transpose();
		K.block<1, 3>(4, 0) += K_04.transpose();
		K.block<1, 3>(5, 0) += K_05.transpose();
		K(3, 3) += t1.transpose() * Jinv_t1;
		const float K_34 = t1.transpose() * Jinv_t2;
		K(3, 4) += K_34;
		const float K_35 = t1.transpose() * Jinv_axis;
		K(3, 5) += K_35;
		K(4, 3) += K_34;
		K(4, 4) += t2.transpose() * Jinv_t2;
		const float K_45 = t2.transpose() * Jinv_axis;
		K(4, 5) += K_45;
		K(5, 3) += K_35;
		K(5, 4) += K_45;
		K(5, 5) += axis.transpose() * Jinv_axis;
	}

	const Eigen::Matrix<float, 6, 6> Kinv = K.inverse();
	const Eigen::Matrix<float, 6, 1> lambda = Kinv * b;
	const Eigen::Vector3f pt = PT * lambda.block<3, 1>(0, 0);
	const Eigen::Vector3f angMomentum = t1*lambda(3, 0) + t2*lambda(4, 0) + axis*lambda(5, 0);

	if (invMass0 != 0.0f)
	{
		corr_x0 = invMass0*pt;
		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt) + angMomentum));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (invMass1 != 0.0f)
	{
		corr_x1 = -invMass1*pt;
		const Eigen::Vector3f ot = (inertiaInverseW1 * (r1.cross(-pt) - angMomentum));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q1.coeffs() = 0.5f *(otQ*q1).coeffs();
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::init_TargetAngleMotorHingeJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	const Eigen::Vector3f &position,
	const Eigen::Vector3f &direction,
	Eigen::Matrix<float, 3, 14> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)	
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)
	// 12:	perpendicular vector on joint axis (normalized) in body 1 (local)
	// 13:	perpendicular vector on joint axis (normalized) in body 1 (global)

	// transform in local coordinates
	const Eigen::Matrix3f rot0T = q0.matrix().transpose();
	const Eigen::Matrix3f rot1T = q1.matrix().transpose();

	// connector in body 0 (local)
	jointInfo.col(0) = rot0T * (position - x0);
	// connector in body 1 (local)
	jointInfo.col(1) = rot1T * (position - x1);
	// connector in body 0 (global)
	jointInfo.col(6) = position;
	// connector in body 1 (global)
	jointInfo.col(7) = position;

	// determine constraint coordinate system
	// with direction as x-axis
	jointInfo.col(8) = direction;
	jointInfo.col(8).normalize();

	Eigen::Vector3f v(1.0f, 0.0f, 0.0f);
	// check if vectors are parallel
	if (fabs(v.dot(jointInfo.col(8))) > 0.99f)
		v = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

	jointInfo.col(9) = jointInfo.col(8).cross(v);
	jointInfo.col(10) = jointInfo.col(8).cross(jointInfo.col(9));
	jointInfo.col(9).normalize();
	jointInfo.col(10).normalize();

	// joint axis in body 1 (global)
	jointInfo.col(11) = jointInfo.col(8);

	// coordinate system of body 0 (local)
	jointInfo.block<3, 3>(0, 2) = rot0T * jointInfo.block<3, 3>(0, 8);

	// joint axis in body 1 (local)
	jointInfo.col(5) = rot1T * jointInfo.col(11);

	// perpendicular vector on joint axis (normalized) in body 1 (global)
	jointInfo.col(13) = jointInfo.col(9);

	// perpendicular vector on joint axis(normalized) in body 1 (local)
	jointInfo.col(12) = rot1T * jointInfo.col(13);

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::update_TargetAngleMotorHingeJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	Eigen::Matrix<float, 3, 14> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)	
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)
	// 12:	perpendicular vector on joint axis (normalized) in body 1 (local)
	// 13:	perpendicular vector on joint axis (normalized) in body 1 (global)


	// compute world space positions of connectors
	const Eigen::Matrix3f rot0 = q0.matrix();
	const Eigen::Matrix3f rot1 = q1.matrix();
	jointInfo.col(6) = rot0 * jointInfo.col(0) + x0;
	jointInfo.col(7) = rot1 * jointInfo.col(1) + x1;

	// transform constraint coordinate system of body 0 to world space
	jointInfo.block<3, 3>(0, 8) = rot0 * jointInfo.block<3, 3>(0, 2);
	// transform joint axis of body 1 to world space
	jointInfo.col(11) = rot1 * jointInfo.col(5);
	// transform perpendicular vector on joint axis of body 1 to world space 
	jointInfo.col(13) = rot1 * jointInfo.col(12);

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::solve_TargetAngleMotorHingeJoint(
	const float invMass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float invMass1,
	const Eigen::Vector3f &x1,
	const Eigen::Matrix3f &inertiaInverseW1,
	const Eigen::Quaternionf &q1,
	const float targetAngle,
	const Eigen::Matrix<float, 3, 14> &jointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)	
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)
	// 12:	perpendicular vector on joint axis (normalized) in body 1 (local)
	// 13:	perpendicular vector on joint axis (normalized) in body 1 (global)

	const Eigen::Vector3f &c0 = jointInfo.col(6);
	const Eigen::Vector3f &c1 = jointInfo.col(7);
	const Eigen::Vector3f &axis0 = jointInfo.col(8);
	const Eigen::Vector3f &axis1 = jointInfo.col(11);
	const Eigen::Vector3f axis = 0.5 * (axis0 + axis1);
	const Eigen::Vector3f u = axis0.cross(axis1);
	const Eigen::Vector3f &t1 = jointInfo.col(9);
	const Eigen::Vector3f &t2 = jointInfo.col(10);
	const Eigen::Vector3f &t3 = jointInfo.col(13);
	const Eigen::Vector3f r0 = c0 - x0;
	const Eigen::Vector3f r1 = c1 - x1;
	Eigen::Matrix3f r0_star, r1_star;
	MathFunctions::crossProductMatrix(r0, r0_star);
	MathFunctions::crossProductMatrix(r1, r1_star);

	Eigen::Matrix<float, 6, 1> b;
	b.block<3, 1>(0, 0) = c1 - c0;
	b(3, 0) = t1.dot(u);
	b(4, 0) = t2.dot(u);

	// determine correction angle
	float delta = targetAngle;
	float c = t1.dot(t3);
	c = std::min(1.0f, c);
	c = std::max(-1.0f, c);
 	if ((t1.cross(t3)).dot(axis) > 0.0f)
 		delta -= acos(c);
  	else
 		delta += acos(c);

	const float pi = (float)M_PI;
	if (delta < -pi)
		delta += 2.0f * pi;
	if (delta > pi)
		delta -= 2.0f * pi;

	b(5, 0) = -delta;

	Eigen::Matrix<float, 6, 6> K;
	K.setZero();
	if (invMass0 != 0.0f)
	{
		// Jacobian for body 0 is
		//
		// (I_3   -r0*)
		// (0     t1^T)
		// (0     t2^T)
		// (0     axis^T)
		//
		// where I_3 is the identity matrix and r0* is the cross product matrix of r0
		//
		// J M^-1 J^T =
		// ( 1/m I_3-r0 * J0^-1 * r0*    -r0 * J0^-1 * t1    -r0 * J0^-1 * t2		-r0 * J0^-1 * axis)
		// ( (-r0 * J0^-1 * t1)^T        t1^T * J0^-1 t1     t1^T * J0^-1 * t2		t1^T * J0^-1 * axis)
		// ( (-r0 * J0^-1 * t2)^T        t2^T * J0^-1 t1     t2^T * J0^-1 * t2		t2^T * J0^-1 axis)
		// ( (-r0 * J0^-1 * axis)^T      axis^T * J0^-1 t1   axis^T * J0^-1 t2		axis^T * J0^-1 axis)

		Eigen::Matrix3f K00;
		computeMatrixK(c0, invMass0, x0, inertiaInverseW0, K00);
		const Eigen::Matrix3f neg_P_r0_star_Jinv = -r0_star * inertiaInverseW0;
		const Eigen::Vector3f Jinv_axis = inertiaInverseW0 * axis;
		const Eigen::Vector3f Jinv_t1 = inertiaInverseW0 * t1;
		const Eigen::Vector3f Jinv_t2 = inertiaInverseW0 * t2;

		K.block<3, 3>(0, 0) = K00;
		K.block<3, 1>(0, 3) = neg_P_r0_star_Jinv * t1;
		K.block<3, 1>(0, 4) = neg_P_r0_star_Jinv * t2;
		K.block<3, 1>(0, 5) = neg_P_r0_star_Jinv * axis;
		K.block<1, 3>(3, 0) = K.block<3, 1>(0, 3).transpose();
		K.block<1, 3>(4, 0) = K.block<3, 1>(0, 4).transpose();
		K.block<1, 3>(5, 0) = K.block<3, 1>(0, 5).transpose();
		K(3, 3) = t1.transpose() * Jinv_t1;
		K(3, 4) = t1.transpose() * Jinv_t2;
		K(3, 5) = t1.transpose() * Jinv_axis;
		K(4, 3) = K(3, 4);
		K(5, 3) = K(3, 5);
		K(4, 4) = t2.transpose() * Jinv_t2;
		K(4, 5) = t2.transpose() * Jinv_axis;
		K(5, 4) = K(4, 5);
		K(5, 5) = axis.transpose() * Jinv_axis;
	}
	if (invMass1 != 0.0f)
	{
		// Jacobian for body 1 is
		//
		// ( -I_3   r1*  )
		// ( 0     -t1^T )
		// ( 0     -t2^T )
		// ( 0     axis^T)
		//
		// where I_3 is the identity matrix and r1* is the cross product matrix of r1
		//
		// J M^-1 J^T =
		// ( 1/m I_3-r1 * J1^-1 * r1*    -r1 * J1^-1 * t1    -r1 * J1^-1 * t2		-r1 * J1^-1 * axis)
		// ( (-r1 * J1^-1 * t1)^T        t1^T * J1^-1 t1     t1^T * J1^-1 t2		t1^T * J1^-1 * axis)
		// ( (-r1 * J1^-1 * t2)^T        t2^T * J1^-1 t1     t2^T * J1^-1 t2		t2^T * J1^-1 axis)
		// ( (-r1 * J1^-1 * axis)^T      axis^T * J1^-1 t1   axis^T * J1^-1	t2		axis^T * J1^-1 axis)

		Eigen::Matrix3f K11;
		computeMatrixK(c1, invMass1, x1, inertiaInverseW1, K11);
		const Eigen::Matrix3f neg_P_r1_star_Jinv = -r1_star * inertiaInverseW1;
		const Eigen::Vector3f Jinv_axis = inertiaInverseW1 * axis;
		const Eigen::Vector3f Jinv_t1 = inertiaInverseW1 * t1;
		const Eigen::Vector3f Jinv_t2 = inertiaInverseW1 * t2;

		K.block<3, 3>(0, 0) += K11;
		const Eigen::Vector3f K_03 = neg_P_r1_star_Jinv * t1;
		const Eigen::Vector3f K_04 = neg_P_r1_star_Jinv * t2;
		const Eigen::Vector3f K_05 = neg_P_r1_star_Jinv * axis;
		K.block<3, 1>(0, 3) += K_03;
		K.block<3, 1>(0, 4) += K_04;
		K.block<3, 1>(0, 5) += K_05;
		K.block<1, 3>(3, 0) += K_03.transpose();
		K.block<1, 3>(4, 0) += K_04.transpose();
		K.block<1, 3>(5, 0) += K_05.transpose();
		K(3, 3) += t1.transpose() * Jinv_t1;
		const float K_34 = t1.transpose() * Jinv_t2;
		K(3, 4) += K_34;
		const float K_35 = t1.transpose() * Jinv_axis;
		K(3, 5) += K_35;
		K(4, 3) += K_34;
		K(4, 4) += t2.transpose() * Jinv_t2;
		const float K_45 = t2.transpose() * Jinv_axis;
		K(4, 5) += K_45;
		K(5, 3) += K_35;	
		K(5, 4) += K_45;
		K(5, 5) += axis.transpose() * Jinv_axis;
	}


	const Eigen::Matrix<float, 6, 6> Kinv = K.inverse();

	const Eigen::Matrix<float, 6, 1> lambda = Kinv * b;
	const Eigen::Vector3f pt = lambda.block<3, 1>(0, 0);

	const Eigen::Vector3f angMomentum = t1*lambda(3, 0) + t2*lambda(4, 0) + axis*lambda(5, 0);

	if (invMass0 != 0.0f)
	{
		corr_x0 = invMass0*pt;
		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt) + angMomentum));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (invMass1 != 0.0f)
	{
		corr_x1 = -invMass1*pt;
		const Eigen::Vector3f ot = (inertiaInverseW1 * (r1.cross(-pt) - angMomentum));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q1.coeffs() = 0.5f *(otQ*q1).coeffs();
	}

	return true;
}


// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::init_TargetVelocityMotorHingeJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	const Eigen::Vector3f &position,
	const Eigen::Vector3f &direction,
	Eigen::Matrix<float, 3, 14> &jointInfo
	)
{
	return init_TargetAngleMotorHingeJoint(x0, q0, x1, q1, position, direction, jointInfo);
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::update_TargetVelocityMotorHingeJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	Eigen::Matrix<float, 3, 14> &jointInfo
	)
{
	return update_TargetAngleMotorHingeJoint(x0, q0, x1, q1, jointInfo);
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::solve_TargetVelocityMotorHingeJoint(
	const float invMass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float invMass1,
	const Eigen::Vector3f &x1,
	const Eigen::Matrix3f &inertiaInverseW1,
	const Eigen::Quaternionf &q1,
	const Eigen::Matrix<float, 3, 14> &jointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1)
{
	Eigen::Matrix<float, 3, 12> hingeJointInfo = jointInfo.block<3, 12>(0, 0);
	const bool res = solve_HingeJoint(	invMass0, x0, inertiaInverseW0, q0, 
												invMass1, x1, inertiaInverseW1, q1, 
												hingeJointInfo, corr_x0, corr_q0, corr_x1, corr_q1);
	return res;
}


// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::velocitySolve_TargetVelocityMotorHingeJoint(
	const float invMass0,
	const Eigen::Vector3f &x0,
	const Eigen::Vector3f &v0,
	const Eigen::Matrix3f &inertiaInverseW0,	
	const Eigen::Vector3f &omega0,
	const float invMass1,
	const Eigen::Vector3f &x1,
	const Eigen::Vector3f &v1,
	const Eigen::Matrix3f &inertiaInverseW1,
	const Eigen::Vector3f &omega1,
	const float targetAngularVelocity,
	const Eigen::Matrix<float, 3, 14> &jointInfo,
	Eigen::Vector3f &corr_v0, Eigen::Vector3f &corr_omega0,
	Eigen::Vector3f &corr_v1, Eigen::Vector3f &corr_omega1)
{
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2-4:	coordinate system of body 0 (local)
	// 5:	joint axis in body 1 (local)
	// 6:	connector in body 0 (global)
	// 7:	connector in body 1 (global)
	// 8-10:coordinate system of body 0 (global)
	// 11:	joint axis in body 1 (global)
	// 12:	perpendicular vector on joint axis (normalized) in body 1 (local)
	// 13:	perpendicular vector on joint axis (normalized) in body 1 (global)

	const Eigen::Vector3f &c0 = jointInfo.col(6);
	const Eigen::Vector3f &c1 = jointInfo.col(7);
	const Eigen::Vector3f &axis0 = jointInfo.col(8);
	const Eigen::Vector3f &axis1 = jointInfo.col(11);
	const Eigen::Vector3f axis = 0.5 * (axis0 + axis1);
	const Eigen::Vector3f u = axis0.cross(axis1);
	const Eigen::Vector3f &t1 = jointInfo.col(9);
	const Eigen::Vector3f &t2 = jointInfo.col(10);
	const Eigen::Vector3f &t3 = jointInfo.col(13);
	const Eigen::Vector3f r0 = c0 - x0;
	const Eigen::Vector3f r1 = c1 - x1;
	Eigen::Matrix3f r0_star, r1_star;
	MathFunctions::crossProductMatrix(r0, r0_star);
	MathFunctions::crossProductMatrix(r1, r1_star);
	Eigen::Vector3f deltaOmega = omega1 - omega0;

	Eigen::Matrix<float, 6, 1> b;
	b.block<3, 1>(0, 0) = v1 - v0;
	b(3, 0) = t1.dot(deltaOmega);
	b(4, 0) = t2.dot(deltaOmega);

 	// determine correction angle
 	float delta = targetAngularVelocity;
	delta -= axis.dot(deltaOmega);

	b(5, 0) = -delta;

	Eigen::Matrix<float, 6, 6> K;
	K.setZero();
	if (invMass0 != 0.0f)
	{
		// Jacobian for body 0 is
		//
		// (I_3   -r0*)
		// (0     t1^T)
		// (0     t2^T)
		// (0     axis^T)
		//
		// where I_3 is the identity matrix and r0* is the cross product matrix of r0
		//
		// J M^-1 J^T =
		// ( 1/m I_3-r0 * J0^-1 * r0*    -r0 * J0^-1 * t1    -r0 * J0^-1 * t2		-r0 * J0^-1 * axis)
		// ( (-r0 * J0^-1 * t1)^T        t1^T * J0^-1 t1     t1^T * J0^-1 * t2		t1^T * J0^-1 * axis)
		// ( (-r0 * J0^-1 * t2)^T        t2^T * J0^-1 t1     t2^T * J0^-1 * t2		t2^T * J0^-1 axis)
		// ( (-r0 * J0^-1 * axis)^T      axis^T * J0^-1 t1   axis^T * J0^-1 t2		axis^T * J0^-1 axis)

		Eigen::Matrix3f K00;
		computeMatrixK(c0, invMass0, x0, inertiaInverseW0, K00);

		K.block<3, 3>(0, 0) = K00;
		K.block<3, 1>(0, 3) = -r0_star * inertiaInverseW0 * t1;
		K.block<3, 1>(0, 4) = -r0_star * inertiaInverseW0 * t2;
		K.block<3, 1>(0, 5) = -r0_star * inertiaInverseW0 * axis;
		K.block<1, 3>(3, 0) = K.block<3, 1>(0, 3).transpose();
		K.block<1, 3>(4, 0) = K.block<3, 1>(0, 4).transpose();
		K.block<1, 3>(5, 0) = K.block<3, 1>(0, 5).transpose();
		K(3, 3) = t1.transpose() * inertiaInverseW0 * t1;
		K(3, 4) = t1.transpose() * inertiaInverseW0 * t2;
		K(3, 5) = t1.transpose() * inertiaInverseW0 * axis;
		K(4, 3) = K(3, 4);
		K(5, 3) = K(3, 5);
		K(4, 4) = t2.transpose() * inertiaInverseW0 * t2;
		K(4, 5) = t2.transpose() * inertiaInverseW0 * axis;
		K(5, 4) = K(4, 5);
		K(5, 5) = axis.transpose() * inertiaInverseW0 * axis;
	}
	if (invMass1 != 0.0f)
	{
		// Jacobian for body 1 is
		//
		// ( -I_3   r1*  )
		// ( 0     -t1^T )
		// ( 0     -t2^T )
		// ( 0     axis^T)
		//
		// where I_3 is the identity matrix and r1* is the cross product matrix of r1
		//
		// J M^-1 J^T =
		// ( 1/m I_3-r1 * J1^-1 * r1*    -r1 * J1^-1 * t1    -r1 * J1^-1 * t2		-r1 * J1^-1 * axis)
		// ( (-r1 * J1^-1 * t1)^T        t1^T * J1^-1 t1     t1^T * J1^-1 t2		t1^T * J1^-1 * axis)
		// ( (-r1 * J1^-1 * t2)^T        t2^T * J1^-1 t1     t2^T * J1^-1 t2		t2^T * J1^-1 axis)
		// ( (-r1 * J1^-1 * axis)^T      axis^T * J1^-1 t1   axis^T * J1^-1	t2		axis^T * J1^-1 axis)

		Eigen::Matrix3f K11;
		computeMatrixK(c1, invMass1, x1, inertiaInverseW1, K11);

		K.block<3, 3>(0, 0) += K11;
		const Eigen::Vector3f K_03 = -r1_star * inertiaInverseW1 * t1;
		const Eigen::Vector3f K_04 = -r1_star * inertiaInverseW1 * t2;
		const Eigen::Vector3f K_05 = -r1_star * inertiaInverseW1 * axis;
		K.block<3, 1>(0, 3) += K_03;
		K.block<3, 1>(0, 4) += K_04;
		K.block<3, 1>(0, 5) += K_05;
		K.block<1, 3>(3, 0) += K_03.transpose();
		K.block<1, 3>(4, 0) += K_04.transpose();
		K.block<1, 3>(5, 0) += K_05.transpose();
		K(3, 3) += t1.transpose() * inertiaInverseW1 * t1;
		const float K_34 = t1.transpose() * inertiaInverseW1 * t2;
		K(3, 4) += K_34;
		const float K_35 = t1.transpose() * inertiaInverseW1 * axis;
		K(3, 5) += K_35;
		K(4, 3) += K_34;
		K(4, 4) += t2.transpose() * inertiaInverseW1 * t2;
		const float K_45 = t2.transpose() * inertiaInverseW1 * axis;
		K(4, 5) += K_45;
		K(5, 3) += K_35;
		K(5, 4) += K_45;
		K(5, 5) += axis.transpose() * inertiaInverseW1 * axis;
	}

	const Eigen::Matrix<float, 6, 6> Kinv = K.inverse();

	const Eigen::Matrix<float, 6, 1> lambda = Kinv * b;
	const Eigen::Vector3f p = lambda.block<3, 1>(0, 0);

	Eigen::Vector3f angMomentum = t1*lambda(3, 0) + t2*lambda(4, 0) + axis*lambda(5, 0);

	if (invMass0 != 0.0f)
	{
		corr_v0 = invMass0*p;
		corr_omega0 = (inertiaInverseW0 * (r0.cross(p) + angMomentum));
	}

	if (invMass1 != 0.0f)
	{
		corr_v1 = -invMass1*p;
		corr_omega1 = (inertiaInverseW1 * (r1.cross(-p) - angMomentum));
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::init_RigidBodyParticleBallJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	Eigen::Matrix<float, 3, 2> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in rigid body (local)
	// 1:	connector in rigid body (global)

	// transform in local coordinates
	const Eigen::Matrix3f rot0T = q0.matrix().transpose();

	jointInfo.col(0) = rot0T * (x1 - x0);
	jointInfo.col(1) = x1;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::update_RigidBodyParticleBallJoint(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	Eigen::Matrix<float, 3, 2> &jointInfo
	)
{
	// jointInfo contains
	// 0:	connector in rigid body (local)
	// 1:	connector in rigid body (global)

	// compute world space position of connector
	const Eigen::Matrix3f rot0 = q0.matrix();
	jointInfo.col(1) = rot0 * jointInfo.col(0) + x0;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::solve_RigidBodyParticleBallJoint(
	const float invMass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float invMass1,
	const Eigen::Vector3f &x1,
	const Eigen::Matrix<float, 3, 2> &jointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1)
{
	// jointInfo contains
	// 0:	connector in rigid body (local)
	// 1:	connector in rigid body (global)

	const Eigen::Vector3f &connector0 = jointInfo.col(1);

	// Compute Kinv
	Eigen::Matrix3f K1, K2;
	computeMatrixK(connector0, invMass0, x0, inertiaInverseW0, K1);

	K2.setZero();
	if (invMass1 != 0.0f)
	{
		K2(0, 0) = invMass1;
		K2(1, 1) = invMass1;
		K2(2, 2) = invMass1;
	}
	const Eigen::Matrix3f Kinv = (K1 + K2).inverse();

	const Eigen::Vector3f pt = Kinv * (x1 - connector0);

	if (invMass0 != 0.0f)
	{
		const Eigen::Vector3f r0 = connector0 - x0;
		corr_x0 = invMass0*pt;

		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (invMass1 != 0.0f)
	{
		corr_x1 = -invMass1*pt;
	}			

	return true;
}
