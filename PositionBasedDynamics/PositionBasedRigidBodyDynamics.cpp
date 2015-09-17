#include "PositionBasedRigidBodyDynamics.h"
#include <cfloat>
#include <iostream>

using namespace PBD;

// ----------------------------------------------------------------------------------------------
void PositionBasedRigidBodyDynamics::computeMatrixK(
	const Eigen::Vector3f &connector,
	const float mass,
	const Eigen::Vector3f &x,
	const Eigen::Matrix3f &inertiaInverseW,
	Eigen::Matrix3f &K)
{
	if (mass != 0.0f)
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

		const float m = (1.0f / mass);
		K(0,0) = c*c*j22 - b*c*(j23 + j23) + b*b*j33 + m;
		K(0,1) = -(c*c*j12) + a*c*j23 + b*c*j13 - a*b*j33;
		K(0,2) = b*c*j12 - a*c*j22 - b*b*j13 + a*b*j23;
		K(1,0) = K(0,1);
		K(1,1) = c*c*j11 - a*c*(j13 + j13) + a*a*j33 + m;
		K(1,2) = -(b*c*j11) + a*c*j12 + a*b*j13 - a*a*j23;
		K(2,0) = K(0,2);
		K(2,1) = K(1,2);
		K(2,2) = b*b*j11 - a*b*(j12 + j12) + a*a*j22 + m;
	}
	else
		K.setZero();
}

// ----------------------------------------------------------------------------------------------
void PositionBasedRigidBodyDynamics::computeMatrixK(
	const Eigen::Vector3f &connector0,
	const Eigen::Vector3f &connector1,
	const float mass,
	const Eigen::Vector3f &x,
	const Eigen::Matrix3f &inertiaInverseW,
	Eigen::Matrix3f &K)
{
	if (mass != 0.0f)
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

		const float m = (1.0f / mass);
		K(0, 0) = c*f*j22 - c*e*j23 - b*f*j23 + b*e*j33 + m;
		K(0, 1) = -(c*f*j12) + c*d*j23 + b*f*j13 - b*d*j33;
		K(0, 2) = c*e*j12 - c*d*j22 - b*e*j13 + b*d*j23;
		K(1, 0) = -(c*f*j12) + c*e*j13 + a*f*j23 - a*e*j33;
		K(1, 1) = c*f*j11 - c*d*j13 - a*f*j13 + a*d*j33 + m;
		K(1, 2) = -(c*e*j11) + c*d*j12 + a*e*j13 - a*d*j23;
		K(2, 0) = b*f*j12 - b*e*j13 - a*f*j22 + a*e*j23;
		K(2, 1) = -(b*f*j11) + b*d*j13 + a*f*j12 - a*d*j23;
		K(2, 2) = b*e*j11 - b*d*j12 - a*e*j12 + a*d*j22 + m;
	}
	else
		K.setZero();
}


// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::initRigidBodyBallJointInfo(
	const Eigen::Vector3f &x0, 						
	const Eigen::Quaternionf &q0,					
	const Eigen::Vector3f &x1, 						
	const Eigen::Quaternionf &q1,					
	const Eigen::Vector3f &ballJointPosition,		
	Eigen::Matrix<float, 3, 4> &ballJointInfo
	)
{
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
bool PositionBasedRigidBodyDynamics::updateRigidBodyBallJointInfo(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	Eigen::Matrix<float, 3, 4> &ballJointInfo
	)
{
	// compute world space positions of connectors
	const Eigen::Matrix3f rot0 = q0.matrix();
	const Eigen::Matrix3f rot1 = q1.matrix();
	ballJointInfo.col(2) = rot0 * ballJointInfo.col(0) + x0;
	ballJointInfo.col(3) = rot1 * ballJointInfo.col(1) + x1;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::solveRigidBodyBallJoint(
	const float mass0,								
	const Eigen::Vector3f &x0, 						
	const Eigen::Matrix3f &inertiaInverseW0,		
	const Eigen::Quaternionf &q0,					
	const float mass1,								
	const Eigen::Vector3f &x1, 						
	const Eigen::Matrix3f &inertiaInverseW1,		
	const Eigen::Quaternionf &q1,	
	const Eigen::Matrix<float, 3, 4> &ballJointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1)
{
	const Eigen::Vector3f &connector0 = ballJointInfo.col(2);
	const Eigen::Vector3f &connector1 = ballJointInfo.col(3);

	// Compute Kinv
	Eigen::Matrix3f K1, K2;
	computeMatrixK(connector0, mass0, x0, inertiaInverseW0, K1);
	computeMatrixK(connector1, mass1, x1, inertiaInverseW1, K2);
	const Eigen::Matrix3f Kinv = (K1 + K2).inverse();

	const Eigen::Vector3f pt = Kinv * (connector1 - connector0);

	if (mass0 != 0.0f)
	{
		const Eigen::Vector3f r0 = connector0 - x0;
		corr_x0 = (1.0f / mass0)*pt;

		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (mass1 != 0.0f)
	{
		const Eigen::Vector3f r1 = connector1 - x1;
		corr_x1 = -(1.0f / mass1)*pt;

		const Eigen::Vector3f ot = (inertiaInverseW1 * (r1.cross(-pt)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q1.coeffs() = 0.5f *(otQ*q1).coeffs();
	}

	return true;
}


// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::initRigidBodyBallOnLineJointInfo(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	const Eigen::Vector3f &position,
	const Eigen::Vector3f &direction,
	Eigen::Matrix<float, 3, 10> &jointInfo
	)
{
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
bool PositionBasedRigidBodyDynamics::updateRigidBodyBallOnLineJointInfo(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	Eigen::Matrix<float, 3, 10> &jointInfo
	)
{
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
bool PositionBasedRigidBodyDynamics::solveRigidBodyBallOnLineJoint(
	const float mass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float mass1,
	const Eigen::Vector3f &x1,
	const Eigen::Matrix3f &inertiaInverseW1,
	const Eigen::Quaternionf &q1,
	const Eigen::Matrix<float, 3, 10> &jointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1)
{
	const Eigen::Vector3f &connector0 = jointInfo.col(5);
	const Eigen::Vector3f &connector1 = jointInfo.col(6);

	// Compute Kinv
	Eigen::Matrix3f K1, K2;
	computeMatrixK(connector0, mass0, x0, inertiaInverseW0, K1);
	computeMatrixK(connector1, mass1, x1, inertiaInverseW1, K2);

	// projection 
	const Eigen::Matrix<float, 3, 2> PT = jointInfo.block<3, 2>(0, 8);
	const Eigen::Matrix<float, 2, 3> P = PT.transpose();

	const Eigen::Matrix2f K = P * (K1 + K2) * PT;
	const Eigen::Matrix2f Kinv = K.inverse();

	const Eigen::Vector2f pt2D = Kinv * (P * (connector1 - connector0));
	const Eigen::Vector3f pt = PT * pt2D;

	if (mass0 != 0.0f)
	{
		const Eigen::Vector3f r0 = connector0 - x0;
		corr_x0 = (1.0f / mass0)*pt;

		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (mass1 != 0.0f)
	{
		const Eigen::Vector3f r1 = connector1 - x1;
		corr_x1 = -(1.0f / mass1)*pt;

		const Eigen::Vector3f ot = (inertiaInverseW1 * (r1.cross(-pt)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q1.coeffs() = 0.5f *(otQ*q1).coeffs();
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::initRigidBodyHingeJointInfo(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	const Eigen::Vector3f &hingeJointPosition,
	const Eigen::Vector3f &hingeJointAxis,
	Eigen::Matrix<float, 3, 14> &hingeJointInfo
	)
{
	Eigen::Vector3f axis = hingeJointAxis;
	axis.normalize();
	const Eigen::Vector3f p1 = hingeJointPosition + 0.5f * axis;
	const Eigen::Vector3f p2 = hingeJointPosition - 0.5f * axis;

	Eigen::Matrix<float, 3, 4> jointInfo1;
	initRigidBodyBallJointInfo(x0, q0, x1, q1, p1, jointInfo1);

	Eigen::Matrix<float, 3, 10> jointInfo2;
	initRigidBodyBallOnLineJointInfo(x0, q0, x1, q1, p2, axis, jointInfo2);

	hingeJointInfo.block<3, 4>(0, 0) = jointInfo1;
	hingeJointInfo.block<3, 10>(0, 4) = jointInfo2;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::updateRigidBodyHingeJointInfo(
	const Eigen::Vector3f &x0,
	const Eigen::Quaternionf &q0,
	const Eigen::Vector3f &x1,
	const Eigen::Quaternionf &q1,
	Eigen::Matrix<float, 3, 14> &hingeJointInfo
	)
{
	Eigen::Matrix<float, 3, 4> jointInfo1 = hingeJointInfo.block<3, 4>(0, 0);
	updateRigidBodyBallJointInfo(x0, q0, x1, q1, jointInfo1);
	hingeJointInfo.block<3, 4>(0, 0) = jointInfo1;

	Eigen::Matrix<float, 3, 10> jointInfo2 = hingeJointInfo.block<3, 10>(0, 4);
	updateRigidBodyBallOnLineJointInfo(x0, q0, x1, q1, jointInfo2);
	hingeJointInfo.block<3, 10>(0, 4) = jointInfo2;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::solveRigidBodyHingeJoint(
	const float mass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float mass1,
	const Eigen::Vector3f &x1,
	const Eigen::Matrix3f &inertiaInverseW1,
	const Eigen::Quaternionf &q1,
	const Eigen::Matrix<float, 3, 14> &hingeJointInfo,
	Eigen::Vector3f &corr_x0, Eigen::Quaternionf &corr_q0,
	Eigen::Vector3f &corr_x1, Eigen::Quaternionf &corr_q1)
{
	//////////////////////////////////////////////////////////////////////////
	// solve ball joint
	//////////////////////////////////////////////////////////////////////////	
	Eigen::Vector3f local_corr_x0, local_corr_x1;
	Eigen::Quaternionf local_corr_q0, local_corr_q1;

	Eigen::Matrix<float, 3, 4> jointInfo1 = hingeJointInfo.block<3, 4>(0, 0);
	const bool res1 = solveRigidBodyBallJoint(mass0, x0, inertiaInverseW0, q0,
		mass1, x1, inertiaInverseW1, q1,
		jointInfo1,
		local_corr_x0, local_corr_q0, local_corr_x1, local_corr_q1);
	if (res1)
	{
		corr_x0 = local_corr_x0;
		corr_x1 = local_corr_x1;
		corr_q0 = local_corr_q0;
		corr_q1 = local_corr_q1;
	}
	else
	{
		corr_x0.setZero();
		corr_x1.setZero();
		corr_q0 = Eigen::Quaternionf(0.0f, 0.0f, 0.0f, 0.0f);
		corr_q1 = Eigen::Quaternionf(0.0f, 0.0f, 0.0f, 0.0f);
	}

	Eigen::Vector3f x0c = x0;
	Eigen::Vector3f x1c = x1;
	Eigen::Quaternionf q0c = q0;
	Eigen::Quaternionf q1c = q1;

	if (mass0 != 0.0f)
	{
		x0c += local_corr_x0;
		q0c.coeffs() += local_corr_q0.coeffs();
		q0c.normalize();
	}
	if (mass1 != 0.0f)
	{
		x1c += local_corr_x1;
		q1c.coeffs() += local_corr_q1.coeffs();
		q1c.normalize();
	}

	//////////////////////////////////////////////////////////////////////////
	// solve ball-on-line-joint
	//////////////////////////////////////////////////////////////////////////
	Eigen::Matrix<float, 3, 10> jointInfo2 = hingeJointInfo.block<3, 10>(0, 4);
	updateRigidBodyBallOnLineJointInfo(x0c, q0c, x1c, q1c, jointInfo2);

	const bool res2 = solveRigidBodyBallOnLineJoint(mass0, x0c, inertiaInverseW0, q0c,
		mass1, x1c, inertiaInverseW1, q1c,
		jointInfo2,
		local_corr_x0, local_corr_q0, local_corr_x1, local_corr_q1);

	if (res2)
	{
		corr_x0 += local_corr_x0;
		corr_x1 += local_corr_x1;
		corr_q0.coeffs() += local_corr_q0.coeffs();
		corr_q1.coeffs() += local_corr_q1.coeffs();
	}

	return (res1 && res2);
}

