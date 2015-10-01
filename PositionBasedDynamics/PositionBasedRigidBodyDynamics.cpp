#include "PositionBasedRigidBodyDynamics.h"
#include "MathFunctions.h"
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
bool PositionBasedRigidBodyDynamics::updateRigidBodyBallJointInfo(
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
	// jointInfo contains
	// 0:	connector in body 0 (local)
	// 1:	connector in body 1 (local)
	// 2:	connector in body 0 (global)
	// 3:	connector in body 1 (global)

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
bool PositionBasedRigidBodyDynamics::updateRigidBodyBallOnLineJointInfo(
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
	if (fabs(v.dot(jointInfo.col(7))) > 0.99f)
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
bool PositionBasedRigidBodyDynamics::updateRigidBodyHingeJointInfo(
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
bool PositionBasedRigidBodyDynamics::solveRigidBodyHingeJoint(
	const float mass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float mass1,
	const Eigen::Vector3f &x1,
	const Eigen::Matrix3f &inertiaInverseW1,
	const Eigen::Quaternionf &q1,
	const Eigen::Matrix<float, 3, 12> &hingeJointInfo,
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

	const Eigen::Vector3f c0 = hingeJointInfo.col(6);
	const Eigen::Vector3f c1 = hingeJointInfo.col(7);
	const Eigen::Vector3f axis0 = hingeJointInfo.col(8);
	const Eigen::Vector3f axis1 = hingeJointInfo.col(11);
	const Eigen::Vector3f u = axis0.cross(axis1);
	const Eigen::Vector3f t1 = hingeJointInfo.col(9);
	const Eigen::Vector3f t2 = hingeJointInfo.col(10);
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
	Eigen::Matrix<float, 5, 6> J0, J1;
	if (mass0 != 0.0f)
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
		// ( (-r0 * J0^-1 * t1)^T        t1^T * J0^-1 t1     t1^T * J0^-1 t2  )
		// ( (-r0 * J0^-1 * t2)^T        t2^T * J0^-1 t1     t2^T * J0^-1 t2  )

		Eigen::Matrix3f K00;
		computeMatrixK(c0, mass0, x0, inertiaInverseW0, K00);

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
	if (mass1 != 0.0f)
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
		// ( (-r1 * J1^-1 * t1)^T        t1^T * J1^-1 t1     t1^T * J1^-1 t2  )
		// ( (-r1 * J1^-1 * t2)^T        t2^T * J1^-1 t1     t2^T * J1^-1 t2  )

		Eigen::Matrix3f K11;
		computeMatrixK(c1, mass1, x1, inertiaInverseW1, K11);

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

	if (mass0 != 0.0f)
	{
		corr_x0 = (1.0f / mass0)*pt;
		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt) + t1*lambda(3, 0) + t2*lambda(4, 0)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (mass1 != 0.0f)
	{
		corr_x1 = -(1.0f / mass1)*pt;
		const Eigen::Vector3f ot = (inertiaInverseW1 * (r1.cross(-pt) - t1*lambda(3, 0) - t2*lambda(4, 0)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q1.coeffs() = 0.5f *(otQ*q1).coeffs();
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedRigidBodyDynamics::initRigidBodyUniversalJointInfo(
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
bool PositionBasedRigidBodyDynamics::updateRigidBodyUniversalJointInfo(
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
bool PositionBasedRigidBodyDynamics::solveRigidBodyUniversalJoint(
	const float mass0,
	const Eigen::Vector3f &x0,
	const Eigen::Matrix3f &inertiaInverseW0,
	const Eigen::Quaternionf &q0,
	const float mass1,
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

	const Eigen::Vector3f c0 = jointInfo.col(4);
	const Eigen::Vector3f c1 = jointInfo.col(5);
	const Eigen::Vector3f axis0 = jointInfo.col(6);
	const Eigen::Vector3f axis1 = jointInfo.col(7);
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
	if (mass0 != 0.0f)
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
		computeMatrixK(c0, mass0, x0, inertiaInverseW0, K00);

		K.block<3, 3>(0, 0) = K00;
		K.block<3, 1>(0, 3) = -r0_star * inertiaInverseW0 * u;
		K.block<1, 3>(3, 0) = K.block<3, 1>(0, 3).transpose();
		K(3, 3) = u.transpose() * inertiaInverseW0 * u;
	}
	if (mass1 != 0.0f)
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
		computeMatrixK(c1, mass1, x1, inertiaInverseW1, K11);

		K.block<3, 3>(0, 0) += K11;
		const Eigen::Vector3f K_03 = -r1_star * inertiaInverseW1 * u;
		K.block<3, 1>(0, 3) += K_03;
		K.block<1, 3>(3, 0) += K_03.transpose();
		K(3, 3) += u.transpose() * inertiaInverseW1 * u;
	}

	const Eigen::Matrix<float, 4, 4> Kinv = K.inverse();

	const Eigen::Matrix<float, 4, 1> lambda = Kinv * b;
	const Eigen::Vector3f pt = lambda.block<3, 1>(0, 0);

	if (mass0 != 0.0f)
	{
		corr_x0 = (1.0f / mass0)*pt;
		const Eigen::Vector3f ot = (inertiaInverseW0 * (r0.cross(pt) + u*lambda(3, 0)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q0.coeffs() = 0.5f *(otQ*q0).coeffs();
	}

	if (mass1 != 0.0f)
	{
		corr_x1 = -(1.0f / mass1)*pt;
		const Eigen::Vector3f ot = (inertiaInverseW1 * (r1.cross(-pt) - u*lambda(3, 0)));
		const Eigen::Quaternionf otQ(0.0f, ot[0], ot[1], ot[2]);
		corr_q1.coeffs() = 0.5f *(otQ*q1).coeffs();
	}

	return true;
}

	// ----------------------------------------------------------------------------------------------
	bool PositionBasedRigidBodyDynamics::initRigidBodyParticleBallJointInfo(
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
	bool PositionBasedRigidBodyDynamics::updateRigidBodyParticleBallJointInfo(
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
	bool PositionBasedRigidBodyDynamics::solveRigidBodyParticleBallJoint(
		const float mass0,
		const Eigen::Vector3f &x0,
		const Eigen::Matrix3f &inertiaInverseW0,
		const Eigen::Quaternionf &q0,
		const float mass1,
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
		computeMatrixK(connector0, mass0, x0, inertiaInverseW0, K1);

		K2.setZero();
		if (mass1 != 0.0f)
		{
			const float invMass1 = 1.0f / mass1;
			K2(0, 0) = invMass1;
			K2(1, 1) = invMass1;
			K2(2, 2) = invMass1;
		}
		const Eigen::Matrix3f Kinv = (K1 + K2).inverse();

		const Eigen::Vector3f pt = Kinv * (x1 - connector0);

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
			corr_x1 = -(1.0f / mass1)*pt;
		}			

		return true;
	}