#include "TimeIntegration.h"

using namespace PBD;


// ----------------------------------------------------------------------------------------------
void TimeIntegration::semiImplicitEuler(
	const float h, 
	const float mass, 
	Eigen::Vector3f &position,
	Eigen::Vector3f &velocity,
	const Eigen::Vector3f &acceleration)
{				
	if (mass != 0.0f)
	{
		velocity += acceleration * h;
		position += velocity * h;
	}
}

// ----------------------------------------------------------------------------------------------
void TimeIntegration::semiImplicitEulerRotation(
	const float h,
	const float mass,
	const Eigen::Matrix3f &invInertiaW,
	Eigen::Quaternionf &rotation,
	Eigen::Vector3f &angularVelocity,	
	const Eigen::Vector3f &torque)
{
	if (mass != 0.0f)
	{
		// simple form without nutation effect
		angularVelocity += h * invInertiaW * torque;

		Eigen::Quaternionf angVelQ(0.0f, angularVelocity[0], angularVelocity[1], angularVelocity[2]);
		rotation.coeffs() += h * 0.5f * (angVelQ * rotation).coeffs();
		rotation.normalize();
	}
}

// ----------------------------------------------------------------------------------------------
void TimeIntegration::velocityUpdateFirstOrder(
	const float h,
	const float mass,
	const Eigen::Vector3f &position,
	const Eigen::Vector3f &oldPosition,
	Eigen::Vector3f &velocity)
{
	if (mass != 0.0f)
		velocity = (1.0f / h) * (position - oldPosition);
}

// ----------------------------------------------------------------------------------------------
void TimeIntegration::angularVelocityUpdateFirstOrder(
	const float h,
	const float mass,
	const Eigen::Quaternionf &rotation,
	const Eigen::Quaternionf &oldRotation,
	Eigen::Vector3f &angularVelocity)
{
	if (mass != 0.0f)
	{
		const Eigen::Quaternionf relRot = (rotation * oldRotation.conjugate());
		angularVelocity = relRot.vec() *(2.0f / h);
	}
}

// ----------------------------------------------------------------------------------------------
void TimeIntegration::velocityUpdateSecondOrder(
	const float h,
	const float mass,
	const Eigen::Vector3f &position,
	const Eigen::Vector3f &oldPosition,
	const Eigen::Vector3f &positionOfLastStep,
	Eigen::Vector3f &velocity)
{
	if (mass != 0.0f)
		velocity = (1.0f / h) * (1.5f*position - 2.0f*oldPosition + 0.5f*positionOfLastStep);
}

// ----------------------------------------------------------------------------------------------
void TimeIntegration::angularVelocityUpdateSecondOrder(
	const float h,
	const float mass,
	const Eigen::Quaternionf &rotation,				
	const Eigen::Quaternionf &oldRotation,			
	const Eigen::Quaternionf &rotationOfLastStep,	
	Eigen::Vector3f &angularVelocity)
{
	// ToDo: is still first order
	if (mass != 0.0f)
	{
		const Eigen::Quaternionf relRot = (rotation * oldRotation.conjugate());
		angularVelocity = relRot.vec() *(2.0f / h);
	}
}