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