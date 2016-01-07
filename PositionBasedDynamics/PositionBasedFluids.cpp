#include "PositionBasedFluids.h"
#include <cfloat>
#include "SPHKernels.h"

using namespace PBD;

// ----------------------------------------------------------------------------------------------
bool PositionBasedFluids::computePBFDensity(
	const unsigned int particleIndex,
	const unsigned int numberOfParticles,
	const Eigen::Vector3f x[],
	const float mass[],
	const Eigen::Vector3f boundaryX[],
	const float boundaryPsi[],
	const unsigned int numNeighbors,
	const unsigned int neighbors[],
	const float density0,
	const bool boundaryHandling,
	float &density_err,
	float &density)
{
	// Compute current density for particle i
	density = mass[particleIndex] * CubicKernel::W_zero();
	for (unsigned int j = 0; j < numNeighbors; j++)
	{
		const unsigned int neighborIndex = neighbors[j];
		if (neighborIndex < numberOfParticles)		// Test if fluid particle
		{
			density += mass[neighborIndex] * CubicKernel::W(x[particleIndex] - x[neighborIndex]);
		}
		else if (boundaryHandling)
		{
			// Boundary: Akinci2012
			density += boundaryPsi[neighborIndex - numberOfParticles] * CubicKernel::W(x[particleIndex] - boundaryX[neighborIndex - numberOfParticles]);
		}
	}

	density_err = std::max(density, density0) - density0;
	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedFluids::computePBFLagrangeMultiplier(
	const unsigned int particleIndex,
	const unsigned int numberOfParticles,
	const Eigen::Vector3f x[],	
	const float mass[],
	const Eigen::Vector3f boundaryX[],
	const float boundaryPsi[],
	const float density,
	const unsigned int numNeighbors,
	const unsigned int neighbors[],
	const float density0,
	const bool boundaryHandling,
	float &lambda)
{
	const float eps = 1.0e-6f;

	// Evaluate constraint function
	const float C = std::max(density / density0 - 1.0f, 0.0f);			// clamp to prevent particle clumping at surface

	if (C != 0.0f)
	{
		// Compute gradients dC/dx_j 
		float sum_grad_C2 = 0.0;
		Eigen::Vector3f gradC_i(0.0f, 0.0f, 0.0f);

		for (unsigned int j = 0; j < numNeighbors; j++)
		{
			const unsigned int neighborIndex = neighbors[j];
			if (neighborIndex < numberOfParticles)		// Test if fluid particle
			{
				const Eigen::Vector3f gradC_j = -mass[neighborIndex] / density0 * CubicKernel::gradW(x[particleIndex] - x[neighborIndex]);
				sum_grad_C2 += gradC_j.squaredNorm();
				gradC_i -= gradC_j;
			}
			else if (boundaryHandling)
			{
				// Boundary: Akinci2012
				const Eigen::Vector3f gradC_j = -boundaryPsi[neighborIndex - numberOfParticles] / density0 * CubicKernel::gradW(x[particleIndex] - boundaryX[neighborIndex - numberOfParticles]);
				sum_grad_C2 += gradC_j.squaredNorm();
				gradC_i -= gradC_j;
			}
		}

		sum_grad_C2 += gradC_i.squaredNorm();

		// Compute lambda
		lambda = -C / (sum_grad_C2 + eps);
	}
	else
		lambda = 0.0f;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedFluids::solveDensityConstraint(
	const unsigned int particleIndex,
	const unsigned int numberOfParticles,
	const Eigen::Vector3f x[],	
	const float mass[],
	const Eigen::Vector3f boundaryX[],
	const float boundaryPsi[],
	const unsigned int numNeighbors,
	const unsigned int neighbors[],
	const float density0,
	const bool boundaryHandling,
	const float lambda[],
	Eigen::Vector3f &corr)
{
	// Compute position correction
	corr.setZero();
	for (unsigned int j = 0; j < numNeighbors; j++)
	{
		const unsigned int neighborIndex = neighbors[j];
		if (neighborIndex < numberOfParticles)		// Test if fluid particle
		{
			const Eigen::Vector3f gradC_j = -mass[neighborIndex] / density0 * CubicKernel::gradW(x[particleIndex] - x[neighborIndex]);
			corr -= (lambda[particleIndex] + lambda[neighborIndex]) * gradC_j;
		}
		else if (boundaryHandling)
		{
			// Boundary: Akinci2012
			const Eigen::Vector3f gradC_j = -boundaryPsi[neighborIndex - numberOfParticles] / density0 * CubicKernel::gradW(x[particleIndex] - boundaryX[neighborIndex - numberOfParticles]);
			corr -= (lambda[particleIndex]) * gradC_j;
		}
	}

	return true;
}
