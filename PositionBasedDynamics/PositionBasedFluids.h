#ifndef POSITION_BASED_FLUIDS_H
#define POSITION_BASED_FLUIDS_H

#include <Eigen/Dense>

// ------------------------------------------------------------------------------------
namespace PBD
{
	class PositionBasedFluids
	{
	public:
		// -------------- Position Based Fluids  -----------------------------------------------------
		
		static bool computePBFDensity(
			const unsigned int particleIndex,				// current fluid particle	
			const unsigned int numberOfParticles,			// number of fluid particles 
			const Eigen::Vector3f x[],						// array of all particle positions
			const float mass[],								// array of all particle masses
			const Eigen::Vector3f boundaryX[],				// array of all boundary particles
			const float boundaryPsi[],						// array of all boundary psi values (Akinci2012)
			const unsigned int numNeighbors,				// number of neighbors 
			const unsigned int neighbors[],					// array with indices of all neighbors (indices larger than numberOfParticles are boundary particles)
			const float density0,							// rest density
			const bool boundaryHandling,					// perform boundary handling (Akinci2012)
			float &density_err,								// returns the clamped density error (can be used for enforcing a maximal global density error)
			float &density);								// return the density

		static bool computePBFLagrangeMultiplier(
			const unsigned int particleIndex,				// current fluid particle	
			const unsigned int numberOfParticles,			// number of fluid particles 
			const Eigen::Vector3f x[],						// array of all particle positions
			const float mass[],								// array of all particle masses
			const Eigen::Vector3f boundaryX[],				// array of all boundary particles
			const float boundaryPsi[],						// array of all boundary psi values (Akinci2012)
			const float density,							// density of current fluid particle
			const unsigned int numNeighbors,				// number of neighbors 
			const unsigned int neighbors[],					// array with indices of all neighbors
			const float density0,							// rest density
			const bool boundaryHandling,					// perform boundary handling (Akinci2012)
			float &lambda);									// returns the Lagrange multiplier

		static bool solveDensityConstraint(
			const unsigned int particleIndex,				// current fluid particle	
			const unsigned int numberOfParticles,			// number of fluid particles 
			const Eigen::Vector3f x[],						// array of all particle positions
			const float mass[],								// array of all particle masses
			const Eigen::Vector3f boundaryX[],				// array of all boundary particles
			const float boundaryPsi[],						// array of all boundary psi values (Akinci2012)
			const unsigned int numNeighbors,				// number of neighbors 
			const unsigned int neighbors[],					// array with indices of all neighbors
			const float density0,							// rest density
			const bool boundaryHandling,					// perform boundary handling (Akinci2012)
			const float lambda[],							// Lagrange multiplier
			Eigen::Vector3f &corr);							// returns the position correction for the current fluid particle
	};
}

#endif