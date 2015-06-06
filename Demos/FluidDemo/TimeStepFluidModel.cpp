#include "TimeStepFluidModel.h"
#include "Demos/Utils/TimeManager.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"
#include "PositionBasedDynamics/TimeIntegration.h"
#include "PositionBasedDynamics/SPHKernels.h"

using namespace PBD;
using namespace std;

TimeStepFluidModel::TimeStepFluidModel()
{
}

TimeStepFluidModel::~TimeStepFluidModel(void)
{
}

void TimeStepFluidModel::step(FluidModel &model)
{
	TimeManager *tm = TimeManager::getCurrent ();
	const float h = tm->getTimeStepSize();
	ParticleData &pd = model.getParticles();

	clearAccelerations(model);

	// Compute viscosity forces
	if (TimeManager::getCurrent()->getTime() > 0.0)			// in the first step we do not know the neighbors
	{		
		computeDensities(model);
		computeViscosityAccels(model);
	}

	// Update time step size by CFL condition
	updateTimeStepSizeCFL(model, 0.0001f, 0.005f);

	// Time integration
	for (unsigned int i = 0; i < pd.size(); i++)
	{ 
		model.getDeltaX(i).setZero();
		pd.getLastPosition(i) = pd.getOldPosition(i);
		pd.getOldPosition(i) = pd.getPosition(i);
		TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i), pd.getVelocity(i), pd.getAcceleration(i));
	}

	// Perform neighborhood search
	model.getNeighborhoodSearch()->neighborhoodSearch(&model.getParticles().getPosition(0), model.numBoundaryParticles(), &model.getBoundaryX(0));

	// Solve density constraint
	constraintProjection(model);

	// Update velocities	
	for (unsigned int i = 0; i < pd.size(); i++)
	{
		if (m_velocityUpdateMethod == 0)
			TimeIntegration::velocityUpdateFirstOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getVelocity(i));
		else
			TimeIntegration::velocityUpdateSecondOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getLastPosition(i), pd.getVelocity(i));
	}

	// Compute new time	
	tm->setTime (tm->getTime () + h);
	model.getNeighborhoodSearch()->update();
}


/** Clear accelerations and add gravitation.
 */
void TimeStepFluidModel::clearAccelerations(FluidModel &model)
{
	ParticleData &pd = model.getParticles();
	const unsigned int count = pd.size();
	const Eigen::Vector3f grav(0.0f, -9.81f, 0.0f);
	for (unsigned int i=0; i < count; i++)
	{
		// Clear accelerations of dynamic particles
		if (pd.getMass(i) != 0.0)
		{
			Eigen::Vector3f &a = pd.getAcceleration(i);
			a = grav;
		}
	}
}

/** Determine densities of all fluid particles. 
*/
void TimeStepFluidModel::computeDensities(FluidModel &model)
{
	ParticleData &pd = model.getParticles();
	const unsigned int numParticles = pd.size();
	unsigned int **neighbors = model.getNeighborhoodSearch()->getNeighbors();
	unsigned int *numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();
	
	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  
		for (int i = 0; i < (int) numParticles; i++)
		{
			float &density = model.getDensity(i);
			float density_err;
			PositionBasedDynamics::computePBFDensity(i, numParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), &model.getBoundaryPsi(0), numNeighbors[i], neighbors[i], model.getDensity0(), true, density_err, density);
		}
	}
}

/** Update time step size by CFL condition.
*/
void TimeStepFluidModel::updateTimeStepSizeCFL(FluidModel &model, const float minTimeStepSize, const float maxTimeStepSize)
{
	const float radius = model.getParticleRadius();
	const float cflFactor = 1.0f;
	float h = TimeManager::getCurrent()->getTimeStepSize();

	// Approximate max. position change due to current velocities
	float maxVel = 0.1f;
	ParticleData &pd = model.getParticles();
	const unsigned int numParticles = pd.size();
	const float diameter = 2.0f*radius;
	for (unsigned int i = 0; i < numParticles; i++)
	{
		const Eigen::Vector3f &vel = pd.getVelocity(i);
		const Eigen::Vector3f &accel = pd.getAcceleration(i);
		const float velMag = (vel + accel*h).squaredNorm();
		if (velMag > maxVel)
			maxVel = velMag;
	}

	// Approximate max. time step size 		
	h = cflFactor * .4f * (diameter / (sqrt(maxVel)));

	h = min(h, maxTimeStepSize);
	h = max(h, minTimeStepSize);

	TimeManager::getCurrent()->setTimeStepSize(h);
}

/** Compute viscosity accelerations.
*/
void TimeStepFluidModel::computeViscosityAccels(FluidModel &model)
{
	ParticleData &pd = model.getParticles();
	const unsigned int numParticles = pd.size();	

	unsigned int **neighbors = model.getNeighborhoodSearch()->getNeighbors();
	unsigned int *numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();

	const float viscosity = model.getViscosity();
	const float h = TimeManager::getCurrent()->getTimeStepSize();

	// Compute viscosity forces (XSPH)
	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  
		for (int i = 0; i < (int)numParticles; i++)
		{
			const Eigen::Vector3f &xi = pd.getPosition(i);
			const Eigen::Vector3f &vi = pd.getVelocity(i);
			const float density_i = model.getDensity(i);
			Eigen::Vector3f &ai = pd.getAcceleration(i);
			for (unsigned int j = 0; j < numNeighbors[i]; j++)
			{
				const unsigned int neighborIndex = neighbors[i][j];
				if (neighborIndex < numParticles)		// Test if fluid particle
				{
					// Viscosity
					const Eigen::Vector3f &xj = pd.getPosition(neighborIndex);
					const Eigen::Vector3f &vj = pd.getVelocity(neighborIndex);
					const float density_j = model.getDensity(neighborIndex);
					ai -= 1.0f / h * viscosity * (pd.getMass(neighborIndex) / density_j) * (vi - vj) * CubicKernel::W(xi - xj);

				}
// 				else 
// 				{
// 					const Eigen::Vector3f &xj = model.getBoundaryX(neighborIndex - numParticles);
// 					ai -= 1.0f / h * viscosity * (model.getBoundaryPsi(neighborIndex - numParticles) / density_i) * (vi)* CubicKernel::W(xi - xj);
// 				}
			}
		}
	}
}




void TimeStepFluidModel::reset(FluidModel &model)
{

}


/** Solve density constraint.
*/
void TimeStepFluidModel::constraintProjection(FluidModel &model)
{
	const unsigned int maxIter = 5;
	unsigned int iter = 0;

	ParticleData &pd = model.getParticles();
	const unsigned int nParticles = pd.size();
	unsigned int **neighbors = model.getNeighborhoodSearch()->getNeighbors();
	unsigned int *numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();

	while (iter < maxIter)
	{
		float avg_density_err = 0.0;

		#pragma omp parallel default(shared)
		{
			#pragma omp for schedule(static)  
			for (int i = 0; i < (int)nParticles; i++)
			{
				float density_err;
				PositionBasedDynamics::computePBFDensity(i, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), &model.getBoundaryPsi(0), numNeighbors[i], neighbors[i], model.getDensity0(), true, density_err, model.getDensity(i));
				PositionBasedDynamics::computePBFLagrangeMultiplier(i, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), &model.getBoundaryPsi(0), model.getDensity(i), numNeighbors[i], neighbors[i], model.getDensity0(), true, model.getLambda(i));
			}
		}
		
		#pragma omp parallel default(shared)
		{
			#pragma omp for schedule(static)  
			for (int i = 0; i < (int)nParticles; i++)
			{
				Eigen::Vector3f corr;
				PositionBasedDynamics::solveDensityConstraint(i, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), &model.getBoundaryPsi(0), numNeighbors[i], neighbors[i], model.getDensity0(), true, &model.getLambda(0), corr);
				model.getDeltaX(i) = corr;
			}
		}

		#pragma omp parallel default(shared)
		{
			#pragma omp for schedule(static)  
			for (int i = 0; i < (int)nParticles; i++)
			{
				pd.getPosition(i) += model.getDeltaX(i);
			}
		}

		iter++;
	}
}

