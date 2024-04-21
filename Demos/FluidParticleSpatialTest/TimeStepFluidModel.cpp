#include "TimeStepFluidModel.h"
#include "Simulation/TimeManager.h"
#include "PositionBasedDynamics/PositionBasedFluids.h"
#include "PositionBasedDynamics/TimeIntegration.h"
#include "PositionBasedDynamics/SPHKernels.h"
#include "Simulation/Simulation.h"
#include "Utils/Timing.h"

#include <set>
#include <numeric>
#include <algorithm>

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
	START_TIMING("simulation step");
	TimeManager *tm = TimeManager::getCurrent ();
	const Real h = tm->getTimeStepSize();
	ParticleData &pd = model.getParticles();

	clearAccelerations(model);

	// Update time step size by CFL condition
	updateTimeStepSizeCFL(model, static_cast<Real>(0.0001), static_cast<Real>(0.005));

	// Time integration
	for (unsigned int i = 0; i < pd.size(); i++)
	{ 
		model.getDeltaX(i).setZero();
		pd.getLastPosition(i) = pd.getOldPosition(i);
		pd.getOldPosition(i) = pd.getPosition(i);
		TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i), pd.getVelocity(i), pd.getAcceleration(i));
	}

	// Perform neighborhood search
	START_TIMING("neighborhood search");
#if defined(FSPH)
	model.getNeighborhoodSearch()->neighborhoodSearch(&model.getParticles().getPosition(0), model.getParticles().size(), model.numBoundaryParticles(), &model.getBoundaryX(0));
#elif defined(nSearch)
	//const float* tmp = model.getParticles().getPosition(0).data();
	model.getNeighborhoodSearch()->neighborhoodSearch(&model.getParticles().getPosition(0), model.getParticles().size(), &model.getBoundaryX(0), model.numBoundaryParticles());
#else
	model.getNeighborhoodSearch()->neighborhoodSearch(&model.getParticles().getPosition(0), model.numBoundaryParticles(), &model.getBoundaryX(0));
#endif
	STOP_TIMING_AVG;

	// Solve density constraint
	START_TIMING("constraint projection");
	constraintProjection(model);
	STOP_TIMING_AVG;

	// Update velocities	
	for (unsigned int i = 0; i < pd.size(); i++)
	{
		if (m_velocityUpdateMethod == 0)
			TimeIntegration::velocityUpdateFirstOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getVelocity(i));
		else
			TimeIntegration::velocityUpdateSecondOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getLastPosition(i), pd.getVelocity(i));
	}

	// Compute viscosity 
	computeXSPHViscosity(model);

	// Compute new time	
	tm->setTime (tm->getTime () + h);
	model.getNeighborhoodSearch()->update();
	STOP_TIMING_AVG;
}


/** Clear accelerations and add gravitation.
 */
void TimeStepFluidModel::clearAccelerations(FluidModel &model)
{
	ParticleData &pd = model.getParticles();
	const unsigned int count = pd.size();
	Simulation* sim = Simulation::getCurrent();
	const Vector3r grav(sim->getVecValue<Real>(Simulation::GRAVITATION));
	for (unsigned int i=0; i < count; i++)
	{
		// Clear accelerations of dynamic particles
		if (pd.getMass(i) != 0.0)
		{
			Vector3r &a = pd.getAcceleration(i);
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
#if defined(FSPH)

#elif defined(nSearch)

#else
	unsigned int** neighbors = model.getNeighborhoodSearch()->getNeighbors();
	unsigned int* numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();
#endif
	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  
		for (int i = 0; i < (int) numParticles; i++)
		{
#if defined(FSPH)

#elif defined(nSearch)
			int sortIdx = i;//model.getNeighborhoodSearch()->sortIdx(i);
			Real& density = model.getDensity(i);
			Real density_err;

			/*PositionBasedFluids::computePBFDensity(corrIdx, numParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
				&model.getBoundaryPsi(0), model.getNeighborhoodSearch()->n_neighbors(corrIdx), model.getNeighborhoodSearch()->neighbors(corrIdx),
				model.getDensity0(), true, density_err, density);*/
			const Real* mass = &pd.getMass(0);
			const Vector3r* boundaryX = &model.getBoundaryX(0);
			const Real* boundaryPsi = &model.getBoundaryPsi(0);
			const Vector3r* x = &pd.getPosition(0);
			const Real density0 = model.getDensity0();
			// Compute current density for particle i
			density = mass[i] * CubicKernel::W_zero();
			for (unsigned int j = 0; j < model.getNeighborhoodSearch()->n_neighbors(sortIdx); j++)
			{
				//const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
				const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
				if (neighborIndex < numParticles)		// Test if fluid particle
				{
					density += mass[neighborIndex] * CubicKernel::W(x[i] - x[neighborIndex]);
				}
				else
				{
					// Boundary: Akinci2012
					density += boundaryPsi[neighborIndex - numParticles] * CubicKernel::W(x[i] - boundaryX[neighborIndex - numParticles]);
				}
			}
			density_err = std::max(density, density0) - density0;
			
#else
			Real& density = model.getDensity(i);
			Real density_err;
			PositionBasedFluids::computePBFDensity(i, numParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), 
				&model.getBoundaryPsi(0), numNeighbors[i], neighbors[i], model.getDensity0(), true, density_err, density);
#endif

			
		}
	}
}

/** Update time step size by CFL condition.
*/
void TimeStepFluidModel::updateTimeStepSizeCFL(FluidModel &model, const Real minTimeStepSize, const Real maxTimeStepSize)
{
	const Real radius = model.getParticleRadius();
	const Real cflFactor = 1.0;
	Real h = TimeManager::getCurrent()->getTimeStepSize();

	// Approximate max. position change due to current velocities
	Real maxVel = static_cast<Real>(0.1);
	ParticleData &pd = model.getParticles();
	const unsigned int numParticles = pd.size();
	const Real diameter = static_cast<Real>(2.0)*radius;
	for (unsigned int i = 0; i < numParticles; i++)
	{
		const Vector3r &vel = pd.getVelocity(i);
		const Vector3r &accel = pd.getAcceleration(i);
		const Real velMag = (vel + accel*h).squaredNorm();
		if (velMag > maxVel)
			maxVel = velMag;
	}

	// Approximate max. time step size 		
	h = cflFactor * static_cast<Real>(0.4) * (diameter / (sqrt(maxVel)));

	h = min(h, maxTimeStepSize);
	h = max(h, minTimeStepSize);

	TimeManager::getCurrent()->setTimeStepSize(h);
}

/** Compute viscosity accelerations.
*/
void TimeStepFluidModel::computeXSPHViscosity(FluidModel &model)
{
	ParticleData &pd = model.getParticles();
	const unsigned int numParticles = pd.size();	
#if defined(FSPH)

#elif defined(nSearch)

#else
	unsigned int** neighbors = model.getNeighborhoodSearch()->getNeighbors();
	unsigned int* numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();
#endif

	const Real viscosity = model.getViscosity();
	const Real h = TimeManager::getCurrent()->getTimeStepSize();

#if defined(FSPH)
	// Compute viscosity forces (XSPH)
	/*#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)
		for (int i = 0; i < (int)numParticles; i++)
		{
			int sortIdx = model.getNeighborhoodSearch()->partIdx(i);
			const Vector3r& xi = pd.getPosition(i);
			Vector3r& vi = pd.getVelocity(i);
			const Real density_i = model.getDensity(i);
			const unsigned int numNeighbors = model.getNeighborhoodSearch()->n_neighbors(sortIdx);
			for (unsigned int j = 0; j < numNeighbors; j++)
			{
				const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
				//const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
				if (neighborIndex < numParticles)		// Test if fluid particle
				{
					// Viscosity
					const Vector3r& xj = pd.getPosition(neighborIndex);
					const Vector3r& vj = pd.getVelocity(neighborIndex);
					const Real density_j = model.getDensity(neighborIndex);
					vi -= viscosity * (pd.getMass(neighborIndex) / density_j) * (vi - vj) * CubicKernel::W(xi - xj);
				}
				// 				else 
				// 				{
				// 					const Vector3r &xj = model.getBoundaryX(neighborIndex - numParticles);
				// 					vi -= viscosity * (model.getBoundaryPsi(neighborIndex - numParticles) / density_i) * (vi)* CubicKernel::W(xi - xj);
				// 				}
			}
		}
	}*/

	//TEST CPU!
	printf("CPU version:\n");
	std:vector<Vector3r> testVelocity(pd.getVelocities());
	NeighborhoodSearchSpatialHashing testNeight(model.getParticles().size(), model.getSupportRadius());
	testNeight.neighborhoodSearch(&model.getParticles().getPosition(0), model.numBoundaryParticles(), &model.getBoundaryX(0));

	unsigned int** neighbors = testNeight.getNeighbors();
	unsigned int* numNeighbors = testNeight.getNumNeighbors();

	// Compute viscosity forces (XSPH)
	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)
		for (int i = 0; i < (int)numParticles; i++)
		{
			const Vector3r& xi = pd.getPosition(i);
			Vector3r& vi = testVelocity[i];
			const Real density_i = model.getDensity(i);
			if (i < 22) printf("%d (%d): ", i, numNeighbors[i]);
			for (unsigned int j = 0; j < numNeighbors[i]; j++)
			{
				const unsigned int neighborIndex = neighbors[i][j];
				if (neighborIndex < numParticles)		// Test if fluid particle
				{
					// Viscosity
					const Vector3r& xj = pd.getPosition(neighborIndex);
					const Vector3r& vj = pd.getVelocity(neighborIndex);
					const Real density_j = model.getDensity(neighborIndex);
					vi -= viscosity * (pd.getMass(neighborIndex) / density_j) * (vi - vj) * CubicKernel::W(xi - xj);

				}
				if (i < 22) printf("%d; ", neighborIndex);
				// 				else 
				// 				{
				// 					const Vector3r &xj = model.getBoundaryX(neighborIndex - numParticles);
				// 					vi -= viscosity * (model.getBoundaryPsi(neighborIndex - numParticles) / density_i) * (vi)* CubicKernel::W(xi - xj);
				// 				}
			}
			if (i < 22) printf("\n");
		}
	}
	printf("\n");

	// Compute viscosity forces (XSPH)
	printf("GPU version:\n");
	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)
		for (int i = 0; i < (int)numParticles; i++)
		{
			int sortIdx = model.getNeighborhoodSearch()->partIdx(i);
			const Vector3r& xi = pd.getPosition(i);
			Vector3r& vi = pd.getVelocity(i);
			const Real density_i = model.getDensity(i);
			const unsigned int numNeighbors = model.getNeighborhoodSearch()->n_neighbors(sortIdx);
			if (i < 22) printf("%d %d (%d): ", i, sortIdx, model.getNeighborhoodSearch()->n_neighbors(sortIdx));
			for (unsigned int j = 0; j < numNeighbors; j++)
			{
				const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
				if (neighborIndex < numParticles)		// Test if fluid particle
				{
					// Viscosity
					const Vector3r& xj = pd.getPosition(neighborIndex);
					const Vector3r& vj = pd.getVelocity(neighborIndex);
					const Real density_j = model.getDensity(neighborIndex);
					vi -= viscosity * (pd.getMass(neighborIndex) / density_j) * (vi - vj) * CubicKernel::W(xi - xj);

				}
				if (i < 22) printf("%d; ", neighborIndex);
				// 				else 
				// 				{
				// 					const Vector3r &xj = model.getBoundaryX(neighborIndex - numParticles);
				// 					vi -= viscosity * (model.getBoundaryPsi(neighborIndex - numParticles) / density_i) * (vi)* CubicKernel::W(xi - xj);
				// 				}
			}
			if (i < 22) printf("\n");
		}
	}
	printf("\n");

	printf("Testing velocity:\n");
	int county = 0;
	std::vector<unsigned int> tmpCPU;
	tmpCPU.reserve(50);
	std::vector<unsigned int> tmpGPU;
	tmpGPU.reserve(50);
	std::vector<unsigned int> difference;
	difference.reserve(50);

	std::set<unsigned int> missingParticles;
	std::vector<unsigned int> wrongParticles;
	std::set<unsigned int> foundParticles;
	wrongParticles.reserve(1408);
	bool same = true;
	for (int i = 0; i < (int)numParticles; i++)
	{
		bool subSame = true;
		Vector3r diff = pd.getVelocity(i) - testVelocity[i];
		int sortIdx = model.getNeighborhoodSearch()->partIdx(i);
		if (diff.squaredNorm() > 0.0001 || model.getNeighborhoodSearch()->n_neighbors(sortIdx) != numNeighbors[i])
		{
			wrongParticles.push_back(i);
			tmpCPU.clear();
			tmpGPU.clear();
			difference.clear();
			printf("## %d ##\n", i);
			county++;
			printf("\t(%f, %f, %f){%f} - (%f, %f, %f){%f} = (%f, %f, %f){%f}\n",
				pd.getVelocity(i)[0], pd.getVelocity(i)[1], pd.getVelocity(i)[2], pd.getVelocity(i).squaredNorm(),
				testVelocity[i][0], testVelocity[i][1], testVelocity[i][2], testVelocity[i].squaredNorm(),
				diff[0], diff[1], diff[2], diff.squaredNorm()
			);

			printf("\tCPU (%d):", numNeighbors[i]);
			for (unsigned int j = 0; j < numNeighbors[i]; j++)
			{
				const unsigned int neighborIndex = neighbors[i][j];
				tmpCPU.push_back(neighborIndex);
				printf("\t%d; ", neighborIndex);
			}
			std::sort(tmpCPU.begin(), tmpCPU.end());
			printf("\n");

			printf("\tGPU (%d): ", model.getNeighborhoodSearch()->n_neighbors(sortIdx));
			for (unsigned int j = 0; j < model.getNeighborhoodSearch()->n_neighbors(sortIdx); j++)
			{
				const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
				tmpGPU.push_back(neighborIndex);
				foundParticles.insert(neighborIndex);
				printf("\t%d; ", neighborIndex);
			}
			std::sort(tmpGPU.begin(), tmpGPU.end());
			printf("\n");


			std::set_difference(tmpGPU.begin(), tmpGPU.end(), tmpCPU.begin(), tmpCPU.end(),
				std::inserter(difference, difference.begin()));
			if (difference.size() > 0)
			{
				printf("\tDifference GPU/CPU (%d): ", difference.size());
				for (unsigned int d : difference)
				{
					printf("\t%d; ", d);
					missingParticles.insert(d);
				}
				printf("\n");
			}

			difference.clear();
			std::set_difference(tmpCPU.begin(), tmpCPU.end(), tmpGPU.begin(), tmpGPU.end(),
				std::inserter(difference, difference.begin()));
			if (difference.size() > 0)
			{
				printf("\tDifference CPU/GPU (%d): ", difference.size());
				for (unsigned int d : difference)
				{
					printf("\t%d; ", d);
					missingParticles.insert(d);
				}
				printf("\n");
			}
		}
		//if (!subSame) same = false;
	}
	//if (same) printf("They all have the same coordinates\n");

	{
		printf("Number of wrongs %d/%d: \n", missingParticles.size(), county);
		bool cursed = true;
		int i = 0;
		for (unsigned int part : missingParticles)
		{
			//printf("%d, ", part);
			if (part < numParticles)
			{
				printf("%d (%f, %f, %f); ", part, pd.getPosition(part)[0], pd.getPosition(part)[1], pd.getPosition(part)[2]);
			}
			else
			{
				printf("%d (%f, %f, %f); ", part, model.getBoundaryX(part - numParticles)[0], model.getBoundaryX(part - numParticles)[1], model.getBoundaryX(part - numParticles)[2]);
			}
			//if (part != wrongParticles[i]) cursed = false;
			i++;
		}
		printf("\n");
		//if (cursed) printf("OH NO IT IS CURSED\n");

		difference.clear();
		std::set_difference(wrongParticles.begin(), wrongParticles.end(), missingParticles.begin(), missingParticles.end(),
			std::inserter(difference, difference.begin()));
		printf("\tDifference of wrong/missing particles (%d): ", difference.size());
		/*for (unsigned int d : difference)
		{
			printf("\t%d; ", d);
		}*/
		printf("\n");

		std::vector<unsigned int> intersection;
		intersection.reserve(1500);
		std::set_intersection(foundParticles.begin(), foundParticles.end(), missingParticles.begin(), missingParticles.end(),
			std::inserter(intersection, intersection.begin()));
		printf("Intersection (%d): ", intersection.size());
		/*for (unsigned int intersec : intersection)
		{
			printf("\t%d; ", intersec);
		}*/
	}
	printf("\n");

	Vector3r cpuVelSum = std::accumulate(testVelocity.begin(), testVelocity.end(), Vector3r(0.0f, 0.0f, 0.0f));
	printf("CPU velocity sum: %f, %f, %f\n", cpuVelSum[0], cpuVelSum[1], cpuVelSum[2]);
	Vector3r gpuVelSum = std::accumulate(&pd.getVelocity(0), &pd.getVelocity(pd.getNumberOfParticles()-1), Vector3r(0.0f, 0.0f, 0.0f));
	printf("GPU velocity sum: %f, %f, %f\n", gpuVelSum[0], gpuVelSum[1], gpuVelSum[2]);
	printf("\n");


#elif defined(nSearch)
	// Compute viscosity forces (XSPH)
	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)
		for (int i = 0; i < (int)numParticles; i++)
		{
			int sortIdx = i;//model.getNeighborhoodSearch()->sortIdx(i);
			const Vector3r& xi = pd.getPosition(i);
			Vector3r& vi = pd.getVelocity(i);
			const Real density_i = model.getDensity(i);
			const unsigned int numNeighbors = model.getNeighborhoodSearch()->n_neighbors(sortIdx);
			for (unsigned int j = 0; j < numNeighbors; j++)
			{
				//const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
				const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
				if (neighborIndex < numParticles)		// Test if fluid particle
				{
					// Viscosity
					const Vector3r& xj = pd.getPosition(neighborIndex);
					const Vector3r& vj = pd.getVelocity(neighborIndex);
					const Real density_j = model.getDensity(neighborIndex);
					vi -= viscosity * (pd.getMass(neighborIndex) / density_j) * (vi - vj) * CubicKernel::W(xi - xj);
				}
				// 				else 
				// 				{
				// 					const Vector3r &xj = model.getBoundaryX(neighborIndex - numParticles);
				// 					vi -= viscosity * (model.getBoundaryPsi(neighborIndex - numParticles) / density_i) * (vi)* CubicKernel::W(xi - xj);
				// 				}
			}
		}
	}

#else
	// Compute viscosity forces (XSPH)
	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)
		for (int i = 0; i < (int)numParticles; i++)
		{
				const Vector3r& xi = pd.getPosition(i);
				Vector3r& vi = pd.getVelocity(i);
				const Real density_i = model.getDensity(i);

				for (unsigned int j = 0; j < numNeighbors[i]; j++)
				{
					const unsigned int neighborIndex = neighbors[i][j];
					if (neighborIndex < numParticles)		// Test if fluid particle
					{
						// Viscosity
						const Vector3r& xj = pd.getPosition(neighborIndex);
						const Vector3r& vj = pd.getVelocity(neighborIndex);
						const Real density_j = model.getDensity(neighborIndex);
						vi -= viscosity * (pd.getMass(neighborIndex) / density_j) * (vi - vj) * CubicKernel::W(xi - xj);

					}
					// 				else 
					// 				{
					// 					const Vector3r &xj = model.getBoundaryX(neighborIndex - numParticles);
					// 					vi -= viscosity * (model.getBoundaryPsi(neighborIndex - numParticles) / density_i) * (vi)* CubicKernel::W(xi - xj);
					// 				}
				}
		}
	}
#endif
}

void TimeStepFluidModel::reset()
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
#if defined(FSPH)

#elif defined(nSearch)

#else
	unsigned int** neighbors = model.getNeighborhoodSearch()->getNeighbors();
	unsigned int* numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();
#endif

	while (iter < maxIter)
	{
		Real avg_density_err = 0.0;

		#pragma omp parallel default(shared)
		{
			#pragma omp for schedule(static)  
			for (int i = 0; i < (int)nParticles; i++)
			{
				Real density_err;
#if defined(FSPH)
				int sortIdx = model.getNeighborhoodSearch()->partIdx(i);
				/*PositionBasedFluids::computePBFDensity(corrIdx, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
					&model.getBoundaryPsi(0), model.getNeighborhoodSearch()->n_neighbors(corrIdx), model.getNeighborhoodSearch()->neighbors(corrIdx),
					model.getDensity0(), true, density_err, model.getDensity(corrIdx));*/
				Real& density = model.getDensity(i);
				const Real* mass = &pd.getMass(0);
				const Vector3r* boundaryX = &model.getBoundaryX(0);
				const Real* boundaryPsi = &model.getBoundaryPsi(0);
				const Vector3r* x = &pd.getPosition(0);
				const Real density0 = model.getDensity0();

				// Compute current density for particle i
				density = mass[i] * CubicKernel::W_zero();
				for (unsigned int j = 0; j < model.getNeighborhoodSearch()->n_neighbors(sortIdx); j++)
				{
					const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
					//const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
					if (neighborIndex < nParticles)		// Test if fluid particle
					{
						density += mass[neighborIndex] * CubicKernel::W(x[i] - x[neighborIndex]);
					}
					else
					{
						// Boundary: Akinci2012
						density += boundaryPsi[neighborIndex - nParticles] * CubicKernel::W(x[i] - boundaryX[neighborIndex - nParticles]);
					}
				}
				density_err = std::max(density, density0) - density0;

				/*PositionBasedFluids::computePBFLagrangeMultiplier(corrIdx, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
					&model.getBoundaryPsi(0), model.getDensity(corrIdx), model.getNeighborhoodSearch()->n_neighbors(corrIdx), model.getNeighborhoodSearch()->neighbors(corrIdx),
					model.getDensity0(), true, model.getLambda(corrIdx));*/
				const Real constDensity = density;
				Real& lambda = model.getLambda(i);

				const Real eps = static_cast<Real>(1.0e-6);

				// Evaluate constraint function
				const Real C = std::max(constDensity / density0 - static_cast<Real>(1.0), static_cast<Real>(0.0));			// clamp to prevent particle clumping at surface

				if (C != 0.0)
				{
					// Compute gradients dC/dx_j 
					Real sum_grad_C2 = 0.0;
					Vector3r gradC_i(0.0, 0.0, 0.0);

					for (unsigned int j = 0; j < model.getNeighborhoodSearch()->n_neighbors(sortIdx); j++)
					{
						const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
						//const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
						if (neighborIndex < nParticles)		// Test if fluid particle
						{
							const Vector3r gradC_j = -mass[neighborIndex] / density0 * CubicKernel::gradW(x[i] - x[neighborIndex]);
							sum_grad_C2 += gradC_j.squaredNorm();
							gradC_i -= gradC_j;
						}
						else
						{
							// Boundary: Akinci2012
							const Vector3r gradC_j = -boundaryPsi[neighborIndex - nParticles] / density0 * CubicKernel::gradW(x[i] - boundaryX[neighborIndex - nParticles]);
							sum_grad_C2 += gradC_j.squaredNorm();
							gradC_i -= gradC_j;
						}
					}

					sum_grad_C2 += gradC_i.squaredNorm();

					// Compute lambda
					lambda = -C / (sum_grad_C2 + eps);
				}
				else
					lambda = 0.0;
#elif defined(nSearch)
				int sortIdx = i;//model.getNeighborhoodSearch()->sortIdx(i);
				/*PositionBasedFluids::computePBFDensity(corrIdx, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
					&model.getBoundaryPsi(0), model.getNeighborhoodSearch()->n_neighbors(corrIdx), model.getNeighborhoodSearch()->neighbors(corrIdx),
					model.getDensity0(), true, density_err, model.getDensity(corrIdx));*/
				Real& density = model.getDensity(i);
				const Real* mass = &pd.getMass(0);
				const Vector3r* boundaryX = &model.getBoundaryX(0);
				const Real* boundaryPsi = &model.getBoundaryPsi(0);
				const Vector3r* x = &pd.getPosition(0);
				const Real density0 = model.getDensity0();
				
				// Compute current density for particle i
				density = mass[i] * CubicKernel::W_zero();
				for (unsigned int j = 0; j < model.getNeighborhoodSearch()->n_neighbors(sortIdx); j++)
				{
					//const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
					const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
					if (neighborIndex < nParticles)		// Test if fluid particle
					{
						density += mass[neighborIndex] * CubicKernel::W(x[i] - x[neighborIndex]);
					}
					else
					{
						// Boundary: Akinci2012
						density += boundaryPsi[neighborIndex - nParticles] * CubicKernel::W(x[i] - boundaryX[neighborIndex - nParticles]);
					}
				}
				density_err = std::max(density, density0) - density0;

				/*PositionBasedFluids::computePBFLagrangeMultiplier(corrIdx, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
					&model.getBoundaryPsi(0), model.getDensity(corrIdx), model.getNeighborhoodSearch()->n_neighbors(corrIdx), model.getNeighborhoodSearch()->neighbors(corrIdx),
					model.getDensity0(), true, model.getLambda(corrIdx));*/
				const Real constDensity = density;
				Real& lambda = model.getLambda(i);

				const Real eps = static_cast<Real>(1.0e-6);

				// Evaluate constraint function
				const Real C = std::max(constDensity / density0 - static_cast<Real>(1.0), static_cast<Real>(0.0));			// clamp to prevent particle clumping at surface

				if (C != 0.0)
				{
					// Compute gradients dC/dx_j 
					Real sum_grad_C2 = 0.0;
					Vector3r gradC_i(0.0, 0.0, 0.0);

					for (unsigned int j = 0; j < model.getNeighborhoodSearch()->n_neighbors(sortIdx); j++)
					{
						//const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
						const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
						if (neighborIndex < nParticles)		// Test if fluid particle
						{
							const Vector3r gradC_j = -mass[neighborIndex] / density0 * CubicKernel::gradW(x[i] - x[neighborIndex]);
							sum_grad_C2 += gradC_j.squaredNorm();
							gradC_i -= gradC_j;
						}
						else
						{
							// Boundary: Akinci2012
							const Vector3r gradC_j = -boundaryPsi[neighborIndex - nParticles] / density0 * CubicKernel::gradW(x[i] - boundaryX[neighborIndex - nParticles]);
							sum_grad_C2 += gradC_j.squaredNorm();
							gradC_i -= gradC_j;
						}
					}

					sum_grad_C2 += gradC_i.squaredNorm();

					// Compute lambda
					lambda = -C / (sum_grad_C2 + eps);
				}
				else
					lambda = 0.0;
#else
				PositionBasedFluids::computePBFDensity(i, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), 
					&model.getBoundaryPsi(0), numNeighbors[i], neighbors[i], model.getDensity0(), true, density_err, model.getDensity(i));
				PositionBasedFluids::computePBFLagrangeMultiplier(i, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), 
					&model.getBoundaryPsi(0), model.getDensity(i), numNeighbors[i], neighbors[i], model.getDensity0(), true, model.getLambda(i));
#endif

				
			}
		}
		
		#pragma omp parallel default(shared)
		{
			#pragma omp for schedule(static)  
			for (int i = 0; i < (int)nParticles; i++)
			{
				Vector3r corr;
#if defined(FSPH)
				int sortIdx = model.getNeighborhoodSearch()->partIdx(i);
				/*PositionBasedFluids::solveDensityConstraint(corrIdx, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), &model.getBoundaryPsi(0),
					model.getNeighborhoodSearch()->n_neighbors(corrIdx), model.getNeighborhoodSearch()->neighbors(corrIdx), model.getDensity0(), true, &model.getLambda(0), corr);*/
				const Real* mass = &pd.getMass(0);
				const Vector3r* boundaryX = &model.getBoundaryX(0);
				const Real* boundaryPsi = &model.getBoundaryPsi(0);
				const Vector3r* x = &pd.getPosition(0);
				const Real density0 = model.getDensity0();
				const Real* lambda = &model.getLambda(0);

				// Compute position correction
				corr.setZero();
				for (unsigned int j = 0; j < model.getNeighborhoodSearch()->n_neighbors(sortIdx); j++)
				{
					const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
					//const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
					if (neighborIndex < nParticles)		// Test if fluid particle
					{
						const Vector3r gradC_j = -mass[neighborIndex] / density0 * CubicKernel::gradW(x[i] - x[neighborIndex]);
						corr -= (lambda[i] + lambda[neighborIndex]) * gradC_j;
					}
					else
					{
						Vector3r tmp0 = x[i] - boundaryX[neighborIndex - nParticles];
						Vector3r tmp1 = CubicKernel::gradW(x[i] - boundaryX[neighborIndex - nParticles]);;
						Real tmp2 = -boundaryPsi[neighborIndex - nParticles];
						// Boundary: Akinci2012
						const Vector3r gradC_j = -boundaryPsi[neighborIndex - nParticles] / density0 * CubicKernel::gradW(x[i] - boundaryX[neighborIndex - nParticles]);
						assert(!std::isnan(gradC_j[0]));
						assert(!std::isnan(gradC_j[1]));
						assert(!std::isnan(gradC_j[2]));
						assert(!std::isinf(gradC_j[0]));
						assert(!std::isinf(gradC_j[1]));
						assert(!std::isinf(gradC_j[2]));
						corr -= (lambda[i]) * gradC_j;
					}
				}
				assert(!std::isnan(corr[0]));
				assert(!std::isnan(corr[1]));
				assert(!std::isnan(corr[2]));
				assert(!std::isinf(corr[0]));
				assert(!std::isinf(corr[1]));
				assert(!std::isinf(corr[2]));
				model.getDeltaX(i) = corr;
#elif defined(nSearch)
				int sortIdx = i;//model.getNeighborhoodSearch()->sortIdx(i);
				/*PositionBasedFluids::solveDensityConstraint(corrIdx, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), &model.getBoundaryPsi(0),
					model.getNeighborhoodSearch()->n_neighbors(corrIdx), model.getNeighborhoodSearch()->neighbors(corrIdx), model.getDensity0(), true, &model.getLambda(0), corr);*/
				const Real* mass = &pd.getMass(0);
				const Vector3r* boundaryX = &model.getBoundaryX(0);
				const Real* boundaryPsi = &model.getBoundaryPsi(0);
				const Vector3r* x = &pd.getPosition(0);
				const Real density0 = model.getDensity0();
				const Real* lambda = &model.getLambda(0);

				// Compute position correction
				corr.setZero();
				for (unsigned int j = 0; j < model.getNeighborhoodSearch()->n_neighbors(sortIdx); j++)
				{
					//const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
					const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
					if (neighborIndex < nParticles)		// Test if fluid particle
					{
						const Vector3r gradC_j = -mass[neighborIndex] / density0 * CubicKernel::gradW(x[i] - x[neighborIndex]);
						corr -= (lambda[i] + lambda[neighborIndex]) * gradC_j;
					}
					else
					{
						// Boundary: Akinci2012
						const Vector3r gradC_j = -boundaryPsi[neighborIndex - nParticles] / density0 * CubicKernel::gradW(x[i] - boundaryX[neighborIndex - nParticles]);
						corr -= (lambda[i]) * gradC_j;
					}
				}

				model.getDeltaX(i) = corr;
#else
				PositionBasedFluids::solveDensityConstraint(i, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), &model.getBoundaryPsi(0), 
					numNeighbors[i], neighbors[i], model.getDensity0(), true, &model.getLambda(0), corr);
				model.getDeltaX(i) = corr;
#endif

				
			}
		}

		Vector3r* tmp = &model.getDeltaX(0);
		int x = 0;

		#pragma omp parallel default(shared)
		{
			#pragma omp for schedule(static)  
			for (int i = 0; i < (int)nParticles; i++)
			{
				assert(!std::isnan(model.getDeltaX(i)[0]));
				assert(!std::isnan(model.getDeltaX(i)[1]));
				assert(!std::isnan(model.getDeltaX(i)[2]));
				assert(!std::isinf(model.getDeltaX(i)[0]));
				assert(!std::isinf(model.getDeltaX(i)[1]));
				assert(!std::isinf(model.getDeltaX(i)[2]));
				pd.getPosition(i) += model.getDeltaX(i);
			}
		}

		iter++;
	}
}

