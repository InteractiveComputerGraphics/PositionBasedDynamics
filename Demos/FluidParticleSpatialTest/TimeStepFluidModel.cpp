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
	//model.getNeighborhoodSearch()->neighborhoodSearch(&model.getParticles().getPosition(0), model.numBoundaryParticles(), &model.getBoundaryX(0));

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

///** Determine densities of all fluid particles. 
//*/
//void TimeStepFluidModel::computeDensities(FluidModel &model)
//{
//	ParticleData &pd = model.getParticles();
//	const unsigned int numParticles = pd.size();
//#if defined(FSPH)
//	NeighborhoodSearchSpatialHashing testNeight(model.getParticles().size(), model.getSupportRadius());
//	testNeight.neighborhoodSearch(&model.getParticles().getPosition(0), model.numBoundaryParticles(), &model.getBoundaryX(0));
//
//	unsigned int** neighbors = testNeight.getNeighbors();
//	unsigned int* numNeighbors = testNeight.getNumNeighbors();
//
//	/*unsigned int** neighbors = model.getNeighborhoodSearch()->getNeighbors();
//	unsigned int* numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();*/
//
//#elif defined(nSearch)
//
//#else
//	unsigned int** neighbors = model.getNeighborhoodSearch()->getNeighbors();
//	unsigned int* numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();
//
//#endif
//
//	#pragma omp parallel default(shared)
//	{
//		#pragma omp for schedule(static)  
//		for (int i = 0; i < (int) numParticles; i++)
//		{
//#if defined(FSPH)
//			Real& density = model.getDensity(i);
//			Real density_err;
//			PositionBasedFluids::computePBFDensity(i, numParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
//				&model.getBoundaryPsi(0), numNeighbors[i], neighbors[i], model.getDensity0(), true, density_err, density);
//
//#elif defined(nSearch)
//			int sortIdx = i;//model.getNeighborhoodSearch()->sortIdx(i);
//			Real& density = model.getDensity(i);
//			Real density_err;
//
//			/*PositionBasedFluids::computePBFDensity(corrIdx, numParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
//				&model.getBoundaryPsi(0), model.getNeighborhoodSearch()->n_neighbors(corrIdx), model.getNeighborhoodSearch()->neighbors(corrIdx),
//				model.getDensity0(), true, density_err, density);*/
//			const Real* mass = &pd.getMass(0);
//			const Vector3r* boundaryX = &model.getBoundaryX(0);
//			const Real* boundaryPsi = &model.getBoundaryPsi(0);
//			const Vector3r* x = &pd.getPosition(0);
//			const Real density0 = model.getDensity0();
//			// Compute current density for particle i
//			density = mass[i] * CubicKernel::W_zero();
//			for (unsigned int j = 0; j < model.getNeighborhoodSearch()->n_neighbors(sortIdx); j++)
//			{
//				//const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
//				const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
//				if (neighborIndex < numParticles)		// Test if fluid particle
//				{
//					density += mass[neighborIndex] * CubicKernel::W(x[i] - x[neighborIndex]);
//				}
//				else
//				{
//					// Boundary: Akinci2012
//					density += boundaryPsi[neighborIndex - numParticles] * CubicKernel::W(x[i] - boundaryX[neighborIndex - numParticles]);
//				}
//			}
//			density_err = std::max(density, density0) - density0;
//			
//#else
//			Real& density = model.getDensity(i);
//			Real density_err;
//			PositionBasedFluids::computePBFDensity(i, numParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), 
//				&model.getBoundaryPsi(0), numNeighbors[i], neighbors[i], model.getDensity0(), true, density_err, density);
//#endif
//
//			
//		}
//	}
//}

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
	/*NeighborhoodSearchSpatialHashing testNeight(model.getParticles().size(), model.getSupportRadius());
	testNeight.neighborhoodSearch(&model.getParticles().getPosition(0), model.numBoundaryParticles(), &model.getBoundaryX(0));

	unsigned int** neighbors = testNeight.getNeighbors();
	unsigned int* numNeighbors = testNeight.getNumNeighbors();*/

	/*unsigned int** neighbors = model.getNeighborhoodSearch()->getNeighbors();
	unsigned int* numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();*/

#elif defined(nSearch)

#else
	unsigned int** neighbors = model.getNeighborhoodSearch()->getNeighbors();
	unsigned int* numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();
#endif

	const Real viscosity = model.getViscosity();
	const Real h = TimeManager::getCurrent()->getTimeStepSize();

#if defined(FSPH)
	// Compute viscosity forces (XSPH)
	//#pragma omp parallel default(shared)
	//{
	//	#pragma omp for schedule(static)
	//	for (int i = 0; i < (int)numParticles; i++)
	//	{
	//		int sortIdx = model.getNeighborhoodSearch()->partIdx(i);
	//		const Vector3r& xi = pd.getPosition(i);
	//		Vector3r& vi = pd.getVelocity(i);
	//		const Real density_i = model.getDensity(i);
	//		const unsigned int numNeighbors = model.getNeighborhoodSearch()->n_neighbors(sortIdx);
	//		for (unsigned int j = 0; j < numNeighbors; j++)
	//		{
	//			const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
	//			//const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
	//			if (neighborIndex < numParticles)		// Test if fluid particle
	//			{
	//				// Viscosity
	//				const Vector3r& xj = pd.getPosition(neighborIndex);
	//				const Vector3r& vj = pd.getVelocity(neighborIndex);
	//				const Real density_j = model.getDensity(neighborIndex);
	//				vi -= viscosity * (pd.getMass(neighborIndex) / density_j) * (vi - vj) * CubicKernel::W(xi - xj);
	//			}
	//			// 				else 
	//			// 				{
	//			// 					const Vector3r &xj = model.getBoundaryX(neighborIndex - numParticles);
	//			// 					vi -= viscosity * (model.getBoundaryPsi(neighborIndex - numParticles) / density_i) * (vi)* CubicKernel::W(xi - xj);
	//			// 				}
	//		}
	//	}
	//}
	//#pragma omp parallel default(shared)
	//{
	//	#pragma omp for schedule(static)
	//	for (int i = 0; i < (int)numParticles; i++)
	//	{
	//		const Vector3r& xi = pd.getPosition(i);
	//		Vector3r& vi = pd.getVelocity(i);
	//		const Real density_i = model.getDensity(i);

	//		for (unsigned int j = 0; j < numNeighbors[i]; j++)
	//		{
	//			const unsigned int neighborIndex = neighbors[i][j];
	//			if (neighborIndex < numParticles)		// Test if fluid particle
	//			{
	//				// Viscosity
	//				const Vector3r& xj = pd.getPosition(neighborIndex);
	//				const Vector3r& vj = pd.getVelocity(neighborIndex);
	//				const Real density_j = model.getDensity(neighborIndex);
	//				vi -= viscosity * (pd.getMass(neighborIndex) / density_j) * (vi - vj) * CubicKernel::W(xi - xj);

	//			}
	//			// 				else 
	//			// 				{
	//			// 					const Vector3r &xj = model.getBoundaryX(neighborIndex - numParticles);
	//			// 					vi -= viscosity * (model.getBoundaryPsi(neighborIndex - numParticles) / density_i) * (vi)* CubicKernel::W(xi - xj);
	//			// 				}
	//		}
	//	}
	//}
	//Vector3r cpuVelSum = std::accumulate(&pd.getVelocity(0), &pd.getVelocity(0) + pd.getNumberOfParticles(), Vector3r(0.0f, 0.0f, 0.0f));
	//printf("CPU velocity sum: %f, %f, %f\n", cpuVelSum[0], cpuVelSum[1], cpuVelSum[2]);

	unsigned int numCPUNeigh = 0;
	unsigned int numGPUNeigh = 0;

	//TEST CPU!
	//printf("CPU version:\n");
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
			numCPUNeigh += numNeighbors[i];
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
	//printf("GPU version:\n");
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
			numGPUNeigh += numNeighbors;
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
				/*else
				{
					printf("Boundry particle neighbour %d:%d\n", i, neighborIndex);
				}*/
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

	printf("Number of CPU neighbours: %d\nNumber of GPU neighbours: %d\n", numCPUNeigh, numGPUNeigh);
	printf("Neighbour difference CPU-GPU=%d\n", numCPUNeigh - numGPUNeigh);

	/*if (numCPUNeigh != numGPUNeigh && numCPUNeigh - numGPUNeigh != 22488)
	{
		printf("Oh shit boooys\n");
	}*/

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
			if (i < numParticles)
			{
				//printf("## %d ## Fluid (%f, %f, %f)\n", i, pd.getPosition(i)[0], pd.getPosition(i)[1], pd.getPosition(i)[2]);
			}
			else
			{
				//printf("## %d ## Boundry (%f, %f, %f)\n", i, model.getBoundaryX(i - numParticles)[0], model.getBoundaryX(i - numParticles)[1], model.getBoundaryX(i - numParticles)[2]);
			}
			county++;
			/*printf("\t(%f, %f, %f){%f} - (%f, %f, %f){%f} = (%f, %f, %f){%f}\n",
				pd.getVelocity(i)[0], pd.getVelocity(i)[1], pd.getVelocity(i)[2], pd.getVelocity(i).squaredNorm(),
				testVelocity[i][0], testVelocity[i][1], testVelocity[i][2], testVelocity[i].squaredNorm(),
				diff[0], diff[1], diff[2], diff.squaredNorm()
			);*/

			if (diff.squaredNorm() > 1.0f)
			{
				printf("WOAH! What are you particles doing? %d\n", i);
			}

			//printf("\tCPU (%d):", numNeighbors[i]);
			bool hasBoundry = false;
			for (unsigned int j = 0; j < numNeighbors[i]; j++)
			{
				const unsigned int neighborIndex = neighbors[i][j];
				tmpCPU.push_back(neighborIndex);
				//printf("\t%d; ", neighborIndex);
				/*if (neighborIndex > numParticles && model.getNeighborhoodSearch()->n_neighbors(sortIdx) != 0)
				{
					printf("WHAAAAAT\n");
				}*/
				if (neighborIndex > numParticles && model.getNeighborhoodSearch()->n_neighbors(sortIdx) == 0)
				{
					hasBoundry = true;
				}
			}
			std::sort(tmpCPU.begin(), tmpCPU.end());
			/*if (!hasBoundry && model.getNeighborhoodSearch()->n_neighbors(sortIdx) == 0)
			{
				printf(" Oh no, my theory is false");
			}*/
			//printf("\n");

			//printf("\tGPU (%d): ", model.getNeighborhoodSearch()->n_neighbors(sortIdx));
			for (unsigned int j = 0; j < model.getNeighborhoodSearch()->n_neighbors(sortIdx); j++)
			{
				const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
				tmpGPU.push_back(neighborIndex);
				foundParticles.insert(neighborIndex);
				//printf("\t%d; ", neighborIndex);
			}
			std::sort(tmpGPU.begin(), tmpGPU.end());
			//printf("\n");


			/*std::set_difference(tmpGPU.begin(), tmpGPU.end(), tmpCPU.begin(), tmpCPU.end(),
				std::inserter(difference, difference.begin()));
			if (difference.size() > 0)
			{
				printf("\tDifference GPU/CPU (%d): ", difference.size());
				for (unsigned int d : difference)
				{
					Vector3r diffNeighDist;
					if (d < numParticles)
					{
						diffNeighDist = pd.getPosition(i) - pd.getPosition(d);
					}
					else
					{
						diffNeighDist = pd.getPosition(i) - model.getBoundaryX(d - numParticles);
					}
					printf("\t%d (%f, %f, %f){%f}; ", d, diffNeighDist[0], diffNeighDist[1], diffNeighDist[2], diffNeighDist.squaredNorm());
					if (diffNeighDist.squaredNorm() > 0.0000001)
					{
						printf("HOLD ON. Why is the difference so big?\n");
					}
					missingParticles.insert(d);
				}
				printf("\n");
			}*/

			bool test0 = false;
			difference.clear();
			std::set_difference(tmpCPU.begin(), tmpCPU.end(), tmpGPU.begin(), tmpGPU.end(),
				std::inserter(difference, difference.begin()));
			if (difference.size() > 0)
			{
				printf("\tDifference CPU/GPU (%d): ", difference.size());
				for (unsigned int d : difference)
				{
					Vector3r diffNeighDist;
					if (d < numParticles)
					{
						diffNeighDist = pd.getPosition(i) - pd.getPosition(d);
					}
					else
					{
						diffNeighDist = pd.getPosition(i) - model.getBoundaryX(d - numParticles);
					}
					printf("\t%d (%f, %f, %f){%f}; ", d, diffNeighDist[0], diffNeighDist[1], diffNeighDist[2], diffNeighDist.squaredNorm());
					if (diffNeighDist.squaredNorm() > 0.009)
					{
						printf("HOLD ON . Why is the difference so big?\n");
					}
					missingParticles.insert(d);
					if (model.getNeighborhoodSearch()->n_neighbors(sortIdx) == 0 && d >= numParticles)
					{
						test0 = true;
					}
				}
				printf("\n");
			}
			if (model.getNeighborhoodSearch()->n_neighbors(sortIdx) == 0 /* && !test0*/)
			{
				printf("HOLD UP! SPICY! %d\n", i);
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
			printf("%d, ", part);
			if (part < numParticles)
			{
				printf("Fluid: %d (%f, %f, %f); ", part, pd.getPosition(part)[0], pd.getPosition(part)[1], pd.getPosition(part)[2]);
				
				//auto it = find(&model.getBoundaryX(0), &model.getBoundaryX(0)+model.numBoundaryParticles(), pd.getPosition(part));
				//if (it != &model.getBoundaryX(0) + model.numBoundaryParticles()) printf("OH SHIT! %d is a boundry particle!\n");
				bool isBound = false;
				for (int j = 0; j < model.numBoundaryParticles(); j++)
				{
					if (pd.getPosition(part)[0] == model.getBoundaryX(j)[0] && pd.getPosition(part)[1] == model.getBoundaryX(j)[1] && pd.getPosition(part)[2] == model.getBoundaryX(j)[2])
					{
						isBound = true;
						break;
					}
				}
				if (isBound) printf("OH SHIT! %d is a boundry particle!\n");
			}
			else
			{
				printf("Boundry: %d (%f, %f, %f); ", part, model.getBoundaryX(part - numParticles)[0], model.getBoundaryX(part - numParticles)[1], model.getBoundaryX(part - numParticles)[2]);
			}
			
			//if (part != wrongParticles[i]) cursed = false;
			i++;
		}
		printf("\n");
		//if (cursed) printf("OH NO IT IS CURSED\n");

		difference.clear();
		std::set_difference(wrongParticles.begin(), wrongParticles.end(), missingParticles.begin(), missingParticles.end(),
			std::inserter(difference, difference.begin()));
		/*printf("\tDifference of wrong/missing particles (%d): ", difference.size());
		for (unsigned int d : difference)
		{
			printf("\t%d; ", d);
		}
		printf("\n");*/

		std::vector<unsigned int> intersection;
		intersection.reserve(1500);
		std::set_intersection(foundParticles.begin(), foundParticles.end(), missingParticles.begin(), missingParticles.end(),
			std::inserter(intersection, intersection.begin()));
		/*printf("Intersection between found and missing particles (%d): ", intersection.size());
		for (unsigned int intersec : intersection)
		{
			printf("\t%d; ", intersec);
		}
		printf("\n");*/
	}
	//printf("\n");

	Vector3r cpuVelSum = std::accumulate(testVelocity.begin(), testVelocity.end(), Vector3r(0.0f, 0.0f, 0.0f));
	printf("CPU velocity sum: %f, %f, %f\n", cpuVelSum[0], cpuVelSum[1], cpuVelSum[2]);
	Vector3r gpuVelSum = std::accumulate(&pd.getVelocity(0), &pd.getVelocity(0)+pd.getNumberOfParticles(), Vector3r(0.0f, 0.0f, 0.0f));
	printf("GPU velocity sum: %f, %f, %f\n", gpuVelSum[0], gpuVelSum[1], gpuVelSum[2]);
	float absVelDiff = (cpuVelSum - gpuVelSum).squaredNorm();
	printf("Absolute difference %f\n", absVelDiff);
	if (absVelDiff != 0.0f)
	{
		printf("AH SHIT\n");
	}
	if (absVelDiff > 1.0f)
	{
		printf("AH SHITx2!!\n");
	}

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
	Vector3r cpuVelSum = std::accumulate(&pd.getVelocity(0), &pd.getVelocity(0) + pd.getNumberOfParticles(), Vector3r(0.0f, 0.0f, 0.0f));
	printf("CPU velocity sum: %f, %f, %f\n", cpuVelSum[0], cpuVelSum[1], cpuVelSum[2]);
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
	/*NeighborhoodSearchSpatialHashing testNeight(model.getParticles().size(), model.getSupportRadius());
	testNeight.neighborhoodSearch(&model.getParticles().getPosition(0), model.numBoundaryParticles(), &model.getBoundaryX(0));

	unsigned int** neighbors = testNeight.getNeighbors();
	unsigned int* numNeighbors = testNeight.getNumNeighbors();*/

	/*unsigned int** neighbors = model.getNeighborhoodSearch()->getNeighbors();
	unsigned int* numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();*/

#elif defined(nSearch)

#else
	unsigned int** neighbors = model.getNeighborhoodSearch()->getNeighbors();
	unsigned int* numNeighbors = model.getNeighborhoodSearch()->getNumNeighbors();
#endif

#if defined(FSPH)
	NeighborhoodSearchSpatialHashing testNeight(model.getParticles().size(), model.getSupportRadius());
	testNeight.neighborhoodSearch(&model.getParticles().getPosition(0), model.numBoundaryParticles(), &model.getBoundaryX(0));

	unsigned int** neighbors = testNeight.getNeighbors();
	unsigned int* numNeighbors = testNeight.getNumNeighbors();

	std:vector<Real> testDensities(model.getDensities());

	Real GPUDensitySum = 0;
	Real CPUDensitySum = 0;

	/*unsigned int numCPUNeigh = 0;
	unsigned int numGPUNeigh = 0;

	for (int i = 0; i < nParticles; i++)
	{
		const int sortIdx = model.getNeighborhoodSearch()->partIdx(i);
		numCPUNeigh += numNeighbors[i];
		numGPUNeigh += model.getNeighborhoodSearch()->n_neighbors(sortIdx);
	}

	printf("Number of CPU neighbours: %d\nNumber of GPU neighbours: %d\n", numCPUNeigh, numGPUNeigh);
	printf("Neighbour difference CPU-GPU=%d\n", numCPUNeigh - numGPUNeigh);*/

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
				Real density_err_CPU;
				Real densityCPU = testDensities[i];
				/*PositionBasedFluids::computePBFDensity(i, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
					&model.getBoundaryPsi(0), numNeighbors[i], neighbors[i], model.getDensity0(), true, density_err, model.getDensity(i));
				PositionBasedFluids::computePBFLagrangeMultiplier(i, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
					&model.getBoundaryPsi(0), model.getDensity(i), numNeighbors[i], neighbors[i], model.getDensity0(), true, model.getLambda(i));*/

				//printf("Testing CPU:\n");
				PositionBasedFluids::computePBFDensity(i, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
					&model.getBoundaryPsi(0), numNeighbors[i], neighbors[i], model.getDensity0(), true, density_err_CPU, densityCPU);

				//printf("Testing GPU:\n");
				const int sortIdx = model.getNeighborhoodSearch()->partIdx(i);

				/*PositionBasedFluids::computePBFDensity(corrIdx, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
					&model.getBoundaryPsi(0), model.getNeighborhoodSearch()->n_neighbors(corrIdx), model.getNeighborhoodSearch()->neighbors(corrIdx),
					model.getDensity0(), true, density_err, model.getDensity(corrIdx));*/
				{
					const unsigned int particleIndex = i;
					const unsigned int numberOfParticles = nParticles;
					const Vector3r* x = &pd.getPosition(0);
					const Real* mass = &pd.getMass(0);
					const Vector3r* boundaryX = &model.getBoundaryX(0);
					const Real* boundaryPsi = &model.getBoundaryPsi(0);
					const unsigned int numNeighbors = model.getNeighborhoodSearch()->n_neighbors(sortIdx);
					//const unsigned int neighbors[];
					const Real density0 = model.getDensity0();
					const bool boundaryHandling = true;
					Real& density_err_0 = density_err;
					Real& density = model.getDensity(i);

					// Compute current density for particle i
					density = mass[particleIndex] * CubicKernel::W_zero();
					for (unsigned int j = 0; j < numNeighbors; j++)
					{
						const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
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

					density_err_0 = std::max(density, density0) - density0;

					//Real& density = model.getDensity(i);
					//const Real* mass = &pd.getMass(0);
					//const Vector3r* boundaryX = &model.getBoundaryX(0);
					//const Real* boundaryPsi = &model.getBoundaryPsi(0);
					//const Vector3r* x = &pd.getPosition(0);
					//const Real density0 = model.getDensity0();

					//// Compute current density for particle i
					//density = mass[i] * CubicKernel::W_zero();
					//for (unsigned int j = 0; j < model.getNeighborhoodSearch()->n_neighbors(sortIdx); j++)
					//{
					//	const unsigned int neighborIndex = model.getNeighborhoodSearch()->invNeighbor(sortIdx, j);
					//	//const unsigned int neighborIndex = model.getNeighborhoodSearch()->neighbor(sortIdx, j);
					//	if (neighborIndex < nParticles)		// Test if fluid particle
					//	{
					//		density += mass[neighborIndex] * CubicKernel::W(x[i] - x[neighborIndex]);
					//	}
					//	else
					//	{
					//		// Boundary: Akinci2012
					//		density += boundaryPsi[neighborIndex - nParticles] * CubicKernel::W(x[i] - boundaryX[neighborIndex - nParticles]);
					//	}
					//}
					//density_err = std::max(density, density0) - density0;
				}
				GPUDensitySum += density_err;
				CPUDensitySum += density_err_CPU;

				/*PositionBasedFluids::computePBFLagrangeMultiplier(corrIdx, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0),
					&model.getBoundaryPsi(0), model.getDensity(corrIdx), model.getNeighborhoodSearch()->n_neighbors(corrIdx), model.getNeighborhoodSearch()->neighbors(corrIdx),
					model.getDensity0(), true, model.getLambda(corrIdx));*/
				{
					
					const Vector3r* x = &pd.getPosition(0);
					const Real* mass = &pd.getMass(0);
					const Vector3r* boundaryX = &model.getBoundaryX(0);
					const Real* boundaryPsi = &model.getBoundaryPsi(0);
					const Real constDensity = model.getDensity(i);
					const Real density0 = model.getDensity0();
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
					{
						lambda = 0.0;
					}
				}
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
				/*PositionBasedFluids::solveDensityConstraint(i, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), &model.getBoundaryPsi(0),
					numNeighbors[i], neighbors[i], model.getDensity0(), true, &model.getLambda(0), corr);
				model.getDeltaX(i) = corr;*/
				int sortIdx = model.getNeighborhoodSearch()->partIdx(i);
				/*PositionBasedFluids::solveDensityConstraint(corrIdx, nParticles, &pd.getPosition(0), &pd.getMass(0), &model.getBoundaryX(0), &model.getBoundaryPsi(0),
					model.getNeighborhoodSearch()->n_neighbors(corrIdx), model.getNeighborhoodSearch()->neighbors(corrIdx), model.getDensity0(), true, &model.getLambda(0), corr);*/
				{
					const Vector3r* x = &pd.getPosition(0);
					const Real* mass = &pd.getMass(0);
					const Vector3r* boundaryX = &model.getBoundaryX(0);
					const Real* boundaryPsi = &model.getBoundaryPsi(0);
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
							assert(!std::isnan(gradC_j[0]));
							assert(!std::isnan(gradC_j[1]));
							assert(!std::isnan(gradC_j[2]));
							assert(!std::isinf(gradC_j[0]));
							assert(!std::isinf(gradC_j[1]));
							assert(!std::isinf(gradC_j[2]));
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

#if defined(FSPH)
		/*printf("CPU density sum: %f\nGPU density sum: %f\n", CPUDensitySum, GPUDensitySum);
		printf("Density difference CPU-GPU=%f\n", CPUDensitySum - GPUDensitySum);*/
#endif

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

