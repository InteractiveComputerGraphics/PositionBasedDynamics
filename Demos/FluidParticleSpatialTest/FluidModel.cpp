#include "FluidModel.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"
#include "PositionBasedDynamics/SPHKernels.h"

#include <set>
#include <numeric>
#include <algorithm>

using namespace PBD;

FluidModel::FluidModel() :
	m_particles()
{	
	m_density0 = static_cast<Real>(1000.0);
	m_particleRadius = static_cast<Real>(0.025);
	viscosity = static_cast<Real>(0.02);
	m_neighborhoodSearch = NULL;
}

FluidModel::~FluidModel(void)
{
	cleanupModel();
}

void FluidModel::cleanupModel()
{
	m_particles.release();
	m_lambda.clear();
	m_density.clear();
	m_deltaX.clear();
	delete m_neighborhoodSearch;
}

void FluidModel::reset()
{
	const unsigned int nPoints = m_particles.size();
	
	for(unsigned int i=0; i < nPoints; i++)
	{
		const Vector3r& x0 = m_particles.getPosition0(i);
		m_particles.getPosition(i) = x0;
		m_particles.getLastPosition(i) = m_particles.getPosition(i);
		m_particles.getOldPosition(i) = m_particles.getPosition(i);
		m_particles.getVelocity(i).setZero();
		m_particles.getAcceleration(i).setZero();
		m_deltaX[i].setZero();
		m_lambda[i] = 0.0;
		m_density[i] = 0.0;
	}
}

ParticleData & PBD::FluidModel::getParticles()
{
	return m_particles;
}

void FluidModel::initMasses()
{
	const int nParticles = (int) m_particles.size();
	const Real diam = static_cast<Real>(2.0)*m_particleRadius;

	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  
		for (int i = 0; i < nParticles; i++)
		{
			m_particles.setMass(i, static_cast<Real>(0.8) * diam*diam*diam * m_density0);		// each particle represents a cube with a side length of r		
																			// mass is slightly reduced to prevent pressure at the beginning of the simulation
		}
	}
}


/** Resize the arrays containing the particle data.
*/
void FluidModel::resizeFluidParticles(const unsigned int newSize)
{
	m_particles.resize(newSize);
	m_lambda.resize(newSize);
	m_density.resize(newSize);
	m_deltaX.resize(newSize);
}


/** Release the arrays containing the particle data.
*/
void FluidModel::releaseFluidParticles()
{
	m_particles.release();
	m_lambda.clear();
	m_density.clear();
	m_deltaX.clear();
}

void FluidModel::initModel(const unsigned int nFluidParticles, Vector3r* fluidParticles, const unsigned int nBoundaryParticles, Vector3r* boundaryParticles)
{
	releaseFluidParticles();
	resizeFluidParticles(nFluidParticles);

	// init kernel
	CubicKernel::setRadius(m_supportRadius);

	// copy fluid positions
	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  
		for (int i = 0; i < (int)nFluidParticles; i++)
		{
			m_particles.getPosition0(i) = fluidParticles[i];
		}
	}

	m_boundaryX.resize(nBoundaryParticles);
	m_boundaryPsi.resize(nBoundaryParticles);

	// copy boundary positions
	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  
		for (int i = 0; i < (int)nBoundaryParticles; i++)
		{
			m_boundaryX[i] = boundaryParticles[i];
		}
	}

	// initialize masses
	initMasses();

	//////////////////////////////////////////////////////////////////////////
	// Compute value psi for boundary particles (boundary handling)
	// (see Akinci et al. "Versatile rigid - fluid coupling for incompressible SPH", Siggraph 2012
	//////////////////////////////////////////////////////////////////////////

	// Search boundary neighborhood
#if defined(FSPH)
	float min_x = std::numeric_limits<float>::max();
	float max_x = std::numeric_limits<float>::min();
	for (int i = 0; i < nBoundaryParticles; i++)
	{
		Vector3r x = m_boundaryX[i];
		if (min_x > x[2])
		{			  
			min_x = x[2];
		}			  
		if (max_x < x[2])
		{			  
			max_x = x[2];
		}
	}
	printf("The min x %f\nThe max x %f\n", min_x, max_x);

	//TEST CPU
	std::vector<Real> testBoundPsi(m_boundaryPsi);
	printf("CPU version:\n");
	std::vector<int> neighboursCPU;
	neighboursCPU.reserve(197384);
	NeighborhoodSearchSpatialHashing testNeight(nBoundaryParticles, m_supportRadius);
	testNeight.neighborhoodSearch(&m_boundaryX[0]);

	unsigned int** neighbors = testNeight.getNeighbors();
	unsigned int* numNeighbors = testNeight.getNumNeighbors();

	/*#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  */
		for (int i = 0; i < (int)nBoundaryParticles; i++)
		{
			Real delta = CubicKernel::W_zero();
			//if (numNeighbors[i] < 9) printf("%d has less than 9 neighbours (%d) \n", i, numNeighbors[i]);
			if (i < 22) printf("%d (%d): ", i, numNeighbors[i]);
			for (unsigned int j = 0; j < numNeighbors[i]; j++)
			{
				const unsigned int neighborIndex = neighbors[i][j];
				delta += CubicKernel::W(m_boundaryX[i] - m_boundaryX[neighborIndex]);
				if (i < 22) printf("%d; ", neighborIndex);
				neighboursCPU.push_back(neighborIndex);
			}
			if (i < 22) printf("\n");
			const Real volume = static_cast<Real>(1.0) / delta;
			testBoundPsi[i] = m_density0 * volume;
		}
	//}
	printf("\n");
	std::sort(neighboursCPU.begin(), neighboursCPU.end());

	printf("GPU version:\n");
	std::vector<int> neighboursGPU;
	neighboursGPU.reserve(200344);
	//m_neighborhoodSearch = new Spatial_FSPH(m_supportRadius, nBoundaryParticles, m_particles.size());
	//Spatial_FSPH& neighborhoodSearchSH = *m_neighborhoodSearch;
	Spatial_FSPH neighborhoodSearchSH(m_supportRadius, 0, nBoundaryParticles);
	neighborhoodSearchSH.neighborhoodSearch(&m_boundaryX[0]);

	/*#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  */
		for (int i = 0; i < (int)nBoundaryParticles; i++)
		{
			const unsigned int sortIdx = neighborhoodSearchSH.partIdx(i);
			Real delta = CubicKernel::W_zero();
			std::vector<int> tmp;
			tmp.reserve(neighborhoodSearchSH.n_neighbors(sortIdx));
			if (i < 22) printf("%d (%d): ", i, neighborhoodSearchSH.n_neighbors(sortIdx));
			for (unsigned int j = 0; j < neighborhoodSearchSH.n_neighbors(sortIdx); j++)
			{
				const unsigned int neighborIndex = neighborhoodSearchSH.invNeighbor(sortIdx, j); //neighborhoodSearchSH.sortIdxBoundry(neighborhoodSearchSH.neighborBoundry(sortIdx, j)); neighborhoodSearchSH.neighborBoundry(sortIdx, j) neighborhoodSearchSH.invNeighborBoundry(sortIdx, j)
				delta += CubicKernel::W(m_boundaryX[i] - m_boundaryX[neighborIndex]);
				if (i < 22) printf("%d; ", neighborIndex);
				neighboursGPU.push_back(neighborIndex);
				tmp.push_back(neighborIndex);
			}
			if (i < 22) printf("\n");
			const Real volume = static_cast<Real>(1.0) / delta;
			m_boundaryPsi[i] = m_density0 * volume;

			std::sort(tmp.begin(), tmp.end());

			auto point = tmp.begin();
			while (point != tmp.end())
			{
				point = std::adjacent_find(point, tmp.end());
				if (point == tmp.end()) break;
				printf("(%d, %d) has duplicate neighbors with index %d\n", i, sortIdx, *point);
			}
		}
	//}
	std::sort(neighboursGPU.begin(), neighboursGPU.end());
	printf("\n");

	printf("Number of CPU neighbours: %d\nNumber of GPU neighbours: %d\n", neighboursCPU.size(), neighboursGPU.size());
	std::vector<unsigned int> neighDiff;
	neighDiff.reserve(3000);

	std::set_difference(neighboursGPU.begin(), neighboursGPU.end(), neighboursCPU.begin(), neighboursCPU.end(),
		std::inserter(neighDiff, neighDiff.begin()));
	printf("Difference(% d) : ", neighDiff.size());
	/*for (unsigned int d : neighDiff)
	{
		printf("\t%d; ", d);
	}*/
	printf("\n");

	printf("Testing pressure:\n");
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
	for (int i = 0; i < (int)nBoundaryParticles; i++)
	{
		bool subSame = true;
		int sortIdx = neighborhoodSearchSH.partIdx(i);
		Real diff = m_boundaryPsi[i] - testBoundPsi[i];
		if (diff > 0.000001 || neighborhoodSearchSH.n_neighbors(sortIdx) != numNeighbors[i])
		{
			wrongParticles.push_back(i);
			tmpCPU.clear();
			tmpGPU.clear();
			difference.clear();
			//printf("## %d ##\n", i);
			county++;
			//printf("\t%f - %f = %f\n", m_boundaryPsi[i], testBoundPsi[i], diff);

			//printf("\tCPU (%d):", numNeighbors[i]);
			for (unsigned int j = 0; j < numNeighbors[i]; j++)
			{
				const unsigned int neighborIndex = neighbors[i][j];
				tmpCPU.push_back(neighborIndex);
				//printf("\t%d; ", neighborIndex);
			}
			std::sort(tmpCPU.begin(), tmpCPU.end());
			//printf("\n");

			const unsigned int sortIdx = neighborhoodSearchSH.partIdx(i);
			//printf("\tGPU (%d): ", neighborhoodSearchSH.n_neighbors(sortIdx));
			for (unsigned int j = 0; j < neighborhoodSearchSH.n_neighbors(sortIdx); j++)
			{
				const unsigned int neighborIndex = neighborhoodSearchSH.invNeighbor(sortIdx, j);
				tmpGPU.push_back(neighborIndex);
				foundParticles.insert(neighborIndex);
				//printf("\t%d; ", neighborIndex);
			}
			std::sort(tmpGPU.begin(), tmpGPU.end());
			//printf("\n");

			

			std::set_difference(tmpGPU.begin(), tmpGPU.end(), tmpCPU.begin(), tmpCPU.end(),
				std::inserter(difference, difference.begin()));
			if (difference.size() > 0)
			{
				printf("## %d ##\n", i);
				printf("\tDifference GPU/CPU (%d): ", difference.size());
				for (unsigned int d : difference)
				{
					printf("\t%d; ", d);
					if (m_boundaryX[i] != m_boundaryX[d]) subSame = false;
					missingParticles.insert(d);
				}
				printf("\n");
			}

			difference.clear();
			std::set_difference(tmpCPU.begin(), tmpCPU.end(), tmpGPU.begin(), tmpGPU.end(),
				std::inserter(difference, difference.begin()));
			if (difference.size() > 0)
			{
				printf("## %d ##\n", i);
				printf("\tDifference CPU/GPU (%d): ", difference.size());
				for (unsigned int d : difference)
				{
					printf("\t%d; ", d);
					if (m_boundaryX[i] != m_boundaryX[d]) subSame = false;
					missingParticles.insert(d);
				}
				printf("\n");
			}
			
		}
		if (!subSame) same = false;
	}

	if (same) printf("They all have the same coordinates\n");

	{
		printf("Number of wrongs %d/%d: \n", missingParticles.size(), county);
		bool cursed = true;
		int i = 0;
		for (unsigned int part : missingParticles)
		{
			/*printf("%d, ", part);
			if (part != wrongParticles[i]) cursed = false;*/
			printf("%d (%f, %f, %f); ", part, m_boundaryX[part][0], m_boundaryX[part][1], m_boundaryX[part][2]);
			i++;
		}
		if (cursed) printf("OH NO IT IS CURSED\n");
		printf("\n");
		difference.clear();
		std::set_difference(missingParticles.begin(), missingParticles.end(), wrongParticles.begin(), wrongParticles.end(),
			std::inserter(difference, difference.begin()));
		printf("\tDifference of missing/wrong particles (%d): ", difference.size());
		/*for (unsigned int d : difference)
		{
			printf("\t%d; ", d);
		}*/
		printf("\n");

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
		for (unsigned int intersec : intersection)
		{
			//printf("\t(%d, [%f, %f, %f]]\n); ", intersec, m_boundaryX[intersec][0], m_boundaryX[intersec][1], m_boundaryX[intersec][2]);
		}
	}
	printf("\n");

	float cpuPsiSum = std::accumulate(testBoundPsi.begin(), testBoundPsi.end(), 0.0f);
	printf("CPU pressure sum: %f\n", cpuPsiSum);
	float gpuPsiSum = std::accumulate(m_boundaryPsi.begin(), m_boundaryPsi.end(), 0.0f);
	printf("GPU pressure sum: %f\n", gpuPsiSum);
	printf("\n");

	// Initialize neighborhood search
	neighborhoodSearchSH.~Spatial_FSPH();
	if (m_neighborhoodSearch == NULL)
		m_neighborhoodSearch = new Spatial_FSPH(m_supportRadius, nBoundaryParticles, m_particles.size());
	/*if (m_neighborhoodSearch == NULL)
		m_neighborhoodSearch = new NeighborhoodSearchSpatialHashing(m_particles.size(), m_supportRadius);
	m_neighborhoodSearch->setRadius(m_supportRadius);*/

#elif defined(nSearch)
	Spatial_hipNSearch neighborhoodSearchSH(m_supportRadius, 0, nBoundaryParticles);
	neighborhoodSearchSH.addBoundry(&m_boundaryX[0], nBoundaryParticles);
	neighborhoodSearchSH.neighborhoodSearchBoundry(&m_boundaryX[0], nBoundaryParticles);

	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  
		for (int i = 0; i < (int)nBoundaryParticles; i++)
		{
			const unsigned int sortIdx = i;//neighborhoodSearchSH.sortIdxBoundry(i);
			Real delta = CubicKernel::W_zero();
			for (unsigned int j = 0; j < neighborhoodSearchSH.n_neighborsBoundry(sortIdx); j++)
			{
				const unsigned int neighborIndex = neighborhoodSearchSH.neighborBoundry(sortIdx, j); //neighborhoodSearchSH.sortIdxBoundry(neighborhoodSearchSH.neighborBoundry(sortIdx, j)); neighborhoodSearchSH.neighborBoundry(sortIdx, j) neighborhoodSearchSH.invNeighborBoundry(sortIdx, j)
				delta += CubicKernel::W(m_boundaryX[i] - m_boundaryX[neighborIndex]);
			}
			const Real volume = static_cast<Real>(1.0) / delta;
			m_boundaryPsi[i] = m_density0 * volume;
		}
	}

	int countSum = 0;
	for (int i = 0; i < (int)nBoundaryParticles; i++)
	{
		countSum += neighborhoodSearchSH.n_neighborsBoundry(i);
	}

	// Initialize neighborhood search
	if (m_neighborhoodSearch == NULL)
		m_neighborhoodSearch = new Spatial_hipNSearch(m_supportRadius, nBoundaryParticles, m_particles.size());
	m_neighborhoodSearch->setRadius(m_supportRadius);
	m_neighborhoodSearch->addParticles(m_particles.size(), &m_boundaryX[0], nBoundaryParticles);

#else
	NeighborhoodSearchSpatialHashing neighborhoodSearchSH(nBoundaryParticles, m_supportRadius);
	neighborhoodSearchSH.neighborhoodSearch(&m_boundaryX[0]);

	unsigned int** neighbors = neighborhoodSearchSH.getNeighbors();
	unsigned int* numNeighbors = neighborhoodSearchSH.getNumNeighbors();

	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  
		for (int i = 0; i < (int)nBoundaryParticles; i++)
		{
			Real delta = CubicKernel::W_zero();
			for (unsigned int j = 0; j < numNeighbors[i]; j++)
			{
				const unsigned int neighborIndex = neighbors[i][j];
				delta += CubicKernel::W(m_boundaryX[i] - m_boundaryX[neighborIndex]);
			}
			const Real volume = static_cast<Real>(1.0) / delta;
			m_boundaryPsi[i] = m_density0 * volume;
		}
	}

	// Initialize neighborhood search
	if (m_neighborhoodSearch == NULL)
		m_neighborhoodSearch = new NeighborhoodSearchSpatialHashing(m_particles.size(), m_supportRadius);
	m_neighborhoodSearch->setRadius(m_supportRadius);

#endif
	

	reset();
}