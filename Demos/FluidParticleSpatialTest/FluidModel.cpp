#include "FluidModel.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"
#include "PositionBasedDynamics/SPHKernels.h"

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

#elif defined(nSearch)
	//TEST CPU
	{
		printf("CPU version: \n");
		std::vector<Real> testBoundPsi(m_boundaryPsi);
		NeighborhoodSearchSpatialHashing testNeight(nBoundaryParticles, m_supportRadius);
		testNeight.neighborhoodSearch(&m_boundaryX[0]);

		unsigned int** neighbors = testNeight.getNeighbors();
		unsigned int* numNeighbors = testNeight.getNumNeighbors();

		#pragma omp parallel default(shared)
		{
			#pragma omp for schedule(static)  
			//for (int i = 0; i < (int)nBoundaryParticles; i++)
			for (int i = 0; i < 22; i++)
			{
				Real delta = CubicKernel::W_zero();
				printf("%d (%d): ", i, numNeighbors[i]);
				for (unsigned int j = 0; j < numNeighbors[i]; j++)
				{
					const unsigned int neighborIndex = neighbors[i][j];
					delta += CubicKernel::W(m_boundaryX[i] - m_boundaryX[neighborIndex]);
					printf("%d; ", neighborIndex);
				}
				printf("\n");
				const Real volume = static_cast<Real>(1.0) / delta;
				testBoundPsi[i] = m_density0 * volume;
			}
		}
		printf("\n");
	}
	
	printf("GPU version: \n");
	Spatial_hipNSearch neighborhoodSearchSH(m_supportRadius, 0, nBoundaryParticles);
	neighborhoodSearchSH.addBoundry(&m_boundaryX[0], nBoundaryParticles);
	neighborhoodSearchSH.neighborhoodSearchBoundry(&m_boundaryX[0], nBoundaryParticles);

	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  
		for (int i = 0; i < (int)nBoundaryParticles; i++)
		{
			const unsigned int sortIdx = neighborhoodSearchSH.sortIdxBoundry(i);
			Real delta = CubicKernel::W_zero();
			if(sortIdx < 22) printf("%d (%d): ", sortIdx, neighborhoodSearchSH.n_neighborsBoundry(sortIdx));
			for (unsigned int j = 0; j < neighborhoodSearchSH.n_neighborsBoundry(sortIdx); j++)
			{
				const unsigned int neighborIndex = neighborhoodSearchSH.sortIdxBoundry(neighborhoodSearchSH.neighborBoundry(sortIdx, j)); //neighborhoodSearchSH.sortIdxBoundry(neighborhoodSearchSH.neighborBoundry(sortIdx, j)); neighborhoodSearchSH.neighborBoundry(sortIdx, j) neighborhoodSearchSH.invNeighborBoundry(sortIdx, j)
				delta += CubicKernel::W(m_boundaryX[i] - m_boundaryX[neighborIndex]);
				if (sortIdx < 22) printf("%d; ", neighborIndex);
			}
			if (sortIdx < 22) printf("\n");
			const Real volume = static_cast<Real>(1.0) / delta;
			m_boundaryPsi[i] = m_density0 * volume;
		}
	}
	printf("\n");

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