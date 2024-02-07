#include "Spatial_FSPH.h"

using namespace PBD;

Spatial_FSPH::Spatial_FSPH(const unsigned int numParticles, const Real radius, const unsigned int maxNeighbors, const unsigned int maxParticlesPerCell)
{

}

Spatial_FSPH::~Spatial_FSPH()
{

}

void Spatial_FSPH::cleanup()
{

}

unsigned int** Spatial_FSPH::getNeighbors() const
{
	return m_neighbors;
}

unsigned int* Spatial_FSPH::getNumNeighbors() const
{
	return m_numNeighbors;
}

unsigned int Spatial_FSPH::getNumParticles() const
{
	return m_numParticles;
}

void Spatial_FSPH::setRadius(const Real radius)
{
	m_cellGridSize = radius;
	m_radius2 = radius * radius;
}

Real Spatial_FSPH::getRadius() const
{
	return sqrt((long double) m_radius2);
}

void Spatial_FSPH::update()
{
	m_currentTimestamp++;
}


void Spatial_FSPH::neighborhoodSearch(Vector3r* x)
{

}

void Spatial_FSPH::neighborhoodSearch(Vector3r* x, const unsigned int numBoundaryParticles, Vector3r* boundaryX)
{

}