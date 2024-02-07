#include "Spatial_hipNSearch.h"

using namespace PBD;

Spatial_hipNSearch::~Spatial_hipNSearch()
{
	
}

void Spatial_hipNSearch::cleanup()
{

}

unsigned int Spatial_hipNSearch::getNumParticles() const
{
	return nSearch.point_set(particleIndex).n_points();
}

void Spatial_hipNSearch::setRadius(const Real radius)
{
	nSearch.set_radius(radius);
}

Real Spatial_hipNSearch::getRadius() const
{
	return nSearch.radius();
}

void Spatial_hipNSearch::update()
{
	m_currentTimestamp++;
}

void Spatial_hipNSearch::addParticles(const unsigned int numParticles, Vector3r* boundryX, const unsigned int numBoundry)
{
	particles.insert(particles.begin(), numParticles, Vector3r(0.0f, 0.0f, 0.0f));
	particles.insert(particles.begin() + numParticles, boundryX, boundryX + numBoundry);
	particleIndex = nSearch.add_point_set(particles[0].data(), particles.size(), true, true);
	nSearch.find_neighbors();
	nSearch.point_set(particleIndex).makeInv();
}

void Spatial_hipNSearch::addBoundry(Vector3r* x, const unsigned int numParticles)
{
	particles.insert(particles.begin(), x, x + numParticles);
	boundryIndex = nSearch.add_point_set(particles[0].data(), particles.size(), true, true);
	nSearch.find_neighbors();
	nSearch.point_set(boundryIndex).makeInv();
}

void Spatial_hipNSearch::neighborhoodSearch(Vector3r* x, const unsigned int numParticles, Vector3r* boundryX, const unsigned int numBoundry)
{
	std::copy(x, x + numParticles, particles.begin()); //particles.insert(particles.begin(), x, x + numParticles);
	std::copy(boundryX, boundryX + numBoundry, particles.begin() + numParticles); //particles.insert(particles.begin() + numParticles, boundryX, boundryX + numBoundry);
	//const float* tmp0 = particles[0].data();
	nSearch.update_point_set(particleIndex);
	nSearch.z_sort();
	nSearch.point_set(particleIndex).sort_field((Vector3r*)nSearch.point_set(particleIndex).GetPoints());
	//nSearch.point_set(particleIndex).makeInverse();
	//const float* tmp1 = nSearch.point_set(particleIndex).GetPoints();
	nSearch.find_neighbors(false);
}

void Spatial_hipNSearch::neighborhoodSearchBoundry(Vector3r* x, const unsigned int numParticles)
{
	std::copy(x, x + numParticles, particles.begin()); //particles.insert(particles.begin(), x, x + numParticles);
	nSearch.update_point_set(boundryIndex);
	nSearch.z_sort();
	nSearch.point_set(boundryIndex).sort_field((Vector3r*)nSearch.point_set(boundryIndex).GetPoints());
	//nSearch.point_set(boundryIndex).makeInverse();
	nSearch.find_neighbors(false);
}