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
	return hipNSearch.point_set(particleIndex).n_points();
}

void Spatial_hipNSearch::setRadius(const Real radius)
{
	hipNSearch.set_radius(radius);
}

Real Spatial_hipNSearch::getRadius() const
{
	return hipNSearch.radius();
}

void Spatial_hipNSearch::update()
{
	m_currentTimestamp++;
}

//void Spatial_hipNSearch::sort(int i)
//{
//	cuNSearch::PointSet& pointSet = nSearch.point_set(i);
//	nSearch.deviceData->updateSort(pointSet);
//	nSearch.z_sort();
//	pointSet.sort_field((Vector3r*)pointSet.GetPoints());
//}

void Spatial_hipNSearch::addParticles(const unsigned int numParticles, Vector3r* boundryX, const unsigned int numBoundry)
{
	particles.insert(particles.begin(), numParticles, Vector3r(0.0f, 0.0f, 0.0f));
	particles.insert(particles.begin() + numParticles, boundryX, boundryX + numBoundry);
	particleIndex = hipNSearch.add_point_set(particles[0].data(), particles.size(), true, true);
	//nSearch.find_neighbors();
	//nSearch.point_set(particleIndex).makeInv();
	
}

void Spatial_hipNSearch::addBoundry(Vector3r* x, const unsigned int numParticles)
{
	particles.insert(particles.begin(), x, x + numParticles);
	boundryIndex = hipNSearch.add_point_set(particles[0].data(), particles.size(), true, true);
	//nSearch.find_neighbors();
	//nSearch.point_set(boundryIndex).makeInv();
	//printf("AH\n");
	//assert(particles.size() == numberOfParticles);
}

void Spatial_hipNSearch::neighborhoodSearch(Vector3r* x, const unsigned int numParticles, Vector3r* boundryX, const unsigned int numBoundry)
{
#ifdef TAKETIME
	START_TIMING("Unnecessary Copy");
#endif // TAKETIME
	std::copy(x, x + numParticles, particles.begin()); //particles.insert(particles.begin(), x, x + numParticles);
	std::copy(boundryX, boundryX + numBoundry, particles.begin() + numParticles); //particles.insert(particles.begin() + numParticles, boundryX, boundryX + numBoundry);
#ifdef TAKETIME
	STOP_TIMING_AVG;
#endif // TAKETIME	
	//const float* tmp0 = particles[0].data();
	//nSearch.sort(nSearch.point_set(particleIndex));
	//nSearch.point_set(particleIndex).makeInverse();
	//const float* tmp1 = nSearch.point_set(particleIndex).GetPoints();
#ifdef TAKETIME
	START_TIMING("Run hipNSearch");
#endif // TAKETIME
	hipNSearch.find_neighbors(true);
#ifdef TAKETIME
	STOP_TIMING_AVG;
#endif // TAKETIME	
	//assert(particles.size() == numberOfParticles);
}

void Spatial_hipNSearch::neighborhoodSearchBoundry(Vector3r* x, const unsigned int numParticles)
{
	std::copy(x, x + numParticles, particles.begin()); //particles.insert(particles.begin(), x, x + numParticles);
	//const float* tmp0 = particles[0].data();
	//const unsigned int* tmp0 = nSearch.point_set(boundryIndex).sortIndices.data();
	//nSearch.sort(nSearch.point_set(boundryIndex));
	//nSearch.point_set(boundryIndex).makeInverse();
	//const float* tmp1 = nSearch.point_set(boundryIndex).GetPoints();
	//const unsigned int* tmp1 = nSearch.point_set(boundryIndex).sortIndices.data();
	hipNSearch.find_neighbors(true);
	//const float* tmp2 = nSearch.point_set(boundryIndex).GetPoints();
	//const unsigned int* tmp2= nSearch.point_set(boundryIndex).sortIndices.data();
	//int i = 0;
	//assert(particles.size() == numberOfParticles);
}