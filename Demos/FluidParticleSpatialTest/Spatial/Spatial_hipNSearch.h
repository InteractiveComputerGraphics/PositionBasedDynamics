#pragma once

#include "Common/Common.h"
#include "hipNSearch/cuNSearch.h"

using Real3 = std::array<Real, 3>;

namespace PBD
{
	class Spatial_hipNSearch
	{
	public:
		Spatial_hipNSearch(const Real radius = 0.1, const unsigned int numBoundry = 0, const unsigned int numParticle = 0) : hipNSearch(radius), m_currentTimestamp(0)
		{
			numberOfParticles = numBoundry + numParticle;
			particles.reserve(numberOfParticles);
			printf("Real3: %d; Vector3r: %d\n", sizeof(Real3), sizeof(Vector3r));
		};
		~Spatial_hipNSearch();

		void cleanup();
		void neighborhoodSearch(Vector3r* x, const unsigned int numParticles, Vector3r* boundryX, const unsigned int numBoundry);
		void neighborhoodSearchBoundry(Vector3r* x, const unsigned int numParticles);
		void addParticles(const unsigned int numParticles = 0, Vector3r* boundryX = nullptr, const unsigned int numBoundry = 0);
		void addBoundry(Vector3r* x, const unsigned int numParticles = 0);
		void update();
		//unsigned int** getNeighbors() const;
		//unsigned int* getNumNeighbors() const;
		const unsigned int getMaxNeighbors() const { return 0; } //TODO

		unsigned int getNumParticles() const;
		void setRadius(const Real radius);
		Real getRadius() const;

		unsigned int n_neighbors(unsigned int i) const
		{
			return hipNSearch.point_set(particleIndex).n_neighbors(particleIndex, i);
		}
		unsigned int* neighbors(unsigned int i) const
		{
			return hipNSearch.point_set(particleIndex).neighbor_list(particleIndex, i);
		}
		unsigned int neighbor(unsigned int i, unsigned int k) const
		{
			return hipNSearch.point_set(particleIndex).neighbor(particleIndex, i, k);
		}
		unsigned int sortIdx(unsigned int i) const
		{
			return hipNSearch.point_set(particleIndex).sortIndices[i];
		}
		unsigned int invSortIdx(unsigned int i) const
		{
			return hipNSearch.point_set(particleIndex).invSortIndices[i];
		}
		unsigned int invNeighbor(unsigned int i, unsigned int k) const
		{
			return invSortIdx(neighbor(i, k));
			//return sortIdx(neighbor(i, k));
		}

		unsigned int n_neighborsBoundry(unsigned int i) const
		{
			return hipNSearch.point_set(boundryIndex).n_neighbors(boundryIndex, i);
		}
		unsigned int neighborBoundry(unsigned int i, unsigned int k) const
		{
			return hipNSearch.point_set(boundryIndex).neighbor(boundryIndex, i, k);
		}
		unsigned int sortIdxBoundry(unsigned int i) const
		{
			return hipNSearch.point_set(boundryIndex).sortIndices[i];
		}
		unsigned int invSortIdxBoundry(unsigned int i) const
		{
			return hipNSearch.point_set(boundryIndex).invSortIndices[i];
		}
		unsigned int invNeighborBoundry(unsigned int i, unsigned int k) const
		{
			return invSortIdxBoundry(neighborBoundry(i, k));
		}

	private:
		int particleIndex;
		int boundryIndex;
		std::vector<Vector3r> particles;
		unsigned int m_currentTimestamp;
		cuNSearch::NeighborhoodSearch hipNSearch;
		int numberOfParticles;

		void Spatial_hipNSearch::sort(int i);
	};
}