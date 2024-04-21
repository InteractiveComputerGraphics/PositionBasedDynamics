#pragma once

#include "Common/Common.h"

#include "sph_kernel.cuh.hip"
#include "sph_arrangement.cuh.hip"
#include "sph_particle.h.hip"

namespace PBD
{
	class Spatial_FSPH
	{
	public:
		Spatial_FSPH(const Real radius = 0.1, const unsigned int numBoundry = 0, const unsigned int numParticles = 0);
		~Spatial_FSPH();

		void cleanup();
		void neighborhoodSearch(Vector3r* x);
		void neighborhoodSearch(Vector3r* x, const unsigned int numParticles, const unsigned int numBoundaryParticles, Vector3r* boundaryX);
		void update();
		unsigned int** getNeighbors() const;
		//const unsigned int getMaxNeighbors() const { return m_maxNeighbors; }

		unsigned int getNumParticles() const;
		void setRadius(const Real radius);
		Real getRadius() const;

		unsigned int n_neighbors(unsigned int i) const
		{
			return neigh->counts[i];
		}

		int* getNumNeighbors() const
		{
			return neigh->counts;
		}

		/*unsigned int* neighbors(unsigned int i) const
		{
			return nSearch.point_set(particleIndex).neighbor_list(particleIndex, i);
		}*/
		unsigned int neighbor(unsigned int i, unsigned int k) const
		{
			return neigh->neighbors[neigh->offsets[i] + k];
		}

		unsigned int partIdx(unsigned int i) const
		{
			return idx2Part[i];
		}
		unsigned int idxIdx(unsigned int i) const
		{
			return part2Idx[i];
		}
		unsigned int invNeighbor(unsigned int i, unsigned int k) const
		{
			return idxIdx(neighbor(i, k));
		}


	private:
		unsigned int m_currentTimestamp;

		Neighbours* neigh;

		std::size_t const N = 120;

		int* idx2Part;
		int* part2Idx;

		/*float const r_omega = static_cast<float>(0.15);
		float const r_omega2 = r_omega * r_omega;
		float const radius = static_cast<float>(2.0) * (static_cast<float>(2.0) * r_omega / static_cast<float>(N - 1));*/

		uint buff_capacity_ = 881968;
		uint nump_ = 0U;
		uint nBoundry = 0U;
		sph::ParticleBufferObject host_buff_;
		sph::ParticleBufferObject device_buff_;
		sph::ParticleBufferObject device_buff_temp_;

		sph::Arrangement* arrangement_;
		sph::SystemParameter sysPara;
	};
}

