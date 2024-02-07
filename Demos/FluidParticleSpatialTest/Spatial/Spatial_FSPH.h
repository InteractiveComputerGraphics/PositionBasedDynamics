#pragma once

#include "Common/Common.h"

namespace PBD
{
	class Spatial_FSPH
	{
	public:
		Spatial_FSPH(const unsigned int numParticles = 0, const Real radius = 0.1, const unsigned int maxNeighbors = 60u, const unsigned int maxParticlesPerCell = 50u);
		~Spatial_FSPH();

		void cleanup();
		void neighborhoodSearch(Vector3r* x);
		void neighborhoodSearch(Vector3r* x, const unsigned int numBoundaryParticles, Vector3r* boundaryX);
		void update();
		unsigned int** getNeighbors() const;
		unsigned int* getNumNeighbors() const;
		const unsigned int getMaxNeighbors() const { return m_maxNeighbors; }

		unsigned int getNumParticles() const;
		void setRadius(const Real radius);
		Real getRadius() const;

		FORCE_INLINE unsigned int n_neighbors(unsigned int i) const
		{
			return m_numNeighbors[i];
		}
		FORCE_INLINE unsigned int neighbor(unsigned int i, unsigned int k) const
		{
			return m_neighbors[i][k];
		}


	private:
		unsigned int m_numParticles;
		unsigned int m_maxNeighbors;
		unsigned int m_maxParticlesPerCell;
		unsigned int** m_neighbors;
		unsigned int* m_numNeighbors;
		Real m_cellGridSize;
		Real m_radius2;
		unsigned int m_currentTimestamp;
	};
}

