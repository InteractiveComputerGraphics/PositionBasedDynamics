#ifndef __FluidModel_h__
#define __FluidModel_h__

#include "Simulation/ParticleData.h"
#include <vector>
#include "Simulation/NeighborhoodSearchSpatialHashing.h"

namespace PBD 
{	
	class FluidModel 
	{
		public:
			FluidModel();
			virtual ~FluidModel();

		protected:	
			Real viscosity;
			Real m_density0;
			Real m_particleRadius;
			Real m_supportRadius;
			ParticleData m_particles;
			std::vector<Vector3r> m_boundaryX;
			std::vector<Real> m_boundaryPsi;
			std::vector<Real> m_density;
			std::vector<Real> m_lambda;		
			std::vector<Vector3r> m_deltaX;
			NeighborhoodSearchSpatialHashing *m_neighborhoodSearch;			

			void initMasses();

			void resizeFluidParticles(const unsigned int newSize);
			void releaseFluidParticles();


		public:
			void cleanupModel();
			virtual void reset();

			ParticleData &getParticles();

			void initModel(const unsigned int nFluidParticles, Vector3r* fluidParticles, const unsigned int nBoundaryParticles, Vector3r* boundaryParticles);

			const unsigned int numBoundaryParticles() const { return (unsigned int)m_boundaryX.size(); }
			Real getDensity0() const { return m_density0; }
			Real getSupportRadius() const { return m_supportRadius; }
			Real getParticleRadius() const { return m_particleRadius; }
			void setParticleRadius(Real val) { m_particleRadius = val; m_supportRadius = static_cast<Real>(4.0)*m_particleRadius; }
			NeighborhoodSearchSpatialHashing* getNeighborhoodSearch() { return m_neighborhoodSearch; }

			Real getViscosity() const { return viscosity; }
			void setViscosity(Real val) { viscosity = val; }

			FORCE_INLINE const Vector3r& getBoundaryX(const unsigned int i) const
			{
				return m_boundaryX[i];
			}

			FORCE_INLINE Vector3r& getBoundaryX(const unsigned int i)
			{
				return m_boundaryX[i];
			}

			FORCE_INLINE void setBoundaryX(const unsigned int i, const Vector3r &val)
			{
				m_boundaryX[i] = val;
			}

			FORCE_INLINE const Real& getBoundaryPsi(const unsigned int i) const
			{
				return m_boundaryPsi[i];
			}

			FORCE_INLINE Real& getBoundaryPsi(const unsigned int i)
			{
				return m_boundaryPsi[i];
			}

			FORCE_INLINE void setBoundaryPsi(const unsigned int i, const Real &val)
			{
				m_boundaryPsi[i] = val;
			}

			FORCE_INLINE const Real& getLambda(const unsigned int i) const
			{
				return m_lambda[i];
			}

			FORCE_INLINE Real& getLambda(const unsigned int i)
			{
				return m_lambda[i];
			}

			FORCE_INLINE void setLambda(const unsigned int i, const Real &val)
			{
				m_lambda[i] = val;
			}

			FORCE_INLINE const Real& getDensity(const unsigned int i) const
			{
				return m_density[i];
			}

			FORCE_INLINE Real& getDensity(const unsigned int i)
			{
				return m_density[i];
			}

			FORCE_INLINE void setDensity(const unsigned int i, const Real &val)
			{
				m_density[i] = val;
			}

			FORCE_INLINE Vector3r &getDeltaX(const unsigned int i)
			{
				return m_deltaX[i];
			}

			FORCE_INLINE const Vector3r &getDeltaX(const unsigned int i) const
			{
				return m_deltaX[i];
			}

			FORCE_INLINE void setDeltaX(const unsigned int i, const Vector3r &val)
			{
				m_deltaX[i] = val;
			}

	};
}

#endif