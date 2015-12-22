#ifndef __FluidModel_h__
#define __FluidModel_h__

#include "Demos/Utils/Config.h"
#include "Demos/Simulation/ParticleData.h"
#include <vector>
#include "Demos/Simulation/NeighborhoodSearchSpatialHashing.h"

namespace PBD 
{	
	class FluidModel 
	{
		public:
			FluidModel();
			virtual ~FluidModel();

		protected:	
			float viscosity;
			float m_density0;
			float m_particleRadius;
			float m_supportRadius;
			ParticleData m_particles;
			std::vector<Eigen::Vector3f> m_boundaryX;
			std::vector<float> m_boundaryPsi;
			std::vector<float> m_density;
			std::vector<float> m_lambda;		
			std::vector<Eigen::Vector3f> m_deltaX;
			NeighborhoodSearchSpatialHashing *m_neighborhoodSearch;			

			void initMasses();

			void resizeFluidParticles(const unsigned int newSize);
			void releaseFluidParticles();


		public:
			void cleanupModel();
			virtual void reset();

			ParticleData &getParticles();

			void initModel(const unsigned int nFluidParticles, Eigen::Vector3f* fluidParticles, const unsigned int nBoundaryParticles, Eigen::Vector3f* boundaryParticles);

			const unsigned int numBoundaryParticles() const { return (unsigned int)m_boundaryX.size(); }
			float getDensity0() const { return m_density0; }
			float getSupportRadius() const { return m_supportRadius; }
			float getParticleRadius() const { return m_particleRadius; }
			void setParticleRadius(float val) { m_particleRadius = val; m_supportRadius = 4.0f*m_particleRadius; }
			NeighborhoodSearchSpatialHashing* getNeighborhoodSearch() { return m_neighborhoodSearch; }

			float getViscosity() const { return viscosity; }
			void setViscosity(float val) { viscosity = val; }

			FORCE_INLINE const Eigen::Vector3f& getBoundaryX(const unsigned int i) const
			{
				return m_boundaryX[i];
			}

			FORCE_INLINE Eigen::Vector3f& getBoundaryX(const unsigned int i)
			{
				return m_boundaryX[i];
			}

			FORCE_INLINE void setBoundaryX(const unsigned int i, const Eigen::Vector3f &val)
			{
				m_boundaryX[i] = val;
			}

			FORCE_INLINE const float& getBoundaryPsi(const unsigned int i) const
			{
				return m_boundaryPsi[i];
			}

			FORCE_INLINE float& getBoundaryPsi(const unsigned int i)
			{
				return m_boundaryPsi[i];
			}

			FORCE_INLINE void setBoundaryPsi(const unsigned int i, const float &val)
			{
				m_boundaryPsi[i] = val;
			}

			FORCE_INLINE const float& getLambda(const unsigned int i) const
			{
				return m_lambda[i];
			}

			FORCE_INLINE float& getLambda(const unsigned int i)
			{
				return m_lambda[i];
			}

			FORCE_INLINE void setLambda(const unsigned int i, const float &val)
			{
				m_lambda[i] = val;
			}

			FORCE_INLINE const float& getDensity(const unsigned int i) const
			{
				return m_density[i];
			}

			FORCE_INLINE float& getDensity(const unsigned int i)
			{
				return m_density[i];
			}

			FORCE_INLINE void setDensity(const unsigned int i, const float &val)
			{
				m_density[i] = val;
			}

			FORCE_INLINE Eigen::Vector3f &getDeltaX(const unsigned int i)
			{
				return m_deltaX[i];
			}

			FORCE_INLINE const Eigen::Vector3f &getDeltaX(const unsigned int i) const
			{
				return m_deltaX[i];
			}

			FORCE_INLINE void setDeltaX(const unsigned int i, const Eigen::Vector3f &val)
			{
				m_deltaX[i] = val;
			}

	};
}

#endif