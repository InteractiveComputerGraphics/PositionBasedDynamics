#ifndef __PARTICLEDATA_H__
#define __PARTICLEDATA_H__

#include "Demos/Utils/Config.h"
#include <vector>
#include <Eigen/Dense>


namespace PBD
{
	/** This class encapsulates the state of all vertices.
	* All parameters are stored in individual arrays.
	*/
	class VertexData
	{
	private:
		std::vector<Eigen::Vector3f> m_x;

	public:
		FORCE_INLINE VertexData(void) :
			m_x()
		{
		}

		FORCE_INLINE ~VertexData(void)
		{
			m_x.clear();
		}

		FORCE_INLINE void addVertex(const Eigen::Vector3f &vertex)
		{
			m_x.push_back(vertex);
		}

		FORCE_INLINE Eigen::Vector3f &getPosition(const unsigned int i)
		{
			return m_x[i];
		}

		FORCE_INLINE const Eigen::Vector3f &getPosition(const unsigned int i) const
		{
			return m_x[i];
		}

		FORCE_INLINE void setPosition(const unsigned int i, const Eigen::Vector3f &pos)
		{
			m_x[i] = pos;
		}

		/** Resize the array containing the particle data.
		*/
		FORCE_INLINE void resize(const unsigned int newSize)
		{
			m_x.resize(newSize);
		}

		/** Reserve the array containing the particle data.
		*/
		FORCE_INLINE void reserve(const unsigned int newSize)
		{
			m_x.reserve(newSize);
		}

		/** Release the array containing the particle data.
		*/
		FORCE_INLINE void release()
		{
			m_x.clear();
		}

		/** Release the array containing the particle data.
		*/
		FORCE_INLINE unsigned int size() const
		{
			return (unsigned int)m_x.size();
		}
	};

	/** This class encapsulates the state of all particles of a particle model.
	 * All parameters are stored in individual arrays.
	 */
	class ParticleData
	{
		private:
			// Mass
			// If the mass is zero, the particle is static
			std::vector<float> m_masses;
			std::vector<float> m_invMasses;

			// Dynamic state
			std::vector<Eigen::Vector3f> m_x0;
			std::vector<Eigen::Vector3f> m_x;
			std::vector<Eigen::Vector3f> m_v;
			std::vector<Eigen::Vector3f> m_a;
			std::vector<Eigen::Vector3f> m_lastX;

		public:
			FORCE_INLINE ParticleData(void)	:
				  m_masses(),
				  m_invMasses(),
				  m_x0(),
				  m_x(),
				  m_v(),
				  m_a(),
				  m_lastX()
			{
			}

			FORCE_INLINE ~ParticleData(void) 
			{
				m_masses.clear();
				m_invMasses.clear();
				m_x0.clear();
				m_x.clear();
				m_v.clear();
				m_a.clear();
				m_lastX.clear();
			}

			FORCE_INLINE void addVertex(const Eigen::Vector3f &vertex)
			{
				m_x0.push_back(vertex);
				m_x.push_back(vertex);
				m_lastX.push_back(vertex);
				m_masses.push_back(1.0);
				m_invMasses.push_back(1.0);
				m_v.push_back(Eigen::Vector3f(0.0, 0.0, 0.0));
				m_a.push_back(Eigen::Vector3f(0.0, 0.0, 0.0));
			}

			FORCE_INLINE Eigen::Vector3f &getPosition(const unsigned int i)
			{
				return m_x[i];
			}

			FORCE_INLINE const Eigen::Vector3f &getPosition(const unsigned int i) const 
			{
				return m_x[i];
			}

			FORCE_INLINE void setPosition(const unsigned int i, const Eigen::Vector3f &pos)
			{
				m_x[i] = pos;
			}

			FORCE_INLINE Eigen::Vector3f &getPosition0(const unsigned int i)
			{
				return m_x0[i];
			}

			FORCE_INLINE const Eigen::Vector3f &getPosition0(const unsigned int i) const
			{
				return m_x0[i];
			}

			FORCE_INLINE void setPosition0(const unsigned int i, const Eigen::Vector3f &pos)
			{
				m_x0[i] = pos;
			}

			FORCE_INLINE Eigen::Vector3f &getLastPosition(const unsigned int i)
			{
				return m_lastX[i];
			}

			FORCE_INLINE const Eigen::Vector3f &getLastPosition(const unsigned int i) const
			{
				return m_lastX[i];
			}

			FORCE_INLINE void setLastPosition(const unsigned int i, const Eigen::Vector3f &pos)
			{
				m_lastX[i] = pos;
			}
			
			FORCE_INLINE Eigen::Vector3f &getVelocity(const unsigned int i)
			{
				return m_v[i];
			}

			FORCE_INLINE const Eigen::Vector3f &getVelocity(const unsigned int i) const 
			{
				return m_v[i];
			}

			FORCE_INLINE void setVelocity(const unsigned int i, const Eigen::Vector3f &vel)
			{
				m_v[i] = vel;
			}

			FORCE_INLINE Eigen::Vector3f &getAcceleration(const unsigned int i)
			{
				return m_a[i];
			}

			FORCE_INLINE const Eigen::Vector3f &getAcceleration(const unsigned int i) const 
			{
				return m_a[i];
			}

			FORCE_INLINE void setAcceleration(const unsigned int i, const Eigen::Vector3f &accel)
			{
				m_a[i] = accel;
			}

			FORCE_INLINE const float getMass(const unsigned int i) const
			{
				return m_masses[i];
			}

			FORCE_INLINE float& getMass(const unsigned int i)
			{
				return m_masses[i];
			}

			FORCE_INLINE void setMass(const unsigned int i, const float mass)
			{
				m_masses[i] = mass;
				if (mass != 0.0f)
					m_invMasses[i] = 1.0f / mass;
				else
					m_invMasses[i] = 0.0f;
			}

			FORCE_INLINE const float getInvMass(const unsigned int i) const
			{
				return m_invMasses[i];
			}

			FORCE_INLINE const unsigned int getNumberOfParticles() const
			{
				return (unsigned int) m_x.size();
			}

			/** Resize the array containing the particle data.
			 */
			FORCE_INLINE void resize(const unsigned int newSize)
			{
				m_masses.resize(newSize);
				m_invMasses.resize(newSize);
				m_x0.resize(newSize);
				m_x.resize(newSize);
				m_v.resize(newSize);
				m_a.resize(newSize);
				m_lastX.resize(newSize);
			}

			/** Reserve the array containing the particle data.
			 */
			FORCE_INLINE void reserve(const unsigned int newSize)
			{
				m_masses.reserve(newSize);
				m_invMasses.reserve(newSize);
				m_x0.reserve(newSize);
				m_x.reserve(newSize);
				m_v.reserve(newSize);
				m_a.reserve(newSize);
				m_lastX.reserve(newSize);
			}

			/** Release the array containing the particle data.
			 */
			FORCE_INLINE void release()
			{
				m_masses.clear();
				m_invMasses.clear();
				m_x0.clear();
				m_x.clear();
				m_v.clear();
				m_a.clear();
				m_lastX.clear();
			}

			/** Release the array containing the particle data.
			 */
			FORCE_INLINE unsigned int size() const 
			{
				return (unsigned int) m_x.size();
			}
	};
}

#endif
