#ifndef SPHKERNELS_H
#define SPHKERNELS_H

#define _USE_MATH_DEFINES
#include <math.h>
#include "Demos/Utils/Config.h"
#include <Eigen/Dense>
#include <algorithm>

#define NO_DISTANCE_TEST

namespace PBD
{
	class CubicKernel
	{
	protected:
		static float m_radius;
		static float m_k;
		static float m_l;
		static float m_W_zero;
	public:
		static float getRadius() { return m_radius; }
		static void setRadius(float val)
		{
			m_radius = val;
			static const float pi = static_cast<float>(M_PI);

			const float h3 = m_radius*m_radius*m_radius;
			m_k = 8.0f / (pi*h3);
			m_l = 48.0f / (pi*h3);
			m_W_zero = W(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
		}

	public:
		//static unsigned int counter;
		static float W(const Eigen::Vector3f &r)
		{
			//counter++;
			float res = 0.0;
			const float rl = r.norm();
			const float q = rl/m_radius;
#ifndef NO_DISTANCE_TEST
			if (q <= 1.0)
#endif
			{
				if (q <= 0.5)
				{
					const float q2 = q*q;
					const float q3 = q2*q;
					res = m_k * (6.0f*q3-6.0f*q2+1.0f);
				}
				else
				{
					res = m_k * (2.0f*pow(1.0f-q,3));
				}
			}
			return res;
		}

		static Eigen::Vector3f gradW(const Eigen::Vector3f &r)
		{
			Eigen::Vector3f res;
			const float rl = r.norm();
			const float q = rl / m_radius;
#ifndef NO_DISTANCE_TEST
			if (q <= 1.0)
#endif
			{
				if (rl > 1.0e-6)
				{
					const Eigen::Vector3f gradq = r * ((float) 1.0 / (rl*m_radius));
					if (q <= 0.5f)
					{
						res = m_l*q*((float) 3.0*q - (float) 2.0)*gradq;
					}
					else
					{
						const float factor = 1.0f - q;
						res = m_l*(-factor*factor)*gradq;
					}
				}
			}
#ifndef NO_DISTANCE_TEST
 			else
 				res.zero();
#endif

			return res;
		}

		static float W_zero()
		{
			return m_W_zero;
		}
	};
}

#endif
