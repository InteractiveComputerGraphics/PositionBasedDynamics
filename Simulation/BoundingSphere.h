#ifndef __BOUNDINGSPHERE_H__
#define __BOUNDINGSPHERE_H__

#include "Common/Common.h"

namespace PBD
{

	class BoundingSphere
	{
	public:

		BoundingSphere() = default;
		BoundingSphere(Vector3r const& x, Real r) : m_x(x), m_r(r) {}

		Vector3r const& x() const { return m_x; }
		Vector3r& x() { return m_x; }

		Real r() const { return m_r; }
		Real& r() { return m_r; }

		bool overlaps(BoundingSphere const& other) const
		{
			Real rr = m_r + other.m_r;
			return (m_x - other.m_x).squaredNorm() < rr * rr;
		}

		bool contains(BoundingSphere const& other) const
		{
			Real rr = r() - other.r();
			return (x() - other.x()).squaredNorm() < rr * rr;
		}

		bool contains(Vector3r const& other) const
		{
			return (x() - other).squaredNorm() < m_r * m_r;
		}

	private:

		Vector3r m_x;
		Real m_r;
	};

}

#endif
