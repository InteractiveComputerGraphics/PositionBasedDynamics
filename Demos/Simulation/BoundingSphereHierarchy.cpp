
#include "BoundingSphereHierarchy.h"
#include "MiniBall.h"

#include <iostream>
#include <unordered_set>
#include <set>

using namespace Eigen;
using pool_set = std::set<unsigned int>;
using namespace PBD;


PointCloudBSH::PointCloudBSH()
    : super(0)
{
}

Vector3r const& PointCloudBSH::entity_position(unsigned int i) const
{
    return m_vertices[i];
}

struct PCBSHCoordAccessor
{
    PCBSHCoordAccessor(const Vector3r *vertices_,
        std::vector<unsigned int> const* lst_)
        : vertices(vertices_), lst(lst_)
    {
    }

    using Pit = unsigned int;
    using Cit = Real const*;
    inline  Cit operator() (Pit it) const {
        return vertices[(*lst)[it]].data();
    }
    const Vector3r *vertices;
    std::vector<unsigned int> const* lst;
};


void PointCloudBSH::compute_hull(unsigned int b, unsigned int n, BoundingSphere& hull) const
{
    auto mb = Miniball::Miniball<PCBSHCoordAccessor>(3, b, b + n,  {m_vertices, &m_lst});

    hull.x() = Map<Vector3r const>(mb.center());
    hull.r() = std::sqrt(mb.squared_radius());
}

void PointCloudBSH::compute_hull_approx(unsigned int b, unsigned int n, BoundingSphere& hull) const
{
	// compute center
	Vector3r x;
	x.setZero();
	for (unsigned int i = b; i < b+n; i++)
		x += m_vertices[m_lst[i]];
	x /= (Real)n;

	Real radius2 = 0.0;
	for (unsigned int i = b; i < b+n; i++)
	{
		radius2 = std::max(radius2, (x - m_vertices[m_lst[i]]).squaredNorm());
	}

	hull.x() = x;
	hull.r() = sqrt(radius2);
}

void PointCloudBSH::init(const Vector3r *vertices, const unsigned int numVertices)
{
	m_lst.resize(numVertices);
	m_vertices = vertices;
	m_numVertices = numVertices;
}

