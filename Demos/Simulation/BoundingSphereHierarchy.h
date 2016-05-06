#ifndef __BOUNDINGSPHEREHIERARCHY_H__
#define __BOUNDINGSPHEREHIERARCHY_H__

#include "Common/Common.h"
#include "BoundingSphere.h"
#include "kdTree.h"

namespace PBD
{
	class PointCloudBSH : public KDTree<BoundingSphere>
	{

	public:

		using super = KDTree<BoundingSphere>;

		PointCloudBSH();

		void init(const Vector3r *vertices, const unsigned int numVertices);
		Vector3r const& entity_position(unsigned int i) const final;
		void compute_hull(unsigned int b, unsigned int n, BoundingSphere& hull)
			const final;
		void compute_hull_approx(unsigned int b, unsigned int n, BoundingSphere& hull)
			const final;

	private:
		const Vector3r *m_vertices;
		unsigned int m_numVertices;
	};

}

#endif
