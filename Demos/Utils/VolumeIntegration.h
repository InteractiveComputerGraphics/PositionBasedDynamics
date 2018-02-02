
#ifndef __VOLUME_INTEGRATION_H__
#define __VOLUME_INTEGRATION_H__

#include <vector>
#include <string>
#include <iostream>

#include "Common/Common.h"

namespace Utilities
{
	class VolumeIntegration
	{

	private:
		int A;
		int B;
		int C;

		// projection integrals 
		Real P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;
		// face integrals 
		Real Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;
		// volume integrals 
		Real T0;
		Real T1[3];
		Real T2[3];
		Real TP[3];

	public:

		VolumeIntegration(const unsigned int nVertices, const unsigned int nFaces, Vector3r * const vertices, const unsigned int* indices);

		/** Compute inertia tensor for given geometry and given density. 
		*/
		void compute_inertia_tensor(Real density);

		/** Return mass of body. */
		Real getMass() const { return m_mass; }
		/** Return volume of body. */
		Real getVolume() const { return m_volume; }
		/** Return inertia tensor of body. */
		Matrix3r const& getInertia() const { return m_theta; }
		/** Return center of mass. */
		Vector3r const& getCenterOfMass() const { return m_r; }

	private:

		void volume_integrals();
		void face_integrals(unsigned int i);

		/** Compute various integrations over projection of face.
		*/
		void projection_integrals(unsigned int i);


		std::vector<Vector3r> m_face_normals;
		std::vector<Real> m_weights;
		unsigned int m_nVertices;
		unsigned int m_nFaces;
		std::vector<Vector3r> m_vertices;
		const unsigned int* m_indices;

		Real m_mass, m_volume;
		Vector3r m_r;
		Vector3r m_x;
		Matrix3r m_theta;
	};
}

#endif 
