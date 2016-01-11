
#ifndef __VOLUME_INTEGRATION_H__
#define __VOLUME_INTEGRATION_H__

#include <vector>
#include <string>
#include <iostream>

#include <Eigen/Dense>
#include "Demos/Utils/IndexedFaceMesh.h"
#include "Demos/Simulation/ParticleData.h"

namespace PBD
{
	class VolumeIntegration
	{

	private:
		int A;
		int B;
		int C;

		// projection integrals 
		float P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;
		// face integrals 
		float Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;
		// volume integrals 
		float T0;
		float T1[3];
		float T2[3];
		float TP[3];

	public:

		VolumeIntegration(IndexedFaceMesh const& mesh, VertexData const&vertices);

		/** Compute inertia tensor for given geometry and given density. 
		*/
		void compute_inertia_tensor(float density);

		/** Return mass of body. */
		float getMass() const { return m_mass; }
		/** Return volume of body. */
		float getVolume() const { return m_volume; }
		/** Return inertia tensor of body. */
		Eigen::Matrix3f const& getInertia() const { return m_theta; }
		/** Return center of mass. */
		Eigen::Vector3f const& getCenterOfMass() const { return m_r; }

	private:

		void volume_integrals();
		void face_integrals(unsigned int i);

		/** Compute various integrations over projection of face.
		*/
		void projection_integrals(unsigned int i);


		std::vector<Eigen::Vector3f> m_face_normals;
		std::vector<float> m_weights;
		IndexedFaceMesh const& m_mesh;
		VertexData const& m_vertices;

		float m_mass, m_volume;
		Eigen::Vector3f m_r;
		Eigen::Matrix3f m_theta;
	};
}

#endif 
