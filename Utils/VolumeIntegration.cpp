
#include "VolumeIntegration.h"
#include <stdio.h> 

using namespace std;
using namespace Eigen;
using namespace Utilities;

#define SQR(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))

VolumeIntegration::VolumeIntegration(const unsigned int nVertices, const unsigned int nFaces, Vector3r * const vertices, const unsigned int* indices)
		: m_nVertices(nVertices), m_nFaces(nFaces), m_indices(indices), m_face_normals(nFaces), m_weights(nFaces)
{
	// compute center of mass
	m_x.setZero();
	for (unsigned int i(0); i < m_nVertices; ++i)
		m_x += vertices[i];
	m_x /= (Real)m_nVertices;

	m_vertices.resize(nVertices);
	for (unsigned int i(0); i < m_nVertices; ++i)
		m_vertices[i] = vertices[i] - m_x;

	for (unsigned int i(0); i < m_nFaces; ++i)
    {
		const Vector3r &a = m_vertices[m_indices[3 * i]];
		const Vector3r &b = m_vertices[m_indices[3 * i + 1]];
		const Vector3r &c = m_vertices[m_indices[3 * i + 2]];

		const Vector3r d1 = b - a;
		const Vector3r d2 = c - a;

		m_face_normals[i] = d1.cross(d2);
        if (m_face_normals[i].isZero(1.e-10)) 
			m_face_normals[i].setZero();
        else 
			m_face_normals[i].normalize();

		m_weights[i] = -m_face_normals[i].dot(a);
    }
}

void VolumeIntegration::compute_inertia_tensor(Real density)
{
    volume_integrals();
    m_volume = static_cast<Real>(T0);

    m_mass = static_cast<Real>(density * T0);

    /* compute center of mass */
    m_r[0] = static_cast<Real>(T1[0] / T0);
    m_r[1] = static_cast<Real>(T1[1] / T0);
    m_r[2] = static_cast<Real>(T1[2] / T0);

    /* compute inertia tensor */
    m_theta(0, 0) = static_cast<Real>(density * (T2[1] + T2[2]));
    m_theta(1, 1) = static_cast<Real>(density * (T2[2] + T2[0]));
    m_theta(2, 2) = static_cast<Real>(density * (T2[0] + T2[1]));
    m_theta(0, 1) = m_theta(1, 0) = -density * static_cast<Real>(TP[0]);
    m_theta(1, 2) = m_theta(2, 1) = -density * static_cast<Real>(TP[1]);
    m_theta(2, 0) = m_theta(0, 2) = -density * static_cast<Real>(TP[2]);

    /* translate inertia tensor to center of mass */
    m_theta(0, 0) -= m_mass * (m_r[1]*m_r[1] + m_r[2]*m_r[2]);
    m_theta(1, 1) -= m_mass * (m_r[2]*m_r[2] + m_r[0]*m_r[0]);
    m_theta(2, 2) -= m_mass * (m_r[0]*m_r[0] + m_r[1]*m_r[1]);
    m_theta(0, 1) = m_theta(1, 0) += m_mass * m_r[0] * m_r[1];
    m_theta(1, 2) = m_theta(2, 1) += m_mass * m_r[1] * m_r[2];
    m_theta(2, 0) = m_theta(0, 2) += m_mass * m_r[2] * m_r[0];

	m_r += m_x;
}


void VolumeIntegration::projection_integrals(unsigned int f)
{
    Real a0, a1, da;
    Real b0, b1, db;
    Real a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
    Real a1_2, a1_3, b1_2, b1_3;
    Real C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
    Real Cab, Kab, Caab, Kaab, Cabb, Kabb;

    P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

    for (int i = 0; i < 3; i++)
    {
		a0 = m_vertices[m_indices[3 * f + i]][A];
		b0 = m_vertices[m_indices[3 * f + i]][B];
		a1 = m_vertices[m_indices[3 * f + ((i + 1) % 3)]][A];
		b1 = m_vertices[m_indices[3 * f + ((i + 1) % 3)]][B];

        da = a1 - a0;
        db = b1 - b0;
        a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
        b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
        a1_2 = a1 * a1; a1_3 = a1_2 * a1;
        b1_2 = b1 * b1; b1_3 = b1_2 * b1;

        C1 = a1 + a0;
        Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
        Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
        Cab = 3 * a1_2 + 2 * a1*a0 + a0_2; Kab = a1_2 + 2 * a1*a0 + 3 * a0_2;
        Caab = a0*Cab + 4 * a1_3; Kaab = a1*Kab + 4 * a0_3;
        Cabb = 4 * b1_3 + 3 * b1_2*b0 + 2 * b1*b0_2 + b0_3;
        Kabb = b1_3 + 2 * b1_2*b0 + 3 * b1*b0_2 + 4 * b0_3;

        P1 += db*C1;
        Pa += db*Ca;
        Paa += db*Caa;
        Paaa += db*Caaa;
        Pb += da*Cb;
        Pbb += da*Cbb;
        Pbbb += da*Cbbb;
        Pab += db*(b1*Cab + b0*Kab);
        Paab += db*(b1*Caab + b0*Kaab);
        Pabb += da*(a1*Cabb + a0*Kabb);
    }

    P1 /= 2.0;
    Pa /= 6.0;
    Paa /= 12.0;
    Paaa /= 20.0;
    Pb /= -6.0;
    Pbb /= -12.0;
    Pbbb /= -20.0;
    Pab /= 24.0;
    Paab /= 60.0;
    Pabb /= -60.0;
}

void VolumeIntegration::face_integrals(unsigned int f)
{
  Real w;
  Vector3r n;
  Real k1, k2, k3, k4;

  projection_integrals(f);

  w = m_weights[f];
  n = m_face_normals[f];
  k1 = (n[C] == 0) ? 0 : 1 / n[C];
  k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

  Fa = k1 * Pa;
  Fb = k1 * Pb;
  Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

  Faa = k1 * Paa;
  Fbb = k1 * Pbb;
  Fcc = k3 * (SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb
     + w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

  Faaa = k1 * Paaa;
  Fbbb = k1 * Pbbb;
  Fccc = -k4 * (CUBE(n[A])*Paaa + 3*SQR(n[A])*n[B]*Paab 
       + 3*n[A]*SQR(n[B])*Pabb + CUBE(n[B])*Pbbb
       + 3*w*(SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb)
       + w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

  Faab = k1 * Paab;
  Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
  Fcca = k3 * (SQR(n[A])*Paaa + 2*n[A]*n[B]*Paab + SQR(n[B])*Pabb
     + w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));
}

void VolumeIntegration::volume_integrals()
{
    Real nx, ny, nz;

    T0  = T1[0] = T1[1] = T1[2] 
        = T2[0] = T2[1] = T2[2] 
        = TP[0] = TP[1] = TP[2] = 0;

	for (unsigned int i(0); i < m_nFaces; ++i)
    {
        Vector3r const& n = m_face_normals[i];
        nx = std::abs(n[0]);
        ny = std::abs(n[1]);
        nz = std::abs(n[2]);
        if (nx > ny && nx > nz) 
            C = 0;
        else 
            C = (ny > nz) ? 1 : 2;
        A = (C + 1) % 3;
        B = (A + 1) % 3;

        face_integrals(i);

        T0 += n[0] * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

        T1[A] += n[A] * Faa;
        T1[B] += n[B] * Fbb;
        T1[C] += n[C] * Fcc;
        T2[A] += n[A] * Faaa;
        T2[B] += n[B] * Fbbb;
        T2[C] += n[C] * Fccc;
        TP[A] += n[A] * Faab;
        TP[B] += n[B] * Fbbc;
        TP[C] += n[C] * Fcca;
    }

    T1[0] /= 2; T1[1] /= 2; T1[2] /= 2;
    T2[0] /= 3; T2[1] /= 3; T2[2] /= 3;
    TP[0] /= 2; TP[1] /= 2; TP[2] /= 2;
}
