#include "PositionBasedDynamics.h"
#include <cfloat>
#include "SPHKernels.h"

using namespace PBD;

//////////////////////////////////////////////////////////////////////////
// MathFunctions
//////////////////////////////////////////////////////////////////////////

// ----------------------------------------------------------------------------------------------
void MathFunctions::jacobiRotate(Eigen::Matrix3f &A, Eigen::Matrix3f &R, int p, int q)
{
	// rotates A through phi in pq-plane to set A(p,q) = 0
	// rotation stored in R whose columns are eigenvectors of A
	if (A(p, q) == 0.0f)
		return;

	float d = (A(p, p) - A(q, q)) / (2.0f*A(p, q));
	float t = 1.0f / (fabs(d) + sqrt(d*d + 1.0f));
	if (d < 0.0f) t = -t;
	float c = 1.0f / sqrt(t*t + 1);
	float s = t*c;
	A(p, p) += t*A(p, q);
	A(q, q) -= t*A(p, q);
	A(p, q) = A(q, p) = 0.0f;
	// transform A
	int k;
	for (k = 0; k < 3; k++) {
		if (k != p && k != q) {
			float Akp = c*A(k, p) + s*A(k, q);
			float Akq = -s*A(k, p) + c*A(k, q);
			A(k, p) = A(p, k) = Akp;
			A(k, q) = A(q, k) = Akq;
		}
	}
	// store rotation in R
	for (k = 0; k < 3; k++) {
		float Rkp = c*R(k, p) + s*R(k, q);
		float Rkq = -s*R(k, p) + c*R(k, q);
		R(k, p) = Rkp;
		R(k, q) = Rkq;
	}
}

// ----------------------------------------------------------------------------------------------
void MathFunctions::eigenDecomposition(const Eigen::Matrix3f &A, Eigen::Matrix3f &eigenVecs, Eigen::Vector3f &eigenVals)
{
	const int numJacobiIterations = 10;
	const float epsilon = 1e-15f;

	Eigen::Matrix3f D = A;

	// only for symmetric matrices!
	eigenVecs.setIdentity();	// unit matrix
	int iter = 0;
	while (iter < numJacobiIterations) {	// 3 off diagonal elements
		// find off diagonal element with maximum modulus
		int p, q;
		float a, max;
		max = fabs(D(0, 1));
		p = 0; q = 1;
		a = fabs(D(0, 2));
		if (a > max) { p = 0; q = 2; max = a; }
		a = fabs(D(1, 2));
		if (a > max) { p = 1; q = 2; max = a; }
		// all small enough -> done
		if (max < epsilon) break;
		// rotate matrix with respect to that element
		jacobiRotate(D, eigenVecs, p, q);
		iter++;
	}
	eigenVals[0] = D(0, 0);
	eigenVals[1] = D(1, 1);
	eigenVals[2] = D(2, 2);
}

/** Perform polar decomposition A = (U D U^T) R
*/
void MathFunctions::polarDecomposition(const Eigen::Matrix3f &A, Eigen::Matrix3f &R, Eigen::Matrix3f &U, Eigen::Matrix3f &D)
{
	// A = SR, where S is symmetric and R is orthonormal
	// -> S = (A A^T)^(1/2)

	// A = U D U^T R

	Eigen::Matrix3f AAT;
	AAT(0, 0) = A(0, 0)*A(0, 0) + A(0, 1)*A(0, 1) + A(0, 2)*A(0, 2);
	AAT(1, 1) = A(1, 0)*A(1, 0) + A(1, 1)*A(1, 1) + A(1, 2)*A(1, 2);
	AAT(2, 2) = A(2, 0)*A(2, 0) + A(2, 1)*A(2, 1) + A(2, 2)*A(2, 2);

	AAT(0, 1) = A(0, 0)*A(1, 0) + A(0, 1)*A(1, 1) + A(0, 2)*A(1, 2);
	AAT(0, 2) = A(0, 0)*A(2, 0) + A(0, 1)*A(2, 1) + A(0, 2)*A(2, 2);
	AAT(1, 2) = A(1, 0)*A(2, 0) + A(1, 1)*A(2, 1) + A(1, 2)*A(2, 2);

	AAT(1, 0) = AAT(0, 1);
	AAT(2, 0) = AAT(0, 2);
	AAT(2, 1) = AAT(1, 2);

	R.setIdentity();
	Eigen::Vector3f eigenVals;
	eigenDecomposition(AAT, U, eigenVals);

	float d0 = sqrt(eigenVals[0]);
	float d1 = sqrt(eigenVals[1]);
	float d2 = sqrt(eigenVals[2]);
	D.setZero();
	D(0, 0) = d0;
	D(1, 1) = d1;
	D(2, 2) = d2;

	const float eps = 1e-15f;

	float l0 = eigenVals[0]; if (l0 <= eps) l0 = 0.0f; else l0 = 1.0f / d0;
	float l1 = eigenVals[1]; if (l1 <= eps) l1 = 0.0f; else l1 = 1.0f / d1;
	float l2 = eigenVals[2]; if (l2 <= eps) l2 = 0.0f; else l2 = 1.0f / d2;

	Eigen::Matrix3f S1;
	S1(0, 0) = l0*U(0, 0)*U(0, 0) + l1*U(0, 1)*U(0, 1) + l2*U(0, 2)*U(0, 2);
	S1(1, 1) = l0*U(1, 0)*U(1, 0) + l1*U(1, 1)*U(1, 1) + l2*U(1, 2)*U(1, 2);
	S1(2, 2) = l0*U(2, 0)*U(2, 0) + l1*U(2, 1)*U(2, 1) + l2*U(2, 2)*U(2, 2);

	S1(0, 1) = l0*U(0, 0)*U(1, 0) + l1*U(0, 1)*U(1, 1) + l2*U(0, 2)*U(1, 2);
	S1(0, 2) = l0*U(0, 0)*U(2, 0) + l1*U(0, 1)*U(2, 1) + l2*U(0, 2)*U(2, 2);
	S1(1, 2) = l0*U(1, 0)*U(2, 0) + l1*U(1, 1)*U(2, 1) + l2*U(1, 2)*U(2, 2);

	S1(1, 0) = S1(0, 1);
	S1(2, 0) = S1(0, 2);
	S1(2, 1) = S1(1, 2);

	R = S1*A;

	// stabilize
	Eigen::Vector3f c0, c1, c2;
	c0 = R.col(0);
	c1 = R.col(1);
	c2 = R.col(2);

	if (c0.squaredNorm() < eps)
		c0 = c1.cross(c2);
	else if (c1.squaredNorm() < eps)
		c1 = c2.cross(c0);
	else
		c2 = c0.cross(c1);
	R.col(0) = c0;
	R.col(1) = c1;
	R.col(2) = c2;
}

/** Return the one norm of the matrix.
*/
float MathFunctions::oneNorm(const Eigen::Matrix3f &A)
{
	const float sum1 = fabs(A(0,0)) + fabs(A(1,0)) + fabs(A(2,0));
	const float sum2 = fabs(A(0,1)) + fabs(A(1,1)) + fabs(A(2,1));
	const float sum3 = fabs(A(0,2)) + fabs(A(1,2)) + fabs(A(2,2));
	float maxSum = sum1;
	if (sum2 > maxSum)
		maxSum = sum2;
	if (sum3 > maxSum)
		maxSum = sum3;
	return maxSum;
}

/** Return the inf norm of the matrix.
*/
float MathFunctions::infNorm(const Eigen::Matrix3f &A)
{
	const float sum1 = fabs(A(0, 0)) + fabs(A(0, 1)) + fabs(A(0, 2));
	const float sum2 = fabs(A(1, 0)) + fabs(A(1, 1)) + fabs(A(1, 2));
	const float sum3 = fabs(A(2, 0)) + fabs(A(2, 1)) + fabs(A(2, 2));
	float maxSum = sum1;
	if (sum2 > maxSum)
		maxSum = sum2;
	if (sum3 > maxSum)
		maxSum = sum3;
	return maxSum;
}

/** Perform a polar decomposition of matrix M and return the rotation matrix R. This method handles the degenerated cases.
*/
void MathFunctions::polarDecompositionStable(const Eigen::Matrix3f &M, const float tolerance, Eigen::Matrix3f &R)
{
	Eigen::Matrix3f Mt = M.transpose();
	float Mone = oneNorm(M);
	float Minf = infNorm(M);
	float Eone;
	Eigen::Matrix3f MadjTt, Et;
	do
	{
		MadjTt.row(0) = Mt.row(1).cross(Mt.row(2));
		MadjTt.row(1) = Mt.row(2).cross(Mt.row(0));
		MadjTt.row(2) = Mt.row(0).cross(Mt.row(1));

		float det = Mt(0,0) * MadjTt(0,0) + Mt(0,1) * MadjTt(0,1) + Mt(0,2) * MadjTt(0,2);

		if (fabs(det) < 1.0e-12)
		{
			Eigen::Vector3f len;
			unsigned int index = 0xffffffff;
			for (unsigned int i = 0; i < 3; i++)
			{
				len[i] = MadjTt.row(i).squaredNorm();
				if (len[i] > 1.0e-12)
				{
					// index of valid cross product
					// => is also the index of the vector in Mt that must be exchanged
					index = i;
					break;
				}
			}
			if (index == 0xffffffff)
			{
				R.setIdentity();
				return;
			}
			else
			{
				Mt.row(index) = Mt.row((index + 1) % 3).cross(Mt.row((index + 2) % 3));
				MadjTt.row((index + 1) % 3) = Mt.row((index + 2) % 3).cross(Mt.row(index));
				MadjTt.row((index + 2) % 3) = Mt.row(index).cross(Mt.row((index + 1) % 3));
				Eigen::Matrix3f M2 = Mt.transpose();
				Mone = oneNorm(M2);
				Minf = infNorm(M2);
				det = Mt(0,0) * MadjTt(0,0) + Mt(0,1) * MadjTt(0,1) + Mt(0,2) * MadjTt(0,2);
			}
		}

		const float MadjTone = oneNorm(MadjTt);
		const float MadjTinf = infNorm(MadjTt);

		const float gamma = sqrt(sqrt((MadjTone*MadjTinf) / (Mone*Minf)) / fabs(det));

		const float g1 = gamma*0.5f;
		const float g2 = 0.5f / (gamma*det);

		for (unsigned char i = 0; i < 3; i++)
		{
			for (unsigned char j = 0; j < 3; j++)
			{
				Et(i,j) = Mt(i,j);
				Mt(i,j) = g1*Mt(i,j) + g2*MadjTt(i,j);
				Et(i,j) -= Mt(i,j);
			}
		}

		Eone = oneNorm(Et);

		Mone = oneNorm(Mt);
		Minf = infNorm(Mt);
	} while (Eone > Mone * tolerance);

	// Q = Mt^T 
	R = Mt.transpose();
}

/** Perform a singular value decomposition of matrix A: A = U * sigma * V^T.
* This function returns two proper rotation matrices U and V^T which do not 
* contain a reflection. Reflections are corrected by the inversion handling
* proposed by Irving et al. 2004.
*/
void MathFunctions::svdWithInversionHandling(const Eigen::Matrix3f &A, Eigen::Vector3f &sigma, Eigen::Matrix3f &U, Eigen::Matrix3f &VT)
{

	Eigen::Matrix3f AT_A, V;
	AT_A = A.transpose() * A;

	Eigen::Vector3f S;

	// Eigen decomposition of A^T * A
	eigenDecomposition(AT_A, V, S);

	// Detect if V is a reflection .
	// Make a rotation out of it by multiplying one column with -1.
	const float detV = V.determinant();
	if (detV < 0.0)
	{
		float minLambda = FLT_MAX;
		unsigned char pos = 0;
		for (unsigned char l = 0; l < 3; l++)
		{
			if (S[l] < minLambda)
			{
				pos = l;
				minLambda = S[l];
			}
		}
		V(0, pos) = -V(0, pos);
		V(1, pos) = -V(1, pos);
		V(2, pos) = -V(2, pos);
	}

	if (S[0] < 0.0f) S[0] = 0.0f;		// safety for sqrt
	if (S[1] < 0.0f) S[1] = 0.0f;
	if (S[2] < 0.0f) S[2] = 0.0f;

	sigma[0] = sqrt(S[0]);
	sigma[1] = sqrt(S[1]);
	sigma[2] = sqrt(S[2]);

	VT = V.transpose();

	//
	// Check for values of hatF near zero
	//
	unsigned char chk = 0;
	unsigned char pos = 0;
	for (unsigned char l = 0; l < 3; l++)
	{
		if (fabs(sigma[l]) < 1.0e-4)
		{
			pos = l;
			chk++;
		}
	}

	if (chk > 0)
	{
		if (chk > 1)
		{
			U.setIdentity();
		}
		else
		{
			U = A * V;
			for (unsigned char l = 0; l < 3; l++)
			{
				if (l != pos)
				{
					for (unsigned char m = 0; m < 3; m++)
					{
						U(m, l) *= 1.0f / sigma[l];
					}
				}
			}

			Eigen::Vector3f v[2];
			unsigned char index = 0;
			for (unsigned char l = 0; l < 3; l++)
			{
				if (l != pos)
				{
					v[index++] = Eigen::Vector3f(U(0, l), U(1, l), U(2, l));
				}
			}
			Eigen::Vector3f vec = v[0].cross(v[1]);
			vec.normalize();
			U(0, pos) = vec[0];
			U(1, pos) = vec[1];
			U(2, pos) = vec[2];
		}
	}
	else
	{
		Eigen::Vector3f sigmaInv(1.0f / sigma[0], 1.0f / sigma[1], 1.0f / sigma[2]);
		U = A * V;
		for (unsigned char l = 0; l < 3; l++)
		{
			for (unsigned char m = 0; m < 3; m++)
			{
				U(m, l) *= sigmaInv[l];
			}
		}
	}

	const float detU = U.determinant();

	// U is a reflection => inversion
	if (detU < 0.0)
	{
		//std::cout << "Inversion!\n";
		float minLambda = FLT_MAX;
		unsigned char pos = 0;
		for (unsigned char l = 0; l < 3; l++)
		{
			if (sigma[l] < minLambda)
			{
				pos = l;
				minLambda = sigma[l];
			}
		}

		// invert values of smallest singular value
		sigma[pos] = -sigma[pos];
		U(0, pos) = -U(0, pos);
		U(1, pos) = -U(1, pos);
		U(2, pos) = -U(2, pos);
	}
}

// ----------------------------------------------------------------------------------------------
float MathFunctions::cotTheta(const Eigen::Vector3f &v, const Eigen::Vector3f &w)
{
	const float cosTheta = v.dot(w);
	const float sinTheta = (v.cross(w)).norm();
	return (cosTheta / sinTheta);
}



//////////////////////////////////////////////////////////////////////////
// PositionBasedDynamics
//////////////////////////////////////////////////////////////////////////


/** Return the position corrections for a distance constraint between 
 * two particles: C=|p0-p1|-restLength = 0.
 */
bool PositionBasedDynamics::solveDistanceConstraint(
	const Eigen::Vector3f &p0, float invMass0, 
	const Eigen::Vector3f &p1, float invMass1,
	const float restLength,
	const float compressionStiffness,
	const float stretchStiffness,
	Eigen::Vector3f &corr0, Eigen::Vector3f &corr1)
{				
	float wSum = invMass0 + invMass1;
	if (wSum == 0.0f)
		return false;

	Eigen::Vector3f n = p1 - p0;
	float d = n.norm();
	n.normalize();
	
	Eigen::Vector3f corr;
	if (d < restLength)
		corr = compressionStiffness * n * (d - restLength) / wSum;
	else
		corr = stretchStiffness * n * (d - restLength) / wSum;

	corr0 =  invMass0 * corr;
	corr1 = -invMass1 * corr;
	return true;
}


/** Return the position corrections for a dihedral bending constraint between
* two triangles (p0, p2, p3) and (p1, p2, p3) with the common edge (p2, p3).
*/
bool PositionBasedDynamics::solveDihedralConstraint(
	const Eigen::Vector3f &p0, float invMass0,		
	const Eigen::Vector3f &p1, float invMass1,
	const Eigen::Vector3f &p2, float invMass2,
	const Eigen::Vector3f &p3, float invMass3,
	const float restAngle,
	const float stiffness,		
	Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3)
{
	// derivatives from Bridson, Simulation of Clothing with Folds and Wrinkles
	// his modes correspond to the derivatives of the bending angle arccos(n1 dot n2) with correct scaling

	if (invMass0 == 0.0f && invMass1 == 0.0f)
		return false;

	Eigen::Vector3f e = p3-p2;
	float  elen = e.norm();
	if (elen < 1e-6f)
		return false;

	float invElen = 1.0f / elen;

	Eigen::Vector3f n1 = (p2-p0).cross(p3-p0); n1 /= n1.squaredNorm();
	Eigen::Vector3f n2 = (p3 - p1).cross(p2 - p1); n2 /= n2.squaredNorm();

	Eigen::Vector3f d0 = elen*n1;
	Eigen::Vector3f d1 = elen*n2;
	Eigen::Vector3f d2 = (p0-p3).dot(e) * invElen * n1 + (p1-p3).dot(e) * invElen * n2;
	Eigen::Vector3f d3 = (p2-p0).dot(e) * invElen * n1 + (p2-p1).dot(e) * invElen * n2;

	n1.normalize();
	n2.normalize();
	float dot = n1.dot(n2);

	if (dot < -1.0f) dot = -1.0f;
	if (dot >  1.0f) dot =  1.0f;
	float phi = acosf(dot);	

	// float phi = (-0.6981317f * dot * dot - 0.8726646f) * dot + 1.570796f;	// fast approximation

	float lambda = 
		invMass0 * d0.squaredNorm() +
		invMass1 * d1.squaredNorm() +
		invMass2 * d2.squaredNorm() +
		invMass3 * d3.squaredNorm();

	if (lambda == 0.0f)
		return false;	

	// stability
	// 1.5 is the largest magic number I found to be stable in all cases :-)
	//if (stiffness > 0.5f && fabs(phi - b.restAngle) > 1.5f)		
	//	stiffness = 0.5f;

	lambda = (phi - restAngle) / lambda * stiffness;

	if (n1.cross(n2).dot(e) > 0.0f)
		lambda = -lambda;	

	corr0 = - invMass0 * lambda * d0;
	corr1 = - invMass1 * lambda * d1;
	corr2 = - invMass2 * lambda * d2;
	corr3 = - invMass3 * lambda * d3;

	return true;
}

/** Return the position corrections for a tetrahedral volume constraint, 
* where the tetrahedron (p0,p1,p2,p3) has the given rest volume.
*/
bool PositionBasedDynamics::solveVolumeConstraint(
	const Eigen::Vector3f &p0, float invMass0,		
	const Eigen::Vector3f &p1, float invMass1,
	const Eigen::Vector3f &p2, float invMass2,
	const Eigen::Vector3f &p3, float invMass3,
	const float restVolume,
	const float negVolumeStiffness,			
	const float posVolumeStiffness,
	Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3)
{
	Eigen::Vector3f d1 = p1 - p0;
	Eigen::Vector3f d2 = p2 - p0;
	Eigen::Vector3f d3 = p3 - p0;
	float volume = 1.0f / 6.0f * (p1 - p0).cross(p2 - p0).dot(p3 - p0);

	corr0.setZero(); corr1.setZero(); corr2.setZero(); corr3.setZero();

	if (posVolumeStiffness == 0.0f && volume > 0.0f)
		return false;

	if (negVolumeStiffness == 0.0f && volume < 0.0f)
		return false;


	Eigen::Vector3f grad0 = (p1 - p2).cross(p3 - p2);
	Eigen::Vector3f grad1 = (p2 - p0).cross(p3 - p0);
	Eigen::Vector3f grad2 = (p0 - p1).cross(p3 - p1);
	Eigen::Vector3f grad3 = (p1 - p0).cross(p2 - p0);

	float lambda = 
		invMass0 * grad0.squaredNorm() +
		invMass1 * grad1.squaredNorm() +
		invMass2 * grad2.squaredNorm() +
		invMass3 * grad3.squaredNorm();

	if (fabs(lambda) < 1.0e-9)
		return false;

	if (volume < 0.0f)
		lambda = negVolumeStiffness * (volume - restVolume) / lambda;
	else
		lambda = posVolumeStiffness * (volume - restVolume) / lambda;

	corr0 = -lambda * invMass0 * grad0;
	corr1 = -lambda * invMass1 * grad1;
	corr2 = -lambda * invMass2 * grad2;
	corr3 = -lambda * invMass3 * grad3;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::computeQuadraticBendingMat(
	const Eigen::Vector3f &p0, 
	const Eigen::Vector3f &p1, 
	const Eigen::Vector3f &p2, 
	const Eigen::Vector3f &p3, 
	Eigen::Matrix4f &Q)
{
	// Compute matrix Q for quadratic bending
	const Eigen::Vector3f *x[4] = { &p2, &p3, &p0, &p1 };

	const Eigen::Vector3f e0 = *x[1] - *x[0];
	const Eigen::Vector3f e1 = *x[2] - *x[0];
	const Eigen::Vector3f e2 = *x[3] - *x[0];
	const Eigen::Vector3f e3 = *x[2] - *x[1];
	const Eigen::Vector3f e4 = *x[3] - *x[1];

	const float c01 = MathFunctions::cotTheta(e0, e1);
	const float c02 = MathFunctions::cotTheta(e0, e2);
	const float c03 = MathFunctions::cotTheta(-e0, e3);
	const float c04 = MathFunctions::cotTheta(-e0, e4);

	const float A0 = 0.5f * (e0.cross(e1)).norm();
	const float A1 = 0.5f * (e0.cross(e2)).norm();

	const float coef = -3.f / (2.f*(A0 + A1));
	const float K[4] = { c03 + c04, c01 + c02, -c01 - c03, -c02 - c04 };
	const float K2[4] = { coef*K[0], coef*K[1], coef*K[2], coef*K[3] };

	for (unsigned char j = 0; j < 4; j++)
	{
		for (unsigned char k = 0; k < j; k++)
		{
			Q(j, k) = Q(k, j) = K[j] * K2[k];
		}
		Q(j, j) = K[j] * K2[j];
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::solveIsometricBendingConstraint(
	const Eigen::Vector3f &p0, float invMass0, 
	const Eigen::Vector3f &p1, float invMass1, 
	const Eigen::Vector3f &p2, float invMass2, 
	const Eigen::Vector3f &p3, float invMass3, 
	const Eigen::Matrix4f &Q, 
	const float stiffness, 
	Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3)
{
	const Eigen::Vector3f *x[4] = { &p2, &p3, &p0, &p1 };
	float invMass[4] = { invMass2, invMass3, invMass0, invMass1 };

	float energy = 0.0;
	for (unsigned char k = 0; k < 4; k++)
		for (unsigned char j = 0; j < 4; j++)
			energy += Q(j, k)*(x[k]->dot(*x[j]));
	energy *= 0.5;

	Eigen::Vector3f gradC[4];
	gradC[0].setZero();
	gradC[1].setZero();
	gradC[2].setZero();
	gradC[3].setZero();
	for (unsigned char k = 0; k < 4; k++)
		for (unsigned char j = 0; j < 4; j++)
			gradC[j] += Q(j,k) * *x[k];


	float sum_normGradC = 0.0;
	for (unsigned int j = 0; j < 4; j++)
	{
		// compute sum of squared gradient norms
		if (invMass[j] != 0.0)
			sum_normGradC += invMass[j] * gradC[j].squaredNorm();
	}

	// exit early if required
	if (fabs(sum_normGradC) > 1.0e-9)
	{
		// compute impulse-based scaling factor
		const float s = energy / sum_normGradC;

		corr0 = -stiffness * (s*invMass[2]) * gradC[2];
		corr1 = -stiffness * (s*invMass[3]) * gradC[3];
		corr2 = -stiffness * (s*invMass[0]) * gradC[0];
		corr3 = -stiffness * (s*invMass[1]) * gradC[1];

		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::solveEdgePointDistConstraint(
	const Eigen::Vector3f &p, float invMass,
	const Eigen::Vector3f &p0, float invMass0,
	const Eigen::Vector3f &p1, float invMass1,
	const float restDist,
	const float compressionStiffness,
	const float stretchStiffness,
	Eigen::Vector3f &corr, Eigen::Vector3f &corr0, Eigen::Vector3f &corr1)
{
	float EPS = 1e-6f;
	Eigen::Vector3f d = p1 - p0;
	float t;
	if ((p0-p1).squaredNorm() < EPS * EPS)
		t = 0.5f;
	else {
		float d2 = d.dot(d);
		float t = d.dot(p - p1) / d2;
		if (t < 0.0f)
			t = 0.0f;
		else if (t > 1.0f)
			t = 1.0f;
	}
	Eigen::Vector3f q = p0 + d*t;	// closest point on edge
	Eigen::Vector3f n = p - q;
	float dist = n.norm();
	n.normalize();
	float C = dist - restDist;
	float b0 = 1.0f - t;
	float b1 = t;
	Eigen::Vector3f grad = n;
	Eigen::Vector3f grad0 = -n * b0;
	Eigen::Vector3f grad1 = -n * b1;

	float s = invMass + invMass0 * b0 * b0 + invMass1 * b1 * b1;
	if (s == 0.0f)
		return false;

	s = C / s;
	if (C < 0.0f)
		s *= compressionStiffness;
	else
		s *= stretchStiffness;

	if (s == 0.0f)
		return false;

	corr = -s * invMass * grad;
	corr0 = -s * invMass0 * grad0;
	corr1 = -s * invMass1 * grad1;
	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::solveTrianglePointDistConstraint(
	const Eigen::Vector3f &p, float invMass,
	const Eigen::Vector3f &p0, float invMass0,
	const Eigen::Vector3f &p1, float invMass1,
	const Eigen::Vector3f &p2, float invMass2,
	const float restDist,
	const float compressionStiffness,
	const float stretchStiffness,
	Eigen::Vector3f &corr, Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2)
{
	// find barycentric coordinates of closest point on triangle

	float b0 = 1.0f / 3.0f;		// for singular case
	float b1 = b0;
	float b2 = b0;

	Eigen::Vector3f d1 = p1 - p0;
	Eigen::Vector3f d2 = p2 - p0;
	Eigen::Vector3f pp0 = p - p0;
	float a = d1.dot(d1);
	float b = d2.dot(d1);
	float c = pp0.dot(d1);
	float d = b;
	float e = d2.dot(d2);
	float f = pp0.dot(d2);
	float det = a*e - b*d;

	if (det != 0.0f) {
		float s = (c*e - b*f) / det;
		float t = (a*f - c*d) / det;
		b0 = 1.0f - s - t;		// inside triangle
		b1 = s;
		b2 = t;
		if (b0 < 0.0f) {		// on edge 1-2
			Eigen::Vector3f d = p2 - p1;
			float d2 = d.dot(d);
			float t = (d2 == 0.0f) ? 0.5f : d.dot(p - p1) / d2;
			if (t < 0.0f) t = 0.0f;	// on point 1
			if (t > 1.0f) t = 1.0f;	// on point 2
			b0 = 0.0f;
			b1 = (1.0f - t);
			b2 = t;
		}
		else if (b1 < 0.0f) {	// on edge 2-0
			Eigen::Vector3f d = p0 - p2;
			float d2 = d.dot(d);
			float t = (d2 == 0.0f) ? 0.5f : d.dot(p - p2) / d2;
			if (t < 0.0f) t = 0.0f;	// on point 2
			if (t > 1.0f) t = 1.0f; // on point 0
			b1 = 0.0f;
			b2 = (1.0f - t);
			b0 = t;
		}
		else if (b2 < 0.0f) {	// on edge 0-1
			Eigen::Vector3f d = p1 - p0;
			float d2 = d.dot(d);
			float t = (d2 == 0.0f) ? 0.5f : d.dot(p - p0) / d2;
			if (t < 0.0f) t = 0.0f;	// on point 0
			if (t > 1.0f) t = 1.0f;	// on point 1
			b2 = 0.0f;
			b0 = (1.0f - t);
			b1 = t;
		}
	}
	Eigen::Vector3f q = p0 * b0 + p1 * b1 + p2 * b2;
	Eigen::Vector3f n = p - q;
	float dist = n.norm();
	n.normalize();
	float C = dist - restDist;
	Eigen::Vector3f grad = n;
	Eigen::Vector3f grad0 = -n * b0;
	Eigen::Vector3f grad1 = -n * b1;
	Eigen::Vector3f grad2 = -n * b2;

	float s = invMass + invMass0 * b0*b0 + invMass1 * b1*b1 + invMass2 * b2*b2;
	if (s == 0.0f)
		return false;

	s = C / s;
	if (C < 0.0f)
		s *= compressionStiffness;
	else
		s *= stretchStiffness;

	if (s == 0.0f)
		return false;

	corr = -s * invMass * grad;
	corr0 = -s * invMass0 * grad0;
	corr1 = -s * invMass1 * grad1;
	corr2 = -s * invMass2 * grad2;
	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::solveEdgeEdgeDistConstraint(
	const Eigen::Vector3f &p0, float invMass0,
	const Eigen::Vector3f &p1, float invMass1,
	const Eigen::Vector3f &p2, float invMass2,
	const Eigen::Vector3f &p3, float invMass3,
	const float restDist,
	const float compressionStiffness,
	const float stretchStiffness,
	Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3)
{
	Eigen::Vector3f d0 = p1 - p0;
	Eigen::Vector3f d1 = p3 - p2;

	float a = d0.squaredNorm();
	float b = -d0.dot(d1);
	float c = d0.dot(d1);
	float d = -d1.squaredNorm();
	float e = (p2 - p0).dot(d0);
	float f = (p2 - p0).dot(d1);
	float det = a*d - b*c;
	float s, t;
	if (det != 0.0f) {
		det = 1.0f / det;
		s = (e*d - b*f) * det;
		t = (a*f - e*c) * det;
	}
	else {	// d0 and d1 parallel
		float s0 = p0.dot(d0);
		float s1 = p1.dot(d0);
		float t0 = p2.dot(d0);
		float t1 = p3.dot(d0);
		bool flip0 = false;
		bool flip1 = false;

		if (s0 > s1) { float f = s0; s0 = s1; s1 = f; flip0 = true; }
		if (t0 > t1) { float f = t0; t0 = t1; t1 = f; flip1 = true; }

		if (s0 >= t1) {
			s = !flip0 ? 0.0f : 1.0f;
			t = !flip1 ? 1.0f : 0.0f;
		}
		else if (t0 >= s1) {
			s = !flip0 ? 1.0f : 0.0f;
			t = !flip1 ? 0.0f : 1.0f;
		}
		else {		// overlap
			float mid = (s0 > t0) ? (s0 + t1) * 0.5f : (t0 + s1) * 0.5f;
			s = (s0 == s1) ? 0.5f : (mid - s0) / (s1 - s0);
			t = (t0 == t1) ? 0.5f : (mid - t0) / (t1 - t0);
		}
	}
	if (s < 0.0f) s = 0.0f;
	if (s > 1.0f) s = 1.0f;
	if (t < 0.0f) t = 0.0f;
	if (t > 1.0f) t = 1.0f;

	float b0 = 1.0f - s;
	float b1 = s;
	float b2 = 1.0f - t;
	float b3 = t;

	Eigen::Vector3f q0 = p0 * b0 + p1 * b1;
	Eigen::Vector3f q1 = p2 * b2 + p3 * b3;
	Eigen::Vector3f n = q0 - q1;
	float dist = n.norm();
	n.normalize();
	float C = dist - restDist;
	Eigen::Vector3f grad0 = n * b0;
	Eigen::Vector3f grad1 = n * b1;
	Eigen::Vector3f grad2 = -n * b2;
	Eigen::Vector3f grad3 = -n * b3;

	s = invMass0 * b0*b0 + invMass1 * b1*b1 + invMass2 * b2*b2 + invMass3 * b3*b3;
	if (s == 0.0f)
		return false;

	s = C / s;
	if (C < 0.0f)
		s *= compressionStiffness;
	else
		s *= stretchStiffness;

	if (s == 0.0f)
		return false;

	corr0 = -s * invMass0 * grad0;
	corr1 = -s * invMass1 * grad1;
	corr2 = -s * invMass2 * grad2;
	corr3 = -s * invMass3 * grad3;
	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::computeShapeMatchingRestInfo(
	const Eigen::Vector3f x0[], const float invMasses[], int numPoints,
	Eigen::Vector3f &restCm, Eigen::Matrix3f &invRestMat)
{
	const float eps = 1e-6f;
	invRestMat.setIdentity();

	// center of mass
	restCm.setZero();
	float wsum = 0.0f;
	for (int i = 0; i < numPoints; i++) {
		float wi = 1.0f / (invMasses[i] + eps);
		restCm += x0[i] * wi;
		wsum += wi;
	}
	if (wsum == 0.0f)
		return false;
	restCm /= wsum;

	// A
	Eigen::Matrix3f A;
	A.setZero();
	for (int i = 0; i < numPoints; i++) {
		const Eigen::Vector3f qi = x0[i] - restCm;
		float wi = 1.0f / (invMasses[i] + eps);
		float x2 = wi * qi[0] * qi[0];
		float y2 = wi * qi[1] * qi[1];
		float z2 = wi * qi[2] * qi[2];
		float xy = wi * qi[0] * qi[1];
		float xz = wi * qi[0] * qi[2];
		float yz = wi * qi[1] * qi[2];
		A(0, 0) += x2; A(0, 1) += xy; A(0, 2) += xz;
		A(1, 0) += xy; A(1, 1) += y2; A(1, 2) += yz;
		A(2, 0) += xz; A(2, 1) += yz; A(2, 2) += z2;
	}
	float det = A.determinant();
	if (fabs(det) > 1.0e-9)
	{
		invRestMat = A.inverse();
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::solveShapeMatchingConstraint(
	const Eigen::Vector3f x0[], const Eigen::Vector3f x[], const float invMasses[], int numPoints,
	const Eigen::Vector3f &restCm, 
	const Eigen::Matrix3f &invRestMat,
	const float stiffness,
	const bool allowStretch,
	Eigen::Vector3f corr[], Eigen::Matrix3f *rot)
{
	const float eps = 1e-6f;

	for (int i = 0; i < numPoints; i++)
		corr[i].setZero();

	// center of mass
	Eigen::Vector3f cm(0.0f, 0.0f, 0.0f);
	float wsum = 0.0f;
	for (int i = 0; i < numPoints; i++) 
	{
		float wi = 1.0f / (invMasses[i] + eps);
		cm += x[i] * wi;
		wsum += wi;
	}
	if (wsum == 0.0f)
		return false;
	cm /= wsum;

	// A
	Eigen::Matrix3f mat;
	mat.setZero();
	for (int i = 0; i < numPoints; i++) {
		Eigen::Vector3f q = x0[i] - restCm;
		Eigen::Vector3f p = x[i] - cm;

		float w = 1.0f / (invMasses[i] + eps);
		p *= w;

		mat(0, 0) += p[0] * q[0]; mat(0, 1) += p[0] * q[1]; mat(0, 2) += p[0] * q[2];
		mat(1, 0) += p[1] * q[0]; mat(1, 1) += p[1] * q[1]; mat(1, 2) += p[1] * q[2];
		mat(2, 0) += p[2] * q[0]; mat(2, 1) += p[2] * q[1]; mat(2, 2) += p[2] * q[2];
	}

	mat = mat * invRestMat;

	Eigen::Matrix3f R, U, D;
	R = mat;
	if (allowStretch)
		R = mat;
	else
		//MathFunctions::polarDecomposition(mat, R, U, D);
		MathFunctions::polarDecompositionStable(mat, 1e-6f, R);

	for (int i = 0; i < numPoints; i++) {
		Eigen::Vector3f goal = cm + R * (x0[i] - restCm);
		corr[i] = (goal - x[i]) * stiffness;
	}

	if (rot)
		*rot = R;

	return true;
}


// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::computeStrainTriangleInvRestMat(
	const Eigen::Vector3f &p0,
	const Eigen::Vector3f &p1,
	const Eigen::Vector3f &p2,
	Eigen::Matrix2f &invRestMat)
{
	float a = p1[0] - p0[0]; float b = p2[0] - p0[0];
	float c = p1[1] - p0[1]; float d = p2[1] - p0[1];

	// inverse
	float det = a*d - b*c;
	if (fabs(det) < 1.0e-9)
		return false;

	float s = 1.0f / det;
	invRestMat(0,0) =  d*s;  invRestMat(0,1) = -b*s;
	invRestMat(1,0) = -c*s;  invRestMat(1,1) =  a*s;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::solveStrainTriangleConstraint(		
		const Eigen::Vector3f &p0, float invMass0, 
		const Eigen::Vector3f &p1, float invMass1,
		const Eigen::Vector3f &p2, float invMass2,
		const Eigen::Matrix2f &invRestMat,
		const float xxStiffness, 
		const float yyStiffness, 
		const float xyStiffness,
		const bool normalizeStretch,
		const bool normalizeShear,
		Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2)
{
	Eigen::Vector3f c[2];
	c[0] = Eigen::Vector3f(invRestMat(0, 0), invRestMat(1, 0), 0.0f);
	c[1] = Eigen::Vector3f(invRestMat(0, 1), invRestMat(1, 1), 0.0f);

	Eigen::Vector3f r[3];

	corr0.setZero();
	corr1.setZero();
	corr2.setZero();

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j <= i; j++) {

// 			r[0] = Eigen::Vector3f(p1[0] - p0[0], p2[0] - p0[0], 0.0f);  // Jacobi
// 			r[1] = Eigen::Vector3f(p1[1] - p0[1], p2[1] - p0[1], 0.0f);
// 			r[2] = Eigen::Vector3f(p1[2] - p0[2], p2[2] - p0[2], 0.0f);

			r[0] = Eigen::Vector3f((p1[0] + corr1[0]) - (p0[0] + corr0[0]), (p2[0] + corr2[0]) - (p0[0] + corr0[0]), 0.0f);		// Gauss - Seidel
			r[1] = Eigen::Vector3f((p1[1] + corr1[1]) - (p0[1] + corr0[1]), (p2[1] + corr2[1]) - (p0[1] + corr0[1]), 0.0f);
			r[2] = Eigen::Vector3f((p1[2] + corr1[2]) - (p0[2] + corr0[2]), (p2[2] + corr2[2]) - (p0[2] + corr0[2]), 0.0f);


			float Sij = 0.0;
			for (int k = 0; k < 3; k++)
				Sij += r[k].dot(c[i]) * r[k].dot(c[j]);

			Eigen::Vector3f d[3];
			d[0] = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

			for (int k = 0; k < 2; k++) {
				d[k+1]  = Eigen::Vector3f(r[0].dot(c[j]), r[1].dot(c[j]), r[2].dot(c[j])) * invRestMat(k, i);
				d[k+1] += Eigen::Vector3f(r[0].dot(c[i]), r[1].dot(c[i]), r[2].dot(c[i])) * invRestMat(k, j);
				d[0] -= d[k+1];
			}

			if (i != j && normalizeShear) {
				float fi2 = 0.0f;
				float fj2 = 0.0f;
				for (int k = 0; k < 3; k++) {
					fi2 += r[k].dot(c[i]) * r[k].dot(c[i]);
					fj2 += r[k].dot(c[j]) * r[k].dot(c[j]);
				}
				float fi = sqrtf(fi2);
				float fj = sqrtf(fj2);

				d[0] = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
				float s = Sij / (fi2*fi*fj2*fj);
				for (int k = 0; k < 2; k++) {
					d[k+1] /= fi * fj;
					d[k+1] -= fj*fj * Eigen::Vector3f(r[0].dot(c[i]), r[1].dot(c[i]), r[2].dot(c[i])) * invRestMat(k, i) * s;
					d[k+1] -= fi*fi * Eigen::Vector3f(r[0].dot(c[j]), r[1].dot(c[j]), r[2].dot(c[j])) * invRestMat(k, j) * s;
					d[0] -= d[k+1];
				}
				Sij = Sij / (fi * fj);
			}

			float lambda = 
				invMass0 * d[0].squaredNorm() +
				invMass1 * d[1].squaredNorm() +
				invMass2 * d[2].squaredNorm();

			if (lambda == 0.0f)
				continue;

			if (i == 0 && j == 0) {
				if (normalizeStretch) {
					float s = sqrt(Sij);
					lambda = 2.0f * s * (s - 1.0f) / lambda * xxStiffness;
				}
				else {
					lambda = (Sij - 1.0f) / lambda * xxStiffness;
				}
			}
			else if (i == 1 && j == 1) {
				if (normalizeStretch) {
					float s = sqrt(Sij);
					lambda = 2.0f * s * (s - 1.0f) / lambda * yyStiffness;
				}
				else {
					lambda = (Sij - 1.0f) / lambda * yyStiffness;
				}
			}
			else {
				lambda = Sij / lambda * xyStiffness;
			}

			corr0 -= lambda * invMass0 * d[0];
			corr1 -= lambda * invMass1 * d[1];
			corr2 -= lambda * invMass2 * d[2];
		}
	}
	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::computeStrainTetraInvRestMat(		
	const Eigen::Vector3f &p0,
	const Eigen::Vector3f &p1,
	const Eigen::Vector3f &p2,
	const Eigen::Vector3f &p3,
	Eigen::Matrix3f &invRestMat)
{
	Eigen::Matrix3f m;
	m.col(0) = p1 - p0;
	m.col(1) = p2 - p0;
	m.col(2) = p3 - p0;

	float det = m.determinant();
	if (fabs(det) > 1.0e-9)
	{
		invRestMat = m.inverse();
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::solveStrainTetraConstraint(
	const Eigen::Vector3f &p0, float invMass0, 
	const Eigen::Vector3f &p1, float invMass1,
	const Eigen::Vector3f &p2, float invMass2,
	const Eigen::Vector3f &p3, float invMass3,
	const Eigen::Matrix3f &invRestMat,
	const Eigen::Vector3f &stretchStiffness,	
	const Eigen::Vector3f &shearStiffness,	
	const bool normalizeStretch,
	const bool normalizeShear,
	Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3)
{
	corr0.setZero();
	corr1.setZero();
	corr2.setZero();
	corr3.setZero();

	Eigen::Vector3f c[3];
	c[0] = invRestMat.col(0);
	c[1] = invRestMat.col(1);
	c[2] = invRestMat.col(2);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j <= i; j++) {

			Eigen::Matrix3f P;
// 			P.col(0) = p1 - p0;		// Jacobi
// 			P.col(1) = p2 - p0;
// 			P.col(2) = p3 - p0;

			P.col(0) = (p1 + corr1) - (p0 + corr0);		// Gauss - Seidel
			P.col(1) = (p2 + corr2) - (p0 + corr0);
			P.col(2) = (p3 + corr3) - (p0 + corr0);

			Eigen::Vector3f fi = P * c[i];
			Eigen::Vector3f fj = P * c[j];

			float Sij = fi.dot(fj);

			float wi,wj,s1,s3;
			if (normalizeShear && i != j) {
				wi = fi.norm();
				wj = fj.norm();
				s1 = 1.0f / (wi*wj);
				s3 = s1 * s1 * s1;
			}

			Eigen::Vector3f d[4];
			d[0] = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

			for (int k = 0; k < 3; k++) {
				d[k+1] = fj * invRestMat(k,i) + fi * invRestMat(k,j);

				if (normalizeShear && i != j) {
					d[k+1] = s1 * d[k+1] - Sij*s3 * (wj*wj * fi*invRestMat(k,i) + wi*wi * fj*invRestMat(k,j));
				}

				d[0] -= d[k+1];
			}

			if (normalizeShear && i != j)
				Sij *= s1;

			float lambda = 
				invMass0 * d[0].squaredNorm() +
				invMass1 * d[1].squaredNorm() +
				invMass2 * d[2].squaredNorm() +
				invMass3 * d[3].squaredNorm();

			if (fabs(lambda) < 1e-6f)		// foo: threshold should be scale dependent
				continue;

			if (i == j) {	// diagonal, stretch
				if (normalizeStretch)  {
					float s = sqrt(Sij);
					lambda = 2.0f * s * (s - 1.0f) / lambda * stretchStiffness[i];
				}
				else {
					lambda = (Sij - 1.0f) / lambda * stretchStiffness[i];
				}
			}
			else {		// off diagonal, shear
				lambda = Sij / lambda * shearStiffness[i + j - 1];
			}

			corr0 -= lambda * invMass0 * d[0];
			corr1 -= lambda * invMass1 * d[1];
			corr2 -= lambda * invMass2 * d[2];
			corr3 -= lambda * invMass3 * d[3];
		}
	}
	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::computeFEMTriangleInvRestMat(	
	const Eigen::Vector3f &p0,
	const Eigen::Vector3f &p1,
	const Eigen::Vector3f &p2,
	float &area, 
	Eigen::Matrix2f &invRestMat)
{
	Eigen::Vector3f normal0 = (p1 - p0).cross(p2 - p0);
	area = normal0.norm() * 0.5f;

	Eigen::Vector3f axis0_1 = p1 - p0;
	axis0_1.normalize();
	Eigen::Vector3f axis0_2 = normal0.cross(axis0_1);
	axis0_2.normalize();

	Eigen::Vector2f p[3];
	p[0] = Eigen::Vector2f(p0.dot(axis0_2), p0.dot(axis0_1));
	p[1] = Eigen::Vector2f(p1.dot(axis0_2), p1.dot(axis0_1));
	p[2] = Eigen::Vector2f(p2.dot(axis0_2), p2.dot(axis0_1));

	Eigen::Matrix2f P;
	P(0, 0) = p[0][0] - p[2][0];
	P(1, 0) = p[0][1] - p[2][1];
	P(0, 1) = p[1][0] - p[2][0];
	P(1, 1) = p[1][1] - p[2][1];

	const float det = P.determinant();
	if (fabs(det) > 1.0e-9)
	{
		invRestMat = P.inverse();
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::solveFEMTriangleConstraint(		
	const Eigen::Vector3f &p0, float invMass0, 
	const Eigen::Vector3f &p1, float invMass1,
	const Eigen::Vector3f &p2, float invMass2,
	const float &area,
	const Eigen::Matrix2f &invRestMat,
	const float youngsModulusX,
	const float youngsModulusY,
	const float youngsModulusShear,
	const float poissonRatioXY,
	const float poissonRatioYX,
	Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2)
{
	// Orthotropic elasticity tensor
	Eigen::Matrix3f C;
	C.setZero();
	C(0, 0) = youngsModulusX / (1.0f - poissonRatioXY*poissonRatioYX);
	C(0, 1) = youngsModulusX*poissonRatioYX / (1.0f - poissonRatioXY*poissonRatioYX);
	C(1, 1) = youngsModulusY / (1.0f - poissonRatioXY*poissonRatioYX);
	C(1, 0) = youngsModulusY*poissonRatioXY / (1.0f - poissonRatioXY*poissonRatioYX);
	C(2, 2) = youngsModulusShear;

	// Determine \partial x/\partial m_i
	Eigen::Matrix<float, 3, 2> F;
	const Eigen::Vector3f p13 = p0 - p2;
	const Eigen::Vector3f p23 = p1 - p2;
	F(0,0) = p13[0] * invRestMat(0,0) + p23[0] * invRestMat(1,0);
	F(0,1) = p13[0] * invRestMat(0,1) + p23[0] * invRestMat(1,1);
	F(1,0) = p13[1] * invRestMat(0,0) + p23[1] * invRestMat(1,0);
	F(1,1) = p13[1] * invRestMat(0,1) + p23[1] * invRestMat(1,1);
	F(2,0) = p13[2] * invRestMat(0,0) + p23[2] * invRestMat(1,0);
	F(2,1) = p13[2] * invRestMat(0,1) + p23[2] * invRestMat(1,1);

	// epsilon = 0.5(F^T * F - I)
	Eigen::Matrix2f epsilon;
	epsilon(0,0) = 0.5f*(F(0,0) * F(0,0) + F(1,0) * F(1,0) + F(2,0) * F(2,0) - 1.0f);		// xx
	epsilon(1,1) = 0.5f*(F(0,1) * F(0,1) + F(1,1) * F(1,1) + F(2,1) * F(2,1) - 1.0f);		// yy
	epsilon(0,1) = 0.5f*(F(0,0) * F(0,1) + F(1,0) * F(1,1) + F(2,0) * F(2,1));			// xy
	epsilon(1,0) = epsilon(0,1);

	// P(F) = det(F) * C*E * F^-T => E = green strain
	Eigen::Matrix2f stress;
	stress(0,0) = C(0,0) * epsilon(0,0) + C(0,1) * epsilon(1,1) + C(0,2) * epsilon(0,1);
	stress(1,1) = C(1,0) * epsilon(0,0) + C(1,1) * epsilon(1,1) + C(1,2) * epsilon(0,1);
	stress(0,1) = C(2,0) * epsilon(0,0) + C(2,1) * epsilon(1,1) + C(2,2) * epsilon(0,1);
	stress(1,0) = stress(0,1);

	const Eigen::Matrix<float, 3, 2> piolaKirchhoffStres = F * stress;

	float psi = 0.0f;
	for (unsigned char j = 0; j < 2; j++)
		for (unsigned char k = 0; k < 2; k++)
			psi += epsilon(j,k) * stress(j,k);
	psi = 0.5f*psi;
	float energy = area*psi;

	// compute gradient
	Eigen::Matrix<float, 3, 2> H = area * piolaKirchhoffStres * invRestMat.transpose();
	Eigen::Vector3f gradC[3];
	for (unsigned char j = 0; j < 3; ++j)
	{
		gradC[0][j] = H(j,0);
		gradC[1][j] = H(j,1);
	}
	gradC[2] = -gradC[0] - gradC[1];


	float sum_normGradC = invMass0 * gradC[0].squaredNorm();
	sum_normGradC += invMass1 * gradC[1].squaredNorm();
	sum_normGradC += invMass2 * gradC[2].squaredNorm();

	// exit early if required
	if (fabs(sum_normGradC) > 1.0e-9)
	{
		// compute scaling factor
		const float s = energy / sum_normGradC;

		// update positions
		corr0 = -(s*invMass0) * gradC[0];
		corr1 = -(s*invMass1) * gradC[1];
		corr2 = -(s*invMass2) * gradC[2];

		return true;
	}

	return false;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::computeFEMTetraInvRestMat(			// compute only when rest shape changes
		const Eigen::Vector3f &p0,
		const Eigen::Vector3f &p1,
		const Eigen::Vector3f &p2,
		const Eigen::Vector3f &p3,
		float &volume,
		Eigen::Matrix3f &invRestMat)
{
	volume = fabs((1.0f / 6.0f) * (p3 - p0).dot((p2 - p0).cross(p1 - p0)));

	Eigen::Matrix3f m;
	m.col(0) = p0 - p3;
	m.col(1) = p1 - p3;
	m.col(2) = p2 - p3;

	float det = m.determinant();
	if (fabs(det) > 1.0e-9)
	{
		invRestMat = m.inverse();
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------------------------
void PositionBasedDynamics::computeGreenStrainAndPiolaStress(
	const Eigen::Vector3f &x1, const Eigen::Vector3f &x2, const Eigen::Vector3f &x3, const Eigen::Vector3f &x4,
	const Eigen::Matrix3f &invRestMat,
	const float restVolume,
	const float mu, const float lambda, Eigen::Matrix3f &epsilon, Eigen::Matrix3f &sigma, float &energy)
{
	// Determine \partial x/\partial m_i
	Eigen::Matrix3f F;
	const Eigen::Vector3f p14 = x1 - x4;
	const Eigen::Vector3f p24 = x2 - x4;
	const Eigen::Vector3f p34 = x3 - x4;
	F(0, 0) = p14[0]*invRestMat(0, 0) + p24[0]*invRestMat(1, 0) + p34[0]*invRestMat(2, 0);
	F(0, 1) = p14[0]*invRestMat(0, 1) + p24[0]*invRestMat(1, 1) + p34[0]*invRestMat(2, 1);
	F(0, 2) = p14[0]*invRestMat(0, 2) + p24[0]*invRestMat(1, 2) + p34[0]*invRestMat(2, 2);

	F(1, 0) = p14[1]*invRestMat(0, 0) + p24[1]*invRestMat(1, 0) + p34[1]*invRestMat(2, 0);
	F(1, 1) = p14[1]*invRestMat(0, 1) + p24[1]*invRestMat(1, 1) + p34[1]*invRestMat(2, 1);
	F(1, 2) = p14[1]*invRestMat(0, 2) + p24[1]*invRestMat(1, 2) + p34[1]*invRestMat(2, 2);

	F(2, 0) = p14[2]*invRestMat(0, 0) + p24[2]*invRestMat(1, 0) + p34[2]*invRestMat(2, 0);
	F(2, 1) = p14[2]*invRestMat(0, 1) + p24[2]*invRestMat(1, 1) + p34[2]*invRestMat(2, 1);
	F(2, 2) = p14[2]*invRestMat(0, 2) + p24[2]*invRestMat(1, 2) + p34[2]*invRestMat(2, 2);

	// epsilon = 1/2 F^T F - I

	epsilon(0, 0) = 0.5f*(F(0, 0) * F(0, 0) + F(1, 0) * F(1, 0) + F(2, 0) * F(2, 0) - 1.0f);		// xx
	epsilon(1, 1) = 0.5f*(F(0, 1) * F(0, 1) + F(1, 1) * F(1, 1) + F(2, 1) * F(2, 1) - 1.0f);		// yy
	epsilon(2, 2) = 0.5f*(F(0, 2) * F(0, 2) + F(1, 2) * F(1, 2) + F(2, 2) * F(2, 2) - 1.0f);		// zz
	epsilon(0, 1) = 0.5f*(F(0, 0) * F(0, 1) + F(1, 0) * F(1, 1) + F(2, 0) * F(2, 1));			// xy
	epsilon(0, 2) = 0.5f*(F(0, 0) * F(0, 2) + F(1, 0) * F(1, 2) + F(2, 0) * F(2, 2));			// xz
	epsilon(1, 2) = 0.5f*(F(0, 1) * F(0, 2) + F(1, 1) * F(1, 2) + F(2, 1) * F(2, 2));			// yz
	epsilon(1, 0) = epsilon(0, 1);
	epsilon(2, 0) = epsilon(0, 2);
	epsilon(2, 1) = epsilon(1, 2);

	// P(F) = F(2 mu E + lambda tr(E)I) => E = green strain
	const float trace = epsilon(0, 0) + epsilon(1, 1) + epsilon(2, 2);
	const float ltrace = lambda*trace;
	sigma = epsilon * 2.0f*mu;
	sigma(0, 0) += ltrace;
	sigma(1, 1) += ltrace;
	sigma(2, 2) += ltrace;
	sigma = F * sigma;

	float psi = 0.0;
	for (unsigned char j = 0; j < 3; j++)
		for (unsigned char k = 0; k < 3; k++)
			psi += epsilon(j, k) * epsilon(j, k);
	psi = mu*psi + 0.5f*lambda * trace*trace;
	energy = restVolume * psi;
}

// ----------------------------------------------------------------------------------------------
void PositionBasedDynamics::computeGradCGreen(float restVolume, const Eigen::Matrix3f &invRestMat, const Eigen::Matrix3f &sigma, Eigen::Vector3f *J)
{
	Eigen::Matrix3f H;
	Eigen::Matrix3f T;
	T = invRestMat.transpose();
	H = sigma * T * restVolume;

	J[0][0] = H(0, 0);
	J[1][0] = H(0, 1);
	J[2][0] = H(0, 2);

	J[0][1] = H(1, 0);
	J[1][1] = H(1, 1);
	J[2][1] = H(1, 2);

	J[0][2] = H(2, 0);
	J[1][2] = H(2, 1);
	J[2][2] = H(2, 2);

	J[3] = -J[0] - J[1] - J[2];
}

// ----------------------------------------------------------------------------------------------
void PositionBasedDynamics::computeGreenStrainAndPiolaStressInversion(
	const Eigen::Vector3f &x1, const Eigen::Vector3f &x2, const Eigen::Vector3f &x3, const Eigen::Vector3f &x4,
	const Eigen::Matrix3f &invRestMat,
	const float restVolume,
	const float mu, const float lambda, Eigen::Matrix3f &epsilon, Eigen::Matrix3f &sigma, float &energy)
{
	// Determine \partial x/\partial m_i
	Eigen::Matrix3f F;
	const Eigen::Vector3f p14 = x1 - x4;
	const Eigen::Vector3f p24 = x2 - x4;
	const Eigen::Vector3f p34 = x3 - x4;
	F(0, 0) = p14[0]*invRestMat(0, 0) + p24[0]*invRestMat(1, 0) + p34[0]*invRestMat(2, 0);
	F(0, 1) = p14[0]*invRestMat(0, 1) + p24[0]*invRestMat(1, 1) + p34[0]*invRestMat(2, 1);
	F(0, 2) = p14[0]*invRestMat(0, 2) + p24[0]*invRestMat(1, 2) + p34[0]*invRestMat(2, 2);

	F(1, 0) = p14[1]*invRestMat(0, 0) + p24[1]*invRestMat(1, 0) + p34[1]*invRestMat(2, 0);
	F(1, 1) = p14[1]*invRestMat(0, 1) + p24[1]*invRestMat(1, 1) + p34[1]*invRestMat(2, 1);
	F(1, 2) = p14[1]*invRestMat(0, 2) + p24[1]*invRestMat(1, 2) + p34[1]*invRestMat(2, 2);

	F(2, 0) = p14[2]*invRestMat(0, 0) + p24[2]*invRestMat(1, 0) + p34[2]*invRestMat(2, 0);
	F(2, 1) = p14[2]*invRestMat(0, 1) + p24[2]*invRestMat(1, 1) + p34[2]*invRestMat(2, 1);
	F(2, 2) = p14[2]*invRestMat(0, 2) + p24[2]*invRestMat(1, 2) + p34[2]*invRestMat(2, 2);

	Eigen::Matrix3f U, VT;
	Eigen::Vector3f hatF;
	MathFunctions::svdWithInversionHandling(F, hatF, U, VT);

	// Clamp small singular values
	const float minXVal = 0.577f;

	for (unsigned char j = 0; j < 3; j++)
	{
		if (hatF[j] < minXVal)
			hatF[j] = minXVal;
	}

	// epsilon for hatF
	Eigen::Vector3f epsilonHatF(0.5f*(hatF[0]*hatF[0] - 1.0f), 0.5f*(hatF[1]*hatF[1] - 1.0f), 0.5f*(hatF[2]*hatF[2] - 1.0f));

	const float trace = epsilonHatF[0] + epsilonHatF[1] + epsilonHatF[2];
	const float ltrace = lambda*trace;
	Eigen::Vector3f sigmaVec = epsilonHatF * 2.0f*mu;
	sigmaVec[0] += ltrace;
	sigmaVec[1] += ltrace;
	sigmaVec[2] += ltrace;
	sigmaVec[0] = hatF[0] * sigmaVec[0];
	sigmaVec[1] = hatF[1] * sigmaVec[1];
	sigmaVec[2] = hatF[2] * sigmaVec[2];

	Eigen::Matrix3f sigmaDiag, epsDiag;

	sigmaDiag.row(0) = Eigen::Vector3f(sigmaVec[0], 0.0f, 0.0f);
	sigmaDiag.row(1) = Eigen::Vector3f(0.0f, sigmaVec[1], 0.0f);
	sigmaDiag.row(2) = Eigen::Vector3f(0.0f, 0.0f, sigmaVec[2]);

	epsDiag.row(0) = Eigen::Vector3f(epsilonHatF[0], 0.0f, 0.0f);
	epsDiag.row(1) = Eigen::Vector3f(0.0f, epsilonHatF[1], 0.0f);
	epsDiag.row(2) = Eigen::Vector3f(0.0f, 0.0f, epsilonHatF[2]);

	epsilon = U*epsDiag*VT;
	sigma = U*sigmaDiag*VT;

	float psi = 0.0f;
	for (unsigned char j = 0; j < 3; j++)
		for (unsigned char k = 0; k < 3; k++)
			psi += epsilon(j, k) * epsilon(j, k);
	psi = mu*psi + 0.5f*lambda * trace*trace;
	energy = restVolume*psi;
}



// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::solveFEMTetraConstraint(
	const Eigen::Vector3f &p0, float invMass0, 
	const Eigen::Vector3f &p1, float invMass1,
	const Eigen::Vector3f &p2, float invMass2,
	const Eigen::Vector3f &p3, float invMass3,
	const float restVolume,
	const Eigen::Matrix3f &invRestMat,
	const float youngsModulus,
	const float poissonRatio,
	const bool  handleInversion,
	Eigen::Vector3f &corr0, Eigen::Vector3f &corr1, Eigen::Vector3f &corr2, Eigen::Vector3f &corr3)
{
	corr0.setZero();
	corr1.setZero();
	corr2.setZero();
	corr3.setZero();

	if (youngsModulus <= 0.0f)
		return true;

	if (poissonRatio < 0.0f || poissonRatio > 0.49f)
		return false;

	float C = 0.0;
	Eigen::Vector3f gradC[4];
	Eigen::Matrix3f epsilon, sigma;
	float volume = (p1 - p0).cross(p2 - p0).dot(p3 - p0) / 6.0f;

	float mu = youngsModulus / 2.0f / (1.0f + poissonRatio);
	float lambda = youngsModulus * poissonRatio / (1.0f + poissonRatio) / (1.0f - 2.0f * poissonRatio);

	if (!handleInversion || volume > 0.0f)
	{
		computeGreenStrainAndPiolaStress(p0, p1, p2, p3, invRestMat, restVolume, mu, lambda, epsilon, sigma, C);
		computeGradCGreen(restVolume, invRestMat, sigma, gradC);
	}
	else
	{
		computeGreenStrainAndPiolaStressInversion(p0, p1, p2, p3, invRestMat, restVolume, mu, lambda, epsilon, sigma, C);
		computeGradCGreen(restVolume, invRestMat, sigma, gradC);
	}

	float sum_normGradC =
		invMass0 * gradC[0].squaredNorm() +
		invMass1 * gradC[1].squaredNorm() +
		invMass2 * gradC[2].squaredNorm() +
		invMass3 * gradC[3].squaredNorm();

	if (sum_normGradC < 1.0e-9f)
		return false;

	// compute scaling factor
	const float s = C / sum_normGradC;

	corr0 = -s * invMass0 * gradC[0];
	corr1 = -s * invMass1 * gradC[1];
	corr2 = -s * invMass2 * gradC[2];
	corr3 = -s * invMass3 * gradC[3];

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::computePBFDensity(
	const unsigned int particleIndex,
	const unsigned int numberOfParticles,
	const Eigen::Vector3f x[],
	const float mass[],
	const Eigen::Vector3f boundaryX[],
	const float boundaryPsi[],
	const unsigned int numNeighbors,
	const unsigned int neighbors[],
	const float density0,
	const bool boundaryHandling,
	float &density_err,
	float &density)
{
	// Compute current density for particle i
	density = mass[particleIndex] * CubicKernel::W_zero();
	for (unsigned int j = 0; j < numNeighbors; j++)
	{
		const unsigned int neighborIndex = neighbors[j];
		if (neighborIndex < numberOfParticles)		// Test if fluid particle
		{
			density += mass[neighborIndex] * CubicKernel::W(x[particleIndex] - x[neighborIndex]);
		}
		else if (boundaryHandling)
		{
			// Boundary: Akinci2012
			density += boundaryPsi[neighborIndex - numberOfParticles] * CubicKernel::W(x[particleIndex] - boundaryX[neighborIndex - numberOfParticles]);
		}
	}

	density_err = std::max(density, density0) - density0;
	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::computePBFLagrangeMultiplier(
	const unsigned int particleIndex,
	const unsigned int numberOfParticles,
	const Eigen::Vector3f x[],	
	const float mass[],
	const Eigen::Vector3f boundaryX[],
	const float boundaryPsi[],
	const float density,
	const unsigned int numNeighbors,
	const unsigned int neighbors[],
	const float density0,
	const bool boundaryHandling,
	float &lambda)
{
	const float eps = 1.0e-6f;

	// Evaluate constraint function
	const float C = std::max(density / density0 - 1.0f, 0.0f);			// clamp to prevent particle clumping at surface

	if (C != 0.0f)
	{
		// Compute gradients dC/dx_j 
		float sum_grad_C2 = 0.0;
		Eigen::Vector3f gradC_i(0.0f, 0.0f, 0.0f);

		for (unsigned int j = 0; j < numNeighbors; j++)
		{
			const unsigned int neighborIndex = neighbors[j];
			if (neighborIndex < numberOfParticles)		// Test if fluid particle
			{
				const Eigen::Vector3f gradC_j = -mass[neighborIndex] / density0 * CubicKernel::gradW(x[particleIndex] - x[neighborIndex]);
				sum_grad_C2 += gradC_j.squaredNorm();
				gradC_i -= gradC_j;
			}
			else if (boundaryHandling)
			{
				// Boundary: Akinci2012
				const Eigen::Vector3f gradC_j = -boundaryPsi[neighborIndex - numberOfParticles] / density0 * CubicKernel::gradW(x[particleIndex] - boundaryX[neighborIndex - numberOfParticles]);
				sum_grad_C2 += gradC_j.squaredNorm();
				gradC_i -= gradC_j;
			}
		}

		sum_grad_C2 += gradC_i.squaredNorm();

		// Compute lambda
		lambda = -C / (sum_grad_C2 + eps);
	}
	else
		lambda = 0.0f;

	return true;
}

// ----------------------------------------------------------------------------------------------
bool PositionBasedDynamics::solveDensityConstraint(
	const unsigned int particleIndex,
	const unsigned int numberOfParticles,
	const Eigen::Vector3f x[],	
	const float mass[],
	const Eigen::Vector3f boundaryX[],
	const float boundaryPsi[],
	const unsigned int numNeighbors,
	const unsigned int neighbors[],
	const float density0,
	const bool boundaryHandling,
	const float lambda[],
	Eigen::Vector3f &corr)
{
	// Compute position correction
	corr.setZero();
	for (unsigned int j = 0; j < numNeighbors; j++)
	{
		const unsigned int neighborIndex = neighbors[j];
		if (neighborIndex < numberOfParticles)		// Test if fluid particle
		{
			const Eigen::Vector3f gradC_j = -mass[neighborIndex] / density0 * CubicKernel::gradW(x[particleIndex] - x[neighborIndex]);
			corr -= (lambda[particleIndex] + lambda[neighborIndex]) * gradC_j;
		}
		else if (boundaryHandling)
		{
			// Boundary: Akinci2012
			const Eigen::Vector3f gradC_j = -boundaryPsi[neighborIndex - numberOfParticles] / density0 * CubicKernel::gradW(x[particleIndex] - boundaryX[neighborIndex - numberOfParticles]);
			corr -= (lambda[particleIndex]) * gradC_j;
		}
	}

	return true;
}

