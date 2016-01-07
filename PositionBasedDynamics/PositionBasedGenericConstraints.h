#ifndef POSITIONBASEDGENERICCONSTRAINTS_H
#define POSITIONBASEDGENERICCONSTRAINTS_H

#include <Eigen/Dense>

// ------------------------------------------------------------------------------------
namespace PBD
{
	/** Generic constraints provide the possibility to experiment with new constraint
	* functions very easily. Only a callback for the constraint function and optionally
	* for the gradient of the constraint must be provided.\n\n
	*
	* Examples and more details can be found here: \ref pageGenericConstraints \n
	*
	*/
	class PositionBasedGenericConstraints
	{
	public:
		/** Determine the position corrections for a constraint function
		 * with a known gradient function.\n\n
		 * More information can be found here: \ref secGenericConstraintsCGFct
		 *
		 * @param  invMass inverse mass of constrained particles
		 * @param  x positions of constrained particles
		 * @param  userData	user data which is required in the callback functions
		 * @param  constraintFct constraint callback
		 * @param  gradientFct gradient callback
		 * @param  corr_x position corrections of constrained particles
		 */		
		template<unsigned int numberOfParticles, unsigned int dim>
		static bool solve_GenericConstraint(
			const float invMass[numberOfParticles],							// inverse mass is zero if particle is static
			const Eigen::Vector3f x[numberOfParticles],						// positions of particles
			void *userData,

			void(*constraintFct)(
				const unsigned int numParticles,
				const float invMass[],							// inverse mass is zero if particle is static
				const Eigen::Vector3f x[],						// positions of particles
				void *userData,
				Eigen::Matrix<float, dim, 1> &constraintValue),

			void(*gradientFct)(
				const unsigned int i,							// compute dC / dxi
				const unsigned int numParticles,
				const float invMass[],							// inverse mass is zero if particle is static
				const Eigen::Vector3f x[],						// positions of particles
				void *userData,
				Eigen::Matrix<float, dim, 3> &jacobian),

			Eigen::Vector3f corr_x[numberOfParticles]);		


		/** Determine the position corrections for a constraint function.
		* This method approximates the gradient using finite differences.
		* Note that it is faster to define the gradient function explicitly.\n\n
		* More information can be found here: \ref secGenericConstraintsCFct
		*
		* @param  invMass inverse mass of constrained particles
		* @param  x positions of constrained particles
		* @param  userData	user data which is required in the callback functions
		* @param  constraintFct constraint callback
		* @param  corr_x position corrections of constrained particles
		*/
		template<unsigned int numberOfParticles, unsigned int dim>
		static bool solve_GenericConstraint(
			const float invMass[numberOfParticles],							// inverse mass is zero if particle is static
			const Eigen::Vector3f x[numberOfParticles],						// positions of particles
			void *userData,

			void(*constraintFct)(
				const unsigned int numParticles,
				const float invMass[],							// inverse mass is zero if particle is static
				const Eigen::Vector3d x[],						// positions of particles
				void *userData,
				Eigen::Matrix<double, dim, 1> &constraintValue),

			Eigen::Vector3f corr_x[numberOfParticles]);

		/** Determines the Jacobian of the i-th particle with finite differences.
		*
		* @param  i index of particle for which the gradient should be computed
		* @param  invMass inverse mass of constrained particles
		* @param  x positions of constrained particles
		* @param  userData	user data which is required in the callback functions
		* @param  constraintFct constraint callback
		* @param  jacobian Jacobian of the i-th particle
		*/
		template<unsigned int numberOfParticles, unsigned int dim>
		static void approximateGradient(
			const unsigned int i,							// compute dC / dxi
			const float invMass[],							// inverse mass is zero if particle is static
			const Eigen::Vector3f x[],						// positions of particles
			void *userData,

			void(*constraintFct)(
				const unsigned int numParticles,
				const float invMass[],							// inverse mass is zero if particle is static
				const Eigen::Vector3d x[],						// positions of particles
				void *userData,
				Eigen::Matrix<double, dim, 1> &constraintValue),

			Eigen::Matrix<float, dim, 3> &jacobian);

	};



	template<unsigned int numberOfParticles, unsigned int dim>
	bool PositionBasedGenericConstraints::solve_GenericConstraint(
		const float invMass[numberOfParticles],							// inverse mass is zero if particle is static
		const Eigen::Vector3f x[numberOfParticles],						// positions of particles
		void *userData,

		void(*constraintFct)(
			const unsigned int numParticles,
			const float invMass[],							// inverse mass is zero if particle is static
			const Eigen::Vector3f x[],						// positions of particles
			void *userData,
			Eigen::Matrix<float, dim, 1> &constraintValue),

		void(*gradientFct)(
			const unsigned int i,							// compute dC / dxi
			const unsigned int numParticles,
			const float invMass[],							// inverse mass is zero if particle is static
			const Eigen::Vector3f x[],						// positions of particles
			void *userData,
			Eigen::Matrix<float, dim, 3> &jacobian),

		Eigen::Vector3f corr_x[numberOfParticles])
	{
		// evaluate constraint function
		Eigen::Matrix<float, dim, 1> C;
		constraintFct(numberOfParticles, invMass, x, userData, C);

		Eigen::Matrix<float, dim, dim> K;
		K.setZero();

		Eigen::Matrix<float, dim, 3> gradients[numberOfParticles];
		for (unsigned int i = 0u; i < numberOfParticles; i++)
		{
			if (invMass[i] != 0.0f)
			{
				// compute gradient
				gradientFct(i, numberOfParticles, invMass, x, userData, gradients[i]);
				K += invMass[i] * gradients[i] * gradients[i].transpose();
			}
		}

		// compute Kinv
		if (K.determinant() < 1.0e-6)
			return false;

		Eigen::Matrix<float, dim, dim> Kinv = K.inverse();

		Eigen::Matrix<float, dim, 1> lambda = -Kinv * C;

		for (unsigned int i = 0u; i < numberOfParticles; i++)
		{
			if (invMass[i] != 0.0f)
			{
				// compute position correction
				corr_x[i] = invMass[i] * gradients[i].transpose() * lambda;
			}
			else
				corr_x[i].setZero();
		}

		return true;
	}

	template<unsigned int numberOfParticles, unsigned int dim>
	bool PositionBasedGenericConstraints::solve_GenericConstraint(
		const float invMass[numberOfParticles],							// inverse mass is zero if particle is static
		const Eigen::Vector3f x[numberOfParticles],						// positions of particles
		void *userData,

		void(*constraintFct)(
			const unsigned int numParticles,
			const float invMass[],							// inverse mass is zero if particle is static
			const Eigen::Vector3d x[],						// positions of particles
			void *userData,
			Eigen::Matrix<double, dim, 1> &constraintValue),

		Eigen::Vector3f corr_x[numberOfParticles])
	{
		// evaluate constraint function
		Eigen::Matrix<double, dim, 1> Cd;
		Eigen::Vector3d xTemp[numberOfParticles];
		for (unsigned int j = 0; j < numberOfParticles; j++)
			xTemp[j] = x[j].template cast<double>();

		constraintFct(numberOfParticles, invMass, xTemp, userData, Cd);
		Eigen::Matrix<float, dim, 1> C = Cd.template cast<float>();

		Eigen::Matrix<float, dim, dim> K;
		K.setZero();

		Eigen::Matrix<float, dim, 3> gradients[numberOfParticles];
		for (unsigned int i = 0u; i < numberOfParticles; i++)
		{
			if (invMass[i] != 0.0f)
			{
				// compute gradient
				approximateGradient<numberOfParticles, dim>(i, invMass, x, userData, constraintFct, gradients[i]);
				K += invMass[i] * gradients[i] * gradients[i].transpose();
			}
		}

		// compute Kinv
		if (K.determinant() < 1.0e-6)
			return false;

		Eigen::Matrix<float, dim, dim> Kinv = K.inverse();

		Eigen::Matrix<float, dim, 1> lambda = -Kinv * C;

		for (unsigned int i = 0u; i < numberOfParticles; i++)
		{
			if (invMass[i] != 0.0f)
			{
				// compute position correction
				corr_x[i] = invMass[i] * gradients[i].transpose() * lambda;
			}
			else
				corr_x[i].setZero();
		}

		return true;
	}

	template<unsigned int numberOfParticles, unsigned int dim>
	void PositionBasedGenericConstraints::approximateGradient(
		const unsigned int i,							// compute dC / dxi
		const float invMass[],							// inverse mass is zero if particle is static
		const Eigen::Vector3f x[],						// positions of particles
		void *userData,

		void(*constraintFct)(
			const unsigned int numParticles,
			const float invMass[],							// inverse mass is zero if particle is static
			const Eigen::Vector3d x[],						// positions of particles
			void *userData,
			Eigen::Matrix<double, dim, 1> &constraintValue),

		Eigen::Matrix<float, dim, 3> &jacobian)
	{
		Eigen::Vector3d xTemp[numberOfParticles];
		for (unsigned int j = 0; j < numberOfParticles; j++)
			xTemp[j] = x[j].template cast<double>();

		double eps = 1.e-6;
		jacobian.setZero();
		Eigen::Vector3d x_i = xTemp[i];
		for (unsigned int j = 0; j < 3; j++)
		{
			xTemp[i][j] += eps;

			Eigen::Matrix<double, dim, 1> e_p, e_m;
			constraintFct(numberOfParticles, invMass, xTemp, userData, e_p);
			xTemp[i][j] = x_i[j] - eps;
			constraintFct(numberOfParticles, invMass, xTemp, userData, e_m);
			xTemp[i][j] = x_i[j];

			Eigen::Matrix<double, dim, 1> res = (e_p - e_m) * (1.0 / (2.0*eps));

			jacobian.template block<dim, 1>(0, j) = res.template cast<float>();
		}
	}

}

#endif