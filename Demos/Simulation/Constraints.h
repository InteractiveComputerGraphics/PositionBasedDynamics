#ifndef _CONSTRAINTS_H
#define _CONSTRAINTS_H

#include "Common/Common.h"
#define _USE_MATH_DEFINES
#include "math.h"

namespace PBD
{
	class SimulationModel;

	class Constraint
	{
	public: 
		unsigned int m_numberOfBodies;
		/** indices of the linked bodies */
		unsigned int *m_bodies;

		Constraint(const unsigned int numberOfBodies) 
		{
			m_numberOfBodies = numberOfBodies; 
			m_bodies = new unsigned int[numberOfBodies]; 
		}

		virtual ~Constraint() { delete[] m_bodies; };
		virtual int &getTypeId() const = 0;

		virtual bool updateConstraint(SimulationModel &model) { return true; };
		virtual bool solvePositionConstraint(SimulationModel &model) { return true; };
		virtual bool solveVelocityConstraint(SimulationModel &model) { return true; };

	public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
		PDB_MAKE_ALIGNED_OPERATOR_NEW
	};

	class BallJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 4> m_jointInfo;

		BallJoint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class BallOnLineJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 10> m_jointInfo;

		BallOnLineJoint() : Constraint(2) {} 
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &dir);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};
 
	class HingeJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 12> m_jointInfo;

		HingeJoint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};
 
	class UniversalJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 8> m_jointInfo;

		UniversalJoint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis1, const Vector3r &axis2);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class SliderJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 14> m_jointInfo;

		SliderJoint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class TargetPositionMotorSliderJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 14> m_jointInfo;
		Real m_targetPosition;

		TargetPositionMotorSliderJoint() : Constraint(2) { m_targetPosition = 0.0; }
		virtual int &getTypeId() const { return TYPE_ID; }

		Real getTargetPosition() const { return m_targetPosition; }
		void setTargetPosition(const Real val) { m_targetPosition = val; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class TargetVelocityMotorSliderJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 14> m_jointInfo;
		Real m_targetVelocity;

		TargetVelocityMotorSliderJoint() : Constraint(2) { m_targetVelocity = 0.0; }
		virtual int &getTypeId() const { return TYPE_ID; }

		Real getTargetVelocity() const { return m_targetVelocity; }
		void setTargetVelocity(const Real val) { m_targetVelocity = val; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
		virtual bool solveVelocityConstraint(SimulationModel &model);
	};

	class TargetAngleMotorHingeJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 14> m_jointInfo;
		Real m_targetAngle;
		TargetAngleMotorHingeJoint() : Constraint(2) { m_targetAngle = 0.0; }
		virtual int &getTypeId() const { return TYPE_ID; }

		Real getTargetAngle() const { return m_targetAngle; }
		void setTargetAngle(const Real val) 
		{ 
			const Real pi = (Real)M_PI;
			m_targetAngle = std::max(val, -pi);
			m_targetAngle = std::min(m_targetAngle, pi);
		}

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class TargetVelocityMotorHingeJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 14> m_jointInfo;
		Real m_targetAngularVelocity;
		TargetVelocityMotorHingeJoint() : Constraint(2) { m_targetAngularVelocity = 0.0; }
		virtual int &getTypeId() const { return TYPE_ID; }

		Real getTargetAngularVelocity() const { return m_targetAngularVelocity; }
		void setTargetAngularVelocity(const Real val)	{ m_targetAngularVelocity = val; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Vector3r &pos, const Vector3r &axis);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
		virtual bool solveVelocityConstraint(SimulationModel &model);
	};
 
	class RigidBodyParticleBallJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<Real, 3, 2> m_jointInfo;

		RigidBodyParticleBallJoint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex, const unsigned int particleIndex);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};
 
	class DistanceConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Real m_restLength;

		DistanceConstraint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class DihedralConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Real m_restAngle;

		DihedralConstraint() : Constraint(4) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
									const unsigned int particle3, const unsigned int particle4);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};
	
	class IsometricBendingConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Matrix4r m_Q;

		IsometricBendingConstraint() : Constraint(4) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
									const unsigned int particle3, const unsigned int particle4);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class FEMTriangleConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Real m_area;
		Matrix2r m_invRestMat;

		FEMTriangleConstraint() : Constraint(3) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
			const unsigned int particle3);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class StrainTriangleConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Matrix2r m_invRestMat;

		StrainTriangleConstraint() : Constraint(3) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
			const unsigned int particle3);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class VolumeConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Real m_restVolume;

		VolumeConstraint() : Constraint(4) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
								const unsigned int particle3, const unsigned int particle4);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class FEMTetConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Real m_volume;
		Matrix3r m_invRestMat;

		FEMTetConstraint() : Constraint(4) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
									const unsigned int particle3, const unsigned int particle4);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class StrainTetConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Matrix3r m_invRestMat;

		StrainTetConstraint() : Constraint(4) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2,
			const unsigned int particle3, const unsigned int particle4);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class ShapeMatchingConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Vector3r m_restCm;
		Matrix3r m_invRestMat;
		Real *m_w;
		Vector3r *m_x0;
		Vector3r *m_x;
		Vector3r *m_corr;
		unsigned int *m_numClusters;

		ShapeMatchingConstraint(const unsigned int numberOfParticles) : Constraint(numberOfParticles)
		{
			m_x = new Vector3r[numberOfParticles];
			m_x0 = new Vector3r[numberOfParticles];
			m_corr = new Vector3r[numberOfParticles];
			m_w = new Real[numberOfParticles];
			m_numClusters = new unsigned int[numberOfParticles];
		}
		virtual ~ShapeMatchingConstraint() 
		{ 
			delete[] m_x; 
			delete[] m_x0;
			delete[] m_corr;
			delete[] m_w;
			delete[] m_numClusters;
		}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particleIndices[], const unsigned int numClusters[]);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class RigidBodyContactConstraint 
	{
	public:
		static int TYPE_ID;
		/** indices of the linked bodies */
		unsigned int m_bodies[2];
		Real m_stiffness; 
		Real m_frictionCoeff;
		Real m_currentCorrection;
		Real m_sum_impulses;
		Eigen::Matrix<Real, 3, 5> m_constraintInfo;

		RigidBodyContactConstraint() { m_currentCorrection = 0.0; }
		~RigidBodyContactConstraint() {}
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, 
			const Vector3r &cp1, const Vector3r &cp2, 
			const Vector3r &normal, const Real dist, 
			const Real restitutionCoeff, const Real stiffness, const Real frictionCoeff);
		virtual bool solveVelocityConstraint(SimulationModel &model);
	};

	class ParticleRigidBodyContactConstraint
	{
	public:
		static int TYPE_ID;
		/** indices of the linked bodies */
		unsigned int m_bodies[2];
		Real m_stiffness;
		Real m_frictionCoeff;
		Real m_currentCorrection;
		Real m_sum_impulses;
		Eigen::Matrix<Real, 3, 5> m_constraintInfo;

		ParticleRigidBodyContactConstraint() { m_currentCorrection = 0.0; }
		~ParticleRigidBodyContactConstraint() {}
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int particleIndex, const unsigned int rbIndex,
			const Vector3r &cp1, const Vector3r &cp2,
			const Vector3r &normal, const Real dist,
			const Real restitutionCoeff, const Real stiffness, const Real frictionCoeff);
		virtual bool solveVelocityConstraint(SimulationModel &model);
	};

	class StretchShearConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Real m_restLength;

		StretchShearConstraint() : Constraint(3) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2, const unsigned int quaternion1);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class BendTwistConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		Quaternionr m_restDarbouxVector;

		BendTwistConstraint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int quaternion1, const unsigned int quaternion2);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};
}

#endif
