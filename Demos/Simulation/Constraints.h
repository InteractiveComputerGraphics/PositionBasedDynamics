#ifndef _CONSTRAINTS_H
#define _CONSTRAINTS_H

#include "Demos/Utils/Config.h"
#include <Eigen/Dense>

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
	};

	class BallJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<float, 3, 4> m_jointInfo;

		BallJoint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class BallOnLineJoint : public Constraint
 	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<float, 3, 10> m_jointInfo;

		BallOnLineJoint() : Constraint(2) {} 
 		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &dir);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
 	};
 
	class HingeJoint : public Constraint
 	{
	public:
		static int TYPE_ID;
 		Eigen::Matrix<float, 3, 12> m_jointInfo;

		HingeJoint() : Constraint(2) {}
 		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
 	};
 
	class UniversalJoint : public Constraint
 	{
	public:
		static int TYPE_ID;
 		Eigen::Matrix<float, 3, 8> m_jointInfo;

		UniversalJoint() : Constraint(2) {}
 		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis1, const Eigen::Vector3f &axis2);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
 	};

	class SliderJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<float, 3, 14> m_jointInfo;

		SliderJoint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class TargetAngleMotorHingeJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<float, 3, 14> m_jointInfo;
		float m_targetAngle;
		float m_maxAngularMomentumPerStep;
		TargetAngleMotorHingeJoint() : Constraint(2) { m_targetAngle = 0.0f; m_maxAngularMomentumPerStep = 0.0f; }
		virtual int &getTypeId() const { return TYPE_ID; }

		float getMaxAngularMomentumPerStep() const { return m_maxAngularMomentumPerStep; }
		void setMaxAngularMomentumPerStep(const float val) { m_maxAngularMomentumPerStep = val; }
		float getTargetAngle() const { return m_targetAngle; }
		void setTargetAngle(const float val) 
		{ 
			const float pi = (float)M_PI;
			m_targetAngle = std::max(val, -pi);
			m_targetAngle = std::min(m_targetAngle, pi);
		}

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class TargetVelocityMotorHingeJoint : public Constraint
	{
	public:
		static int TYPE_ID;
		Eigen::Matrix<float, 3, 14> m_jointInfo;
		float m_targetAngularVelocity;
		float m_maxAngularMomentumPerStep;
		TargetVelocityMotorHingeJoint() : Constraint(2) { m_targetAngularVelocity = 0.0f; m_maxAngularMomentumPerStep = 0.0f; }
		virtual int &getTypeId() const { return TYPE_ID; }

		float getMaxAngularMomentumPerStep() const { return m_maxAngularMomentumPerStep; }
		void setMaxAngularMomentumPerStep(const float val) { m_maxAngularMomentumPerStep = val; }
		float getTargetAngularVelocity() const { return m_targetAngularVelocity; }
		void setTargetAngularVelocity(const float val)	{ m_targetAngularVelocity = val; }

		bool initConstraint(SimulationModel &model, const unsigned int rbIndex1, const unsigned int rbIndex2, const Eigen::Vector3f &pos, const Eigen::Vector3f &axis);
		virtual bool updateConstraint(SimulationModel &model);
		virtual bool solvePositionConstraint(SimulationModel &model);
		virtual bool solveVelocityConstraint(SimulationModel &model);
	};
 
	class RigidBodyParticleBallJoint : public Constraint
 	{
	public:
		static int TYPE_ID;
 		Eigen::Matrix<float, 3, 2> m_jointInfo;

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
		float m_restLength;

		DistanceConstraint() : Constraint(2) {}
		virtual int &getTypeId() const { return TYPE_ID; }

		virtual bool initConstraint(SimulationModel &model, const unsigned int particle1, const unsigned int particle2);
		virtual bool solvePositionConstraint(SimulationModel &model);
	};

	class DihedralConstraint : public Constraint
	{
	public:
		static int TYPE_ID;
		float m_restAngle;

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
		Eigen::Matrix4f m_Q;

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
		float m_area;
		Eigen::Matrix2f m_invRestMat;

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
		Eigen::Matrix2f m_invRestMat;

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
		float m_restVolume;

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
		float m_volume;
		Eigen::Matrix3f m_invRestMat;

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
		Eigen::Matrix3f m_invRestMat;

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
		Eigen::Vector3f m_restCm;
		Eigen::Matrix3f m_invRestMat;
		float *m_w;
		Eigen::Vector3f *m_x0;
		Eigen::Vector3f *m_x;
		Eigen::Vector3f *m_corr;
		unsigned int *m_numClusters;

		ShapeMatchingConstraint(const unsigned int numberOfParticles) : Constraint(numberOfParticles)
		{
			m_x = new Eigen::Vector3f[numberOfParticles];
			m_x0 = new Eigen::Vector3f[numberOfParticles];
			m_corr = new Eigen::Vector3f[numberOfParticles];
			m_w = new float[numberOfParticles];
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
}

#endif
