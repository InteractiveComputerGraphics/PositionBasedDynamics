#ifndef __RIGIDBODY_H__
#define __RIGIDBODY_H__

#include "Demos/Utils/Config.h"
#include <vector>
#include <Eigen/Dense>


namespace PBD
{
	/** This class encapsulates the state of a rigid body.
	 */
	class RigidBody
	{
		private:
			/** mass */
			float m_mass;
			/** center of mass */
			Eigen::Vector3f m_x;
			Eigen::Vector3f m_lastX;
			Eigen::Vector3f m_oldX;
			Eigen::Vector3f m_x0;
			/** center of mass velocity */
			Eigen::Vector3f m_v;
			/** acceleration (by external forces) */
			Eigen::Vector3f m_a;

			/** Inertia tensor in the principal axis system: \n
			* After the main axis transformation the inertia tensor is a diagonal matrix.
			* So only three values are required to store the inertia tensor. These values
			* are constant over time.
			*/
			Eigen::Vector3f m_inertiaTensor;
			/** Inverse inertia tensor in body space */
			Eigen::Vector3f m_inertiaTensorInverse;
			/** 3x3 matrix, inverse of the inertia tensor in world space */
			Eigen::Matrix3f m_inertiaTensorInverseW;
			/** Quaternion that describes the rotation of the body in world space */
			Eigen::Quaternionf m_q;
			Eigen::Quaternionf m_lastQ;
			Eigen::Quaternionf m_oldQ;
			Eigen::Quaternionf m_q0;
			/** rotationMatrix = 3x3 matrix. 
			* Important for the transformation from world in body space and vice versa.
			* When using quaternions the rotation matrix is computed out of the quaternion.
			*/
			Eigen::Matrix3f m_rot;
			/** Angular velocity, defines rotation axis and velocity (magnitude of the vector) */
			Eigen::Vector3f m_omega;
			/** external torque */
			Eigen::Vector3f m_torque;
			
		public:
			RigidBody(void) 
			{
			}

			~RigidBody(void)
			{
			}

			void initBody(const float mass, const Eigen::Vector3f &x, const Eigen::Vector3f &inertiaTensor, const Eigen::Quaternionf &rotation)
			{
				m_mass = mass;
				m_x = x; 
				m_x0 = x;
				m_lastX = x;
				m_oldX = x;
				m_v.setZero();
				m_a.setZero();

				setInertiaTensor(inertiaTensor);
				m_q = rotation;
				m_q0 = rotation;
				m_lastQ = rotation;
				m_oldQ = rotation;
				m_rot = m_q.matrix();
				rotationUpdated();
				m_omega.setZero();
				m_torque.setZero();
			}

			void rotationUpdated()
			{
				if (m_mass != 0.0)
				{
					m_rot = m_q.matrix();
					updateInverseInertiaW();
				}
			}

			void updateInverseInertiaW()
			{
				if (m_mass != 0.0)
				{
					m_inertiaTensorInverseW = m_rot * m_inertiaTensorInverse.asDiagonal() * m_rot.transpose();
				}
			}

			FORCE_INLINE float &getMass()
			{
				return m_mass;
			}

			FORCE_INLINE const float &getMass() const
			{
				return m_mass;
			}

			FORCE_INLINE void setMass(const float &value)
			{
				m_mass = value;
			}

			FORCE_INLINE Eigen::Vector3f &getPosition()
			{
				return m_x;
			}

			FORCE_INLINE const Eigen::Vector3f &getPosition() const 
			{
				return m_x;
			}

			FORCE_INLINE void setPosition(const Eigen::Vector3f &pos)
			{
				m_x = pos;
			}

			FORCE_INLINE Eigen::Vector3f &getLastPosition()
			{
				return m_lastX;
			}

			FORCE_INLINE const Eigen::Vector3f &getLastPosition() const
			{
				return m_lastX;
			}

			FORCE_INLINE void setLastPosition(const Eigen::Vector3f &pos)
			{
				m_lastX = pos;
			}

			FORCE_INLINE Eigen::Vector3f &getOldPosition()
			{
				return m_oldX;
			}

			FORCE_INLINE const Eigen::Vector3f &getOldPosition() const
			{
				return m_oldX;
			}

			FORCE_INLINE void setOldPosition(const Eigen::Vector3f &pos)
			{
				m_oldX = pos;
			}

			FORCE_INLINE Eigen::Vector3f &getPosition0()
			{
				return m_x0;
			}

			FORCE_INLINE const Eigen::Vector3f &getPosition0() const
			{
				return m_x0;
			}

			FORCE_INLINE void setPosition0(const Eigen::Vector3f &pos)
			{
				m_x0 = pos;
			}

			FORCE_INLINE Eigen::Vector3f &getVelocity()
			{
				return m_v;
			}

			FORCE_INLINE const Eigen::Vector3f &getVelocity() const
			{
				return m_v;
			}

			FORCE_INLINE void setVelocity(const Eigen::Vector3f &value)
			{
				m_v = value;
			}			

			FORCE_INLINE Eigen::Vector3f &getAcceleration()
			{
				return m_a;
			}

			FORCE_INLINE const Eigen::Vector3f &getAcceleration() const 
			{
				return m_a;
			}

			FORCE_INLINE void setAcceleration(const Eigen::Vector3f &accel)
			{
				m_a = accel;
			}

			FORCE_INLINE const Eigen::Vector3f &getInertiaTensor() const
			{
				return m_inertiaTensor;
			}

			FORCE_INLINE void setInertiaTensor(const Eigen::Vector3f &value)
			{
				m_inertiaTensor = value;
				m_inertiaTensorInverse = Eigen::Vector3f(1.0f / value[0], 1.0f / value[1], 1.0f / value[2]);
			}

			FORCE_INLINE const Eigen::Vector3f &getInertiaTensorInverse() const
			{
				return m_inertiaTensorInverse;
			}

			FORCE_INLINE Eigen::Matrix3f &getInertiaTensorInverseW()
			{
				return m_inertiaTensorInverseW;
			}

			FORCE_INLINE const Eigen::Matrix3f &getInertiaTensorInverseW() const
			{
				return m_inertiaTensorInverseW;
			}

			FORCE_INLINE void setInertiaTensorInverseW(const Eigen::Matrix3f &value)
			{
				m_inertiaTensorInverseW = value;
			}

			FORCE_INLINE Eigen::Quaternionf &getRotation()
			{
				return m_q;
			}

			FORCE_INLINE const Eigen::Quaternionf &getRotation() const
			{
				return m_q;
			}

			FORCE_INLINE void setRotation(const Eigen::Quaternionf &value)
			{
				m_q = value;
			}

			FORCE_INLINE Eigen::Quaternionf &getLastRotation()
			{
				return m_lastQ;
			}

			FORCE_INLINE const Eigen::Quaternionf &getLastRotation() const
			{
				return m_lastQ;
			}

			FORCE_INLINE void setLastRotation(const Eigen::Quaternionf &value)
			{
				m_lastQ = value;
			}

			FORCE_INLINE Eigen::Quaternionf &getOldRotation()
			{
				return m_oldQ;
			}

			FORCE_INLINE const Eigen::Quaternionf &getOldRotation() const
			{
				return m_oldQ;
			}

			FORCE_INLINE void setOldRotation(const Eigen::Quaternionf &value)
			{
				m_oldQ = value;
			}

			FORCE_INLINE Eigen::Quaternionf &getRotation0()
			{
				return m_q0;
			}

			FORCE_INLINE const Eigen::Quaternionf &getRotation0() const
			{
				return m_q0;
			}

			FORCE_INLINE void setRotation0(const Eigen::Quaternionf &value)
			{
				m_q0 = value;
			}

			FORCE_INLINE Eigen::Matrix3f &getRotationMatrix()
			{
				return m_rot;
			}

			FORCE_INLINE const Eigen::Matrix3f &getRotationMatrix() const
			{
				return m_rot;
			}

			FORCE_INLINE void setRotationMatrix(const Eigen::Matrix3f &value)
			{
				m_rot = value;
			}

			FORCE_INLINE Eigen::Vector3f &getAngularVelocity()
			{
				return m_omega;
			}

			FORCE_INLINE const Eigen::Vector3f &getAngularVelocity() const
			{
				return m_omega;
			}

			FORCE_INLINE void setAngularVelocity(const Eigen::Vector3f &value)
			{
				m_omega = value;
			}

			FORCE_INLINE Eigen::Vector3f &getTorque()
			{
				return m_torque;
			}

			FORCE_INLINE const Eigen::Vector3f &getTorque() const
			{
				return m_torque;
			}

			FORCE_INLINE void setTorque(const Eigen::Vector3f &value)
			{
				m_torque = value;
			}
	};
}

#endif
