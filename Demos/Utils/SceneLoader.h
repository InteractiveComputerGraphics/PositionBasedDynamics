#ifndef __SCENELOADER_H__
#define __SCENELOADER_H__

#include <string>
#include "ObjectArray.h"
#include "extern/json/json.hpp"
#include "Common/Common.h"

namespace PBD
{
	class SceneLoader
	{
	protected:
		nlohmann::json m_json;

	public:
		struct RigidBodyData
		{
			enum CollisionObjectTypes {
				No_Collision_Object = 0, Sphere, Box, Cylinder, Torus, HollowSphere, HollowBox
			};
			unsigned int m_id;
			std::string m_modelFile;
			bool m_isDynamic;
			Real m_density;
			Vector3r m_x;
			Quaternionr m_q;
			Vector3r m_scale;	
			Vector3r m_v;
			Vector3r m_omega;
			Real m_restitutionCoeff;
			Real m_frictionCoeff;
			int m_collisionObjectType;
			bool m_testMesh;
			Vector3r m_collisionObjectScale;
			bool m_invertSDF;
			Real m_thicknessSDF;

			nlohmann::json m_json;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			PDB_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct TriangleModelData
		{
			unsigned int m_id;
			std::string m_modelFile;
			Vector3r m_x;
			Quaternionr m_q;
			Vector3r m_scale;
			ObjectArray<unsigned int> m_staticParticles;
			Real m_restitutionCoeff;
			Real m_frictionCoeff;
			nlohmann::json m_json;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			PDB_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct TetModelData
		{
			unsigned int m_id;
			std::string m_modelFileTet;
			std::string m_modelFileNodes;
			std::string m_modelFileElements;
			std::string m_modelFileVis;
			Vector3r m_x;
			Quaternionr m_q;
			Vector3r m_scale;
			ObjectArray<unsigned int> m_staticParticles;
			Real m_restitutionCoeff;
			Real m_frictionCoeff;
			nlohmann::json m_json;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct BallJointData
		{
			unsigned int m_bodyID[2];
			Vector3r m_position;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			PDB_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct BallOnLineJointData
		{
			unsigned int m_bodyID[2];
			Vector3r m_position;
			Vector3r m_axis;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			PDB_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct HingeJointData
		{
			unsigned int m_bodyID[2];
			Vector3r m_position;
			Vector3r m_axis;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			PDB_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct UniversalJointData
		{
			unsigned int m_bodyID[2];
			Vector3r m_position;
			Vector3r m_axis[2];

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			PDB_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct SliderJointData
		{
			unsigned int m_bodyID[2];
			Vector3r m_position;
			Vector3r m_axis;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			PDB_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct RigidBodyParticleBallJointData
		{
			unsigned int m_bodyID[2];
		};

		struct TargetAngleMotorHingeJointData
		{
			unsigned int m_bodyID[2];
			Vector3r m_position;
			Vector3r m_axis;
			Real m_target;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			PDB_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct TargetVelocityMotorHingeJointData
		{
			unsigned int m_bodyID[2];
			Vector3r m_position;
			Vector3r m_axis;
			Real m_target;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct TargetPositionMotorSliderJointData
		{
			unsigned int m_bodyID[2];
			Vector3r m_position;
			Vector3r m_axis;
			Real m_target;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct TargetVelocityMotorSliderJointData
		{
			unsigned int m_bodyID[2];
			Vector3r m_position;
			Vector3r m_axis;
			Real m_target;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		struct SceneData
		{
			enum TriangleModelSimulationMethodTypes { No_Simulation_Method = 0, Distance_Constraints, FEM_Based_PBD, Strain_Based_Dynamics };
			enum TriangleModelBendingMethodTypes { No_Bending_Method = 0, Dihedral_Angle, Isometric_Bending };
			enum VelocityUpdateMethodTypes { First_Order_Update = 0, Second_Order_Update };

			std::string m_sceneName;
			Vector3r m_camPosition;
			Vector3r m_camLookat;

			Real m_timeStepSize;
			Vector3r m_gravity;
			Real m_contactTolerance;
			int m_triangleModelSimulationMethod;
			int m_triangleModelBendingMethod;
			int m_tetModelSimulationMethod;
			Real m_contactStiffnessRigidBody;
			Real m_contactStiffnessParticleRigidBody;

			unsigned int m_velocityUpdateMethod;
			unsigned int m_maxIter;
			unsigned int m_maxIterVel;

			Real m_cloth_stiffness;
			Real m_cloth_bendingStiffness;
			Real m_cloth_xxStiffness;
			Real m_cloth_yyStiffness;
			Real m_cloth_xyStiffness;
			Real m_cloth_xyPoissonRatio;
			Real m_cloth_yxPoissonRatio;
			bool  m_cloth_normalizeStretch;
			bool  m_cloth_normalizeShear;

			Real m_solid_stiffness;
			Real m_solid_poissonRatio;
			bool m_solid_normalizeStretch;
			bool m_solid_normalizeShear;

			ObjectArray<RigidBodyData> m_rigidBodyData;			
			ObjectArray<TriangleModelData> m_triangleModelData;
			ObjectArray<TetModelData> m_tetModelData;
			ObjectArray<BallJointData> m_ballJointData;
			ObjectArray<BallOnLineJointData> m_ballOnLineJointData;
			ObjectArray<HingeJointData> m_hingeJointData;
			ObjectArray<UniversalJointData> m_universalJointData;
			ObjectArray<SliderJointData> m_sliderJointData;
			ObjectArray<RigidBodyParticleBallJointData> m_rigidBodyParticleBallJointData;
			ObjectArray<TargetAngleMotorHingeJointData> m_targetAngleMotorHingeJointData;
			ObjectArray<TargetVelocityMotorHingeJointData> m_targetVelocityMotorHingeJointData;
			ObjectArray<TargetPositionMotorSliderJointData> m_targetPositionMotorSliderJointData;
			ObjectArray<TargetVelocityMotorSliderJointData> m_targetVelocityMotorSliderJointData;

		public:	//BES: 23.8.2016 - make sure the class is aligned to 16 bytes even for x86 build
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		void readScene(const std::string &fileName, SceneData &sceneData);
		void readSimulation(const nlohmann::json &child, const std::string &key, SceneData &sceneData);
		void readRigidBodies(const nlohmann::json &child, const std::string &key, const std::string &basePath, SceneData &sceneData);
		void readTriangleModels(const nlohmann::json &child, const std::string &key, const std::string &basePath, SceneData &sceneData);
		void readTetModels(const nlohmann::json &child, const std::string &key, const std::string &basePath, SceneData &sceneData);
		void readBallJoints(const nlohmann::json &child, const std::string &key, SceneData &sceneData);
		void readBallOnLineJoints(const nlohmann::json &child, const std::string &key, SceneData &sceneData);
		void readHingeJoints(const nlohmann::json &child, const std::string &key, SceneData &sceneData);
		void readUniversalJoints(const nlohmann::json &child, const std::string &key, SceneData &sceneData);
		void readSliderJoints(const nlohmann::json &child, const std::string &key, SceneData &sceneData);
		void readRigidBodyParticleBallJoints(const nlohmann::json &child, const std::string &key, SceneData &sceneData);
		void readTargetAngleMotorHingeJoints(const nlohmann::json &child, const std::string &key, SceneData &sceneData);
		void readTargetVelocityMotorHingeJoints(const nlohmann::json &child, const std::string &key, SceneData &sceneData);
		void readTargetPositionMotorSliderJoints(const nlohmann::json &child, const std::string &key, SceneData &sceneData);
		void readTargetVelocityMotorSliderJoints(const nlohmann::json &child, const std::string &key, SceneData &sceneData);

		template <typename T>
		static bool readValue(const nlohmann::json &j, const std::string &key, T &v)
		{
			if (j.find(key) == j.end())
				return false;

			v = j[key].get<T>();
			return true;
		}

		template <typename T, int size>
		static bool readVector(const nlohmann::json &j, const std::string &key, Eigen::Matrix<T, size, 1> &vec)
		{
			if (j.find(key) == j.end())
				return false;

			std::vector<T> values = j[key].get<std::vector<T>>();
			for (unsigned int i = 0; i < values.size(); i++)
				vec[i] = values[i];
			return true;
		}
	};

	template <>
	bool SceneLoader::readValue<bool>(const nlohmann::json &j, const std::string &key, bool &v);	
}

#endif