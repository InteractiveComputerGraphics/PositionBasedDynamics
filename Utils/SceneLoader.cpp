#include "SceneLoader.h"
#include <iostream>
#include <fstream>
#include "FileSystem.h"
#include "Logger.h"

using namespace Utilities;
using namespace GenParam;


void SceneLoader::readScene(const std::string &fileName, SceneData &sceneData)
{
	LOG_INFO << "Load scene file: " << fileName;
	try
	{
		std::ifstream input_file(fileName);
		if (!input_file.is_open())
		{
			LOG_ERR << "Cannot open file!\n";
			return;
		}
		m_json << input_file;

		std::string basePath = FileSystem::getFilePath(fileName);

		readValue(m_json, "Name", sceneData.m_sceneName);
		
		sceneData.m_camPosition = Vector3r(5.0, 10.0, 30.0);
		readVector(m_json, "cameraPosition", sceneData.m_camPosition);
		sceneData.m_camLookat = Vector3r(5.0, 0.0, 0.0);
		readVector(m_json, "cameraLookat", sceneData.m_camLookat);
		
		//////////////////////////////////////////////////////////////////////////
		// read general 
		//////////////////////////////////////////////////////////////////////////
		sceneData.m_timeStepSize = 0.005;
		sceneData.m_gravity = Vector3r(0, -9.81, 0);
		sceneData.m_maxIter = 5;
		sceneData.m_maxIterVel = 5;
		sceneData.m_velocityUpdateMethod = 0;
		sceneData.m_triangleModelSimulationMethod = -1;
		sceneData.m_triangleModelBendingMethod = -1;
		sceneData.m_tetModelSimulationMethod = -1;
		sceneData.m_contactTolerance = 0.0;
		sceneData.m_contactStiffnessRigidBody = 1.0;
		sceneData.m_contactStiffnessParticleRigidBody = 100.0;
		sceneData.m_cloth_stiffness = 1.0;
		sceneData.m_cloth_bendingStiffness = 0.01;
		sceneData.m_cloth_xxStiffness = 1.0;
		sceneData.m_cloth_yyStiffness = 1.0;
		sceneData.m_cloth_xyStiffness = 1.0;
		sceneData.m_cloth_xyPoissonRatio = 0.3;
		sceneData.m_cloth_yxPoissonRatio = 0.3;
		sceneData.m_cloth_normalizeStretch = false;
		sceneData.m_cloth_normalizeShear = false;
		sceneData.m_cloth_stiffness = 1.0;
		sceneData.m_cloth_stiffness = 0.3;
		sceneData.m_solid_normalizeStretch = false;
		sceneData.m_solid_normalizeShear = false;
		if (m_json.find("Simulation") != m_json.end())
			readSimulation(m_json, "Simulation", sceneData);
	
		//////////////////////////////////////////////////////////////////////////
		// read rigid bodies
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("RigidBodies") != m_json.end())
			readRigidBodies(m_json, "RigidBodies", basePath, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read triangle models
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("TriangleModels") != m_json.end())
			readTriangleModels(m_json, "TriangleModels", basePath, sceneData);
		
		//////////////////////////////////////////////////////////////////////////
		// read tet models
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("TetModels") != m_json.end())
			readTetModels(m_json, "TetModels", basePath, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read ball joints
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("BallJoints") != m_json.end())
			readBallJoints(m_json, "BallJoints", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read ball-on-line joints
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("BallOnLineJoints") != m_json.end())
			readBallOnLineJoints(m_json, "BallOnLineJoints", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read hinge joints
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("HingeJoints") != m_json.end())
			readHingeJoints(m_json, "HingeJoints", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read universal joints
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("UniversalJoints") != m_json.end())
			readUniversalJoints(m_json, "UniversalJoints", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read slider joints
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("SliderJoints") != m_json.end())
			readSliderJoints(m_json, "SliderJoints", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read RigidBodyParticleBallJoints
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("RigidBodyParticleBallJoints") != m_json.end())
			readRigidBodyParticleBallJoints(m_json, "RigidBodyParticleBallJoints", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read RigidBodySprings
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("RigidBodySprings") != m_json.end())
			readRigidBodySprings(m_json, "RigidBodySprings", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read DistanceJoints
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("DistanceJoints") != m_json.end())
			readDistanceJoints(m_json, "DistanceJoints", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read DamperJoints
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("DamperJoints") != m_json.end())
			readDamperJoints(m_json, "DamperJoints", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read TargetAngleMotorHingeJoint
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("TargetAngleMotorHingeJoints") != m_json.end())
			readTargetAngleMotorHingeJoints(m_json, "TargetAngleMotorHingeJoints", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read TargetVelocityMotorHingeJoint
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("TargetVelocityMotorHingeJoints") != m_json.end())
			readTargetVelocityMotorHingeJoints(m_json, "TargetVelocityMotorHingeJoints", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read TargetPositionMotorSliderJoint
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("TargetPositionMotorSliderJoints") != m_json.end())
			readTargetPositionMotorSliderJoints(m_json, "TargetPositionMotorSliderJoints", sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read TargetVelocityMotorSliderJoint
		//////////////////////////////////////////////////////////////////////////
		if (m_json.find("TargetVelocityMotorSliderJoints") != m_json.end())
			readTargetVelocityMotorSliderJoints(m_json, "TargetVelocityMotorSliderJoints", sceneData);
	}
	catch (std::exception& e)
	{
		LOG_ERR << e.what();
		exit(1);
	}
}

void SceneLoader::readSimulation(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	readValue(child, "timeStepSize", sceneData.m_timeStepSize);
	readVector(child, "gravity", sceneData.m_gravity);
	readValue(child, "maxIter", sceneData.m_maxIter);
	readValue(child, "maxIterVel", sceneData.m_maxIterVel);
	readValue(child, "velocityUpdateMethod", sceneData.m_velocityUpdateMethod);
	readValue(child, "triangleModelSimulationMethod", sceneData.m_triangleModelSimulationMethod);
	readValue(child, "triangleModelBendingMethod", sceneData.m_triangleModelBendingMethod);
	readValue(child, "tetModelSimulationMethod", sceneData.m_tetModelSimulationMethod);
	readValue(child, "contactTolerance", sceneData.m_contactTolerance);
	readValue(child, "contactStiffnessRigidBody", sceneData.m_contactStiffnessRigidBody);
	readValue(child, "contactStiffnessParticleRigidBody", sceneData.m_contactStiffnessParticleRigidBody);

	// stiffness
	readValue(child, "cloth_stiffness", sceneData.m_cloth_stiffness);

	// bendingStiffness
	readValue(child, "cloth_bendingStiffness", sceneData.m_cloth_bendingStiffness);

	// xxStiffness, yyStiffness, xyStiffness
	readValue(child, "cloth_xxStiffness", sceneData.m_cloth_xxStiffness);
	readValue(child, "cloth_yyStiffness", sceneData.m_cloth_yyStiffness);
	readValue(child, "cloth_xyStiffness", sceneData.m_cloth_xyStiffness);

	// xyPoissonRatio, yxPoissonRatio				  
	readValue(child, "cloth_xyPoissonRatio", sceneData.m_cloth_xyPoissonRatio);
	readValue(child, "cloth_yxPoissonRatio", sceneData.m_cloth_yxPoissonRatio);

	// normalize
	readValue(child, "cloth_normalizeStretch", sceneData.m_cloth_normalizeStretch);
	readValue(child, "cloth_normalizeShear", sceneData.m_cloth_normalizeShear);

	// solid stiffness
	readValue(child, "solid_stiffness", sceneData.m_solid_stiffness);

	// Poisson ratio
	readValue(child, "solid_poissonRatio", sceneData.m_solid_poissonRatio);

	// normalize
	readValue(child, "solid_normalizeStretch", sceneData.m_solid_normalizeStretch);
	readValue(child, "solid_normalizeShear", sceneData.m_solid_normalizeShear);
}

void SceneLoader::readRigidBodies(const nlohmann::json &j, const std::string &key, const std::string &basePath, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	sceneData.m_rigidBodyData.reserve(5000);

	for (auto& rigidBody : child)
	{
		std::string geomFileName;
		if (readValue<std::string>(rigidBody, "geometryFile", geomFileName))
		{
			if (FileSystem::isRelativePath(geomFileName))
			{
				geomFileName = basePath + "/" + geomFileName;
			}

			sceneData.m_rigidBodyData.emplace_back(RigidBodyData());
			RigidBodyData &rbd = sceneData.m_rigidBodyData.back();
			rbd.m_modelFile = geomFileName;

			// id
			rbd.m_id = 0;
			readValue(rigidBody, "id", rbd.m_id);

			// is dynamic body
			rbd.m_isDynamic = true;
			readValue(rigidBody, "isDynamic", rbd.m_isDynamic);

			// density
			rbd.m_density = 1.0;
			readValue(rigidBody, "density", rbd.m_density);

			// translation
			rbd.m_x.setZero();
			readVector(rigidBody, "translation", rbd.m_x);
			
			// rotation axis
			Vector3r axis;
			axis.setZero();
			Real angle = 0.0;
			if (readVector(rigidBody, "rotationAxis", axis) &&
				readValue<Real>(rigidBody, "rotationAngle", angle))
			{
				axis.normalize();
				rbd.m_q = Quaternionr(AngleAxisr(angle, axis));
			}
			else
				rbd.m_q = Quaternionr(1.0, 0.0, 0.0, 0.0);
				
			// scale
			rbd.m_scale = Vector3r(1.0, 1.0, 1.0);
			readVector(rigidBody, "scale", rbd.m_scale);

			// velocity
			rbd.m_v.setZero();
			readVector(rigidBody, "velocity", rbd.m_v);

			// angular velocity
			rbd.m_omega.setZero();
			readVector(rigidBody, "angularVelocity", rbd.m_omega);

			// restitution
			rbd.m_restitutionCoeff = 0.6;
			readValue(rigidBody, "restitution", rbd.m_restitutionCoeff);

			// friction
			rbd.m_frictionCoeff = 0.2;
			readValue(rigidBody, "friction", rbd.m_frictionCoeff);

			// collision object type
			rbd.m_collisionObjectType = CollisionObjectTypes::No_Collision_Object;
			readValue(rigidBody, "collisionObjectType", rbd.m_collisionObjectType);

			rbd.m_collisionObjectFileName = "";
			readValue(rigidBody, "collisionObjectFileName", rbd.m_collisionObjectFileName);

			// test mesh
			rbd.m_testMesh = true;
			readValue(rigidBody, "testMesh", rbd.m_testMesh);

			// scale
			rbd.m_collisionObjectScale = Vector3r(1.0, 1.0, 1.0);
			readVector(rigidBody, "collisionObjectScale", rbd.m_collisionObjectScale);

			rbd.m_invertSDF = false;
			readValue(rigidBody, "invertSDF", rbd.m_invertSDF);

			rbd.m_thicknessSDF = 0.1;
			readValue(rigidBody, "thicknessSDF", rbd.m_thicknessSDF);

			rbd.m_resolutionSDF = Eigen::Matrix<unsigned int, 3, 1, Eigen::DontAlign>(10, 10, 10);
			readVector(rigidBody, "resolutionSDF", rbd.m_resolutionSDF);
			
			rbd.m_json = rigidBody;
		}
	}
}

void SceneLoader::readTriangleModels(const nlohmann::json &j, const std::string &key, const std::string &basePath, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& triModel : child) 
	{
		std::string geomFileName;
		if (readValue<std::string>(triModel, "geometryFile", geomFileName))
		{
			if (FileSystem::isRelativePath(geomFileName))
			{
				geomFileName = basePath + "/" + geomFileName;
			}

			sceneData.m_triangleModelData.emplace_back(TriangleModelData());
			TriangleModelData &data = sceneData.m_triangleModelData.back();
			data.m_modelFile = geomFileName;

			// id
			data.m_id = 0;
			readValue(triModel, "id", data.m_id);

			// translation
			data.m_x.setZero();
			readVector(triModel, "translation", data.m_x);

			// rotation axis
			Vector3r axis;
			axis.setZero();
			Real angle = 0.0;
			if (readVector(triModel, "rotationAxis", axis) &&
				readValue<Real>(triModel, "rotationAngle", angle))
			{
				axis.normalize();
				data.m_q = Quaternionr(AngleAxisr(angle, axis));
			}
			else
				data.m_q = Quaternionr(1.0, 0.0, 0.0, 0.0);

			// scale
			data.m_scale = Vector3r(1.0, 1.0, 1.0);
			readVector(triModel, "scale", data.m_scale);

			// static particles
			unsigned int index = 0;
			if (triModel.find("staticParticles") != triModel.end())
			{
				data.m_staticParticles.reserve((unsigned int)triModel["staticParticles"].size());
				for (auto& item : triModel["staticParticles"])
					data.m_staticParticles.push_back(item.get<unsigned int>());
			}

			// restitution
			data.m_restitutionCoeff = 0.1;
			readValue(triModel, "restitution", data.m_restitutionCoeff);

			// friction
			data.m_frictionCoeff = 0.2;
			readValue(triModel, "friction", data.m_frictionCoeff);

			data.m_json = triModel;
		}
	}
}

void SceneLoader::readTetModels(const nlohmann::json &j, const std::string &key, const std::string &basePath, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& tetModel : child)
	{
		std::string nodeFileName, eleFileName;
		if (readValue<std::string>(tetModel, "nodeFile", nodeFileName) &&
			readValue<std::string>(tetModel, "eleFile", eleFileName))
		{
			if (FileSystem::isRelativePath(nodeFileName))
			{
				nodeFileName = basePath + "/" + nodeFileName;
			}

			if (FileSystem::isRelativePath(eleFileName))
			{
				eleFileName = basePath + "/" + eleFileName;
			}

			std::string visFileName = "";
			if (readValue<std::string>(tetModel, "visFile", visFileName))
			{
				if (FileSystem::isRelativePath(visFileName))
				{
					visFileName = basePath + "/" + visFileName;
				}
			}

			sceneData.m_tetModelData.emplace_back(TetModelData());
			TetModelData &data = sceneData.m_tetModelData.back();
			data.m_modelFileNodes = nodeFileName;
			data.m_modelFileElements = eleFileName;
			data.m_modelFileVis = visFileName;

			// id
			data.m_id = 0;
			readValue(tetModel, "id", data.m_id);

			// translation
			data.m_x.setZero();
			readVector(tetModel, "translation", data.m_x);

			// rotation axis
			Vector3r axis;
			axis.setZero();
			Real angle = 0.0;
			if (readVector(tetModel, "rotationAxis", axis) &&
				readValue<Real>(tetModel, "rotationAngle", angle))
			{
				axis.normalize();
				data.m_q = Quaternionr(AngleAxisr(angle, axis));
			}
			else
				data.m_q = Quaternionr(1.0, 0.0, 0.0, 0.0);

			// scale
			data.m_scale = Vector3r(1.0, 1.0, 1.0);
			readVector(tetModel, "scale", data.m_scale);

			// static particles
			unsigned int index = 0;
			if (tetModel.find("staticParticles") != tetModel.end())
			{
				data.m_staticParticles.reserve((unsigned int)tetModel["staticParticles"].size());
				for (auto& item : tetModel["staticParticles"])
					data.m_staticParticles.push_back(item.get<unsigned int>());
			}

			// restitution
			data.m_restitutionCoeff = 0.1;
			readValue(tetModel, "restitution", data.m_restitutionCoeff);

			// friction
			data.m_frictionCoeff = 0.2;
			readValue(tetModel, "friction", data.m_frictionCoeff);

			// collision object type
			data.m_collisionObjectType = CollisionObjectTypes::No_Collision_Object;
			readValue(tetModel, "collisionObjectType", data.m_collisionObjectType);

			data.m_collisionObjectFileName = "";
			readValue(tetModel, "collisionObjectFileName", data.m_collisionObjectFileName);

			// test mesh
			data.m_testMesh = true;
			readValue(tetModel, "testMesh", data.m_testMesh);

			// scale
			data.m_collisionObjectScale = Vector3r(1.0, 1.0, 1.0);
			readVector(tetModel, "collisionObjectScale", data.m_collisionObjectScale);

			data.m_invertSDF = false;
			readValue(tetModel, "invertSDF", data.m_invertSDF);

			data.m_thicknessSDF = 0.1;
			readValue(tetModel, "thicknessSDF", data.m_thicknessSDF);

			data.m_resolutionSDF = Eigen::Matrix<unsigned int, 3, 1, Eigen::DontAlign>(10, 10, 10);
			readVector(tetModel, "resolutionSDF", data.m_resolutionSDF);

			data.m_json = tetModel;
		}
	}
}

void SceneLoader::readBallJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		BallJointData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "position", jd.m_position))
		{
			sceneData.m_ballJointData.push_back(jd);
		}
	}
}


void SceneLoader::readBallOnLineJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		BallOnLineJointData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "position", jd.m_position) &&
			readVector(joint, "axis", jd.m_axis))
		{
			sceneData.m_ballOnLineJointData.push_back(jd);
		}
	}
}

void SceneLoader::readHingeJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		HingeJointData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "position", jd.m_position) &&
			readVector(joint, "axis", jd.m_axis))
		{
			sceneData.m_hingeJointData.push_back(jd);
		}
	}
}

void SceneLoader::readUniversalJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		UniversalJointData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "position", jd.m_position) &&
			readVector(joint, "axis1", jd.m_axis[0]) &&
			readVector(joint, "axis2", jd.m_axis[1]))
		{
			sceneData.m_universalJointData.push_back(jd);
		}
	}
}

void SceneLoader::readSliderJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		SliderJointData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "axis", jd.m_axis))
		{
			sceneData.m_sliderJointData.push_back(jd);
		}
	}
}

void SceneLoader::readRigidBodyParticleBallJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		RigidBodyParticleBallJointData jd;
		if (readValue(joint, "rbID", jd.m_bodyID[0]) &&
			readValue(joint, "particleID", jd.m_bodyID[1]))
		{
			sceneData.m_rigidBodyParticleBallJointData.push_back(jd);
		}
	}
}

void SceneLoader::readTargetAngleMotorHingeJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		TargetAngleMotorHingeJointData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "position", jd.m_position) &&
			readVector(joint, "axis", jd.m_axis))
		{
			if (!readValue(joint, "target", jd.m_target))
			{
				jd.m_target = 0.0;

				// target sequence
				unsigned int index = 0;
				if (joint.find("targetSequence") != joint.end())
				{
					jd.m_targetSequence.reserve((unsigned int)joint["targetSequence"].size());
					for (auto& item : joint["targetSequence"])
						jd.m_targetSequence.push_back(item.get<Real>());

					// be sure that we have an even number of values
					if (jd.m_targetSequence.size() % 2 == 1)
						jd.m_targetSequence.pop_back();
					readValue(joint, "repeatSequence", jd.m_repeat);
				}
			}
			sceneData.m_targetAngleMotorHingeJointData.push_back(jd);
		}
	}
}

void SceneLoader::readTargetVelocityMotorHingeJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		TargetVelocityMotorHingeJointData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "position", jd.m_position) &&
			readVector(joint, "axis", jd.m_axis))
		{
			if (!readValue(joint, "target", jd.m_target))
			{
				jd.m_target = 0.0;

				// target sequence
				unsigned int index = 0;
				if (joint.find("targetSequence") != joint.end())
				{
					jd.m_targetSequence.reserve((unsigned int)joint["targetSequence"].size());
					for (auto& item : joint["targetSequence"])
						jd.m_targetSequence.push_back(item.get<Real>());

					// be sure that we have an even number of values
					if (jd.m_targetSequence.size() % 2 == 1)
						jd.m_targetSequence.pop_back();
					readValue(joint, "repeatSequence", jd.m_repeat);
				}
			}
			sceneData.m_targetVelocityMotorHingeJointData.push_back(jd);
		}
	}
}

void SceneLoader::readTargetPositionMotorSliderJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		TargetPositionMotorSliderJointData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "axis", jd.m_axis))
		{
			if (!readValue(joint, "target", jd.m_target))
			{
				jd.m_target = 0.0;

				// target sequence
				unsigned int index = 0;
				if (joint.find("targetSequence") != joint.end())
				{
					jd.m_targetSequence.reserve((unsigned int)joint["targetSequence"].size());
					for (auto& item : joint["targetSequence"])
						jd.m_targetSequence.push_back(item.get<Real>());

					// be sure that we have an even number of values
					if (jd.m_targetSequence.size() % 2 == 1)
						jd.m_targetSequence.pop_back();
					readValue(joint, "repeatSequence", jd.m_repeat);
				}
			}
			sceneData.m_targetPositionMotorSliderJointData.push_back(jd);
		}
	}
}

void SceneLoader::readTargetVelocityMotorSliderJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		TargetVelocityMotorSliderJointData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "axis", jd.m_axis))
		{
			if (!readValue(joint, "target", jd.m_target))
			{
				jd.m_target = 0.0;

				// target sequence
				unsigned int index = 0;
				if (joint.find("targetSequence") != joint.end())
				{
					jd.m_targetSequence.reserve((unsigned int)joint["targetSequence"].size());
					for (auto& item : joint["targetSequence"])
						jd.m_targetSequence.push_back(item.get<Real>());

					// be sure that we have an even number of values
					if (jd.m_targetSequence.size() % 2 == 1)
						jd.m_targetSequence.pop_back();
					readValue(joint, "repeatSequence", jd.m_repeat);
				}
			}
			sceneData.m_targetVelocityMotorSliderJointData.push_back(jd);
		}
	}
}

void SceneLoader::readRigidBodySprings(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		RigidBodySpringData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "position1", jd.m_position1) &&
			readVector(joint, "position2", jd.m_position2) &&
			readValue(joint, "stiffness", jd.m_stiffness))
		{
			sceneData.m_rigidBodySpringData.push_back(jd);
		}
	}
}

void SceneLoader::readDistanceJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		DistanceJointData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "position1", jd.m_position1) &&
			readVector(joint, "position2", jd.m_position2))
		{
			sceneData.m_distanceJointData.push_back(jd);
		}
	}
}

void SceneLoader::readDamperJoints(const nlohmann::json &j, const std::string &key, SceneData &sceneData)
{
	const nlohmann::json &child = j[key];

	for (auto& joint : child)
	{
		DamperJointData jd;
		if (readValue(joint, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joint, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joint, "axis", jd.m_axis) &&
			readValue(joint, "stiffness", jd.m_stiffness))
		{
			sceneData.m_damperJointData.push_back(jd);
		}
	}
}

template <>
bool SceneLoader::readValue<bool>(const nlohmann::json &j, const std::string &key, bool &v)
{
	if (j.find(key) == j.end())
		return false;

	if (j[key].is_number_integer())
	{
		int val = j[key].get<int>();
		v = val != 0;
	}
	else
		v = j[key].get<bool>();
	return true;
}

void SceneLoader::readParameterObject(ParameterObject *paramObj)
{
	if (paramObj == nullptr)
		return;

	const unsigned int numParams = paramObj->numParameters();


	//////////////////////////////////////////////////////////////////////////
	// read configuration 
	//////////////////////////////////////////////////////////////////////////
	if (m_json.find("Simulation") != m_json.end())
	{
		nlohmann::json config = m_json["Simulation"];
		std::vector<std::string> newParamList;

		for (unsigned int i = 0; i < numParams; i++)
		{
			ParameterBase *paramBase = paramObj->getParameter(i);

			if (paramBase->getType() == RealParameterType)
			{
				Real val;
				if (readValue(config, paramBase->getName(), val))
					static_cast<NumericParameter<Real>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::UINT32)
			{
				unsigned int val;
				if (readValue(config, paramBase->getName(), val))
					static_cast<NumericParameter<unsigned int>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::UINT16)
			{
				unsigned short val;
				if (readValue(config, paramBase->getName(), val))
					static_cast<NumericParameter<unsigned short>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::UINT8)
			{
				unsigned char val;
				if (readValue(config, paramBase->getName(), val))
					static_cast<NumericParameter<unsigned char>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::INT32)
			{
				int val;
				if (readValue(config, paramBase->getName(), val))
					static_cast<NumericParameter<int>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::INT16)
			{
				short val;
				if (readValue(config, paramBase->getName(), val))
					static_cast<NumericParameter<short>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::INT8)
			{
				char val;
				if (readValue(config, paramBase->getName(), val))
					static_cast<NumericParameter<char>*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::ENUM)
			{
				int val;
				if (readValue(config, paramBase->getName(), val))
					static_cast<EnumParameter*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == ParameterBase::BOOL)
			{
				bool val;
				if (readValue(config, paramBase->getName(), val))
					static_cast<BoolParameter*>(paramBase)->setValue(val);
			}
			else if (paramBase->getType() == RealVectorParameterType)
			{
				if (static_cast<VectorParameter<Real>*>(paramBase)->getDim() == 3)
				{
					Vector3r val;
					if (readVector(config, paramBase->getName(), val))
						static_cast<VectorParameter<Real>*>(paramBase)->setValue(val.data());
				}
			}
		}
	}
}
