#include "SceneLoader.h"
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

using namespace PBD;


void SceneLoader::readScene(const std::string &fileName, SceneData &sceneData)
{
	FILE *file = fopen(fileName.c_str(), "r");
	if (file == NULL)
		return;
	fclose(file);

	try
	{
		boost::property_tree::json_parser::read_json(fileName, m_ptree);

		readValue(m_ptree, "Name", sceneData.m_sceneName);
		
		sceneData.m_camPosition = Vector3r(5.0, 10.0, 30.0);
		readVector(m_ptree, "cameraPosition", sceneData.m_camPosition);
		sceneData.m_camLookat = Vector3r(5.0, 0.0, 0.0);
		readVector(m_ptree, "cameraLookat", sceneData.m_camLookat);

		//////////////////////////////////////////////////////////////////////////
		// read general 
		//////////////////////////////////////////////////////////////////////////
		boost::optional<boost::property_tree::ptree&> child = m_ptree.get_child_optional("Simulation");
		if (child)
			readSimulation(child, sceneData);
	
		//////////////////////////////////////////////////////////////////////////
		// read rigid bodies
		//////////////////////////////////////////////////////////////////////////
		boost::filesystem::path basePath = boost::filesystem::path(fileName).parent_path();
		child = m_ptree.get_child_optional("RigidBodies");
		if (child)
			readRigidBodies(child, basePath, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read triangle models
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("TriangleModels");
		if (child)
			readTriangleModels(child, basePath, sceneData);
		
		//////////////////////////////////////////////////////////////////////////
		// read tet models
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("TetModels");
		if (child)
			readTetModels(child, basePath, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read ball joints
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("BallJoints");
		if (child)
			readBallJoints(child, sceneData);		

		//////////////////////////////////////////////////////////////////////////
		// read ball-on-line joints
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("BallOnLineJoints");
		if (child)
			readBallOnLineJoints(child, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read hinge joints
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("HingeJoints");
		if (child)
			readHingeJoints(child, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read universal joints
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("UniversalJoints");
		if (child)
			readUniversalJoints(child, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read slider joints
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("SliderJoints");
		if (child)
			readSliderJoints(child, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read RigidBodyParticleBallJoints
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("RigidBodyParticleBallJoints");
		if (child)
			readRigidBodyParticleBallJoints(child, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read TargetAngleMotorHingeJoint
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("TargetAngleMotorHingeJoints");
		if (child)
			readTargetAngleMotorHingeJoints(child, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read TargetVelocityMotorHingeJoint
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("TargetVelocityMotorHingeJoints");
		if (child)
			readTargetVelocityMotorHingeJoints(child, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read TargetPositionMotorSliderJoint
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("TargetPositionMotorSliderJoints");
		if (child)
			readTargetPositionMotorSliderJoints(child, sceneData);

		//////////////////////////////////////////////////////////////////////////
		// read TargetVelocityMotorSliderJoint
		//////////////////////////////////////////////////////////////////////////
		child = m_ptree.get_child_optional("TargetVelocityMotorSliderJoints");
		if (child)
			readTargetVelocityMotorSliderJoints(child, sceneData);
	}
	catch (std::exception& e)
	{
		std::cout << e.what() << "\n";
		exit(1);
	}
}

void SceneLoader::readSimulation(const boost::optional<boost::property_tree::ptree&> child, SceneData &sceneData)
{
	sceneData.m_timeStepSize = 0.005;
	readValue(child.get(), "timeStepSize", sceneData.m_timeStepSize);

	sceneData.m_gravity = Vector3r(0, -9.81, 0);
	readVector(child.get(), "gravity", sceneData.m_gravity);

	sceneData.m_maxIter = 5;
	readValue(child.get(), "maxIter", sceneData.m_maxIter);

	sceneData.m_maxIterVel = 5;
	readValue(child.get(), "maxIterVel", sceneData.m_maxIterVel);

	sceneData.m_velocityUpdateMethod = 0;
	readValue(child.get(), "velocityUpdateMethod", sceneData.m_velocityUpdateMethod);

	sceneData.m_triangleModelSimulationMethod = -1;
	readValue(child.get(), "triangleModelSimulationMethod", sceneData.m_triangleModelSimulationMethod);

	sceneData.m_triangleModelBendingMethod = -1;
	readValue(child.get(), "triangleModelBendingMethod", sceneData.m_triangleModelBendingMethod);

	sceneData.m_tetModelSimulationMethod = -1;
	readValue(child.get(), "tetModelSimulationMethod", sceneData.m_tetModelSimulationMethod);

	sceneData.m_contactTolerance = 0.0;
	readValue(child.get(), "contactTolerance", sceneData.m_contactTolerance);

	sceneData.m_contactStiffnessRigidBody = 1.0;
	readValue(child.get(), "contactStiffnessRigidBody", sceneData.m_contactStiffnessRigidBody);

	sceneData.m_contactStiffnessParticleRigidBody = 100.0;
	readValue(child.get(), "contactStiffnessParticleRigidBody", sceneData.m_contactStiffnessParticleRigidBody);

	// stiffness
	sceneData.m_cloth_stiffness = 1.0;
	readValue(child.get(), "cloth_stiffness", sceneData.m_cloth_stiffness);

	// bendingStiffness
	sceneData.m_cloth_bendingStiffness = 0.01;
	readValue(child.get(), "cloth_bendingStiffness", sceneData.m_cloth_bendingStiffness);

	// xxStiffness, yyStiffness, xyStiffness
	sceneData.m_cloth_xxStiffness = 1.0;
	sceneData.m_cloth_yyStiffness = 1.0;
	sceneData.m_cloth_xyStiffness = 1.0;
	readValue(child.get(), "cloth_xxStiffness", sceneData.m_cloth_xxStiffness);
	readValue(child.get(), "cloth_yyStiffness", sceneData.m_cloth_yyStiffness);
	readValue(child.get(), "cloth_xyStiffness", sceneData.m_cloth_xyStiffness);

	// xyPoissonRatio, yxPoissonRatio				  
	sceneData.m_cloth_xyPoissonRatio = 0.3;
	sceneData.m_cloth_yxPoissonRatio = 0.3;
	readValue(child.get(), "cloth_xyPoissonRatio", sceneData.m_cloth_xyPoissonRatio);
	readValue(child.get(), "cloth_yxPoissonRatio", sceneData.m_cloth_yxPoissonRatio);

	// normalize
	sceneData.m_cloth_normalizeStretch = false;
	sceneData.m_cloth_normalizeShear = false;
	readValue(child.get(), "cloth_normalizeStretch", sceneData.m_cloth_normalizeStretch);
	readValue(child.get(), "cloth_normalizeShear", sceneData.m_cloth_normalizeShear);

	// solid stiffness
	sceneData.m_cloth_stiffness = 1.0;
	readValue(child.get(), "solid_stiffness", sceneData.m_solid_stiffness);

	// Poisson ratio
	sceneData.m_cloth_stiffness = 0.3;
	readValue(child.get(), "solid_poissonRatio", sceneData.m_solid_poissonRatio);

	// normalize
	sceneData.m_solid_normalizeStretch = false;
	sceneData.m_solid_normalizeShear = false;
	readValue(child.get(), "solid_normalizeStretch", sceneData.m_solid_normalizeStretch);
	readValue(child.get(), "solid_normalizeShear", sceneData.m_solid_normalizeShear);
}

void SceneLoader::readRigidBodies(const boost::optional<boost::property_tree::ptree&> child, const boost::filesystem::path &basePath, SceneData &sceneData)
{
	sceneData.m_rigidBodyData.reserve(5000);
	BOOST_FOREACH(boost::property_tree::ptree::value_type &rigidBodies, child.get())
	{
		std::string geomFileName;
		if (readValue<std::string>(rigidBodies.second, "geometryFile", geomFileName))
		{
			boost::filesystem::path p(geomFileName);
			if (p.is_relative())
			{
				geomFileName = boost::filesystem::path(basePath / geomFileName).string();
			}

			RigidBodyData &rbd = sceneData.m_rigidBodyData.create();
			rbd.m_modelFile = geomFileName;

			// id
			rbd.m_id = 0;
			readValue(rigidBodies.second, "id", rbd.m_id);

			// is dynamic body
			rbd.m_isDynamic = true;
			readValue(rigidBodies.second, "isDynamic", rbd.m_isDynamic);

			// density
			rbd.m_density = 1.0;
			readValue(rigidBodies.second, "density", rbd.m_density);

			// translation
			rbd.m_x.setZero();
			readVector(rigidBodies.second, "translation", rbd.m_x);

			// rotation axis
			Vector3r axis;
			axis.setZero();
			Real angle = 0.0;
			if (readVector(rigidBodies.second, "rotationAxis", axis) &&
				readValue<Real>(rigidBodies.second, "rotationAngle", angle))
			{
				axis.normalize();
				rbd.m_q = Quaternionr(AngleAxisr(angle, axis));
			}
			else
				rbd.m_q = Quaternionr(1.0, 0.0, 0.0, 0.0);

			// scale
			rbd.m_scale = Vector3r(1.0, 1.0, 1.0);
			readVector(rigidBodies.second, "scale", rbd.m_scale);

			// velocity
			rbd.m_v.setZero();
			readVector(rigidBodies.second, "velocity", rbd.m_v);

			// angular velocity
			rbd.m_omega.setZero();
			readVector(rigidBodies.second, "angularVelocity", rbd.m_omega);

			// restitution
			rbd.m_restitutionCoeff = 0.6;
			readValue(rigidBodies.second, "restitution", rbd.m_restitutionCoeff);

			// friction
			rbd.m_frictionCoeff = 0.2;
			readValue(rigidBodies.second, "friction", rbd.m_frictionCoeff);

			// collision object type
			rbd.m_collisionObjectType = RigidBodyData::CollisionObjectTypes::No_Collision_Object;
			readValue(rigidBodies.second, "collisionObjectType", rbd.m_collisionObjectType);

			// test mesh
			rbd.m_testMesh = true;
			readValue(rigidBodies.second, "testMesh", rbd.m_testMesh);

			// scale
			rbd.m_collisionObjectScale = Vector3r(1.0, 1.0, 1.0);
			readVector(rigidBodies.second, "collisionObjectScale", rbd.m_collisionObjectScale);

			rbd.pt = &rigidBodies.second;
		}
	}
}

void SceneLoader::readTriangleModels(const boost::optional<boost::property_tree::ptree&> child, const boost::filesystem::path &basePath, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &triModels, child.get())
	{
		std::string geomFileName;
		if (readValue<std::string>(triModels.second, "geometryFile", geomFileName))
		{
			boost::filesystem::path p(geomFileName);
			if (p.is_relative())
			{
				geomFileName = boost::filesystem::path(basePath / geomFileName).string();
			}

			TriangleModelData &data = sceneData.m_triangleModelData.create();
			data.m_modelFile = geomFileName;

			// id
			data.m_id = 0;
			readValue(triModels.second, "id", data.m_id);

			// translation
			data.m_x.setZero();
			readVector(triModels.second, "translation", data.m_x);

			// rotation axis
			Vector3r axis;
			axis.setZero();
			Real angle = 0.0;
			if (readVector(triModels.second, "rotationAxis", axis) &&
				readValue<Real>(triModels.second, "rotationAngle", angle))
			{
				axis.normalize();
				data.m_q = Quaternionr(AngleAxisr(angle, axis));
			}
			else
				data.m_q = Quaternionr(1.0, 0.0, 0.0, 0.0);

			// scale
			data.m_scale = Vector3r(1.0, 1.0, 1.0);
			readVector(triModels.second, "scale", data.m_scale);

			// static particles
			unsigned int index = 0;
			boost::optional<boost::property_tree::ptree&> child2 = triModels.second.get_child_optional("staticParticles");
			if (child2)
			{
				data.m_staticParticles.reserve((unsigned int)child2.get().size());
				for (auto& item : child2.get())
					data.m_staticParticles.push_back(item.second.get_value<unsigned int>());
			}

			// restitution
			data.m_restitutionCoeff = 0.1;
			readValue(triModels.second, "restitution", data.m_restitutionCoeff);

			// friction
			data.m_frictionCoeff = 0.2;
			readValue(triModels.second, "friction", data.m_frictionCoeff);

			data.pt = &triModels.second;
		}
	}
}

void SceneLoader::readTetModels(const boost::optional<boost::property_tree::ptree&> child, const boost::filesystem::path &basePath, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &tetModels, child.get())
	{
		std::string nodeFileName, eleFileName;
		if (readValue<std::string>(tetModels.second, "nodeFile", nodeFileName) &&
			readValue<std::string>(tetModels.second, "eleFile", eleFileName))
		{
			boost::filesystem::path pNode(nodeFileName);
			if (pNode.is_relative())
			{
				nodeFileName = boost::filesystem::path(basePath / nodeFileName).string();
			}

			boost::filesystem::path pEle(eleFileName);
			if (pEle.is_relative())
			{
				eleFileName = boost::filesystem::path(basePath / eleFileName).string();
			}

			std::string visFileName = "";
			if (readValue<std::string>(tetModels.second, "visFile", visFileName))
			{
				boost::filesystem::path pVis(visFileName);
				if (pVis.is_relative())
				{
					visFileName = boost::filesystem::path(basePath / visFileName).string();
				}
			}

			TetModelData &data = sceneData.m_tetModelData.create();
			data.m_modelFileNodes = nodeFileName;
			data.m_modelFileElements = eleFileName;
			data.m_modelFileVis = visFileName;

			// id
			data.m_id = 0;
			readValue(tetModels.second, "id", data.m_id);

			// translation
			data.m_x.setZero();
			readVector(tetModels.second, "translation", data.m_x);

			// rotation axis
			Vector3r axis;
			axis.setZero();
			Real angle = 0.0;
			if (readVector(tetModels.second, "rotationAxis", axis) &&
				readValue<Real>(tetModels.second, "rotationAngle", angle))
			{
				axis.normalize();
				data.m_q = Quaternionr(AngleAxisr(angle, axis));
			}
			else
				data.m_q = Quaternionr(1.0, 0.0, 0.0, 0.0);

			// scale
			data.m_scale = Vector3r(1.0, 1.0, 1.0);
			readVector(tetModels.second, "scale", data.m_scale);

			// static particles
			unsigned int index = 0;
			boost::optional<boost::property_tree::ptree&> child2 = tetModels.second.get_child_optional("staticParticles");
			if (child2)
			{
				data.m_staticParticles.reserve((unsigned int)child2.get().size());
				for (auto& item : child2.get())
					data.m_staticParticles.push_back(item.second.get_value<unsigned int>());
			}

			// restitution
			data.m_restitutionCoeff = 0.1;
			readValue(tetModels.second, "restitution", data.m_restitutionCoeff);

			// friction
			data.m_frictionCoeff = 0.2;
			readValue(tetModels.second, "friction", data.m_frictionCoeff);

			data.pt = &tetModels.second;
		}
	}
}

void SceneLoader::readBallJoints(const boost::optional<boost::property_tree::ptree&> child, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &joints, child.get())
	{
		BallJointData jd;
		if (readValue(joints.second, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joints.second, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joints.second, "position", jd.m_position))
		{
			sceneData.m_ballJointData.push_back(jd);
		}
	}
}


void SceneLoader::readBallOnLineJoints(const boost::optional<boost::property_tree::ptree&> child, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &joints, child.get())
	{
		BallOnLineJointData jd;
		if (readValue(joints.second, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joints.second, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joints.second, "position", jd.m_position) &&
			readVector(joints.second, "axis", jd.m_axis))
		{
			sceneData.m_ballOnLineJointData.push_back(jd);
		}
	}
}

void SceneLoader::readHingeJoints(const boost::optional<boost::property_tree::ptree&> child, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &joints, child.get())
	{
		HingeJointData jd;
		if (readValue(joints.second, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joints.second, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joints.second, "position", jd.m_position) &&
			readVector(joints.second, "axis", jd.m_axis))
		{
			sceneData.m_hingeJointData.push_back(jd);
		}
	}
}

void SceneLoader::readUniversalJoints(const boost::optional<boost::property_tree::ptree&> child, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &joints, child.get())
	{
		UniversalJointData jd;
		if (readValue(joints.second, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joints.second, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joints.second, "position", jd.m_position) &&
			readVector(joints.second, "axis1", jd.m_axis[0]) &&
			readVector(joints.second, "axis2", jd.m_axis[1]))
		{
			sceneData.m_universalJointData.push_back(jd);
		}
	}
}

void SceneLoader::readSliderJoints(const boost::optional<boost::property_tree::ptree&> child, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &joints, child.get())
	{
		SliderJointData jd;
		if (readValue(joints.second, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joints.second, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joints.second, "position", jd.m_position) &&
			readVector(joints.second, "axis", jd.m_axis))
		{
			sceneData.m_sliderJointData.push_back(jd);
		}
	}
}

void SceneLoader::readRigidBodyParticleBallJoints(const boost::optional<boost::property_tree::ptree&> child, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &joints, child.get())
	{
		RigidBodyParticleBallJointData jd;
		if (readValue(joints.second, "rbID", jd.m_bodyID[0]) &&
			readValue(joints.second, "particleID", jd.m_bodyID[1]))
		{
			sceneData.m_rigidBodyParticleBallJointData.push_back(jd);
		}
	}
}

void SceneLoader::readTargetAngleMotorHingeJoints(const boost::optional<boost::property_tree::ptree&> child, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &joints, child.get())
	{
		TargetAngleMotorHingeJointData jd;
		if (readValue(joints.second, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joints.second, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joints.second, "position", jd.m_position) &&
			readVector(joints.second, "axis", jd.m_axis) &&
			readValue(joints.second, "target", jd.m_target))
		{
			sceneData.m_targetAngleMotorHingeJointData.push_back(jd);
		}
	}
}

void SceneLoader::readTargetVelocityMotorHingeJoints(const boost::optional<boost::property_tree::ptree&> child, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &joints, child.get())
	{
		TargetVelocityMotorHingeJointData jd;
		if (readValue(joints.second, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joints.second, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joints.second, "position", jd.m_position) &&
			readVector(joints.second, "axis", jd.m_axis) &&
			readValue(joints.second, "target", jd.m_target))
		{
			sceneData.m_targetVelocityMotorHingeJointData.push_back(jd);
		}
	}
}

void SceneLoader::readTargetPositionMotorSliderJoints(const boost::optional<boost::property_tree::ptree&> child, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &joints, child.get())
	{
		TargetPositionMotorSliderJointData jd;
		if (readValue(joints.second, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joints.second, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joints.second, "position", jd.m_position) &&
			readVector(joints.second, "axis", jd.m_axis) &&
			readValue(joints.second, "target", jd.m_target))
		{
			sceneData.m_targetPositionMotorSliderJointData.push_back(jd);
		}
	}
}

void SceneLoader::readTargetVelocityMotorSliderJoints(const boost::optional<boost::property_tree::ptree&> child, SceneData &sceneData)
{
	BOOST_FOREACH(boost::property_tree::ptree::value_type &joints, child.get())
	{
		TargetVelocityMotorSliderJointData jd;
		if (readValue(joints.second, "bodyID1", jd.m_bodyID[0]) &&
			readValue(joints.second, "bodyID2", jd.m_bodyID[1]) &&
			readVector(joints.second, "position", jd.m_position) &&
			readVector(joints.second, "axis", jd.m_axis) &&
			readValue(joints.second, "target", jd.m_target))
		{
			sceneData.m_targetVelocityMotorSliderJointData.push_back(jd);
		}
	}
}