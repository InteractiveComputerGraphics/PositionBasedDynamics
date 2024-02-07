#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeStepController.h"
#include <iostream>
#include "Demos/Visualization/Visualization.h"
#include "Simulation/DistanceFieldCollisionDetection.h"
#include "Utils/SceneLoader.h"
#include "Utils/TetGenLoader.h"
#include "Simulation/CubicSDFCollisionDetection.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
#include "Demos/Common/DemoBase.h"
#include "Simulation/Simulation.h"

#define _USE_MATH_DEFINES
#include "math.h"

// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
	#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

void timeStep ();
void buildModel ();
void readScene(const bool readFile);
void render ();
void reset();


DemoBase *base;
Vector3r camPos;
Vector3r camLookat;
CubicSDFCollisionDetection *cd;


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	std::string exePath = FileSystem::getProgramPath();

	std::string sceneFileName;
	if (argc > 1)
		sceneFileName = string(argv[1]);
	else
	{
		std::cerr << "Usage: SceneLoaderDemo [scene_file]\n";
		exit(1);
	}

	base = new DemoBase();
	base->init(argc, argv, sceneFileName.c_str());

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	cd = new CubicSDFCollisionDetection();
	cd->init();

	buildModel();

	base->createParameterGUI();

	// reset simulation when cloth simulation/bending method has changed
	model->setClothSimulationMethodChangedCallback([&]() { reset(); });
	model->setClothBendingMethodChangedCallback([&]() { reset(); });
	model->setSolidSimulationMethodChangedCallback([&]() { reset(); });

	base->readParameters();

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport(40.0f, 0.1f, 500.0f, camPos, camLookat);
	
	SimulationModel::TriangleModelVector &triModels = model->getTriangleModels();
	SimulationModel::TetModelVector &tetModels = model->getTetModels();
	MiniGL::mainLoop();

	base->cleanup();

	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	delete Simulation::getCurrent();
	delete base;
	delete model;
	delete cd;
	
	return 0;
}

void reset()
{
	const Real h = TimeManager::getCurrent()->getTimeStepSize();
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Simulation::getCurrent()->reset();
	base->getSelectedParticles().clear();

	Simulation::getCurrent()->getModel()->cleanup();
	Simulation::getCurrent()->getTimeStep()->getCollisionDetection()->cleanup();

	readScene(false);
	TimeManager::getCurrent()->setTimeStepSize(h);
}

void timeStep ()
{
	const Real pauseAt = base->getValue<Real>(DemoBase::PAUSE_AT);
	if ((pauseAt > 0.0) && (pauseAt < TimeManager::getCurrent()->getTime()))
		base->setValue(DemoBase::PAUSE, true);

	if (base->getValue<bool>(DemoBase::PAUSE))
		return;

	// Simulation code
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const unsigned int numSteps = base->getValue<unsigned int>(DemoBase::NUM_STEPS_PER_RENDER);
	for (unsigned int i = 0; i < numSteps; i++)
	{
		START_TIMING("SimStep");
		Simulation::getCurrent()->getTimeStep()->step(*model);
		STOP_TIMING_AVG;

		base->step();
	}

	// Update visualization models
	const ParticleData &pd = model->getParticles();
	for (unsigned int i = 0; i < model->getTetModels().size(); i++)
	{
		model->getTetModels()[i]->updateMeshNormals(pd);
		model->getTetModels()[i]->updateVisMesh(pd);
	}
	for (unsigned int i = 0; i < model->getTriangleModels().size(); i++)
	{
		model->getTriangleModels()[i]->updateMeshNormals(pd);
	}
	//base->setValue(DemoBase::PAUSE, true);
}

void render()
{
	base->render();
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.005));

	SimulationModel *model = Simulation::getCurrent()->getModel();
	Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, cd);

	readScene(true);
}

void initTriangleModelConstraints()
{
	// init constraints
	SimulationModel *model = Simulation::getCurrent()->getModel();
	for (unsigned int cm = 0; cm < model->getTriangleModels().size(); cm++)
	{
		model->setClothStiffness(1.0);
		if (model->getClothSimulationMethod() == 4)
			model->setClothStiffness(100000);
		model->addClothConstraints(model->getTriangleModels()[cm], model->getClothSimulationMethod(), model->getClothStiffness(), model->getClothStiffnessXX(),
			model->getClothStiffnessYY(), model->getClothStiffnessXY(), model->getClothPoissonRatioXY(), model->getClothPoissonRatioYX(),
			model->getClothNormalizeStretch(), model->getClothNormalizeShear());

		model->setClothBendingStiffness(0.01);
		if (model->getClothBendingMethod() == 3)
			model->setClothBendingStiffness(100.0);
		model->addBendingConstraints(model->getTriangleModels()[cm], model->getClothBendingMethod(), model->getClothBendingStiffness());
	}
}

void initTetModelConstraints()
{
	// init constraints
	SimulationModel *model = Simulation::getCurrent()->getModel();
	model->setSolidStiffness(1.0);
	if (model->getSolidSimulationMethod() == 3)
		model->setSolidStiffness(1000000);
	if (model->getSolidSimulationMethod() == 6)
		model->setSolidStiffness(100000);

	model->setSolidVolumeStiffness(1.0);
	if (model->getSolidSimulationMethod() == 6)
		model->setSolidVolumeStiffness(100000);
	for (unsigned int cm = 0; cm < model->getTetModels().size(); cm++)
	{
		model->addSolidConstraints(model->getTetModels()[cm], model->getSolidSimulationMethod(), model->getSolidStiffness(),
			model->getSolidPoissonRatio(), model->getSolidVolumeStiffness(), model->getSolidNormalizeStretch(), model->getSolidNormalizeShear());
	}
}

CubicSDFCollisionDetection::GridPtr generateSDF(const std::string &modelFile, const std::string &collisionObjectFileName, const Eigen::Matrix<unsigned int, 3, 1> &resolutionSDF, 
	VertexData &vd, IndexedFaceMesh &mesh)
{
	const std::string basePath = FileSystem::getFilePath(base->getSceneFile());
	const string cachePath = basePath + "/Cache";
	const std::string modelFileName = FileSystem::getFileNameWithExt(modelFile);
	CubicSDFCollisionDetection::GridPtr distanceField;
	
	if (collisionObjectFileName == "")
	{
		std::string md5FileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + ".md5");
		string md5Str = FileSystem::getFileMD5(modelFile);
		bool md5 = false;
		if (FileSystem::fileExists(md5FileName))
			md5 = FileSystem::checkMD5(md5Str, md5FileName);

		// check MD5 if cache file is available
		const string resStr = to_string(resolutionSDF[0]) + "_" + to_string(resolutionSDF[1]) + "_" + to_string(resolutionSDF[2]);
		const string sdfFileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + "_" + resStr + ".csdf");
		bool foundCacheFile = FileSystem::fileExists(sdfFileName);

		if (foundCacheFile && md5)
		{
			LOG_INFO << "Load cached SDF: " << sdfFileName;
			distanceField = std::make_shared<CubicSDFCollisionDetection::Grid>(sdfFileName);
		}
		else
		{
			std::vector<unsigned int> &faces = mesh.getFaces();
			const unsigned int nFaces = mesh.numFaces();

//#ifdef USE_DOUBLE
//			Discregrid::TriangleMesh sdfMesh(&vd.getPosition(0)[0], faces.data(), vd.size(), nFaces);
//#else
			// if type is float, copy vector to double vector
			std::vector<double> doubleVec;
			doubleVec.resize(3 * vd.size());
			for (unsigned int i = 0; i < vd.size(); i++)
				for (unsigned int j = 0; j < 3; j++)
					doubleVec[3 * i + j] = vd.getPosition(i)[j];
			Discregrid::TriangleMesh sdfMesh(&doubleVec[0], faces.data(), vd.size(), nFaces);
//#endif
			Discregrid::TriangleMeshDistance md(sdfMesh);
			Eigen::AlignedBox3d domain;
			for (auto const& x : sdfMesh.vertices())
			{
				domain.extend(x);
			}
			domain.max() += 0.1 * Eigen::Vector3d::Ones();
			domain.min() -= 0.1 * Eigen::Vector3d::Ones();

			LOG_INFO << "Set SDF resolution: " << resolutionSDF[0] << ", " << resolutionSDF[1] << ", " << resolutionSDF[2];
			distanceField = std::make_shared<CubicSDFCollisionDetection::Grid>(domain, std::array<unsigned int, 3>({ resolutionSDF[0], resolutionSDF[1], resolutionSDF[2] }));
			auto func = Discregrid::DiscreteGrid::ContinuousFunction{};
			func = [&md](Eigen::Vector3d const& xi) {return md.signed_distance(xi).distance; };
			LOG_INFO << "Generate SDF for " << modelFile;
			distanceField->addFunction(func, true);
			if (FileSystem::makeDir(cachePath) == 0)
			{
				LOG_INFO << "Save SDF: " << sdfFileName;
				distanceField->save(sdfFileName);
				FileSystem::writeMD5File(modelFile, md5FileName);
			}
		}
	}
	else
	{
		std::string fileName = collisionObjectFileName;
		if (FileSystem::isRelativePath(fileName))
		{
			fileName = FileSystem::normalizePath(basePath + "/" + fileName);
		}
		LOG_INFO << "Load SDF: " << fileName;
		distanceField = std::make_shared<CubicSDFCollisionDetection::Grid>(fileName);
	}
	return distanceField;
}


/** Create the rigid body model
*/
void readScene(const bool readFile)
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
	SimulationModel::TriangleModelVector &triModels = model->getTriangleModels();
	SimulationModel::TetModelVector &tetModels = model->getTetModels();
	SimulationModel::ConstraintVector &constraints = model->getConstraints();

	if (readFile)
	{
		base->cleanup();
		base->readScene();
	}
	SceneLoader::SceneData &data = base->getSceneData();

	camPos = data.m_camPosition;
	camLookat = data.m_camLookat;

	TimeManager::getCurrent()->setTimeStepSize(data.m_timeStepSize);

	//////////////////////////////////////////////////////////////////////////
	// rigid bodies
	//////////////////////////////////////////////////////////////////////////

	// map file names to loaded geometry to prevent multiple imports of same files
	std::map<std::string, pair<VertexData, IndexedFaceMesh>> objFiles;
	std::map<std::string, CubicSDFCollisionDetection::GridPtr> distanceFields;
	for (unsigned int i = 0; i < data.m_rigidBodyData.size(); i++)
	{
		SceneLoader::RigidBodyData &rbd = data.m_rigidBodyData[i];

		// Check if already loaded
		rbd.m_modelFile = FileSystem::normalizePath(rbd.m_modelFile);
		if (objFiles.find(rbd.m_modelFile) == objFiles.end())
		{
			IndexedFaceMesh mesh;
			VertexData vd;
			DemoBase::loadMesh(rbd.m_modelFile, vd, mesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
			objFiles[rbd.m_modelFile] = { vd, mesh };
		}

		const std::string basePath = FileSystem::getFilePath(base->getSceneFile());
		const string cachePath = basePath + "/Cache";
		const string resStr = to_string(rbd.m_resolutionSDF[0]) + "_" + to_string(rbd.m_resolutionSDF[1]) + "_" + to_string(rbd.m_resolutionSDF[2]);
		const std::string modelFileName = FileSystem::getFileNameWithExt(rbd.m_modelFile);
		const string sdfFileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + "_" + resStr + ".csdf");

		std::string sdfKey = rbd.m_collisionObjectFileName;
		if (sdfKey == "")
		{
			sdfKey = sdfFileName;
		}
		if (distanceFields.find(sdfKey) == distanceFields.end())
		{
			// Generate SDF
			if (rbd.m_collisionObjectType == SceneLoader::SDF)
			{
				if (rbd.m_collisionObjectFileName == "")
				{
					std::string md5FileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + ".md5");
					string md5Str = FileSystem::getFileMD5(rbd.m_modelFile);
					bool md5 = false;
					if (FileSystem::fileExists(md5FileName))
						md5 = FileSystem::checkMD5(md5Str, md5FileName);

					// check MD5 if cache file is available
					const string resStr = to_string(rbd.m_resolutionSDF[0]) + "_" + to_string(rbd.m_resolutionSDF[1]) + "_" + to_string(rbd.m_resolutionSDF[2]);
					const string sdfFileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + "_" + resStr + ".csdf");
					bool foundCacheFile = FileSystem::fileExists(sdfFileName);

					if (foundCacheFile && md5)
					{
						LOG_INFO << "Load cached SDF: " << sdfFileName;
						distanceFields[sdfFileName] = std::make_shared<CubicSDFCollisionDetection::Grid>(sdfFileName);
					}
					else
					{
						VertexData &vd = objFiles[rbd.m_modelFile].first;
						IndexedFaceMesh &mesh = objFiles[rbd.m_modelFile].second;

						std::vector<unsigned int> &faces = mesh.getFaces();
						const unsigned int nFaces = mesh.numFaces();

//#ifdef USE_DOUBLE
//						Discregrid::TriangleMesh sdfMesh(&vd.getPosition(0)[0], faces.data(), vd.size(), nFaces);
//#else
						// if type is float, copy vector to double vector
						std::vector<double> doubleVec;
						doubleVec.resize(3 * vd.size());
						for (unsigned int i = 0; i < vd.size(); i++)
							for (unsigned int j = 0; j < 3; j++)
								doubleVec[3 * i + j] = vd.getPosition(i)[j];
						Discregrid::TriangleMesh sdfMesh(&doubleVec[0], faces.data(), vd.size(), nFaces);
//#endif
						Discregrid::TriangleMeshDistance md(sdfMesh);
						Eigen::AlignedBox3d domain;
						for (auto const& x : sdfMesh.vertices())
						{
							domain.extend(x);
						}
						domain.max() += 0.1 * Eigen::Vector3d::Ones();
						domain.min() -= 0.1 * Eigen::Vector3d::Ones();

						LOG_INFO << "Set SDF resolution: " << rbd.m_resolutionSDF[0] << ", " << rbd.m_resolutionSDF[1] << ", " << rbd.m_resolutionSDF[2];
						distanceFields[sdfFileName] = std::make_shared<CubicSDFCollisionDetection::Grid>(domain, std::array<unsigned int, 3>({ rbd.m_resolutionSDF[0], rbd.m_resolutionSDF[1], rbd.m_resolutionSDF[2] }));
						auto func = Discregrid::DiscreteGrid::ContinuousFunction{};
						func = [&md](Eigen::Vector3d const& xi) {return md.signed_distance(xi).distance; };
						LOG_INFO << "Generate SDF for " << rbd.m_modelFile;
						distanceFields[sdfFileName]->addFunction(func, true);
						if (FileSystem::makeDir(cachePath) == 0)
						{
							LOG_INFO << "Save SDF: " << sdfFileName;
							distanceFields[sdfFileName]->save(sdfFileName);
							FileSystem::writeMD5File(rbd.m_modelFile, md5FileName);
						}
					}
				}
				else
				{
					std::string fileName = rbd.m_collisionObjectFileName;
					if (FileSystem::isRelativePath(fileName))
					{
						fileName = FileSystem::normalizePath(basePath + "/" + fileName);
					}
					LOG_INFO << "Load SDF: " << fileName;
					distanceFields[rbd.m_collisionObjectFileName] = std::make_shared<CubicSDFCollisionDetection::Grid>(fileName);
				}
			}
		}
	}

	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		const SceneLoader::TetModelData &tmd = data.m_tetModelData[i];

		// Check if already loaded
		if ((tmd.m_modelFileVis != "") &&
			(objFiles.find(tmd.m_modelFileVis) == objFiles.end()))
		{
			IndexedFaceMesh mesh;
			VertexData vd;
			DemoBase::loadMesh(FileSystem::normalizePath(tmd.m_modelFileVis), vd, mesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
			objFiles[tmd.m_modelFileVis] = { vd, mesh };
		}

		const std::string basePath = FileSystem::getFilePath(base->getSceneFile());
		const string cachePath = basePath + "/Cache";
		const string resStr = to_string(tmd.m_resolutionSDF[0]) + "_" + to_string(tmd.m_resolutionSDF[1]) + "_" + to_string(tmd.m_resolutionSDF[2]);
		const std::string modelFileName = FileSystem::getFileNameWithExt(tmd.m_modelFileVis);
		const string sdfFileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + "_" + resStr + ".csdf");

		std::string sdfKey = tmd.m_collisionObjectFileName;
		if (sdfKey == "")
		{
			sdfKey = sdfFileName;
		}
		if (distanceFields.find(sdfKey) == distanceFields.end())
		{
			// Generate SDF
			if (tmd.m_collisionObjectType == SceneLoader::SDF)
			{
				VertexData &vd = objFiles[tmd.m_modelFileVis].first;
				IndexedFaceMesh &mesh = objFiles[tmd.m_modelFileVis].second;

				CubicSDFCollisionDetection::GridPtr distanceField = generateSDF(tmd.m_modelFileVis, tmd.m_collisionObjectFileName, tmd.m_resolutionSDF, vd, mesh);

				if (tmd.m_collisionObjectFileName == "")
					distanceFields[sdfFileName] = distanceField;
				else
					distanceFields[tmd.m_collisionObjectFileName] = distanceField;
			}
		}
	}


	rb.resize(data.m_rigidBodyData.size());
	std::map<unsigned int, unsigned int> id_index;
	for (unsigned int i = 0; i < data.m_rigidBodyData.size(); i++)
	{
		const SceneLoader::RigidBodyData &rbd = data.m_rigidBodyData[i];

		if (objFiles.find(rbd.m_modelFile) == objFiles.end())
			continue;

		id_index[rbd.m_id] = i;

		VertexData &vd = objFiles[rbd.m_modelFile].first;
		IndexedFaceMesh &mesh = objFiles[rbd.m_modelFile].second;
		mesh.setFlatShading(rbd.m_flatShading);

		rb[i] = new RigidBody();
		rb[i]->initBody(rbd.m_density,
			rbd.m_x,
			rbd.m_q,
			vd, mesh,
			rbd.m_scale);

		if (!rbd.m_isDynamic)
			rb[i]->setMass(0.0);
		else
		{
			rb[i]->setVelocity(rbd.m_v);
			rb[i]->setAngularVelocity(rbd.m_omega);
		}
		rb[i]->setRestitutionCoeff(rbd.m_restitutionCoeff);
		rb[i]->setFrictionCoeff(rbd.m_frictionCoeff);

		const std::vector<Vector3r> &vertices = rb[i]->getGeometry().getVertexDataLocal().getVertices();
		const unsigned int nVert = static_cast<unsigned int>(vertices.size());

		switch (rbd.m_collisionObjectType)
		{
			case SceneLoader::No_Collision_Object: break;
			case SceneLoader::Sphere: 
				cd->addCollisionSphere(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, rbd.m_collisionObjectScale[0], rbd.m_testMesh, rbd.m_invertSDF);
				break;
			case SceneLoader::Box:
				cd->addCollisionBox(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, rbd.m_collisionObjectScale, rbd.m_testMesh, rbd.m_invertSDF);
				break;
			case SceneLoader::Cylinder:
				cd->addCollisionCylinder(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, rbd.m_collisionObjectScale.head<2>(), rbd.m_testMesh, rbd.m_invertSDF);
				break;
			case SceneLoader::Torus:
				cd->addCollisionTorus(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, rbd.m_collisionObjectScale.head<2>(), rbd.m_testMesh, rbd.m_invertSDF);
				break;
			case SceneLoader::HollowSphere:
				cd->addCollisionHollowSphere(i, PBD::CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, rbd.m_collisionObjectScale[0], rbd.m_thicknessSDF, rbd.m_testMesh, rbd.m_invertSDF);
				break;
			case SceneLoader::HollowBox:
				cd->addCollisionHollowBox(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, rbd.m_collisionObjectScale, rbd.m_thicknessSDF, rbd.m_testMesh, rbd.m_invertSDF);
				break;
			case SceneLoader::SDF:
			{	
				if (rbd.m_collisionObjectFileName == "")
				{
					const std::string basePath = FileSystem::getFilePath(base->getSceneFile());
					const string cachePath = basePath + "/Cache";
					const string resStr = to_string(rbd.m_resolutionSDF[0]) + "_" + to_string(rbd.m_resolutionSDF[1]) + "_" + to_string(rbd.m_resolutionSDF[2]);
					const std::string modelFileName = FileSystem::getFileNameWithExt(rbd.m_modelFile);
					const string sdfFileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + "_" + resStr + ".csdf");
					cd->addCubicSDFCollisionObject(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, distanceFields[sdfFileName], rbd.m_collisionObjectScale, rbd.m_testMesh, rbd.m_invertSDF);
				}
				else
					cd->addCubicSDFCollisionObject(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, distanceFields[rbd.m_collisionObjectFileName], rbd.m_collisionObjectScale, rbd.m_testMesh, rbd.m_invertSDF);
				break;
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// triangle models
	//////////////////////////////////////////////////////////////////////////

	// map file names to loaded geometry to prevent multiple imports of same files
	std::map<std::string, pair<VertexData, IndexedFaceMesh>> triFiles;
	for (unsigned int i = 0; i < data.m_triangleModelData.size(); i++)
	{
		const SceneLoader::TriangleModelData &tmd = data.m_triangleModelData[i];

		// Check if already loaded
		if (triFiles.find(tmd.m_modelFile) == triFiles.end())
		{
			IndexedFaceMesh mesh;
			VertexData vd;
			DemoBase::loadMesh(FileSystem::normalizePath(tmd.m_modelFile), vd, mesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
			triFiles[tmd.m_modelFile] = { vd, mesh };
		}
	}

	triModels.reserve(data.m_triangleModelData.size());
	std::map<unsigned int, unsigned int> tm_id_index;
	for (unsigned int i = 0; i < data.m_triangleModelData.size(); i++)
	{
		const SceneLoader::TriangleModelData &tmd = data.m_triangleModelData[i];

		if (triFiles.find(tmd.m_modelFile) == triFiles.end())
			continue;

		tm_id_index[tmd.m_id] = i;

		VertexData vd = triFiles[tmd.m_modelFile].first;
		IndexedFaceMesh &mesh = triFiles[tmd.m_modelFile].second;

		const Matrix3r R = tmd.m_q.matrix();
		for (unsigned int j = 0; j < vd.size(); j++)
		{
			vd.getPosition(j) = R * (vd.getPosition(j).cwiseProduct(tmd.m_scale)) + tmd.m_x;
		}

		model->addTriangleModel(vd.size(), mesh.numFaces(), &vd.getPosition(0), mesh.getFaces().data(), mesh.getUVIndices(), mesh.getUVs());

		TriangleModel *tm = triModels[triModels.size() - 1];
		ParticleData &pd = model->getParticles();
		unsigned int offset = tm->getIndexOffset();

		for (unsigned int j = 0; j < tmd.m_staticParticles.size(); j++)
		{
			const unsigned int index = tmd.m_staticParticles[j] + offset;
			pd.setMass(index, 0.0);
		}

		tm->setRestitutionCoeff(tmd.m_restitutionCoeff);
		tm->setFrictionCoeff(tmd.m_frictionCoeff);
	}

	initTriangleModelConstraints();

	//////////////////////////////////////////////////////////////////////////
	// tet models
	//////////////////////////////////////////////////////////////////////////

	// map file names to loaded geometry to prevent multiple imports of same files
	std::map<pair<string, string>, pair<vector<Vector3r>, vector<unsigned int>>> tetFiles;
	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		const SceneLoader::TetModelData &tmd = data.m_tetModelData[i];

		// Check if already loaded
		pair<string, string> fileNames = { tmd.m_modelFileNodes, tmd.m_modelFileElements };
		if (tetFiles.find(fileNames) == tetFiles.end())
		{
			vector<Vector3r> vertices;
			vector<unsigned int> tets;
			TetGenLoader::loadTetgenModel(FileSystem::normalizePath(tmd.m_modelFileNodes), FileSystem::normalizePath(tmd.m_modelFileElements), vertices, tets);
			tetFiles[fileNames] = { vertices, tets };
		}
	}

	tetModels.reserve(data.m_tetModelData.size());
	std::map<unsigned int, unsigned int> tm_id_index2;
	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		const SceneLoader::TetModelData &tmd = data.m_tetModelData[i];

		pair<string, string> fileNames = { tmd.m_modelFileNodes, tmd.m_modelFileElements };
		auto geo = tetFiles.find(fileNames);
		if (geo == tetFiles.end())
			continue;

		tm_id_index2[tmd.m_id] = i;

		vector<Vector3r> vertices = geo->second.first;
		vector<unsigned int> &tets = geo->second.second;

		const Matrix3r R = tmd.m_q.matrix();
		for (unsigned int j = 0; j < vertices.size(); j++)
		{
			vertices[j] = R * (vertices[j].cwiseProduct(tmd.m_scale)) + tmd.m_x;
		}

		model->addTetModel((unsigned int)vertices.size(), (unsigned int)tets.size() / 4, vertices.data(), tets.data());

		TetModel *tm = tetModels[tetModels.size() - 1];
		ParticleData &pd = model->getParticles();
		unsigned int offset = tm->getIndexOffset();

		tm->setInitialX(tmd.m_x);
		tm->setInitialR(R);
		tm->setInitialScale(tmd.m_scale);
	
		for (unsigned int j = 0; j < tmd.m_staticParticles.size(); j++)
		{
			const unsigned int index = tmd.m_staticParticles[j] + offset;
			pd.setMass(index, 0.0);
		}

		// read visualization mesh
		if (tmd.m_modelFileVis != "")
		{ 
			if (objFiles.find(tmd.m_modelFileVis) != objFiles.end())
			{
				IndexedFaceMesh &visMesh = tm->getVisMesh();
				VertexData &vdVis = tm->getVisVertices();
				vdVis = objFiles[tmd.m_modelFileVis].first;
				visMesh = objFiles[tmd.m_modelFileVis].second;

				for (unsigned int j = 0; j < vdVis.size(); j++)
					vdVis.getPosition(j) = R * (vdVis.getPosition(j).cwiseProduct(tmd.m_scale)) + tmd.m_x;

				tm->updateMeshNormals(pd);
				tm->attachVisMesh(pd);
				tm->updateVisMesh(pd);
			}
		}

		tm->setRestitutionCoeff(tmd.m_restitutionCoeff);
		tm->setFrictionCoeff(tmd.m_frictionCoeff);

		tm->updateMeshNormals(pd);
	}

	initTetModelConstraints();

	// init collision objects for deformable models
	ParticleData &pd = model->getParticles();
	for (unsigned int i = 0; i < data.m_triangleModelData.size(); i++)
	{
		TriangleModel *tm = triModels[i];
		unsigned int offset = tm->getIndexOffset();
		const unsigned int nVert = tm->getParticleMesh().numVertices();
		cd->addCollisionObjectWithoutGeometry(i, CollisionDetection::CollisionObject::TriangleModelCollisionObjectType, &pd.getPosition(offset), nVert, true);

	}
	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		TetModel *tm = tetModels[i];
		unsigned int offset = tm->getIndexOffset();
		const unsigned int nVert = tm->getParticleMesh().numVertices();
		const IndexedTetMesh &tetMesh = tm->getParticleMesh();

		const SceneLoader::TetModelData &tmd = data.m_tetModelData[i];

		switch (tmd.m_collisionObjectType)
		{
		case SceneLoader::No_Collision_Object: 
			cd->addCollisionObjectWithoutGeometry(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, true);
			break;
		case SceneLoader::Sphere:
			cd->addCollisionSphere(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, tmd.m_collisionObjectScale[0], tmd.m_testMesh, tmd.m_invertSDF);
			break;
		case SceneLoader::Box:
			cd->addCollisionBox(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, tmd.m_collisionObjectScale, tmd.m_testMesh, tmd.m_invertSDF);
			break;
		case SceneLoader::Cylinder:
			cd->addCollisionCylinder(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, tmd.m_collisionObjectScale.head<2>(), tmd.m_testMesh, tmd.m_invertSDF);
			break;
		case SceneLoader::Torus:
			cd->addCollisionTorus(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, tmd.m_collisionObjectScale.head<2>(), tmd.m_testMesh, tmd.m_invertSDF);
			break;
		case SceneLoader::HollowSphere:
			cd->addCollisionHollowSphere(i, PBD::CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, tmd.m_collisionObjectScale[0], tmd.m_thicknessSDF, tmd.m_testMesh, tmd.m_invertSDF);
			break;
		case SceneLoader::HollowBox:
			cd->addCollisionHollowBox(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, tmd.m_collisionObjectScale, tmd.m_thicknessSDF, tmd.m_testMesh, tmd.m_invertSDF);
			break;
		case SceneLoader::SDF:
		{
			if (tmd.m_collisionObjectFileName == "")
			{
				const std::string basePath = FileSystem::getFilePath(base->getSceneFile());
				const string cachePath = basePath + "/Cache";
				const string resStr = to_string(tmd.m_resolutionSDF[0]) + "_" + to_string(tmd.m_resolutionSDF[1]) + "_" + to_string(tmd.m_resolutionSDF[2]);
				const std::string modelFileName = FileSystem::getFileNameWithExt(tmd.m_modelFileVis);
				const string sdfFileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + "_" + resStr + ".csdf");
				cd->addCubicSDFCollisionObject(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, distanceFields[sdfFileName], tmd.m_collisionObjectScale, tmd.m_testMesh, tmd.m_invertSDF);
			}
			else
				cd->addCubicSDFCollisionObject(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, distanceFields[tmd.m_collisionObjectFileName], tmd.m_collisionObjectScale, tmd.m_testMesh, tmd.m_invertSDF);
			break;
		}
		}
	}

	// init tet BVH
	std::vector<CollisionDetection::CollisionObject*> &collisionObjects = cd->getCollisionObjects();
	for (unsigned int k = 0; k < collisionObjects.size(); k++)
	{
		if (cd->isDistanceFieldCollisionObject(collisionObjects[k]) &&
			(collisionObjects[k]->m_bodyType == CollisionDetection::CollisionObject::TetModelCollisionObjectType))
		{
			const unsigned int modelIndex = collisionObjects[k]->m_bodyIndex;
			TetModel *tm = tetModels[modelIndex];
			const unsigned int offset = tm->getIndexOffset();
			const IndexedTetMesh &mesh = tm->getParticleMesh();

			((DistanceFieldCollisionDetection::DistanceFieldCollisionObject*) collisionObjects[k])->initTetBVH(&pd.getPosition(offset), mesh.numVertices(), mesh.getTets().data(), mesh.numTets(), cd->getTolerance());
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// joints
	//////////////////////////////////////////////////////////////////////////

	for (unsigned int i = 0; i < data.m_ballJointData.size(); i++)
	{
		const SceneLoader::BallJointData &jd = data.m_ballJointData[i];
		model->addBallJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position);
	}

	for (unsigned int i = 0; i < data.m_ballOnLineJointData.size(); i++)
	{
		const SceneLoader::BallOnLineJointData &jd = data.m_ballOnLineJointData[i];
		model->addBallOnLineJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
	}

	for (unsigned int i = 0; i < data.m_hingeJointData.size(); i++)
	{
		const SceneLoader::HingeJointData &jd = data.m_hingeJointData[i];
		model->addHingeJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
	}

	for (unsigned int i = 0; i < data.m_universalJointData.size(); i++)
	{
		const SceneLoader::UniversalJointData &jd = data.m_universalJointData[i];
		model->addUniversalJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis[0], jd.m_axis[1]);
	}

	for (unsigned int i = 0; i < data.m_sliderJointData.size(); i++)
	{
		const SceneLoader::SliderJointData &jd = data.m_sliderJointData[i];
		model->addSliderJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_axis);
	}

	for (unsigned int i = 0; i < data.m_rigidBodyParticleBallJointData.size(); i++)
	{
		const SceneLoader::RigidBodyParticleBallJointData &jd = data.m_rigidBodyParticleBallJointData[i];
		model->addRigidBodyParticleBallJoint(id_index[jd.m_bodyID[0]], jd.m_bodyID[1]);
	}

	for (unsigned int i = 0; i < data.m_targetAngleMotorHingeJointData.size(); i++)
	{
		const SceneLoader::TargetAngleMotorHingeJointData &jd = data.m_targetAngleMotorHingeJointData[i];
		model->addTargetAngleMotorHingeJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((MotorJoint*)constraints[constraints.size() - 1])->setTarget(jd.m_target);
		((MotorJoint*)constraints[constraints.size() - 1])->setTargetSequence(jd.m_targetSequence);
		((MotorJoint*)constraints[constraints.size() - 1])->setRepeatSequence(jd.m_repeat);
	}

	for (unsigned int i = 0; i < data.m_targetVelocityMotorHingeJointData.size(); i++)
	{
		const SceneLoader::TargetVelocityMotorHingeJointData &jd = data.m_targetVelocityMotorHingeJointData[i];
		model->addTargetVelocityMotorHingeJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((MotorJoint*)constraints[constraints.size() - 1])->setTarget(jd.m_target);
		((MotorJoint*)constraints[constraints.size() - 1])->setTargetSequence(jd.m_targetSequence);
		((MotorJoint*)constraints[constraints.size() - 1])->setRepeatSequence(jd.m_repeat);
	}

	for (unsigned int i = 0; i < data.m_targetPositionMotorSliderJointData.size(); i++)
	{
		const SceneLoader::TargetPositionMotorSliderJointData &jd = data.m_targetPositionMotorSliderJointData[i];
		model->addTargetPositionMotorSliderJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_axis);
		((MotorJoint*)constraints[constraints.size() - 1])->setTarget(jd.m_target);
		((MotorJoint*)constraints[constraints.size() - 1])->setTargetSequence(jd.m_targetSequence);
		((MotorJoint*)constraints[constraints.size() - 1])->setRepeatSequence(jd.m_repeat);
	}

	for (unsigned int i = 0; i < data.m_targetVelocityMotorSliderJointData.size(); i++)
	{
		const SceneLoader::TargetVelocityMotorSliderJointData &jd = data.m_targetVelocityMotorSliderJointData[i];
		model->addTargetVelocityMotorSliderJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_axis);
		((MotorJoint*)constraints[constraints.size() - 1])->setTarget(jd.m_target);
		((MotorJoint*)constraints[constraints.size() - 1])->setTargetSequence(jd.m_targetSequence);
		((MotorJoint*)constraints[constraints.size() - 1])->setRepeatSequence(jd.m_repeat);
	}

	for (unsigned int i = 0; i < data.m_damperJointData.size(); i++)
	{
		const SceneLoader::DamperJointData &jd = data.m_damperJointData[i];
		model->addDamperJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_axis, jd.m_stiffness);
	}

	for (unsigned int i = 0; i < data.m_rigidBodySpringData.size(); i++)
	{
		const SceneLoader::RigidBodySpringData &jd = data.m_rigidBodySpringData[i];
		model->addRigidBodySpring(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position1, jd.m_position2, jd.m_stiffness);
	}

	for (unsigned int i = 0; i < data.m_distanceJointData.size(); i++)
	{
		const SceneLoader::DistanceJointData &jd = data.m_distanceJointData[i];
		model->addDistanceJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position1, jd.m_position2);
	}
}


