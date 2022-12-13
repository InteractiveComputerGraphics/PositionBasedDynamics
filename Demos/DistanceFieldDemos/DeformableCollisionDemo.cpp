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
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
#include "Demos/Common/DemoBase.h"
#include "Simulation/Simulation.h"

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
void createMesh();
void render ();
void reset();

DemoBase *base;
DistanceFieldCollisionDetection *cd;

const unsigned int width = 30;
const unsigned int depth = 5;
const unsigned int height = 5; 

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Bar collision demo");

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	cd = new DistanceFieldCollisionDetection();
	cd->init();

	buildModel();

	base->createParameterGUI();

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3r (5.0, 10.0, 30.0), Vector3r (5.0, 0.0, 0.0));
	MiniGL::mainLoop ();	

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
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Simulation::getCurrent()->reset();
	base->getSelectedParticles().clear();

	Simulation::getCurrent()->getModel()->cleanup();
	Simulation::getCurrent()->getTimeStep()->getCollisionDetection()->cleanup();

	buildModel();
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

	for (unsigned int i = 0; i < model->getTetModels().size(); i++)
		model->getTetModels()[i]->updateMeshNormals(model->getParticles());
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.005));
	SimulationModel *model = Simulation::getCurrent()->getModel();

	createMesh();

	// create static rigid body
	string fileName = FileSystem::normalizePath(base->getExePath() + "/resources/models/cube.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	DemoBase::loadMesh(fileName, vd, mesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
	mesh.setFlatShading(true);

	string fileNameTorus = FileSystem::normalizePath(base->getExePath() + "/resources/models/torus.obj");
	IndexedFaceMesh meshTorus;
	VertexData vdTorus;
	DemoBase::loadMesh(fileNameTorus, vdTorus, meshTorus, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());

	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
	rb.resize(2);

	// floor
	rb[0] = new RigidBody();
	rb[0]->initBody(1.0,
		Vector3r(0.0, -5.5, 0.0),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh,
		Vector3r(100.0, 1.0, 100.0));
	rb[0]->setMass(0.0);

	// torus
	rb[1] = new RigidBody();
	rb[1]->initBody(1.0,
		Vector3r(5.0, -1.5, 0.0),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vdTorus, meshTorus,
		Vector3r(3.0, 3.0, 3.0));
	rb[1]->setMass(0.0);
	rb[1]->setFrictionCoeff(static_cast<Real>(0.1));

	Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, cd);
	cd->setTolerance(static_cast<Real>(0.05));
	
	const std::vector<Vector3r> &vertices1 = rb[0]->getGeometry().getVertexDataLocal().getVertices();
	const unsigned int nVert1 = static_cast<unsigned int>(vertices1.size());
	cd->addCollisionBox(0, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices1.data(), nVert1, Vector3r(100.0, 1.0, 100.0));

	const std::vector<Vector3r> &vertices2 = rb[1]->getGeometry().getVertexDataLocal().getVertices();
	const unsigned int nVert2 = static_cast<unsigned int>(vertices2.size());
	cd->addCollisionTorus(1, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices2.data(), nVert2, Vector2r(3.0, 1.5));

	SimulationModel::TetModelVector &tm = model->getTetModels();
	ParticleData &pd = model->getParticles();
	for (unsigned int i = 0; i < tm.size(); i++)
	{
		const unsigned int nVert = tm[i]->getParticleMesh().numVertices();
		unsigned int offset = tm[i]->getIndexOffset();
		tm[i]->setFrictionCoeff(static_cast<Real>(0.1));
		cd->addCollisionObjectWithoutGeometry(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, true);
	}
}


void render ()
{
	base->render();
}


void createMesh()
{
	SimulationModel* model = Simulation::getCurrent()->getModel();
	model->addRegularTetModel(width, height, depth,
		Vector3r(4.5, 3, 0), Matrix3r::Identity(), Vector3r(9.0, 1.5, 1.5));

	// init constraints
	model->setSolidStiffness(1.0);
	if (model->getSolidSimulationMethod() == 3)
		model->setSolidStiffness(1000000);
	if (model->getSolidSimulationMethod() == 6)
		model->setSolidStiffness(100000);

	model->setSolidVolumeStiffness(1.0);
	if (model->getSolidSimulationMethod() == 6)
		model->setSolidVolumeStiffness(100000);

	ParticleData& pd = model->getParticles();
	for (unsigned int cm = 0; cm < model->getTetModels().size(); cm++)
	{
		model->addSolidConstraints(model->getTetModels()[cm], model->getSolidSimulationMethod(), model->getSolidStiffness(),
			model->getSolidPoissonRatio(), model->getSolidVolumeStiffness(), model->getSolidNormalizeStretch(), model->getSolidNormalizeShear());

		model->getTetModels()[cm]->updateMeshNormals(pd);

		LOG_INFO << "Number of tets: " << model->getTetModels()[cm]->getParticleMesh().numTets();
		LOG_INFO << "Number of vertices: " << width * height * depth;
	}
}

