#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeStepController.h"
#include <iostream>
#include "Simulation/Constraints.h"
#include "Demos/Visualization/Visualization.h"
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
void createRigidBodyModel();
void createClothMesh();
void render ();
void reset();

DemoBase *base;
const int nRows = 20;
const int nCols = 20;
const Real clothWidth = 10.0;
const Real clothHeight = 10.0;
const Real width = static_cast<Real>(0.2);
const Real height = static_cast<Real>(2.0);
const Real depth = static_cast<Real>(0.2);


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Rigid-cloth coupling demo");

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	buildModel();

	base->createParameterGUI();

	// reset simulation when cloth simulation/bending method has changed
	model->setClothSimulationMethodChangedCallback([&]() { reset(); });
	model->setClothBendingMethodChangedCallback([&]() { reset(); });

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);
	MiniGL::setViewport (40.0, 0.1f, 500.0, Vector3r (0.0, 10.0, 30.0), Vector3r (0.0, 0.0, 0.0));

	MiniGL::mainLoop();	

	base->cleanup();

	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	delete Simulation::getCurrent();
	delete base;
	delete model;

	return 0;
}

void reset()
{
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Simulation::getCurrent()->reset();
	base->getSelectedParticles().clear();

	Simulation::getCurrent()->getModel()->cleanup();

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

	for (unsigned int i = 0; i < model->getTriangleModels().size(); i++)
		model->getTriangleModels()[i]->updateMeshNormals(model->getParticles());
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.005));

	createClothMesh();
	createRigidBodyModel();	
}

void render ()
{
	base->render();
}

// Compute diagonal inertia tensor
Vector3r computeInertiaTensorBox(const Real mass, const Real width, const Real height, const Real depth)
{
	const Real Ix = (mass / static_cast<Real>(12.0)) * (height*height + depth*depth);
	const Real Iy = (mass / static_cast<Real>(12.0)) * (width*width + depth*depth);
	const Real Iz = (mass / static_cast<Real>(12.0)) * (width*width + height*height);
	return Vector3r(Ix, Iy, Iz);
}


/** Create the model
*/
void createRigidBodyModel()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
	
	string fileName = FileSystem::normalizePath(base->getExePath() + "/resources/models/cube.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	DemoBase::loadMesh(fileName, vd, mesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r(width, height, depth));
	mesh.setFlatShading(true);

	IndexedFaceMesh mesh_static;
	VertexData vd_static;
	DemoBase::loadMesh(fileName, vd_static, mesh_static, Vector3r::Zero(), Matrix3r::Identity(), Vector3r(0.5, 0.5, 0.5));
	mesh_static.setFlatShading(true);

	rb.resize(12);

	//////////////////////////////////////////////////////////////////////////
	// -5, -5
	//////////////////////////////////////////////////////////////////////////
	rb[0] = new RigidBody();
	rb[0]->initBody(0.0,
		Vector3r(-5.0, 0.0, -5.0),
		computeInertiaTensorBox(1.0, 0.5, 0.5, 0.5),
		Quaternionr(1.0, 0.0, 0.0, 0.0), 
		vd_static, mesh_static);

	// dynamic body
	rb[1] = new RigidBody();
	rb[1]->initBody(1.0,
		Vector3r(-5.0, 1.0, -5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	// dynamic body
	rb[2] = new RigidBody();
	rb[2]->initBody(1.0,
		Vector3r(-5.0, 3.0, -5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	model->addBallJoint(0, 1, Vector3r(-5.0, 0.0, -5.0));
	model->addBallJoint(1, 2, Vector3r(-5.0, 2.0, -5.0));

	//////////////////////////////////////////////////////////////////////////
	// 5, -5
	//////////////////////////////////////////////////////////////////////////
	rb[3] = new RigidBody();
	rb[3]->initBody(0.0,
		Vector3r(5.0, 0.0, -5.0),
		computeInertiaTensorBox(1.0, 0.5, 0.5, 0.5),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd_static, mesh_static);

	// dynamic body
	rb[4] = new RigidBody();
	rb[4]->initBody(1.0,
		Vector3r(5.0, 1.0, -5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	// dynamic body
	rb[5] = new RigidBody();
	rb[5]->initBody(1.0,
		Vector3r(5.0, 3.0, -5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	model->addBallJoint(3, 4, Vector3r(5.0, 0.0, -5.0));
	model->addBallJoint(4, 5, Vector3r(5.0, 2.0, -5.0));

	//////////////////////////////////////////////////////////////////////////
	// 5, 5
	//////////////////////////////////////////////////////////////////////////
	rb[6] = new RigidBody();
	rb[6]->initBody(0.0,
		Vector3r(5.0, 0.0, 5.0),
		computeInertiaTensorBox(1.0, 0.5, 0.5, 0.5),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd_static, mesh_static);

	// dynamic body
	rb[7] = new RigidBody();
	rb[7]->initBody(1.0,
		Vector3r(5.0, 1.0, 5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	// dynamic body
	rb[8] = new RigidBody();
	rb[8]->initBody(1.0,
		Vector3r(5.0, 3.0, 5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	model->addBallJoint(6, 7, Vector3r(5.0, 0.0, 5.0));
	model->addBallJoint(7, 8, Vector3r(5.0, 2.0, 5.0));

	//////////////////////////////////////////////////////////////////////////
	// -5, 5
	//////////////////////////////////////////////////////////////////////////
	rb[9] = new RigidBody();
	rb[9]->initBody(0.0,
		Vector3r(-5.0, 0.0, 5.0),
		computeInertiaTensorBox(1.0, 0.5, 0.5, 0.5),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd_static, mesh_static);

	// dynamic body
	rb[10] = new RigidBody();
	rb[10]->initBody(1.0,
		Vector3r(-5.0, 1.0, 5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	// dynamic body
	rb[11] = new RigidBody();
	rb[11]->initBody(1.0,
		Vector3r(-5.0, 3.0, 5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	model->addBallJoint(9, 10, Vector3r(-5.0, 0.0, 5.0));
	model->addBallJoint(10, 11, Vector3r(-5.0, 2.0, 5.0));
	

	model->addRigidBodyParticleBallJoint(2, 0);
	model->addRigidBodyParticleBallJoint(5, nCols - 1);
	model->addRigidBodyParticleBallJoint(8, nRows*nCols - 1);
	model->addRigidBodyParticleBallJoint(11, (nRows -1)*nCols);

}


/** Create a particle model mesh
*/
void createClothMesh()
{
	SimulationModel* model = Simulation::getCurrent()->getModel();
	model->addRegularTriangleModel(nCols, nRows,
		Vector3r(-5, 4, -5), AngleAxisr(M_PI * 0.5, Vector3r(1, 0, 0)).matrix(), Vector2r(clothWidth, clothHeight));


	// init constraints
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

	LOG_INFO << "Number of triangles: " << model->getTriangleModels()[0]->getParticleMesh().numFaces();
	LOG_INFO << "Number of vertices: " << nRows*nCols;

}
