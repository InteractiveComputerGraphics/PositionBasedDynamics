#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "GenericConstraintsModel.h"
#include <iostream>
#include "Simulation/TimeStepController.h"
#include "Demos/Visualization/Visualization.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
#include "Demos/Common/DemoBase.h"
#include "Demos/Common/TweakBarParameters.h"
#include "Simulation/Simulation.h"

// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
	#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

void initParameters();
void timeStep ();
void buildModel ();
void createBodyModel();
void render ();
void reset();

DemoBase *base;

const Real width = static_cast<Real>(0.4);
const Real height = static_cast<Real>(0.4);
const Real depth = static_cast<Real>(2.0);


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Generic rigid body demo");

	GenericConstraintsModel *model = new GenericConstraintsModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	buildModel();

	initParameters();

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport(60.0, 0.1f, 500.0, Vector3r(2.0, 4.0, 15.0), Vector3r(2.0, -2.0, 0.0));

	MiniGL::mainLoop ();	

	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	delete Simulation::getCurrent();
	delete base;
	delete model;

	return 0;
}

void initParameters()
{
	TwRemoveAllVars(MiniGL::getTweakBar());
	TweakBarParameters::cleanup();

	MiniGL::initTweakBarParameters();

	TweakBarParameters::createParameterGUI();
	TweakBarParameters::createParameterObjectGUI(base);
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent());
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent()->getModel());
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent()->getTimeStep());
}

void reset()
{
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Simulation::getCurrent()->reset();
	base->getSelectedParticles().clear();
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
	}
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.005));

	createBodyModel();
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

void createBodyModel()
{
	GenericConstraintsModel *model = (GenericConstraintsModel*)Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();

	string fileName = FileSystem::normalizePath(base->getExePath() + "/resources/models/cube.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	DemoBase::loadMesh(fileName, vd, mesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r(width, height, depth));
	mesh.setFlatShading(true);
	IndexedFaceMesh meshStatic;
	VertexData vdStatic;
	DemoBase::loadMesh(fileName, vdStatic, meshStatic, Vector3r::Zero(), Matrix3r::Identity(), Vector3r(0.5, 0.5, 0.5));
	meshStatic.setFlatShading(true);

	// static body
	const unsigned int numberOfBodies = 6;
	rb.resize(numberOfBodies);
	Real startX = 0.0;
	Real startY = 1.0;
	for (unsigned int i = 0; i < 2; i++)
	{
		rb[3 * i] = new RigidBody();
		rb[3 * i]->initBody(0.0,
			Vector3r(startX, startY, 1.0),
			computeInertiaTensorBox(1.0, 0.5, 0.5, 0.5),
			Quaternionr(1.0, 0.0, 0.0, 0.0),
			vdStatic, meshStatic);

		// dynamic body
		rb[3 * i + 1] = new RigidBody();
		rb[3 * i + 1]->initBody(1.0,
			Vector3r(startX, startY - static_cast<Real>(0.25), static_cast<Real>(2.0)),
			computeInertiaTensorBox(1.0, width, height, depth),
			Quaternionr(1.0, 0.0, 0.0, 0.0),
			vd, mesh);

		// dynamic body
		rb[3 * i + 2] = new RigidBody();
		rb[3 * i + 2]->initBody(1.0,
			Vector3r(startX, startY - static_cast<Real>(0.25), static_cast<Real>(4.0)),
			computeInertiaTensorBox(1.0, width, height, depth),
			Quaternionr(1.0, 0.0, 0.0, 0.0),
			vd, mesh);

		startX += 4.0;

		if (i == 3)
		{
			startY -= 5.5;
			startX = 0.0;
		}
	}

	Real jointY = 0.75;
	model->addGenericHingeJoint(0, 1, Vector3r(0.0, jointY, 1.0), Vector3r(1.0, 0.0, 0.0));
	model->addGenericBallJoint(1, 2, Vector3r(0.25, jointY, 3.0));

	model->addGenericSliderJoint(3, 4, Vector3r(4.0, jointY, 1.0), Vector3r(1.0, 0.0, 0.0));
	model->addGenericBallJoint(4, 5, Vector3r(4.25, jointY, 3.0));
}


