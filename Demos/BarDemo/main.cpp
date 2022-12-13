#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeStepController.h"
#include <iostream>
#include "Demos/Visualization/Visualization.h"
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
const unsigned int width = 30;
const unsigned int depth = 5;
const unsigned int height = 5; 


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Bar demo");

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	buildModel();

	base->createParameterGUI();

	// reset simulation when solid simulation method has changed
	model->setSolidSimulationMethodChangedCallback([&]() { reset(); });

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3r (5.0, 10.0, 30.0), Vector3r (5.0, 0.0, 0.0));
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

	for (unsigned int i = 0; i < model->getTetModels().size(); i++)
	{
		model->getTetModels()[i]->updateMeshNormals(model->getParticles());
	} 	
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.005));

	createMesh();
}


void render ()
{
	base->render();
}


void createMesh()
{
	SimulationModel* model = Simulation::getCurrent()->getModel();
	model->addRegularTetModel(width, height, depth,
		Vector3r(5, 0, 0), Matrix3r::Identity(), Vector3r(10.0, 1.5, 1.5));

	ParticleData& pd = model->getParticles();
	for (unsigned int i = 0; i < 1; i++)
	{
		for (unsigned int j = 0; j < height; j++)
		{
			for (unsigned int k = 0; k < depth; k++)
				pd.setMass(i*height*depth + j*depth + k, 0.0);
		}
	}

	// init constraints
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

		model->getTetModels()[cm]->updateMeshNormals(pd);

		LOG_INFO << "Number of tets: " << model->getTetModels()[cm]->getParticleMesh().numTets();
		LOG_INFO << "Number of vertices: " << width * height * depth;
	}
}

