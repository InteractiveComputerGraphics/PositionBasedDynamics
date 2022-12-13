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

const int nRows = 50;
const int nCols = 50;
const Real width = 10.0;
const Real height = 10.0;
DemoBase *base;

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Cloth demo");

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

	for (unsigned int i = 0; i < model->getTriangleModels().size(); i++)
		model->getTriangleModels()[i]->updateMeshNormals(model->getParticles());
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


/** Create a particle model mesh 
*/
void createMesh()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	model->addRegularTriangleModel(nCols, nRows,  
		Vector3r(0,1,0), AngleAxisr(M_PI*0.5, Vector3r(1,0,0)).matrix(), Vector2r(width, height));
	
	// Set mass of points to zero => make it static
	ParticleData& pd = model->getParticles();
	pd.setMass(0, 0.0);
	pd.setMass(nRows-1, 0.0);

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

