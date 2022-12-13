#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "PositionBasedElasticRodsModel.h"
#include "PositionBasedElasticRodsConstraints.h"
#include "PositionBasedElasticRodsTSC.h"
#include <iostream>
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
void createRod();
void render ();
void reset();

DemoBase *base;
PositionBasedElasticRodsTSC sim;

const int numberOfPoints = 32;

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Elastic rod demo");

	PositionBasedElasticRodsModel *model = new PositionBasedElasticRodsModel();
	model->init();
	Simulation::getCurrent()->setModel(model);
	PositionBasedElasticRodsTSC *tsc = new PositionBasedElasticRodsTSC();
	tsc->init();
	delete Simulation::getCurrent()->getTimeStep();
	Simulation::getCurrent()->setTimeStep(tsc);

	buildModel();

	base->createParameterGUI();

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3r (5.0, 5.0, 10.0), Vector3r (5.0, 0.0, 0.0));
	MiniGL::mainLoop ();	

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
}

void buildModel()
{
	TimeManager::getCurrent()->setTimeStepSize(0.002f);
	PositionBasedElasticRodsModel *model = (PositionBasedElasticRodsModel*)Simulation::getCurrent()->getModel();
	model->setBendingAndTwistingStiffness(Vector3r(0.5, 0.5, 0.5));

	sim.setDamping(0.001f);

	createRod();
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
}


void render()
{
	// Draw sim model
	PositionBasedElasticRodsModel *model = (PositionBasedElasticRodsModel*) Simulation::getCurrent()->getModel();
	ParticleData &pd = model->getParticles();
	ParticleData &ghostParticles = model->getGhostParticles();
	SimulationModel::ConstraintVector &constraints = model->getConstraints();

	float selectionColor[4] = { 0.8f, 0.0f, 0.0f, 1 };
	float pointColor[4] = { 0.1f, 0.2f, 0.6f, 1 };
	float ghostPointColor[4] = { 0.1f, 0.1f, 0.1f, 0.5f };
	float edgeColor[4] = { 0.0f, 0.6f, 0.2f, 1 };

	for (unsigned int i = 0; i < numberOfPoints; i++)
	{
		MiniGL::drawSphere(pd.getPosition(i), 0.07f, pointColor);
	}
	
	for (unsigned int i = 0; i < numberOfPoints-1; i++)
	{
		MiniGL::drawSphere(ghostParticles.getPosition(i), 0.07f, ghostPointColor);
		MiniGL::drawVector(pd.getPosition(i), pd.getPosition(i + 1), 0.2f, edgeColor);
	}
	base->render();
}


/** Create the elastic rod model
*/
void createRod()
{
	PositionBasedElasticRodsModel *model = (PositionBasedElasticRodsModel*)Simulation::getCurrent()->getModel();
	ParticleData &particles = model->getParticles();
	ParticleData &ghostParticles = model->getGhostParticles();
	SimulationModel::ConstraintVector &constraints = model->getConstraints();

	//centreline points
	for (unsigned int i = 0; i < numberOfPoints; i++)
	{	
		particles.addVertex(Vector3r(static_cast<Real>(0.25 * i), 0.0, 0.0));
	}

	//edge ghost points
	for (unsigned int i = 0; i < numberOfPoints-1; i++)
	{
		ghostParticles.addVertex(Vector3r(static_cast<Real>(0.25 * i) + static_cast<Real>(0.125), static_cast<Real>(0.25), 0.0));
	}

	//lock two first particles and first ghost point
	particles.setMass(0, 0.0f);
	particles.setMass(1, 0.0f);
	ghostParticles.setMass(0, 0.0f);

	for (unsigned int i = 0; i < numberOfPoints - 1; i++)
	{
		model->addDistanceConstraint(i, i + 1, model->getRodStretchingStiffness());
		model->addPerpendiculaBisectorConstraint(i, i + 1, i);
		model->addGhostPointEdgeDistanceConstraint(i, i + 1, i);
		
		if (i < numberOfPoints - 2)
		{	
			//  Single rod element:
			//      D   E		//ghost points
			//		|	|
			//  --A---B---C--	// rod points
			int pA = i;
			int pB = i + 1;
			int pC = i + 2;
			int pD = i;
			int pE = i + 1;
			model->addDarbouxVectorConstraint(pA, pB, pC, pD, pE);
		}
	}
}
