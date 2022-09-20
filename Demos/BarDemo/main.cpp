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

void timeStep ();
void buildModel ();
void createMesh();
void render ();
void reset();
void TW_CALL setSimulationMethod(const void *value, void *clientData);
void TW_CALL getSimulationMethod(void *value, void *clientData);
void TW_CALL setStiffness(const void* value, void* clientData);
void TW_CALL getStiffness(void* value, void* clientData);
void TW_CALL setPoissonRatio(const void* value, void* clientData);
void TW_CALL getPoissonRatio(void* value, void* clientData);
void TW_CALL setVolumeStiffness(const void* value, void* clientData);
void TW_CALL getVolumeStiffness(void* value, void* clientData);
void TW_CALL setNormalizeStretch(const void* value, void* clientData);
void TW_CALL getNormalizeStretch(void* value, void* clientData);
void TW_CALL setNormalizeShear(const void* value, void* clientData);
void TW_CALL getNormalizeShear(void* value, void* clientData);


DemoBase *base;
const unsigned int width = 30;
const unsigned int depth = 5;
const unsigned int height = 5; 
short simulationMethod = 2;
Real stiffness = 1.0;
Real poissonRatio = 0.3;
bool normalizeStretch = false;
bool normalizeShear = false;
Real volumeStiffness = 1.0;

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

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3r (5.0, 10.0, 30.0), Vector3r (5.0, 0.0, 0.0));

	TwType enumType2 = TwDefineEnum("SimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "SimulationMethod", enumType2, setSimulationMethod, getSimulationMethod, &simulationMethod,
			" label='Simulation method' enum='0 {None}, 1 {Volume constraints}, 2 {FEM based PBD}, 3 {FEM based XPBD}, \
			4 {Strain based dynamics (no inversion handling)}, 5 {Shape matching (no inversion handling)}, 6 {XPBD volume constraints}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "stiffness", TW_TYPE_REAL, setStiffness, getStiffness, model, " label='Stiffness'  min=0.0 step=0.1 precision=4 group='Solid' ");
	TwAddVarCB(MiniGL::getTweakBar(), "poissonRatio", TW_TYPE_REAL, setPoissonRatio, getPoissonRatio, model, " label='Poisson ratio'  min=0.0 step=0.1 precision=4 group='Solid' ");
	TwAddVarCB(MiniGL::getTweakBar(), "normalizeStretch", TW_TYPE_BOOL32, setNormalizeStretch, getNormalizeStretch, model, " label='Normalize stretch' group='Solid' ");
	TwAddVarCB(MiniGL::getTweakBar(), "normalizeShear", TW_TYPE_BOOL32, setNormalizeShear, getNormalizeShear, model, " label='Normalize shear' group='Solid' ");
	TwAddVarCB(MiniGL::getTweakBar(), "volumeStiffness", TW_TYPE_REAL, setVolumeStiffness, getVolumeStiffness, model, " label='Volume stiffness'  min=0.0 step=0.1 precision=4 group='Solid' ");
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
	stiffness = 1.0;
	if (simulationMethod == 3) 
		stiffness = 1000000;
	if (simulationMethod == 6)
		stiffness = 100000;

	volumeStiffness = 1.0;
	if (simulationMethod == 6)
		volumeStiffness = 100000;
	for (unsigned int cm = 0; cm < model->getTetModels().size(); cm++)
	{
		model->addSolidConstraints(model->getTetModels()[cm], simulationMethod, stiffness, 
			poissonRatio, volumeStiffness, normalizeStretch, normalizeShear);

		model->getTetModels()[cm]->updateMeshNormals(pd);

		LOG_INFO << "Number of tets: " << model->getTetModels()[cm]->getParticleMesh().numTets();
		LOG_INFO << "Number of vertices: " << width * height * depth;
	}
}

void TW_CALL setSimulationMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	*((short*)clientData) = val;
	reset();
}

void TW_CALL getSimulationMethod(void *value, void *clientData)
{
	*(short *)(value) = *((short*)clientData);
}

void TW_CALL setStiffness(const void* value, void* clientData)
{
	stiffness = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<FEMTetConstraint, Real, &FEMTetConstraint::m_stiffness>(stiffness);
	((SimulationModel*)clientData)->setConstraintValue<XPBD_FEMTetConstraint, Real, &XPBD_FEMTetConstraint::m_stiffness>(stiffness);
	((SimulationModel*)clientData)->setConstraintValue<StrainTetConstraint, Real, &StrainTetConstraint::m_stretchStiffness>(stiffness);
	((SimulationModel*)clientData)->setConstraintValue<StrainTetConstraint, Real, &StrainTetConstraint::m_shearStiffness>(stiffness);
	((SimulationModel*)clientData)->setConstraintValue<DistanceConstraint, Real, &DistanceConstraint::m_stiffness>(stiffness);
	((SimulationModel*)clientData)->setConstraintValue<DistanceConstraint_XPBD, Real, &DistanceConstraint_XPBD::m_stiffness>(stiffness);
	((SimulationModel*)clientData)->setConstraintValue<ShapeMatchingConstraint, Real, &ShapeMatchingConstraint::m_stiffness>(stiffness);
}

void TW_CALL getStiffness(void* value, void* clientData)
{
	*(Real*)(value) = stiffness;
}

void TW_CALL setVolumeStiffness(const void* value, void* clientData)
{
	volumeStiffness = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<VolumeConstraint, Real, &VolumeConstraint::m_stiffness>(volumeStiffness);
	((SimulationModel*)clientData)->setConstraintValue<VolumeConstraint_XPBD, Real, &VolumeConstraint_XPBD::m_stiffness>(volumeStiffness);
}

void TW_CALL getVolumeStiffness(void* value, void* clientData)
{
	*(Real*)(value) = volumeStiffness;
}

void TW_CALL getPoissonRatio(void* value, void* clientData)
{
	*(Real*)(value) = poissonRatio;
}

void TW_CALL setPoissonRatio(const void* value, void* clientData)
{
	poissonRatio = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<FEMTetConstraint, Real, &FEMTetConstraint::m_poissonRatio>(poissonRatio);
	((SimulationModel*)clientData)->setConstraintValue<XPBD_FEMTetConstraint, Real, &XPBD_FEMTetConstraint::m_poissonRatio>(poissonRatio);
}

void TW_CALL getNormalizeStretch(void* value, void* clientData)
{
	*(bool*)(value) = normalizeStretch;
}

void TW_CALL setNormalizeStretch(const void* value, void* clientData)
{
	normalizeStretch = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<StrainTetConstraint, bool, &StrainTetConstraint::m_normalizeStretch>(normalizeStretch);
}

void TW_CALL getNormalizeShear(void* value, void* clientData)
{
	*(bool*)(value) = normalizeShear;
}

void TW_CALL setNormalizeShear(const void* value, void* clientData)
{
	normalizeShear = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<StrainTetConstraint, bool, &StrainTetConstraint::m_normalizeShear>(normalizeShear);
}