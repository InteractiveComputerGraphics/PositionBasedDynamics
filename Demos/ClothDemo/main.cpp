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
void TW_CALL setBendingMethod(const void *value, void *clientData);
void TW_CALL getBendingMethod(void *value, void *clientData);
void TW_CALL setSimulationMethod(const void *value, void *clientData);
void TW_CALL getSimulationMethod(void *value, void *clientData);
void TW_CALL setBendingStiffness(const void* value, void* clientData);
void TW_CALL getBendingStiffness(void* value, void* clientData);
void TW_CALL setDistanceStiffness(const void* value, void* clientData);
void TW_CALL getDistanceStiffness(void* value, void* clientData);
void TW_CALL setXXStiffness(const void* value, void* clientData);
void TW_CALL getXXStiffness(void* value, void* clientData);
void TW_CALL setYYStiffness(const void* value, void* clientData);
void TW_CALL getYYStiffness(void* value, void* clientData);
void TW_CALL setXYStiffness(const void* value, void* clientData);
void TW_CALL getXYStiffness(void* value, void* clientData);
void TW_CALL setXYPoissonRatio(const void* value, void* clientData);
void TW_CALL getXYPoissonRatio(void* value, void* clientData);
void TW_CALL setYXPoissonRatio(const void* value, void* clientData);
void TW_CALL getYXPoissonRatio(void* value, void* clientData);
void TW_CALL setNormalizeStretch(const void* value, void* clientData);
void TW_CALL getNormalizeStretch(void* value, void* clientData);
void TW_CALL setNormalizeShear(const void* value, void* clientData);
void TW_CALL getNormalizeShear(void* value, void* clientData);


const int nRows = 50;
const int nCols = 50;
const Real width = 10.0;
const Real height = 10.0;
short simulationMethod = 2;
short bendingMethod = 2;
Real distanceStiffness = 1.0;
Real xxStiffness = 1.0;
Real yyStiffness = 1.0;
Real xyStiffness = 1.0;
Real xyPoissonRatio = 0.3;
Real yxPoissonRatio = 0.3;
bool normalizeStretch = false;
bool normalizeShear = false;
Real bendingStiffness = 0.01;
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

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3r (5.0, 10.0, 30.0), Vector3r (5.0, 0.0, 0.0));

	TwType enumType2 = TwDefineEnum("SimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "SimulationMethod", enumType2, setSimulationMethod, getSimulationMethod, &simulationMethod, 
		" label='Simulation method' enum='0 {None}, 1 {Distance constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics}, 4 {XPBD distance constraints}' group=Simulation");
	TwType enumType3 = TwDefineEnum("BendingMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "BendingMethod", enumType3, setBendingMethod, getBendingMethod, &bendingMethod, 
		" label='Bending method' enum='0 {None}, 1 {Dihedral angle}, 2 {Isometric bending}, 3 {XPBD isometric bending}' group=Bending");
	TwAddVarCB(MiniGL::getTweakBar(), "BendingStiffness", TW_TYPE_REAL, setBendingStiffness, getBendingStiffness, model, " label='Bending stiffness'  min=0.0 step=0.1 precision=4 group='Bending' ");
	TwAddVarCB(MiniGL::getTweakBar(), "DistanceStiffness", TW_TYPE_REAL, setDistanceStiffness, getDistanceStiffness, model, " label='Distance constraint stiffness'  min=0.0 step=0.1 precision=4 group='Cloth' ");
	TwAddVarCB(MiniGL::getTweakBar(), "xxStiffness", TW_TYPE_REAL, setXXStiffness, getXXStiffness, model, " label='xx stiffness'  min=0.0 step=0.1 precision=4 group='Cloth' ");
	TwAddVarCB(MiniGL::getTweakBar(), "yyStiffness", TW_TYPE_REAL, setYYStiffness, getYYStiffness, model, " label='yy stiffness'  min=0.0 step=0.1 precision=4 group='Cloth' ");
	TwAddVarCB(MiniGL::getTweakBar(), "xyStiffness", TW_TYPE_REAL, setXYStiffness, getXYStiffness, model, " label='xy stiffness'  min=0.0 step=0.1 precision=4 group='Cloth' ");
	TwAddVarCB(MiniGL::getTweakBar(), "xyPoissonRatio", TW_TYPE_REAL, setXYPoissonRatio, getXYPoissonRatio, model, " label='xy Poisson ratio'  min=0.0 step=0.1 precision=4 group='Cloth' ");
	TwAddVarCB(MiniGL::getTweakBar(), "yxPoissonRatio", TW_TYPE_REAL, setYXPoissonRatio, getYXPoissonRatio, model, " label='yx Poisson ratio'  min=0.0 step=0.1 precision=4 group='Cloth' ");
	TwAddVarCB(MiniGL::getTweakBar(), "normalizeStretch", TW_TYPE_BOOL32, setNormalizeStretch, getNormalizeStretch, model, " label='Normalize stretch' group='Cloth' ");
	TwAddVarCB(MiniGL::getTweakBar(), "normalizeShear", TW_TYPE_BOOL32, setNormalizeShear, getNormalizeShear, model, " label='Normalize shear' group='Cloth' ");

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
		distanceStiffness = 1.0;
		if (simulationMethod == 4)
			distanceStiffness = 100000;
		model->addClothConstraints(model->getTriangleModels()[cm], simulationMethod, distanceStiffness, xxStiffness, 
			yyStiffness, xyStiffness, xyPoissonRatio, yxPoissonRatio, normalizeStretch, normalizeShear);

		bendingStiffness = 0.01;
		if (bendingMethod == 3)
			bendingStiffness = 100.0;
		model->addBendingConstraints(model->getTriangleModels()[cm], bendingMethod, bendingStiffness);
	}

	LOG_INFO << "Number of triangles: " << model->getTriangleModels()[0]->getParticleMesh().numFaces();
	LOG_INFO << "Number of vertices: " << nRows*nCols;

}

void TW_CALL setBendingMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	*((short*)clientData) = val;
	reset();
}

void TW_CALL getBendingMethod(void *value, void *clientData)
{
	*(short *)(value) = *((short*)clientData);
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

void TW_CALL setBendingStiffness(const void* value, void* clientData)
{
	bendingStiffness = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<DihedralConstraint, Real, &DihedralConstraint::m_stiffness>(bendingStiffness);
	((SimulationModel*)clientData)->setConstraintValue<IsometricBendingConstraint, Real, &IsometricBendingConstraint::m_stiffness>(bendingStiffness);
	((SimulationModel*)clientData)->setConstraintValue<IsometricBendingConstraint_XPBD, Real, &IsometricBendingConstraint_XPBD::m_stiffness>(bendingStiffness);
}

void TW_CALL getBendingStiffness(void* value, void* clientData)
{
	*(Real*)(value) = bendingStiffness;
}

void TW_CALL setDistanceStiffness(const void* value, void* clientData)
{
	distanceStiffness = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<DistanceConstraint, Real, &DistanceConstraint::m_stiffness>(distanceStiffness);
	((SimulationModel*)clientData)->setConstraintValue<DistanceConstraint_XPBD, Real, &DistanceConstraint_XPBD::m_stiffness>(distanceStiffness);
}

void TW_CALL getDistanceStiffness(void* value, void* clientData)
{
	*(Real*)(value) = distanceStiffness;
}

void TW_CALL setXXStiffness(const void* value, void* clientData)
{
	xxStiffness = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<FEMTriangleConstraint, Real, &FEMTriangleConstraint::m_xxStiffness>(xxStiffness);
	((SimulationModel*)clientData)->setConstraintValue<StrainTriangleConstraint, Real, &StrainTriangleConstraint::m_xxStiffness>(xxStiffness);
}

void TW_CALL getXXStiffness(void* value, void* clientData)
{
	*(Real*)(value) = xxStiffness;
}

void TW_CALL getYYStiffness(void* value, void* clientData)
{
	*(Real*)(value) = yyStiffness;
}

void TW_CALL setYYStiffness(const void* value, void* clientData)
{
	yyStiffness = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<FEMTriangleConstraint, Real, &FEMTriangleConstraint::m_yyStiffness>(yyStiffness);
	((SimulationModel*)clientData)->setConstraintValue<StrainTriangleConstraint, Real, &StrainTriangleConstraint::m_yyStiffness>(yyStiffness);
}

void TW_CALL getXYStiffness(void* value, void* clientData)
{
	*(Real*)(value) = xyStiffness;
}

void TW_CALL setXYStiffness(const void* value, void* clientData)
{
	xyStiffness = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<FEMTriangleConstraint, Real, &FEMTriangleConstraint::m_xyStiffness>(xyStiffness);
	((SimulationModel*)clientData)->setConstraintValue<StrainTriangleConstraint, Real, &StrainTriangleConstraint::m_xyStiffness>(xyStiffness);
}

void TW_CALL getXYPoissonRatio(void* value, void* clientData)
{
	*(Real*)(value) = xyPoissonRatio;
}

void TW_CALL setXYPoissonRatio(const void* value, void* clientData)
{
	xyPoissonRatio = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<FEMTriangleConstraint, Real, &FEMTriangleConstraint::m_xyPoissonRatio>(xyPoissonRatio);
}

void TW_CALL getYXPoissonRatio(void* value, void* clientData)
{
	*(Real*)(value) = yxPoissonRatio;
}

void TW_CALL setYXPoissonRatio(const void* value, void* clientData)
{
	yxPoissonRatio = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<FEMTriangleConstraint, Real, &FEMTriangleConstraint::m_yxPoissonRatio>(yxPoissonRatio);
}

void TW_CALL getNormalizeStretch(void* value, void* clientData)
{
	*(bool*)(value) = normalizeStretch;
}

void TW_CALL setNormalizeStretch(const void* value, void* clientData)
{
	normalizeStretch = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<StrainTriangleConstraint, bool, &StrainTriangleConstraint::m_normalizeStretch>(normalizeStretch);
}

void TW_CALL getNormalizeShear(void* value, void* clientData)
{
	*(bool*)(value) = normalizeShear;
}

void TW_CALL setNormalizeShear(const void* value, void* clientData)
{
	normalizeShear= *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<StrainTriangleConstraint, bool, &StrainTriangleConstraint::m_normalizeShear>(normalizeShear);
}