#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Demos/Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "PositionBasedElasticRodsModel.h"
#include "PositionBasedElasticRodsConstraints.h"
#include "PositionBasedElasticRodsTSC.h"
#include <iostream>

#define _USE_MATH_DEFINES
#include "math.h"

// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
	#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;

void timeStep ();
void buildModel ();
void createRod();
void render ();
void reset();
void cleanup();
void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end);

void TW_CALL setRestDarbouxX(const void *value, void *clientData);
void TW_CALL setRestDarbouxY(const void *value, void *clientData);
void TW_CALL setRestDarbouxZ(const void *value, void *clientData);

void TW_CALL getRestDarbouxX(void *value, void *clientData);
void TW_CALL getRestDarbouxY(void *value, void *clientData);
void TW_CALL getRestDarbouxZ(void *value, void *clientData);

void TW_CALL getBendingAndTwistingStiffnessX(void *value, void *clientData);
void TW_CALL getBendingAndTwistingStiffnessY(void *value, void *clientData);
void TW_CALL getBendingAndTwistingStiffnessZ(void *value, void *clientData);

void TW_CALL setBendingAndTwistingStiffnessX(const void *value, void *clientData);
void TW_CALL setBendingAndTwistingStiffnessY(const void *value, void *clientData);
void TW_CALL setBendingAndTwistingStiffnessZ(const void *value, void *clientData);

void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData);
void TW_CALL getVelocityUpdateMethod(void *value, void *clientData);


PositionBasedElasticRodsModel model;
PositionBasedElasticRodsTSC sim;

const int numberOfPoints = 32;
bool doPause = false;
std::vector<unsigned int> selectedParticles;
Vector3r oldMousePos;

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	// OpenGL
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Elastic rod demo");
	MiniGL::initLights ();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection);

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3r (5.0, -10.0, 30.0), Vector3r (5.0, 0.0, 0.0));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_REAL, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation ");

	TwAddVarCB(MiniGL::getTweakBar(), "RestDarbouxX", TW_TYPE_REAL, setRestDarbouxX, getRestDarbouxX, &model, " label='Rest Darboux X'  min=-1.0 max = 1.0 step=0.01 precision=2 group=BendTwist ");
	TwAddVarCB(MiniGL::getTweakBar(), "RestDarbouxY", TW_TYPE_REAL, setRestDarbouxY, getRestDarbouxY, &model, " label='Rest Darboux Y'  min=-1.0 max = 1.0 step=0.01 precision=2 group=BendTwist ");
	TwAddVarCB(MiniGL::getTweakBar(), "RestDarbouxZ", TW_TYPE_REAL, setRestDarbouxZ, getRestDarbouxZ, &model, " label='Rest Darboux Z'  min=-1.0 max = 1.0 step=0.01 precision=2 group=BendTwist ");

	TwAddVarCB(MiniGL::getTweakBar(), "BendingAndTwistingStiffnessX", TW_TYPE_REAL, setBendingAndTwistingStiffnessX, getBendingAndTwistingStiffnessX, &model, " label='Bending X stiffness'  min=0.01 max = 1.0 step=0.01 precision=2 group=BendTwist ");
	TwAddVarCB(MiniGL::getTweakBar(), "BendingAndTwistingStiffnessY", TW_TYPE_REAL, setBendingAndTwistingStiffnessY, getBendingAndTwistingStiffnessY, &model, " label='Bending Y stiffness'  min=0.01 max = 1.0 step=0.01 precision=2 group=BendTwist ");
	TwAddVarCB(MiniGL::getTweakBar(), "BendingAndTwistingStiffnessZ", TW_TYPE_REAL, setBendingAndTwistingStiffnessZ, getBendingAndTwistingStiffnessZ, &model, " label='Twisting stiffness'  min=0.01 max = 1.0 step=0.01 precision=2 group=BendTwist ");

	glutMainLoop ();	

	cleanup ();
	
	return 0;
}

void cleanup()
{
	delete TimeManager::getCurrent();
}

void reset()
{
	model.reset();
	sim.reset();
	TimeManager::getCurrent()->setTime(0.0);
}

void mouseMove(int x, int y)
{
	Vector3r mousePos;
	MiniGL::unproject(x, y, mousePos);
	const Vector3r diff = mousePos - oldMousePos;

	TimeManager *tm = TimeManager::getCurrent();
	const Real h = tm->getTimeStepSize();

	ParticleData &pd = model.getParticles();
	for (unsigned int j = 0; j < selectedParticles.size(); j++)
	{
		pd.getVelocity(selectedParticles[j]) += 5.0*diff / h;
	}
	oldMousePos = mousePos;
}

void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end)
{
	std::vector<unsigned int> hits;
	selectedParticles.clear();
	ParticleData &pd = model.getParticles();
	Selection::selectRect(start, end, &pd.getPosition(0), &pd.getPosition(pd.size() - 1), selectedParticles);
	if (selectedParticles.size() > 0)
		MiniGL::setMouseMoveFunc(GLUT_MIDDLE_BUTTON, mouseMove);
	else
		MiniGL::setMouseMoveFunc(-1, NULL);

	MiniGL::unproject(end[0], end[1], oldMousePos);
}

void buildModel()
{
	TimeManager::getCurrent()->setTimeStepSize(0.002f);
	model.setBendingAndTwistingStiffness(Vector3r(0.5, 0.5, 0.5));

	sim.setDamping(0.001f);

	createRod();
}

void timeStep ()
{
	if (doPause)
		return;

	// Simulation code
	for (unsigned int i = 0; i < 8; i++)
		sim.step(model);
}


void render()
{
	MiniGL::coordinateSystem();

	// Draw sim model

	ParticleData &particles = model.getParticles();
	ParticleData &ghostParticles = model.getGhostParticles();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();

	float selectionColor[4] = { 0.8f, 0.0f, 0.0f, 1 };
	float pointColor[4] = { 0.1f, 0.2f, 0.6f, 1 };
	float ghostPointColor[4] = { 0.1f, 0.1f, 0.1f, 0.5f };
	float edgeColor[4] = { 0.0f, 0.6f, 0.2f, 1 };

	float red[4] = { 0.8f, 0.0f, 0.0f, 1 };
	const ParticleData &pd = model.getParticles();
	for (unsigned int j = 0; j < selectedParticles.size(); j++)
	{
		MiniGL::drawSphere(pd.getPosition(selectedParticles[j]), 0.21f, red);
	}

	for (unsigned int i = 0; i < numberOfPoints; i++)
	{
		MiniGL::drawSphere(particles.getPosition(i), 0.2f, pointColor);
	}
	
	for (unsigned int i = 0; i < numberOfPoints-1; i++)
	{
		MiniGL::drawSphere(ghostParticles.getPosition(i), 0.1f, ghostPointColor);
		MiniGL::drawVector(particles.getPosition(i), particles.getPosition(i + 1), 0.2f, edgeColor);
	}

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
}


/** Create the elastic rod model
*/
void createRod()
{
	ParticleData &particles = model.getParticles();
	ParticleData &ghostParticles = model.getGhostParticles();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();

	//centreline points
	for (unsigned int i = 0; i < numberOfPoints; i++)
	{	
		particles.addVertex(Vector3r((Real)i*1.0, 0.0, 0.0));
	}

	//edge ghost points
	for (unsigned int i = 0; i < numberOfPoints-1; i++)
	{
		ghostParticles.addVertex(Vector3r((Real)i*1.0 + 0.5, 1.0, 0.0));
	}

	//lock two first particles and first ghost point
	particles.setMass(0, 0.0f);
	particles.setMass(1, 0.0f);
	ghostParticles.setMass(0, 0.0f);

	for (unsigned int i = 0; i < numberOfPoints - 1; i++)
	{
		model.addDistanceConstraint(i, i + 1);
		model.addPerpendiculaBisectorConstraint(i, i + 1, i);
		model.addGhostPointEdgeDistanceConstraint(i, i + 1, i);
		
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
			model.addDarbouxVectorConstraint(pA, pB, pC, pD, pE);
		}
	}
}


void TW_CALL setRestDarbouxX(const void *value, void *clientData)
{
	((PositionBasedElasticRodsModel*)clientData)->getRestDarbouxVector()[0] = *(const Real *)(value);
}

void TW_CALL setRestDarbouxY(const void *value, void *clientData)
{
	((PositionBasedElasticRodsModel*)clientData)->getRestDarbouxVector()[1] = *(const Real *)(value);
}

void TW_CALL setRestDarbouxZ(const void *value, void *clientData)
{
	((PositionBasedElasticRodsModel*)clientData)->getRestDarbouxVector()[2] = *(const Real *)(value);
}

void TW_CALL getRestDarbouxX(void *value, void *clientData)
{
	*(Real *)(value) = ((PositionBasedElasticRodsModel*)clientData)->getRestDarbouxVector()[0];
}

void TW_CALL getRestDarbouxY(void *value, void *clientData)
{
	*(Real *)(value) = ((PositionBasedElasticRodsModel*)clientData)->getRestDarbouxVector()[1];
}

void TW_CALL getRestDarbouxZ(void *value, void *clientData)
{
	*(Real *)(value) = ((PositionBasedElasticRodsModel*)clientData)->getRestDarbouxVector()[2];
}


void TW_CALL setBendingAndTwistingStiffnessX(const void *value, void *clientData)
{
	((PositionBasedElasticRodsModel*)clientData)->getBendingAndTwistingStiffness()[0] = *(const Real *)(value);
}

void TW_CALL setBendingAndTwistingStiffnessY(const void *value, void *clientData)
{
	((PositionBasedElasticRodsModel*)clientData)->getBendingAndTwistingStiffness()[1] = *(const Real *)(value);
}

void TW_CALL setBendingAndTwistingStiffnessZ(const void *value, void *clientData)
{
	((PositionBasedElasticRodsModel*)clientData)->getBendingAndTwistingStiffness()[2] = *(const Real *)(value);
}

void TW_CALL getBendingAndTwistingStiffnessX(void *value, void *clientData)
{
	*(Real *)(value) = ((PositionBasedElasticRodsModel*)clientData)->getBendingAndTwistingStiffness()[0];
}

void TW_CALL getBendingAndTwistingStiffnessY(void *value, void *clientData)
{
	*(Real *)(value) = ((PositionBasedElasticRodsModel*)clientData)->getBendingAndTwistingStiffness()[1];
}

void TW_CALL getBendingAndTwistingStiffnessZ(void *value, void *clientData)
{
	*(Real *)(value) = ((PositionBasedElasticRodsModel*)clientData)->getBendingAndTwistingStiffness()[2];
}

void TW_CALL setTimeStep(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	TimeManager::getCurrent()->setTimeStepSize(val);
}

void TW_CALL getTimeStep(void *value, void *clientData)
{
	*(Real *)(value) = TimeManager::getCurrent()->getTimeStepSize();
}

void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	((TimeStepController*)clientData)->setVelocityUpdateMethod((unsigned int)val);
}

void TW_CALL getVelocityUpdateMethod(void *value, void *clientData)
{
	*(short *)(value) = (short)((TimeStepController*)clientData)->getVelocityUpdateMethod();
}

