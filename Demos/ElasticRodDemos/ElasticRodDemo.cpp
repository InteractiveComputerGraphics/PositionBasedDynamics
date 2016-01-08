#include "Demos/Utils/Config.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Demos/Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Demos/Simulation/SimulationModel.h"
#include "Demos/Simulation/TimeStepController.h"
#include <iostream>

#define _USE_MATH_DEFINES
#include "math.h"

// Enable memory leak detection
#ifdef _DEBUG
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

void TW_CALL setBendAndTwistStiffness(const void *value, void *clientData);
void TW_CALL getBendAndTwistStiffness(void *value, void *clientData);

void TW_CALL setRestDarbouxX(const void *value, void *clientData);
void TW_CALL setRestDarbouxY(const void *value, void *clientData);
void TW_CALL setRestDarbouxZ(const void *value, void *clientData);

void TW_CALL getRestDarbouxX(void *value, void *clientData);
void TW_CALL getRestDarbouxY(void *value, void *clientData);
void TW_CALL getRestDarbouxZ(void *value, void *clientData);

void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData);
void TW_CALL getVelocityUpdateMethod(void *value, void *clientData);


SimulationModel model;
TimeStepController sim;

const int numberOfPoints = 32;
bool doPause = false;
std::vector<unsigned int> selectedBodies;
Eigen::Vector3f oldMousePos;
Eigen::Vector3f restDarboux;
// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	// OpenGL
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Rigid body demo");
	MiniGL::initLights ();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection);

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3f (5.0, -10.0, 30.0), Vector3f (5.0, 0.0, 0.0));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_FLOAT, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation ");

	TwAddVarCB(MiniGL::getTweakBar(), "BendAndTwistKs", TW_TYPE_FLOAT, setBendAndTwistStiffness, getBendAndTwistStiffness, &model, " label='Bend and Twist Stiffness'  min=0.0 max = 1.0 step=0.1 precision=2 group=BendTwist ");

	TwAddVarCB(MiniGL::getTweakBar(), "RestDarbouxX", TW_TYPE_FLOAT, setRestDarbouxX, getRestDarbouxX, &model, " label='Rest Darboux X'  min=-1.0 max = 1.0 step=0.01 precision=2 group=BendTwist ");
	TwAddVarCB(MiniGL::getTweakBar(), "RestDarbouxY", TW_TYPE_FLOAT, setRestDarbouxY, getRestDarbouxY, &model, " label='Rest Darboux Y'  min=-1.0 max = 1.0 step=0.01 precision=2 group=BendTwist ");
	TwAddVarCB(MiniGL::getTweakBar(), "RestDarbouxZ", TW_TYPE_FLOAT, setRestDarbouxZ, getRestDarbouxZ, &model, " label='Rest Darboux Z'  min=-1.0 max = 1.0 step=0.01 precision=2 group=BendTwist ");



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
	Eigen::Vector3f mousePos;
	MiniGL::unproject(x, y, mousePos);
	const Eigen::Vector3f diff = mousePos - oldMousePos;

	TimeManager *tm = TimeManager::getCurrent();
	const float h = tm->getTimeStepSize();

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	for (size_t j = 0; j < selectedBodies.size(); j++)
	{
		rb[selectedBodies[j]]->getVelocity() += 1.0f / h * diff;
	}
	oldMousePos = mousePos;
}

void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end)
{

 	MiniGL::unproject(end[0], end[1], oldMousePos);
}

void buildModel()
{

	TimeManager::getCurrent()->setTimeStepSize(0.002f);

	sim.setDamping(0.001f);

	model.setElasticRodBendAndTwistStiffness(0.5f);
	model.setElasticRodStretchStiffness(1.0f);
	restDarboux = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

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
	float pointColor[4] = { 0.1f, 0.1f, 0.5f, 1 };
	float ghostPointColor[4] = { 0.1f, 0.1f, 0.1f, 0.5f };
	float edgeColor[4] = { 0.0f, 0.6f, 0.2f, 1 };

	for (size_t i = 0; i < numberOfPoints; i++)
	{
		MiniGL::drawSphere(particles.getPosition(i), 0.2f, pointColor);
	}
	
	for (size_t i = 0; i < numberOfPoints-1; i++)
	{
		MiniGL::drawSphere(ghostParticles.getPosition(i), 0.1f, ghostPointColor);
		MiniGL::drawVector(particles.getPosition(i), particles.getPosition(i + 1), 0.2f, edgeColor);
	}

	for (size_t i = 0; i < constraints.size(); i++)
	{
		if (constraints[i]->getTypeId() == ElasticRodBendAndTwistConstraint::TYPE_ID)
		{
			((ElasticRodBendAndTwistConstraint*)constraints[i])->m_restDarbouxVector = restDarboux;
		}
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
		particles.addVertex(Eigen::Vector3f((float)i*1.0f, 0.0f, 0.0f));
	}

	//edge ghost points
	for (unsigned int i = 0; i < numberOfPoints-1; i++)
	{
		ghostParticles.addVertex(Eigen::Vector3f((float)i*1.0f + 0.5f, 1.0f, 0.0f));
	}

	//lock two first particles and first ghost point
	particles.setMass(0, 0.0f);
	particles.setMass(1, 0.0f);
	ghostParticles.setMass(0, 0.0f);

	for (unsigned int i = 0; i < numberOfPoints - 1; i++)
	{
		model.addElasticRodEdgeConstraint(i, i + 1, i);
		
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
			model.addElasticRodBendAndTwistConstraint(pA, pB, pC, pD, pE);
		}
	}
}



void TW_CALL setBendAndTwistStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	model.setElasticRodBendAndTwistStiffness(val);
}

void TW_CALL getBendAndTwistStiffness(void *value, void *clientData)
{
	*(float *)(value) = model.getElasticRodBendAndTwistStiffness();
}

void TW_CALL setRestDarbouxX(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	restDarboux[0] = val;
}

void TW_CALL setRestDarbouxY(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	restDarboux[1] = val;
}

void TW_CALL setRestDarbouxZ(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	restDarboux[2] = val;
}

void TW_CALL getRestDarbouxX(void *value, void *clientData)
{
	*(float *)(value) = restDarboux[0];
}

void TW_CALL getRestDarbouxY(void *value, void *clientData)
{
	*(float *)(value) = restDarboux[1];
}

void TW_CALL getRestDarbouxZ(void *value, void *clientData)
{
	*(float *)(value) = restDarboux[2];
}

void TW_CALL setTimeStep(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	TimeManager::getCurrent()->setTimeStepSize(val);
}

void TW_CALL getTimeStep(void *value, void *clientData)
{
	*(float *)(value) = TimeManager::getCurrent()->getTimeStepSize();
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

