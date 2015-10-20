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
void createBodyModel();
void render ();
void reset();
void cleanup();
void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData);
void TW_CALL getVelocityUpdateMethod(void *value, void *clientData);


SimulationModel model;
TimeStepController sim;

const int numberOfBodies = 10;
const float width = 1.0f;
const float height = 0.1f;
const float depth = 0.1f;
bool doPause = true;
std::vector<unsigned int> selectedBodies;
Eigen::Vector3f oldMousePos;

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
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");

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
 	std::vector<unsigned int> hits;
 	selectedBodies.clear();
 
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > x;
	x.resize(rb.size());
 	for (unsigned int i = 0; i < rb.size(); i++)
 	{
 		x[i] = rb[i]->getPosition();
 	}
 
 	Selection::selectRect(start, end, &x[0], &x[rb.size() - 1], selectedBodies);
 	if (selectedBodies.size() > 0)
 		MiniGL::setMouseMoveFunc(GLUT_MIDDLE_BUTTON, mouseMove);
 	else
 		MiniGL::setMouseMoveFunc(-1, NULL);
 
 	MiniGL::unproject(end[0], end[1], oldMousePos);
}

void timeStep ()
{
	if (doPause)
		return;

	// Simulation code
	for (unsigned int i = 0; i < 8; i++)
		sim.step(model);
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.005f);

	createBodyModel();
}

void renderBallJoint(BallJoint &bj)
{
	float jointColor[4] = { 0.0f, 0.6f, 0.2f, 1 };
	MiniGL::drawSphere(bj.m_jointInfo.col(2), 1.25f*height, jointColor);
}

void render ()
{
	MiniGL::coordinateSystem();
	
	// Draw sim model
	
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();

	float selectionColor[4] = { 0.8f, 0.0f, 0.0f, 1 };
	float surfaceColor[4] = { 0.1f, 0.4f, 0.8f, 1 };

	for (size_t i = 0; i < rb.size(); i++)
	{
		bool selected = false;
		for (unsigned int j = 0; j < selectedBodies.size(); j++)
		{
			if (selectedBodies[j] == i)
				selected = true;
		}

		if (!selected)
			MiniGL::drawCube(rb[i]->getPosition(), rb[i]->getRotationMatrix().transpose(), width, height, depth, surfaceColor);
		else
			MiniGL::drawCube(rb[i]->getPosition(), rb[i]->getRotationMatrix().transpose(), width, height, depth, selectionColor);
	}

	for (size_t i = 0; i < constraints.size(); i++)
	{
		if (constraints[i]->getTypeId() == BallJoint::TYPE_ID)
		{
			renderBallJoint(*(BallJoint*)constraints[i]);
		}
	}

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
}


/** Create the rigid body model
*/
void createBodyModel()
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();

	rb.resize(numberOfBodies);
	const float mass = 1.0f;
	for (unsigned int i = 0; i < numberOfBodies; i++)
	{	
		// Compute diagonal inertia tensor
		const float Ix = (mass / 12.0f) * (height*height + depth*depth);
		const float Iy = (mass / 12.0f) * (width*width + depth*depth);
		const float Iz = (mass / 12.0f) * (width*width + height*height);

		rb[i] = new RigidBody();
		rb[i]->initBody(1.0f,
			Eigen::Vector3f((float)i*width, 0.0f, 0.0f),
			Eigen::Vector3f(Ix, Iy, Iz),
			Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));
	}
	// Make first body static
	rb[0]->setMass(0.0f);

	constraints.reserve(numberOfBodies - 1);
	for (unsigned int i = 0; i < numberOfBodies-1; i++)
	{
		model.addBallJoint(i, i + 1, Eigen::Vector3f((float)i*width + 0.5f*width, 0.0f, 0.0f));
	}
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

