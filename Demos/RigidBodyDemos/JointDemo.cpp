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
void TW_CALL setMaxIterations(const void *value, void *clientData);
void TW_CALL getMaxIterations(void *value, void *clientData);

SimulationModel model;
TimeStepController sim;

const float width = 0.4f;
const float height = 0.4f;
const float depth = 2.0f;
bool doPause = true;
std::vector<unsigned int> selectedBodies;
Eigen::Vector3f oldMousePos;
float jointColor[4] = { 0.0f, 0.4f, 0.2f, 1.0f };
float dynamicBodyColor[4] = { 0.1f, 0.4f, 0.8f, 1 };
float staticBodyColor[4] = { 0.4f, 0.4f, 0.4f, 1.0f };

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	// OpenGL
	MiniGL::init (argc, argv, 1280, 960, 0, 0, "Rigid body demo");
	MiniGL::initLights ();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection);

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (60.0f, 0.1f, 500.0f, Vector3f (6.0f, -5.5f, 15.0f), Vector3f (6.0f, -3.0f, 0.0f));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_FLOAT, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "MaxIter", TW_TYPE_UINT32, setMaxIterations, getMaxIterations, &sim, " label='Max. iterations'  min=1 step=1 group=Simulation ");

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
	{
		sim.step(model);

		// set target angle of motors for an animation
		const float currentTargetAngle = (float)M_PI * 0.5f - (float)M_PI * 0.5f * cos(0.25f*TimeManager::getCurrent()->getTime());
		SimulationModel::ConstraintVector &constraints = model.getConstraints();
		TargetAngleMotorHingeJoint &joint1 = (*(TargetAngleMotorHingeJoint*)constraints[8]);
		TargetVelocityMotorHingeJoint &joint2 = (*(TargetVelocityMotorHingeJoint*)constraints[9]);
		joint1.setTargetAngle(currentTargetAngle);
		joint2.setTargetAngularVelocity(3.5f);

		const float currentTargetPos = 1.5f*sin(2.0f*TimeManager::getCurrent()->getTime());
		TargetPositionMotorSliderJoint &joint3 = (*(TargetPositionMotorSliderJoint*)constraints[12]);
		joint3.setTargetPosition(currentTargetPos);

		float currentTargetVel = 0.25f;
		if (((int) (0.25f*TimeManager::getCurrent()->getTime())) % 2 == 1)
			currentTargetVel = -currentTargetVel;
		TargetVelocityMotorSliderJoint &joint4 = (*(TargetVelocityMotorSliderJoint*)constraints[14]);
		joint4.setTargetVelocity(currentTargetVel);
	}
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.005f);

	createBodyModel();
}

void renderBallJoint(BallJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(2), 0.1f, jointColor);
}

void renderBallOnLineJoint(BallOnLineJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(5), 0.1f, jointColor);
	MiniGL::drawCylinder(bj.m_jointInfo.col(5) - bj.m_jointInfo.col(7), bj.m_jointInfo.col(5) + bj.m_jointInfo.col(7), jointColor, 0.05f);
}

void renderHingeJoint(HingeJoint &hj)
{
	MiniGL::drawSphere(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawSphere(hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawCylinder(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), jointColor, 0.05f);
}

void renderUniversalJoint(UniversalJoint &uj)
{
	MiniGL::drawSphere(uj.m_jointInfo.col(4) - 0.5*uj.m_jointInfo.col(6), 0.1f, jointColor);
	MiniGL::drawSphere(uj.m_jointInfo.col(4) + 0.5*uj.m_jointInfo.col(6), 0.1f, jointColor);
	MiniGL::drawSphere(uj.m_jointInfo.col(5) - 0.5*uj.m_jointInfo.col(7), 0.1f, jointColor);
	MiniGL::drawSphere(uj.m_jointInfo.col(5) + 0.5*uj.m_jointInfo.col(7), 0.1f, jointColor);
	MiniGL::drawCylinder(uj.m_jointInfo.col(4) - 0.5*uj.m_jointInfo.col(6), uj.m_jointInfo.col(4) + 0.5*uj.m_jointInfo.col(6), jointColor, 0.05f);
	MiniGL::drawCylinder(uj.m_jointInfo.col(5) - 0.5*uj.m_jointInfo.col(7), uj.m_jointInfo.col(5) + 0.5*uj.m_jointInfo.col(7), jointColor, 0.05f);
}

void renderSliderJoint(SliderJoint &joint)
{
	MiniGL::drawSphere(joint.m_jointInfo.col(6), 0.1f, jointColor);
	MiniGL::drawCylinder(joint.m_jointInfo.col(7) - joint.m_jointInfo.col(8), joint.m_jointInfo.col(7) + joint.m_jointInfo.col(8), jointColor, 0.05f);
}

void renderTargetPositionMotorSliderJoint(TargetPositionMotorSliderJoint &joint)
{
	MiniGL::drawSphere(joint.m_jointInfo.col(6), 0.1f, jointColor);
	MiniGL::drawCylinder(joint.m_jointInfo.col(7) - joint.m_jointInfo.col(8), joint.m_jointInfo.col(7) + joint.m_jointInfo.col(8), jointColor, 0.05f);
}

void renderTargetVelocityMotorSliderJoint(TargetVelocityMotorSliderJoint &joint)
{
	MiniGL::drawSphere(joint.m_jointInfo.col(6), 0.1f, jointColor);
	MiniGL::drawCylinder(joint.m_jointInfo.col(7) - joint.m_jointInfo.col(8), joint.m_jointInfo.col(7) + joint.m_jointInfo.col(8), jointColor, 0.05f);
}

void renderTargetAngleMotorHingeJoint(TargetAngleMotorHingeJoint &hj)
{
	MiniGL::drawSphere(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawSphere(hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawCylinder(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), jointColor, 0.05f);
}

void renderTargetVelocityMotorHingeJoint(TargetVelocityMotorHingeJoint &hj)
{
	MiniGL::drawSphere(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawSphere(hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawCylinder(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), jointColor, 0.05f);
}

void render ()
{
	MiniGL::coordinateSystem();
	
	// Draw sim model
	
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();

	float selectionColor[4] = { 0.8f, 0.0f, 0.0f, 1 };	

	for (size_t i = 0; i < rb.size(); i++)
	{
		bool selected = false;
		for (unsigned int j = 0; j < selectedBodies.size(); j++)
		{
			if (selectedBodies[j] == i)
				selected = true;
		}

		if (rb[i]->getMass() == 0.0f)
			MiniGL::drawCube(rb[i]->getPosition(), rb[i]->getRotationMatrix().transpose(), 0.5f, 0.5f, 0.5f, staticBodyColor);
		else
		{
			if (!selected)
				MiniGL::drawCube(rb[i]->getPosition(), rb[i]->getRotationMatrix().transpose(), width, height, depth, dynamicBodyColor);
			else
				MiniGL::drawCube(rb[i]->getPosition(), rb[i]->getRotationMatrix().transpose(), width, height, depth, selectionColor);
		}
	}

	for (size_t i = 0; i < constraints.size(); i++)
	{
		if (constraints[i]->getTypeId() == BallJoint::TYPE_ID)
		{
			renderBallJoint(*(BallJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == BallOnLineJoint::TYPE_ID)
		{
			renderBallOnLineJoint(*(BallOnLineJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == HingeJoint::TYPE_ID)
		{
			renderHingeJoint(*(HingeJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == UniversalJoint::TYPE_ID)
		{
			renderUniversalJoint(*(UniversalJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == SliderJoint::TYPE_ID)
		{
			renderSliderJoint(*(SliderJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == TargetAngleMotorHingeJoint::TYPE_ID)
		{
			renderTargetAngleMotorHingeJoint(*(TargetAngleMotorHingeJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == TargetVelocityMotorHingeJoint::TYPE_ID)
		{
			renderTargetVelocityMotorHingeJoint(*(TargetVelocityMotorHingeJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == TargetPositionMotorSliderJoint::TYPE_ID)
		{
			renderTargetPositionMotorSliderJoint(*(TargetPositionMotorSliderJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == TargetVelocityMotorSliderJoint::TYPE_ID)
		{
			renderTargetVelocityMotorSliderJoint(*(TargetVelocityMotorSliderJoint*)constraints[i]);
		}
	}

	float textColor[4] = { 0.0f, .2f, .4f, 1 };
	MiniGL::drawStrokeText(-0.5f, 1.5f, 1.0f, 0.002f, "ball joint", 11, textColor);
	MiniGL::drawStrokeText(3.0f, 1.5f, 1.0f, 0.002f, "ball-on-line joint", 19, textColor);
	MiniGL::drawStrokeText(7.3f, 1.5f, 1.0f, 0.002f, "hinge joint", 12, textColor);
	MiniGL::drawStrokeText(11.2f, 1.5f, 1.0f, 0.002f, "universal joint", 15, textColor);

	MiniGL::drawStrokeText(-1.0f, -4.0f, 1.0f, 0.002f, "motor hinge joint", 17, textColor);
	MiniGL::drawStrokeText(3.4f, -4.0f, 1.0f, 0.002f, "slider joint", 12, textColor);
	MiniGL::drawStrokeText(6.6f, -4.0f, 1.0f, 0.002f, "target position motor", 21, textColor);
	MiniGL::drawStrokeText(10.6f, -4.0f, 1.0f, 0.002f, "target velocity motor", 21, textColor);

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
}

// Compute diagonal inertia tensor
Eigen::Vector3f computeInertiaTensorBox(const float mass, const float width, const float height, const float depth)
{
	const float Ix = (mass / 12.0f) * (height*height + depth*depth);
	const float Iy = (mass / 12.0f) * (width*width + depth*depth);
	const float Iz = (mass / 12.0f) * (width*width + height*height);
	return Eigen::Vector3f(Ix, Iy, Iz);
}


/** Create the rigid body model
*/
void createBodyModel()
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	// static body
	rb.resize(24);
	float startX = 0.0f;
	float startY = 1.0f;
	for (unsigned int i = 0; i < 8; i++)
	{
		rb[3*i] = new RigidBody();
		rb[3*i]->initBody(0.0f,
			Eigen::Vector3f(startX, startY, 1.0f),
			computeInertiaTensorBox(1.0f, 0.5f, 0.5f, 0.5f),
			Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

		// dynamic body
		rb[3*i+1] = new RigidBody();
		rb[3*i+1]->initBody(1.0f,
			Eigen::Vector3f(startX, startY-0.25f, 2.0f),
			computeInertiaTensorBox(1.0f, width, height, depth),
			Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

		// dynamic body
		rb[3 * i + 2] = new RigidBody();
		rb[3 * i + 2]->initBody(1.0f,
			Eigen::Vector3f(startX, startY - 0.25f, 4.0f),
			computeInertiaTensorBox(1.0f, width, height, depth),
			Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));
		
		startX += 4.0f;

		if (i == 3)
		{
			startY -= 5.5f;
			startX = 0.0f;
		}
	}

	float jointY = 0.75f;
	model.addBallJoint(0, 1, Eigen::Vector3f(0.25f, jointY, 1.0f));
	model.addBallJoint(1, 2, Eigen::Vector3f(0.25f, jointY, 3.0f));
	
	model.addBallOnLineJoint(3, 4, Eigen::Vector3f(4.25f, jointY, 1.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
	model.addBallJoint(4, 5, Eigen::Vector3f(4.25f, jointY, 3.0f));
	
	model.addHingeJoint(6, 7, Eigen::Vector3f(8.0f, jointY, 1.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
	model.addBallJoint(7, 8, Eigen::Vector3f(8.25f, jointY, 3.0f));

	model.addUniversalJoint(9, 10, Eigen::Vector3f(12.0f, jointY, 1.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
	model.addBallJoint(10, 11, Eigen::Vector3f(12.25f, jointY, 3.0f));

	jointY -= 5.5f;
	model.addTargetAngleMotorHingeJoint(12, 13, Eigen::Vector3f(0.0f, jointY, 1.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
	model.addTargetVelocityMotorHingeJoint(13, 14, Eigen::Vector3f(0.0f, jointY, 3.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
  
	model.addSliderJoint(15, 16, Eigen::Vector3f(4.0f, jointY, 1.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
	model.addBallJoint(16, 17, Eigen::Vector3f(4.25f, jointY, 3.0f));

	model.addTargetPositionMotorSliderJoint(18, 19, Eigen::Vector3f(8.0f, jointY, 1.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
	model.addBallJoint(19, 20, Eigen::Vector3f(8.25f, jointY, 3.0f));

	model.addTargetVelocityMotorSliderJoint(21, 22, Eigen::Vector3f(12.0f, jointY, 1.0f), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
	model.addBallJoint(22, 23, Eigen::Vector3f(12.25f, jointY, 3.0f));
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

void TW_CALL setMaxIterations(const void *value, void *clientData)
{
	const unsigned int val = *(const unsigned int *)(value);
	((TimeStepController*)clientData)->setMaxIterations(val);
}

void TW_CALL getMaxIterations(void *value, void *clientData)
{
	*(unsigned int *)(value) = ((TimeStepController*)clientData)->getMaxIterations();
}
