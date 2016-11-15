#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Demos/Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Demos/Simulation/SimulationModel.h"
#include "Demos/Simulation/TimeStepController.h"
#include <iostream>
#include "Demos/Utils/OBJLoader.h"
#include "Demos/Visualization/Visualization.h"
#include "Demos/Utils/Utilities.h"
#include "Demos/Utils/Timing.h"

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
void createBodyModel();
void render ();
void reset();
void cleanup();
void initShader();
void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData);
void TW_CALL getVelocityUpdateMethod(void *value, void *clientData);
void TW_CALL setMaxIterations(const void *value, void *clientData);
void TW_CALL getMaxIterations(void *value, void *clientData);

SimulationModel model;
TimeStepController sim;

const Real width = 0.4;
const Real height = 0.4;
const Real depth = 2.0;
bool doPause = true;
std::vector<unsigned int> selectedBodies;
Vector3r oldMousePos;
Shader *shader;
string exePath;
string dataPath;
float jointColor[4] = { 0.0f, 0.4f, 0.2f, 1.0 };
float dynamicBodyColor[4] = { 0.1f, 0.4f, 0.8f, 1.0 };
float staticBodyColor[4] = { 0.4f, 0.4f, 0.4f, 1.0 };

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	exePath = Utilities::getFilePath(argv[0]);
	dataPath = exePath + "/" + std::string(PBD_DATA_PATH);

	// OpenGL
	MiniGL::init (argc, argv, 1280, 960, 0, 0, "Rigid body demo");
	MiniGL::initLights ();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection);
	initShader();

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (60.0, 0.1f, 500.0, Vector3r (6.0, 1.0, 15.0), Vector3r (6.0, -3.0, 0.0));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_REAL, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "MaxIter", TW_TYPE_UINT32, setMaxIterations, getMaxIterations, &sim, " label='Max. iterations'  min=1 step=1 group=Simulation ");

	glutMainLoop ();	

	cleanup ();

	Timing::printAverageTimes();
	
	return 0;
}

void initShader()
{
	std::string vertFile = dataPath + "/shaders/vs_flat.glsl";
	std::string geomFile = dataPath + "/shaders/gs_flat.glsl";
	std::string fragFile = dataPath + "/shaders/fs_flat.glsl";
	shader = MiniGL::createShader(vertFile, geomFile, fragFile);

	if (shader == NULL)
		return;

	shader->begin();
	shader->addUniform("modelview_matrix");
	shader->addUniform("projection_matrix");
	shader->addUniform("surface_color");
	shader->addUniform("shininess");
	shader->addUniform("specular_factor");
	shader->end();
}

void cleanup()
{
	delete TimeManager::getCurrent();
	delete shader;
}

void reset()
{
	Timing::printAverageTimes();
	Timing::reset();

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

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	for (size_t j = 0; j < selectedBodies.size(); j++)
	{
		rb[selectedBodies[j]]->getVelocity() += 1.0 / h * diff;
	}
	oldMousePos = mousePos;
}

void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end)
{
	std::vector<unsigned int> hits;
	selectedBodies.clear();
 
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	std::vector<Vector3r, Eigen::aligned_allocator<Vector3r> > x;
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
		const Real currentTargetAngle = (Real)M_PI * 0.5 - (Real)M_PI * 0.5 * cos(0.25*TimeManager::getCurrent()->getTime());
		SimulationModel::ConstraintVector &constraints = model.getConstraints();
		TargetAngleMotorHingeJoint &joint1 = (*(TargetAngleMotorHingeJoint*)constraints[8]);
		TargetVelocityMotorHingeJoint &joint2 = (*(TargetVelocityMotorHingeJoint*)constraints[9]);
		joint1.setTargetAngle(currentTargetAngle);
		joint2.setTargetAngularVelocity(3.5);

		const Real currentTargetPos = 1.5*sin(2.0*TimeManager::getCurrent()->getTime());
		TargetPositionMotorSliderJoint &joint3 = (*(TargetPositionMotorSliderJoint*)constraints[12]);
		joint3.setTargetPosition(currentTargetPos);

		Real currentTargetVel = 0.25;
		if (((int) (0.25*TimeManager::getCurrent()->getTime())) % 2 == 1)
			currentTargetVel = -currentTargetVel;
		TargetVelocityMotorSliderJoint &joint4 = (*(TargetVelocityMotorSliderJoint*)constraints[14]);
		joint4.setTargetVelocity(currentTargetVel);
	}
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.005);

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

	if (shader)
	{
		shader->begin();
		glUniform1f(shader->getUniform("shininess"), 5.0);
		glUniform1f(shader->getUniform("specular_factor"), 0.2f);

		GLfloat matrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
		glUniformMatrix4fv(shader->getUniform("modelview_matrix"), 1, GL_FALSE, matrix);
		GLfloat pmatrix[16];
		glGetFloatv(GL_PROJECTION_MATRIX, pmatrix);
		glUniformMatrix4fv(shader->getUniform("projection_matrix"), 1, GL_FALSE, pmatrix);
	}

	for (size_t i = 0; i < rb.size(); i++)
	{
		bool selected = false;
		for (unsigned int j = 0; j < selectedBodies.size(); j++)
		{
			if (selectedBodies[j] == i)
				selected = true;
		}

		const VertexData &vd = rb[i]->getGeometry().getVertexData();
		const IndexedFaceMesh &mesh = rb[i]->getGeometry().getMesh();
		if (rb[i]->getMass() == 0.0)
		{
			if (shader)
				glUniform3fv(shader->getUniform("surface_color"), 1, staticBodyColor);
			Visualization::drawMesh(vd, mesh, 0, staticBodyColor);
		}
		else
		{
			if (!selected)
			{
				if (shader)
					glUniform3fv(shader->getUniform("surface_color"), 1, dynamicBodyColor);
				Visualization::drawMesh(vd, mesh, 0, dynamicBodyColor);
			}
			else
			{
				if (shader)
					glUniform3fv(shader->getUniform("surface_color"), 1, selectionColor);
				Visualization::drawMesh(vd, mesh, 0, selectionColor);
			}
		}		
	}
	if (shader)
		shader->end();

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

	float textColor[4] = { 0.0, .2f, .4f, 1 };
	MiniGL::drawStrokeText(-0.5, 1.5, 1.0, 0.002f, "ball joint", 11, textColor);
	MiniGL::drawStrokeText(3.0, 1.5, 1.0, 0.002f, "ball-on-line joint", 19, textColor);
	MiniGL::drawStrokeText(7.3f, 1.5, 1.0, 0.002f, "hinge joint", 12, textColor);
	MiniGL::drawStrokeText(11.2f, 1.5, 1.0, 0.002f, "universal joint", 15, textColor);

	MiniGL::drawStrokeText(-1.0, -4.0, 1.0, 0.002f, "motor hinge joint", 17, textColor);
	MiniGL::drawStrokeText(3.4f, -4.0, 1.0, 0.002f, "slider joint", 12, textColor);
	MiniGL::drawStrokeText(6.6f, -4.0, 1.0, 0.002f, "target position motor", 21, textColor);
	MiniGL::drawStrokeText(10.6f, -4.0, 1.0, 0.002f, "target velocity motor", 21, textColor);

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
}

// Compute diagonal inertia tensor
Vector3r computeInertiaTensorBox(const Real mass, const Real width, const Real height, const Real depth)
{
	const Real Ix = (mass / 12.0) * (height*height + depth*depth);
	const Real Iy = (mass / 12.0) * (width*width + depth*depth);
	const Real Iz = (mass / 12.0) * (width*width + height*height);
	return Vector3r(Ix, Iy, Iz);
}


/** Create the rigid body model
*/
void createBodyModel()
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();

	string fileName = Utilities::normalizePath(dataPath + "/models/cube.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	OBJLoader::loadObj(fileName, vd, mesh, Vector3r(width, height, depth));
	IndexedFaceMesh meshStatic;
	VertexData vdStatic;
	OBJLoader::loadObj(fileName, vdStatic, meshStatic, Vector3r(0.5, 0.5, 0.5));

	// static body
	const unsigned int numberOfBodies = 24;
	rb.resize(numberOfBodies);
	Real startX = 0.0;
	Real startY = 1.0;
	for (unsigned int i = 0; i < 8; i++)
	{
		rb[3*i] = new RigidBody();
		rb[3*i]->initBody(0.0,
			Vector3r(startX, startY, 1.0),
			computeInertiaTensorBox(1.0, 0.5, 0.5, 0.5),
			Quaternionr(1.0, 0.0, 0.0, 0.0),
			vdStatic, meshStatic);

		// dynamic body
		rb[3*i+1] = new RigidBody();
		rb[3*i+1]->initBody(1.0,
			Vector3r(startX, startY-0.25, 2.0),
			computeInertiaTensorBox(1.0, width, height, depth),
			Quaternionr(1.0, 0.0, 0.0, 0.0),
			vd, mesh);

		// dynamic body
		rb[3 * i + 2] = new RigidBody();
		rb[3 * i + 2]->initBody(1.0,
			Vector3r(startX, startY - 0.25, 4.0),
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

// 	// create geometries
// 	for (unsigned int i = 0; i < numberOfBodies; i++)
// 	{
// 		if (rb[i]->getMass() != 0.0)
// 			rb[i]->getGeometry().initMesh(vd.size(), mesh.numFaces(), &vd.getPosition(0), mesh.getFaces().data(), mesh.getUVIndices(), mesh.getUVs());
// 		else
// 			rb[i]->getGeometry().initMesh(vdStatic.size(), meshStatic.numFaces(), &vdStatic.getPosition(0), meshStatic.getFaces().data(), meshStatic.getUVIndices(), meshStatic.getUVs());
// 		rb[i]->getGeometry().updateMeshTransformation(rb[i]->getPosition(), rb[i]->getRotationMatrix());
// 	}

	Real jointY = 0.75;
	model.addBallJoint(0, 1, Vector3r(0.25, jointY, 1.0));
	model.addBallJoint(1, 2, Vector3r(0.25, jointY, 3.0));
	
	model.addBallOnLineJoint(3, 4, Vector3r(4.25, jointY, 1.0), Vector3r(1.0, 0.0, 0.0));
	model.addBallJoint(4, 5, Vector3r(4.25, jointY, 3.0));
	
	model.addHingeJoint(6, 7, Vector3r(8.0, jointY, 1.0), Vector3r(1.0, 0.0, 0.0));
	model.addBallJoint(7, 8, Vector3r(8.25, jointY, 3.0));

	model.addUniversalJoint(9, 10, Vector3r(12.0, jointY, 1.0), Vector3r(1.0, 0.0, 0.0), Vector3r(0.0, 1.0, 0.0));
	model.addBallJoint(10, 11, Vector3r(12.25, jointY, 3.0));

	jointY -= 5.5;
	model.addTargetAngleMotorHingeJoint(12, 13, Vector3r(0.0, jointY, 1.0), Vector3r(1.0, 0.0, 0.0));
	model.addTargetVelocityMotorHingeJoint(13, 14, Vector3r(0.0, jointY, 3.0), Vector3r(0.0, 1.0, 0.0));
  
	model.addSliderJoint(15, 16, Vector3r(4.0, jointY, 1.0), Vector3r(1.0, 0.0, 0.0));
	model.addBallJoint(16, 17, Vector3r(4.25, jointY, 3.0));

	model.addTargetPositionMotorSliderJoint(18, 19, Vector3r(8.0, jointY, 1.0), Vector3r(1.0, 0.0, 0.0));
	model.addBallJoint(19, 20, Vector3r(8.25, jointY, 3.0));

	model.addTargetVelocityMotorSliderJoint(21, 22, Vector3r(12.0, jointY, 1.0), Vector3r(1.0, 0.0, 0.0));
	model.addBallJoint(22, 23, Vector3r(12.25, jointY, 3.0));
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

void TW_CALL setMaxIterations(const void *value, void *clientData)
{
	const unsigned int val = *(const unsigned int *)(value);
	((TimeStepController*)clientData)->setMaxIterations(val);
}

void TW_CALL getMaxIterations(void *value, void *clientData)
{
	*(unsigned int *)(value) = ((TimeStepController*)clientData)->getMaxIterations();
}
