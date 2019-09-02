#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeStepController.h"
#include <iostream>
#include "Utils/OBJLoader.h"
#include "Demos/Visualization/Visualization.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
#include "Demos/Common/DemoBase.h"
#include "Demos/Common/TweakBarParameters.h"
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

void initParameters();
void timeStep ();
void buildModel ();
void createBodyModel();
void render ();
void reset();

DemoBase *base;

const Real width = static_cast<Real>(0.4);
const Real height = static_cast<Real>(0.4);
const Real depth = static_cast<Real>(2.0);


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Rigid body demo");

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	buildModel ();

	initParameters();

	Simulation::getCurrent()->setSimulationMethodChangedCallback([&]() { reset(); initParameters(); base->getSceneLoader()->readParameterObject(Simulation::getCurrent()->getTimeStep()); });

	// OpenGL
	MiniGL::setClientIdleFunc(50, timeStep);
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setClientSceneFunc(render);
	MiniGL::setViewport(60.0, 0.1f, 500.0, Vector3r(6.0, 0.0, 18.0), Vector3r(6.0, -4.0, 0.0));

	glutMainLoop ();	

	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	delete Simulation::getCurrent();
	delete base;
	delete model;
	
	return 0;
}

void initParameters()
{
	TwRemoveAllVars(MiniGL::getTweakBar());
	TweakBarParameters::cleanup();

	MiniGL::initTweakBarParameters();

	TweakBarParameters::createParameterGUI();
	TweakBarParameters::createParameterObjectGUI(base);
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent());
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent()->getModel());
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent()->getTimeStep());
}

void reset()
{
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Simulation::getCurrent()->reset();
	base->getSelectedParticles().clear();
}

void timeStep()
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

		// set target angle of motors for an animation
		const Real currentTargetAngle = static_cast<Real>(M_PI * 0.5) - static_cast<Real>(M_PI * 0.5) * cos(static_cast<Real>(0.25)*TimeManager::getCurrent()->getTime());
		SimulationModel::ConstraintVector &constraints = model->getConstraints();
		TargetAngleMotorHingeJoint &joint1 = (*(TargetAngleMotorHingeJoint*)constraints[8]);
		TargetVelocityMotorHingeJoint &joint2 = (*(TargetVelocityMotorHingeJoint*)constraints[9]);
		joint1.setTarget(currentTargetAngle);
		joint2.setTarget(3.5);

		const Real currentTargetPos = static_cast<Real>(1.5)*sin(static_cast<Real>(2.0)*TimeManager::getCurrent()->getTime());
		TargetPositionMotorSliderJoint &joint3 = (*(TargetPositionMotorSliderJoint*)constraints[12]);
		joint3.setTarget(currentTargetPos);

		Real currentTargetVel = 0.25;
		if (((int)(0.25*TimeManager::getCurrent()->getTime())) % 2 == 1)
			currentTargetVel = -currentTargetVel;
		TargetVelocityMotorSliderJoint &joint4 = (*(TargetVelocityMotorSliderJoint*)constraints[14]);
		joint4.setTarget(currentTargetVel);
	}
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.005));

	createBodyModel();
}

void render ()
{
	base->render();

	float textColor[4] = { 0.0, .2f, .4f, 1 };
	MiniGL::drawStrokeText(-0.5, 1.5, 1.0, 0.002f, "ball joint", 11, textColor);
	MiniGL::drawStrokeText(3.0, 1.5, 1.0, 0.002f, "ball-on-line joint", 19, textColor);
	MiniGL::drawStrokeText(7.3f, 1.5, 1.0, 0.002f, "hinge joint", 12, textColor);
	MiniGL::drawStrokeText(11.2f, 1.5, 1.0, 0.002f, "universal joint", 15, textColor);

	MiniGL::drawStrokeText(-1.0, -4.0, 1.0, 0.002f, "motor hinge joint", 17, textColor);
	MiniGL::drawStrokeText(3.4f, -4.0, 1.0, 0.002f, "slider joint", 12, textColor);
	MiniGL::drawStrokeText(6.6f, -4.0, 1.0, 0.002f, "target position motor", 21, textColor);
	MiniGL::drawStrokeText(10.6f, -4.0, 1.0, 0.002f, "target velocity motor", 21, textColor);

	MiniGL::drawStrokeText(-0.25, -9.5, 1.0, 0.002f, "spring", 6, textColor);
	MiniGL::drawStrokeText(3.3, -9.5, 1.0, 0.002f, "distance joint", 14, textColor);

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
}

// Compute diagonal inertia tensor
Vector3r computeInertiaTensorBox(const Real mass, const Real width, const Real height, const Real depth)
{
	const Real Ix = (mass / static_cast<Real>(12.0)) * (height*height + depth*depth);
	const Real Iy = (mass / static_cast<Real>(12.0)) * (width*width + depth*depth);
	const Real Iz = (mass / static_cast<Real>(12.0)) * (width*width + height*height);
	return Vector3r(Ix, Iy, Iz);
}

void loadObj(const std::string &filename, VertexData &vd, IndexedFaceMesh &mesh, const Vector3r &scale)
{
	std::vector<OBJLoader::Vec3f> x;
	std::vector<OBJLoader::Vec3f> normals;
	std::vector<OBJLoader::Vec2f> texCoords;
	std::vector<MeshFaceIndices> faces;
	OBJLoader::Vec3f s = { (float)scale[0], (float)scale[1], (float)scale[2] };
	OBJLoader::loadObj(filename, &x, &faces, &normals, &texCoords, s);

	mesh.release();
	const unsigned int nPoints = (unsigned int)x.size();
	const unsigned int nFaces = (unsigned int)faces.size();
	const unsigned int nTexCoords = (unsigned int)texCoords.size();
	mesh.initMesh(nPoints, nFaces * 2, nFaces);
	vd.reserve(nPoints);
	for (unsigned int i = 0; i < nPoints; i++)
	{
		vd.addVertex(Vector3r(x[i][0], x[i][1], x[i][2]));
	}
	for (unsigned int i = 0; i < nTexCoords; i++)
	{
		mesh.addUV(texCoords[i][0], texCoords[i][1]);
	}
	for (unsigned int i = 0; i < nFaces; i++)
	{
		// Reduce the indices by one
		int posIndices[3];
		int texIndices[3];
		for (int j = 0; j < 3; j++)
		{
			posIndices[j] = faces[i].posIndices[j] - 1;
			if (nTexCoords > 0)
			{
				texIndices[j] = faces[i].texIndices[j] - 1;
				mesh.addUVIndex(texIndices[j]);
			}
		}

		mesh.addFace(&posIndices[0]);
	}
	mesh.buildNeighbors();

	mesh.updateNormals(vd, 0);
	mesh.updateVertexNormals(vd);

	LOG_INFO << "Number of triangles: " << nFaces;
	LOG_INFO << "Number of vertices: " << nPoints;
}

/** Create the rigid body model
*/
void createBodyModel()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();

	string fileName = FileSystem::normalizePath(base->getDataPath() + "/models/cube.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	loadObj(fileName, vd, mesh, Vector3r(width, height, depth));
	IndexedFaceMesh meshStatic;
	VertexData vdStatic;
	loadObj(fileName, vdStatic, meshStatic, Vector3r(0.5, 0.5, 0.5));

	// static body
	const unsigned int numberOfBodies = 30;
	rb.resize(numberOfBodies);
	Real startX = 0.0;
	Real startY = 6.5;
	for (unsigned int i = 0; i < 10; i++)
	{
		if (i % 4 == 0)
		{
			startY -= 5.5;
			startX = 0.0;
		}

		rb[3*i] = new RigidBody();
		rb[3*i]->initBody(0.0,
			Vector3r(startX, startY, 1.0),
			computeInertiaTensorBox(1.0, 0.5, 0.5, 0.5),
			Quaternionr(1.0, 0.0, 0.0, 0.0),
			vdStatic, meshStatic);

		// dynamic body
		rb[3*i+1] = new RigidBody();
		rb[3*i+1]->initBody(1.0,
			Vector3r(startX, startY- static_cast<Real>(0.25), static_cast<Real>(2.0)),
			computeInertiaTensorBox(1.0, width, height, depth),
			Quaternionr(1.0, 0.0, 0.0, 0.0),
			vd, mesh);

		// dynamic body
		rb[3 * i + 2] = new RigidBody();
		rb[3 * i + 2]->initBody(1.0,
			Vector3r(startX, startY - static_cast<Real>(0.25), static_cast<Real>(4.0)),
			computeInertiaTensorBox(1.0, width, height, depth),
			Quaternionr(1.0, 0.0, 0.0, 0.0),
			vd, mesh);
		
		startX += 4.0;
	}

	Real jointY = 0.75;
	model->addBallJoint(0, 1, Vector3r(0.25, jointY, 1.0));
	model->addBallJoint(1, 2, Vector3r(0.25, jointY, 3.0));
	
	model->addBallOnLineJoint(3, 4, Vector3r(4.25, jointY, 1.0), Vector3r(1.0, 0.0, 0.0));
	model->addBallJoint(4, 5, Vector3r(4.25, jointY, 3.0));
	
	model->addHingeJoint(6, 7, Vector3r(8.0, jointY, 1.0), Vector3r(1.0, 0.0, 0.0));
	model->addBallJoint(7, 8, Vector3r(8.25, jointY, 3.0));

	model->addUniversalJoint(9, 10, Vector3r(12.0, jointY, 1.0), Vector3r(1.0, 0.0, 0.0), Vector3r(0.0, 1.0, 0.0));
	model->addBallJoint(10, 11, Vector3r(12.25, jointY, 3.0));

	jointY -= 5.5;
	model->addTargetAngleMotorHingeJoint(12, 13, Vector3r(0.0, jointY, 1.0), Vector3r(1.0, 0.0, 0.0));
	model->addTargetVelocityMotorHingeJoint(13, 14, Vector3r(0.0, jointY, 3.0), Vector3r(0.0, 1.0, 0.0));
  
	model->addSliderJoint(15, 16, Vector3r(1.0, 0.0, 0.0));
	model->addBallJoint(16, 17, Vector3r(4.25, jointY, 3.0));

	model->addTargetPositionMotorSliderJoint(18, 19, Vector3r(1.0, 0.0, 0.0));
	model->addBallJoint(19, 20, Vector3r(8.25, jointY, 3.0));

	model->addTargetVelocityMotorSliderJoint(21, 22, Vector3r(1.0, 0.0, 0.0));
	model->addBallJoint(22, 23, Vector3r(12.25, jointY, 3.0));

	jointY -= 5.5;
	model->addRigidBodySpring(24, 25, Vector3r(0.25, jointY, 1.0), Vector3r(0.25, jointY, 1.0), 50.0);
	model->addBallJoint(25, 26, Vector3r(0.25, jointY, 3.0));

	model->addDistanceJoint(27, 28, Vector3r(4.25, jointY, 1.0), Vector3r(4.25, jointY, 2.0));
	model->addBallJoint(28, 29, Vector3r(4.25, jointY, 3.0));
}



