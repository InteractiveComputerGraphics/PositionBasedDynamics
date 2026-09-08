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
#include "Simulation/DistanceFieldCollisionDetection.h"

/*
* Trying to reproduce the results of plectonemes formation demonstrated in
* https://youtu.be/EFH9xt4omls?si=JgBwHavp-9jNoqIN&t=10
*/

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
void createRodModel();
void render ();
void reset();
void changeSolver();

DemoBase *base;
DistanceFieldCollisionDetection* cd;

const int numberOfBodies = 60;
const Real width = static_cast<Real>(0.5);
const Real height = static_cast<Real>(0.3);
const Real depth = static_cast<Real>(0.3);
const Real youngsModulus = static_cast<Real>(0.2e9);
const Real torsionModulus = static_cast<Real>(0.1e9);
const Real density = 780.;

Vector3r initialPos;
Quaternionr initialRot;
bool directSolver = true;


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Plectonemes (Stiff Rod) Demo");

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	cd = new DistanceFieldCollisionDetection();
	cd->init();	

	buildModel();

	base->createParameterGUI();

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::addKeyFunc('s', changeSolver);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0, 0.1f, 500.0, Vector3r (5.0, 10.0, 30.0), Vector3r (5.0, 0.0, 0.0));

	MiniGL::mainLoop();	

	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	delete Simulation::getCurrent();
	delete base;
	delete model;
	delete cd;

	return 0;
}

void reset()
{
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Simulation::getCurrent()->reset();
	base->getSelectedParticles().clear();

	Simulation::getCurrent()->getModel()->cleanup();
	Simulation::getCurrent()->getTimeStep()->getCollisionDetection()->cleanup();
	
	buildModel();

}

void changeSolver()
{
	directSolver = !directSolver;
	LOG_INFO << "using direct solver: " << directSolver;
	reset();
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
	
	auto rodLeftEdge = model->getRigidBodies()[model->getRigidBodies().size()-1];
	TimeManager* tm = TimeManager::getCurrent();
	
	const unsigned int numSteps = base->getValue<unsigned int>(DemoBase::NUM_STEPS_PER_RENDER);
	const float totalRodLength = width * numberOfBodies;
	for (unsigned int i = 0; i < numSteps; i++)
	{
		auto curOffset = tm->getTime()*2;
		if (curOffset < totalRodLength*0.6)
			rodLeftEdge->setPosition(initialPos + Vector3r(curOffset,0,0));

		const Real maxRotationAngle = M_PI * 10;
		auto angle = tm->getTime()*2;
		if (angle < maxRotationAngle)
		{
			AngleAxisr angleAxis(angle, Vector3r(0, 1, 0));
			rodLeftEdge->setRotation(initialRot * Quaternionr(angleAxis));
		}
			
		rodLeftEdge->setRotationMatrix(rodLeftEdge->getRotation().matrix());
						
		START_TIMING("SimStep");
		Simulation::getCurrent()->getTimeStep()->step(*model);
		STOP_TIMING_AVG;

		base->step();		
	}	
	
	rodLeftEdge->getGeometry().updateMeshTransformation(rodLeftEdge->getPosition(), rodLeftEdge->getRotationMatrix());
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.0004));
	Simulation::getCurrent()->getTimeStep()->setValue<unsigned int>(TimeStepController::NUM_SUB_STEPS, 1);
	cd->setValue<Real>(CollisionDetection::CONTACT_TOLERANCE, 0.1);	
	
	SimulationModel* model = Simulation::getCurrent()->getModel();
	Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, cd);
	model->setValue<Real>(SimulationModel::CONTACT_STIFFNESS_RB, 100);

	createRodModel();
}

void render ()
{
	base->render();
}

void createDirectSolverConstraints(SimulationModel* model, const unsigned int numConstraints, const int firstRbIndex)
{

	// create zero-stretch, bending and twisting constraints	
	std::vector<std::pair<unsigned int, unsigned int>>  constraintSegmentIndices;
	std::vector<Vector3r> constraintPositions;
	std::vector<Real> averageRadii;
	std::vector<Real> averageSegmentLengths;
	std::vector<Real> youngsModuli(numConstraints, youngsModulus);
	std::vector<Real> torsionModuli(numConstraints, torsionModulus);

	constraintSegmentIndices.reserve(numConstraints);
	constraintPositions.reserve(numConstraints);
	averageRadii.reserve(numConstraints);
	averageSegmentLengths.reserve(numConstraints);

	for (unsigned int cID = 0; cID < numConstraints; ++cID)
	{		
		constraintSegmentIndices.push_back(std::pair<unsigned int, unsigned int>(cID + firstRbIndex, cID + firstRbIndex + 1));
		auto rb0 = model->getRigidBodies()[cID + firstRbIndex];
		auto rb1 = model->getRigidBodies()[cID + firstRbIndex + 1];
		
		Vector3r x = (rb0->getPosition() + rb1->getPosition()) * 0.5f;
		constraintPositions.push_back(x);

		// compute average length
		Real avgLength = width;
		averageSegmentLengths.push_back(avgLength);

		// compute average radius
		averageRadii.push_back((height+depth)*0.25f);
	}

	model->addDirectPositionBasedSolverForStiffRodsConstraint(constraintSegmentIndices,
		constraintPositions, averageRadii, averageSegmentLengths, youngsModuli, torsionModuli);
}

/** Create the rod model
*/
void createRodModel()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model->getConstraints();

	string fileName = FileSystem::normalizePath(base->getExePath() + "/resources/models/cube.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	DemoBase::loadMesh(fileName, vd, mesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r(height, width, depth));
	mesh.setFlatShading(true);

	VertexData vdEdges;
	DemoBase::loadMesh(fileName, vdEdges, mesh, Vector3r::Zero(), Matrix3r::Identity(), Vector3r(height*4, width, depth*4));
	
	rb.resize(numberOfBodies);	
	for (unsigned int i = 0; i < numberOfBodies; i++)
	{			
		rb[i] = new RigidBody();

		Real mass(static_cast<Real>(0.25 * M_PI) * width * height * depth * density);

		const Real Iy = static_cast<Real>(1. / 12.) * mass*(static_cast<Real>(3.)*(static_cast<Real>(0.25)*height*height) + width*width);
		const Real Iz = static_cast<Real>(1. / 12.) * mass*(static_cast<Real>(3.)*(static_cast<Real>(0.25)*depth*depth) + width*width);
		const Real Ix = static_cast<Real>(0.25) * mass*(static_cast<Real>(0.25)*(height*height + depth*depth));
		Vector3r inertia(Iy, Ix, Iz); // rod axis along y-axis
		
		Real angle(static_cast<Real>(M_PI_2));
		Vector3r axis(0., 0., 1.);
		AngleAxisr angleAxis(angle, axis);
		Quaternionr rotation(angleAxis);

		VertexData& curVd = (i == 0 || i == numberOfBodies - 1) ? vdEdges : vd;

		rb[i]->initBody(mass,
			Vector3r((Real)i*width, 0.0, 0.0),
			inertia,
			rotation,
			curVd, mesh);

		const std::vector<Vector3r>& vertices = rb[i]->getGeometry().getVertexDataLocal().getVertices();
		const unsigned int nVert = static_cast<unsigned int>(vertices.size());
		cd->addCollisionCylinder(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, Vector2r(height, width));
	}
	// Make the last end point static
	rb[numberOfBodies-1]->setMass(0.0);
	// create a kinematic body connected to the first rod element
	// DirectPositionBasedSolverForStiffRods doesn't seem to work when both rod edges are not dynamic (i.e. have infinite mass)
	auto kinematic = new RigidBody();
	kinematic->initBody(0, rb[0]->getPosition(), rb[0]->getRotation(), vdEdges, mesh);
	rb.push_back(kinematic);
	model->addTargetAngleMotorHingeJoint(0, numberOfBodies, kinematic->getPosition(), Vector3r(1, 0, 0));

	initialPos = rb[0]->getPosition();
	initialRot = rb[0]->getRotation();

	// disable collision between adjacent elements
	for (unsigned int i = 0; i < numberOfBodies-1; ++i)	
		cd->addExclusionPair(i, i + 1);	

	if (directSolver)
	{
		createDirectSolverConstraints(model, numberOfBodies-1, 0);
	}
	else
	{
		constraints.reserve(numberOfBodies);
		for (unsigned int i = 0; i < numberOfBodies - 1; i++)
		{
			model->addStretchBendingTwistingConstraint(i, i + 1, Vector3r(static_cast<Real>(i) * width + static_cast<Real>(0.5) * width, 0.0, 0.0),
				0.25 * (height + depth), width, youngsModulus, torsionModulus);
		}
	}	
	
	Simulation::getCurrent()->getTimeStep()->setValue<unsigned int>(TimeStepController::MAX_ITERATIONS, 1);
}

