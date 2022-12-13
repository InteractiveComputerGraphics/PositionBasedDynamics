#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeStepController.h"
#include <iostream>
#include "Demos/Visualization/Visualization.h"
#include "Simulation/DistanceFieldCollisionDetection.h"
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
void createBodyModel();
void render ();
void reset();

DemoBase *base;
DistanceFieldCollisionDetection *cd;


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Rigid body collision demo");

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
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0, Vector3r (5.0, 30.0, 70.0), Vector3r (5.0, 0.0, 0.0));
	MiniGL::mainLoop();

	base->cleanup();

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

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.005));

	SimulationModel *model = Simulation::getCurrent()->getModel();
	Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, cd);

	createBodyModel();
}

void render ()
{
	base->render();
}

/** Create the rigid body model
*/
void createBodyModel()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model->getConstraints();

	string fileNameBox = FileSystem::normalizePath(base->getExePath() + "/resources/models/cube.obj");
	IndexedFaceMesh meshBox;
	VertexData vdBox;
	DemoBase::loadMesh(fileNameBox, vdBox, meshBox, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
	meshBox.setFlatShading(true);

	string fileNameCylinder = FileSystem::normalizePath(base->getExePath() + "/resources/models/cylinder.obj");
	IndexedFaceMesh meshCylinder;
	VertexData vdCylinder;
	DemoBase::loadMesh(fileNameCylinder, vdCylinder, meshCylinder, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());

	string fileNameTorus = FileSystem::normalizePath(base->getExePath() + "/resources/models/torus.obj");
	IndexedFaceMesh meshTorus;
	VertexData vdTorus;
	DemoBase::loadMesh(fileNameTorus, vdTorus, meshTorus, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());

	string fileNameCube = FileSystem::normalizePath(base->getExePath() + "/resources/models/cube_5.obj");
	IndexedFaceMesh meshCube;
	VertexData vdCube;
	DemoBase::loadMesh(fileNameCube, vdCube, meshCube, Vector3r::Zero(), Matrix3r::Identity(), Vector3r::Ones());
	meshCube.setFlatShading(true);

	string fileNameSphere = FileSystem::normalizePath(base->getExePath() + "/resources/models/sphere.obj");
	IndexedFaceMesh meshSphere;
	VertexData vdSphere;
	DemoBase::loadMesh(fileNameSphere, vdSphere, meshSphere, Vector3r::Zero(), Matrix3r::Identity(), 2.0*Vector3r::Ones());


	const unsigned int num_piles_x = 5;
	const unsigned int num_piles_z = 5;
	const Real dx_piles = 4.0;
	const Real dz_piles = 4.0;
	const Real startx_piles = -0.5 * (Real)(num_piles_x - 1)*dx_piles;
	const Real startz_piles = -0.5 * (Real)(num_piles_z - 1)*dz_piles;
	const unsigned int num_piles = num_piles_x * num_piles_z;
	const unsigned int num_bodies_x = 3;
	const unsigned int num_bodies_y = 5;
	const unsigned int num_bodies_z = 3;
	const Real dx_bodies = 6.0;
	const Real dy_bodies = 6.0;
	const Real dz_bodies = 6.0;
	const Real startx_bodies = -0.5 * (Real)(num_bodies_x - 1)*dx_bodies;
	const Real starty_bodies = 14.0;
	const Real startz_bodies = -0.5 * (Real)(num_bodies_z - 1)*dz_bodies;
	const unsigned int num_bodies = num_bodies_x * num_bodies_y * num_bodies_z;
	rb.resize(num_piles + num_bodies + 1);
	unsigned int rbIndex = 0;

	// floor
	rb[rbIndex] = new RigidBody();
	rb[rbIndex]->initBody(1.0,
		Vector3r(0.0, -0.5, 0.0),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vdBox, meshBox, Vector3r(100.0, 1.0, 100.0));
	rb[rbIndex]->setMass(0.0);

	const std::vector<Vector3r> &vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
	const unsigned int nVert = static_cast<unsigned int>(vertices.size());

	cd->addCollisionBox(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, Vector3r(100.0, 1.0, 100.0));
	rbIndex++;

	Real current_z = startz_piles;
	for (unsigned int i = 0; i < num_piles_z; i++)
	{ 
		Real current_x = startx_piles;
		for (unsigned int j = 0; j < num_piles_x; j++)
		{
			rb[rbIndex] = new RigidBody();
			rb[rbIndex]->initBody(100.0,
				Vector3r(current_x, 5.0, current_z),
				Quaternionr(1.0, 0.0, 0.0, 0.0),
				vdCylinder, meshCylinder, 
				Vector3r(0.5, 10.0, 0.5));
			rb[rbIndex]->setMass(0.0);

			const std::vector<Vector3r> &vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
			const unsigned int nVert = static_cast<unsigned int>(vertices.size());
			cd->addCollisionCylinder(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, Vector2r(0.5, 10.0));
			current_x += dx_piles;
			rbIndex++;
		}
		current_z += dz_piles;
	}

	srand((unsigned int) time(NULL));

	Real current_y = starty_bodies;
	unsigned int currentType = 0;
	for (unsigned int i = 0; i < num_bodies_y; i++)
	{
		Real current_x = startx_bodies;
		for (unsigned int j = 0; j < num_bodies_x; j++)
		{
			Real current_z = startz_bodies;
			for (unsigned int k = 0; k < num_bodies_z; k++)
			{
				rb[rbIndex] = new RigidBody();

				Real ax = static_cast <Real> (rand()) / static_cast <Real> (RAND_MAX);
				Real ay = static_cast <Real> (rand()) / static_cast <Real> (RAND_MAX);
				Real az = static_cast <Real> (rand()) / static_cast <Real> (RAND_MAX);
				Real w = static_cast <Real> (rand()) / static_cast <Real> (RAND_MAX);
				Quaternionr q(w, ax, ay, az);
				q.normalize();

				currentType = rand() % 4;
				if (currentType == 0)
				{
					rb[rbIndex]->initBody(100.0,
						Vector3r(current_x, current_y, current_z),
						q, //Quaternionr(1.0, 0.0, 0.0, 0.0),
						vdTorus, meshTorus,
						Vector3r(2.0, 2.0, 2.0));
					
					const std::vector<Vector3r> &vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
					const unsigned int nVert = static_cast<unsigned int>(vertices.size());
					cd->addCollisionTorus(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, Vector2r(2.0, 1.0));
				}
				else if (currentType == 1)
				{
					rb[rbIndex]->initBody(100.0,
						Vector3r(current_x, current_y, current_z),
						q, //Quaternionr(1.0, 0.0, 0.0, 0.0),
						vdCube, meshCube, 
						Vector3r(4.0, 1.0, 1.0));

					const std::vector<Vector3r> &vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
					const unsigned int nVert = static_cast<unsigned int>(vertices.size());
					cd->addCollisionBox(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, Vector3r(4.0, 1.0, 1.0));
				}
				else if (currentType == 2)
				{
					rb[rbIndex]->initBody(100.0,
						Vector3r(current_x, current_y, current_z),
						q, //Quaternionr(1.0, 0.0, 0.0, 0.0),
						vdSphere, meshSphere);

					const std::vector<Vector3r> &vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
					const unsigned int nVert = static_cast<unsigned int>(vertices.size());
					cd->addCollisionSphere(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, 2.0);
				}
				else if (currentType == 3)
				{
					rb[rbIndex]->initBody(100.0,
						Vector3r(current_x, current_y, current_z),
						q, //Quaternionr(1.0, 0.0, 0.0, 0.0),
						vdCylinder, meshCylinder, 
						Vector3r(0.75, 5.0, 0.75));

					const std::vector<Vector3r> &vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
					const unsigned int nVert = static_cast<unsigned int>(vertices.size());
					cd->addCollisionCylinder(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices.data(), nVert, Vector2r(0.75, 5.0));
			}
				currentType = (currentType + 1) % 4;
				current_z += dz_bodies;
				rbIndex++;
			}
			current_x += dx_bodies;
		}
		current_y += dy_bodies;
	}
}

