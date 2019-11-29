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
#include "Simulation/DistanceFieldCollisionDetection.h"
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
void TW_CALL setContactTolerance(const void *value, void *clientData);
void TW_CALL getContactTolerance(void *value, void *clientData);
void TW_CALL setContactStiffnessRigidBody(const void *value, void *clientData);
void TW_CALL getContactStiffnessRigidBody(void *value, void *clientData);
void TW_CALL setContactStiffnessParticleRigidBody(const void *value, void *clientData);
void TW_CALL getContactStiffnessParticleRigidBody(void *value, void *clientData);


DemoBase *base;
DistanceFieldCollisionDetection cd;


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

		base = new DemoBase();
	base->init(argc, argv, "Rigid body collision demo");

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	buildModel();

	initParameters();

	Simulation::getCurrent()->setSimulationMethodChangedCallback([&]() { reset(); initParameters(); base->getSceneLoader()->readParameterObject(Simulation::getCurrent()->getTimeStep()); });

	// OpenGL
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0, Vector3r (5.0, 30.0, 70.0), Vector3r (5.0, 0.0, 0.0));

	TwAddVarCB(MiniGL::getTweakBar(), "ContactTolerance", TW_TYPE_REAL, setContactTolerance, getContactTolerance, &cd, " label='Contact tolerance'  min=0.0 step=0.001 precision=3 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessRigidBody", TW_TYPE_REAL, setContactStiffnessRigidBody, getContactStiffnessRigidBody, model, " label='Contact stiffness RB'  min=0.0 step=0.1 precision=2 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessParticleRigidBody", TW_TYPE_REAL, setContactStiffnessParticleRigidBody, getContactStiffnessParticleRigidBody, model, " label='Contact stiffness Particle-RB'  min=0.0 step=0.1 precision=2 group=Simulation ");


	glutMainLoop ();	

	base->cleanup();

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
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.005));

	SimulationModel *model = Simulation::getCurrent()->getModel();
	Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, &cd);

	createBodyModel();
}

void render ()
{
	base->render();
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
	SimulationModel::ConstraintVector &constraints = model->getConstraints();

	string fileNameBox = FileSystem::normalizePath(base->getDataPath() + "/models/cube.obj");
	IndexedFaceMesh meshBox;
	VertexData vdBox;
	loadObj(fileNameBox, vdBox, meshBox, Vector3r::Ones());

	string fileNameCylinder = FileSystem::normalizePath(base->getDataPath() + "/models/cylinder.obj");
	IndexedFaceMesh meshCylinder;
	VertexData vdCylinder;
	loadObj(fileNameCylinder, vdCylinder, meshCylinder, Vector3r::Ones());

	string fileNameTorus = FileSystem::normalizePath(base->getDataPath() + "/models/torus.obj");
	IndexedFaceMesh meshTorus;
	VertexData vdTorus;
	loadObj(fileNameTorus, vdTorus, meshTorus, Vector3r::Ones());

	string fileNameCube = FileSystem::normalizePath(base->getDataPath() + "/models/cube_5.obj");
	IndexedFaceMesh meshCube;
	VertexData vdCube;
	loadObj(fileNameCube, vdCube, meshCube, Vector3r::Ones());

	string fileNameSphere = FileSystem::normalizePath(base->getDataPath() + "/models/sphere.obj");
	IndexedFaceMesh meshSphere;
	VertexData vdSphere;
	loadObj(fileNameSphere, vdSphere, meshSphere, 2.0*Vector3r::Ones());


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

	const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
	const unsigned int nVert = static_cast<unsigned int>(vertices->size());

	cd.addCollisionBox(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, Vector3r(100.0, 1.0, 100.0));
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

			const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
			const unsigned int nVert = static_cast<unsigned int>(vertices->size());
			cd.addCollisionCylinder(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, Vector2r(0.5, 10.0));
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
					
					const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
					const unsigned int nVert = static_cast<unsigned int>(vertices->size());
					cd.addCollisionTorus(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, Vector2r(2.0, 1.0));
				}
				else if (currentType == 1)
				{
					rb[rbIndex]->initBody(100.0,
						Vector3r(current_x, current_y, current_z),
						q, //Quaternionr(1.0, 0.0, 0.0, 0.0),
						vdCube, meshCube, 
						Vector3r(4.0, 1.0, 1.0));

					const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
					const unsigned int nVert = static_cast<unsigned int>(vertices->size());
					cd.addCollisionBox(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, Vector3r(4.0, 1.0, 1.0));
				}
				else if (currentType == 2)
				{
					rb[rbIndex]->initBody(100.0,
						Vector3r(current_x, current_y, current_z),
						q, //Quaternionr(1.0, 0.0, 0.0, 0.0),
						vdSphere, meshSphere);

					const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
					const unsigned int nVert = static_cast<unsigned int>(vertices->size());
					cd.addCollisionSphere(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, 2.0);
				}
				else if (currentType == 3)
				{
					rb[rbIndex]->initBody(100.0,
						Vector3r(current_x, current_y, current_z),
						q, //Quaternionr(1.0, 0.0, 0.0, 0.0),
						vdCylinder, meshCylinder, 
						Vector3r(0.75, 5.0, 0.75));

					const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
					const unsigned int nVert = static_cast<unsigned int>(vertices->size());
					cd.addCollisionCylinder(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, Vector2r(0.75, 5.0));
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

void TW_CALL setContactStiffnessRigidBody(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setContactStiffnessRigidBody(val);
}

void TW_CALL getContactStiffnessRigidBody(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getContactStiffnessRigidBody();
}

void TW_CALL setContactStiffnessParticleRigidBody(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setContactStiffnessParticleRigidBody(val);
}

void TW_CALL getContactStiffnessParticleRigidBody(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getContactStiffnessParticleRigidBody();
}

void TW_CALL setContactTolerance(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((DistanceFieldCollisionDetection*)clientData)->setTolerance(val);
}

void TW_CALL getContactTolerance(void *value, void *clientData)
{
	*(Real *)(value) = ((DistanceFieldCollisionDetection*)clientData)->getTolerance();
}

