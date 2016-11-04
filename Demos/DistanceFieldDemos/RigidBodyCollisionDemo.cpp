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
#include "Demos/Simulation/DistanceFieldCollisionDetection.h"

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
void TW_CALL setContactTolerance(const void *value, void *clientData);
void TW_CALL getContactTolerance(void *value, void *clientData);
void TW_CALL setContactStiffnessRigidBody(const void *value, void *clientData);
void TW_CALL getContactStiffnessRigidBody(void *value, void *clientData);
void TW_CALL setContactStiffnessParticleRigidBody(const void *value, void *clientData);
void TW_CALL getContactStiffnessParticleRigidBody(void *value, void *clientData);


SimulationModel model;
DistanceFieldCollisionDetection cd;
TimeStepController sim;

bool doPause = true;
std::vector<unsigned int> selectedBodies;
Vector3r oldMousePos;
Shader *shader;
Shader *shaderTex;
bool renderContacts = false;
string exePath;
string dataPath;

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	exePath = Utilities::getFilePath(argv[0]);
	dataPath = exePath + "/" + std::string(PBD_DATA_PATH);

	// OpenGL
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Rigid body collision demo");
	MiniGL::initLights ();
	MiniGL::initTexture();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection);
	initShader();

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0, Vector3r (5.0, 30.0, 70.0), Vector3r (5.0, 0.0, 0.0));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarRW(MiniGL::getTweakBar(), "RenderContacts", TW_TYPE_BOOLCPP, &renderContacts, " label='Render contacts' group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_REAL, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "MaxIter", TW_TYPE_UINT32, setMaxIterations, getMaxIterations, &sim, " label='Max. iterations'  min=1 step=1 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactTolerance", TW_TYPE_REAL, setContactTolerance, getContactTolerance, &cd, " label='Contact tolerance'  min=0.0 step=0.001 precision=3 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessRigidBody", TW_TYPE_REAL, setContactStiffnessRigidBody, getContactStiffnessRigidBody, &model, " label='Contact stiffness RB'  min=0.0 step=0.1 precision=2 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessParticleRigidBody", TW_TYPE_REAL, setContactStiffnessParticleRigidBody, getContactStiffnessParticleRigidBody, &model, " label='Contact stiffness Particle-RB'  min=0.0 step=0.1 precision=2 group=Simulation ");


	glutMainLoop ();	

	cleanup ();
	
	return 0;
}

void initShader()
{
	std::string vertFile = dataPath + "/shaders/vs_smooth.glsl";
	std::string fragFile = dataPath + "/shaders/fs_smooth.glsl";
	shader = MiniGL::createShader(vertFile, "", fragFile);

	if (shader == NULL)
		return;

	shader->begin();
	shader->addUniform("modelview_matrix");
	shader->addUniform("projection_matrix");
	shader->addUniform("surface_color");
	shader->addUniform("shininess");
	shader->addUniform("specular_factor");
	shader->end();

	std::string vertFileTex = dataPath + "/shaders/vs_smoothTex.glsl";
	std::string fragFileTex = dataPath + "/shaders/fs_smoothTex.glsl";
	shaderTex = MiniGL::createShader(vertFileTex, "", fragFileTex);

	if (shaderTex == NULL)
		return;

	shaderTex->begin();
	shaderTex->addUniform("modelview_matrix");
	shaderTex->addUniform("projection_matrix");
	shaderTex->addUniform("surface_color");
	shaderTex->addUniform("shininess");
	shaderTex->addUniform("specular_factor");
	shaderTex->end();
}

void cleanup()
{
	delete TimeManager::getCurrent();
	delete shader;
	delete shaderTex;
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

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	for (size_t j = 0; j < selectedBodies.size(); j++)
	{
		if (rb[selectedBodies[j]]->getMass() != 0.0)
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
		sim.step(model);
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.005);

	sim.setCollisionDetection(model, &cd);

	createBodyModel();
}

void renderBallJoint(BallJoint &bj)
{
	float jointColor[4] = { 0.0, 0.6f, 0.2f, 1 };
	MiniGL::drawSphere(bj.m_jointInfo.col(2), 0.15f, jointColor);
}

void renderContact(RigidBodyContactConstraint &cc)
{
	float col1[4] = { 0.0, 0.6f, 0.2f, 1 };
	float col2[4] = { 0.6f, 0.0, 0.2f, 1 };
	MiniGL::drawPoint(cc.m_constraintInfo.col(0), 5.0, col1);
	MiniGL::drawPoint(cc.m_constraintInfo.col(1), 5.0, col2);
	MiniGL::drawVector(cc.m_constraintInfo.col(1), cc.m_constraintInfo.col(1) + cc.m_constraintInfo.col(2), 1.0, col2);
}

void render ()
{
	MiniGL::coordinateSystem();
	
	// Draw sim model
	
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();
	SimulationModel::RigidBodyContactConstraintVector &contacts = model.getRigidBodyContactConstraints();

	float selectionColor[4] = { 0.8f, 0.0f, 0.0f, 1 };
	float surfaceColor[4] = { 0.3f, 0.5f, 0.8f, 1 };
	float staticColor[4] = { 0.5f, 0.5f, 0.5f, 1 };

	if (renderContacts)
	{
		for (unsigned int i = 0; i < contacts.size(); i++)
			renderContact(contacts[i]);
	}


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
		if (shader)
		{
			if (rb[i]->getMass() != 0.0)
			{
				if (!selected)
				{
					glUniform3fv(shader->getUniform("surface_color"), 1, surfaceColor);
					Visualization::drawMesh(vd, mesh, 0, surfaceColor);
				}
				else
				{
					glUniform3fv(shader->getUniform("surface_color"), 1, selectionColor);
					Visualization::drawMesh(vd, mesh, 0, selectionColor);
				}
			}
		}
	}

	if (shaderTex)
		shaderTex->end();

	if (shaderTex)
	{
		shaderTex->begin();
		glUniform1f(shaderTex->getUniform("shininess"), 5.0);
		glUniform1f(shaderTex->getUniform("specular_factor"), 0.2f);

		GLfloat matrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
		glUniformMatrix4fv(shaderTex->getUniform("modelview_matrix"), 1, GL_FALSE, matrix);
		GLfloat pmatrix[16];
		glGetFloatv(GL_PROJECTION_MATRIX, pmatrix);
		glUniformMatrix4fv(shaderTex->getUniform("projection_matrix"), 1, GL_FALSE, pmatrix);
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
		if (shaderTex)
		{
			if (rb[i]->getMass() == 0.0)
			{
				if (!selected)
				{
					glUniform3fv(shaderTex->getUniform("surface_color"), 1, staticColor);
					Visualization::drawTexturedMesh(vd, mesh, 0, staticColor);
				}
				else
				{
					glUniform3fv(shaderTex->getUniform("surface_color"), 1, selectionColor);
					Visualization::drawTexturedMesh(vd, mesh, 0, selectionColor);
				}
			}
		}
	}

	if (shaderTex)
		shaderTex->end();

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

	string fileNameBox = dataPath + "/models/cube.obj";
	IndexedFaceMesh meshBox;
	VertexData vdBox;
	OBJLoader::loadObj(fileNameBox, vdBox, meshBox);

	string fileNameCylinder = dataPath + "/models/cylinder.obj";
	IndexedFaceMesh meshCylinder;
	VertexData vdCylinder;
	OBJLoader::loadObj(fileNameCylinder, vdCylinder, meshCylinder);

	string fileNameTorus = dataPath + "/models/torus.obj";
	IndexedFaceMesh meshTorus;
	VertexData vdTorus;
	OBJLoader::loadObj(fileNameTorus, vdTorus, meshTorus);

	string fileNameCube = dataPath + "/models/cube_5.obj";
	IndexedFaceMesh meshCube;
	VertexData vdCube;
	OBJLoader::loadObj(fileNameCube, vdCube, meshCube);

	string fileNameSphere = dataPath + "/models/sphere.obj";
	IndexedFaceMesh meshSphere;
	VertexData vdSphere;
	OBJLoader::loadObj(fileNameSphere, vdSphere, meshSphere);


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
					cd.addCollisionSphere(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, 1.0);
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

