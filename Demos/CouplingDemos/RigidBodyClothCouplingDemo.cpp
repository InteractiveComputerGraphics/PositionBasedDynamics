#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Demos/Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Demos/Simulation/SimulationModel.h"
#include "Demos/Simulation/TimeStepController.h"
#include <iostream>
#include "Demos/Simulation/Constraints.h"
#include "Demos/Visualization/Visualization.h"
#include "Demos/Utils/OBJLoader.h"
#include "Demos/Utils/Logger.h"
#include "Demos/Utils/Timing.h"
#include "Demos/Utils/FileSystem.h"

#define _USE_MATH_DEFINES
#include "math.h"

// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
	#define new DEBUG_NEW 
#endif

INIT_TIMING
INIT_LOGGING

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

void timeStep ();
void buildModel ();
void createRigidBodyModel();
void createClothMesh();
void render ();
void reset();
void cleanup();
void initShader();
void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData);
void TW_CALL getVelocityUpdateMethod(void *value, void *clientData);
void TW_CALL setStiffness(const void *value, void *clientData);
void TW_CALL getStiffness(void *value, void *clientData);
void TW_CALL setXXStiffness(const void *value, void *clientData);
void TW_CALL getXXStiffness(void *value, void *clientData);
void TW_CALL setYYStiffness(const void *value, void *clientData);
void TW_CALL getYYStiffness(void *value, void *clientData);
void TW_CALL setXYStiffness(const void *value, void *clientData);
void TW_CALL getXYStiffness(void *value, void *clientData);
void TW_CALL setXYPoissonRatio(const void *value, void *clientData);
void TW_CALL getXYPoissonRatio(void *value, void *clientData);
void TW_CALL setYXPoissonRatio(const void *value, void *clientData);
void TW_CALL getYXPoissonRatio(void *value, void *clientData);
void TW_CALL setNormalizeStretch(const void *value, void *clientData);
void TW_CALL getNormalizeStretch(void *value, void *clientData);
void TW_CALL setNormalizeShear(const void *value, void *clientData);
void TW_CALL getNormalizeShear(void *value, void *clientData);
void TW_CALL setBendingStiffness(const void *value, void *clientData);
void TW_CALL getBendingStiffness(void *value, void *clientData);
void TW_CALL setBendingMethod(const void *value, void *clientData);
void TW_CALL getBendingMethod(void *value, void *clientData);
void TW_CALL setSimulationMethod(const void *value, void *clientData);
void TW_CALL getSimulationMethod(void *value, void *clientData);


SimulationModel model;
TimeStepController sim;

const int nRows = 20;
const int nCols = 20;
const Real clothWidth = 10.0;
const Real clothHeight = 10.0;
const Real width = 0.2;
const Real height = 2.0;
const Real depth = 0.2;
short simulationMethod = 2;
short bendingMethod = 2;
bool doPause = true;
std::vector<unsigned int> selectedBodies;
std::vector<unsigned int> selectedParticles;
Vector3r oldMousePos;
Shader *shader;
Shader *shaderTex;
string exePath;
string dataPath;
float jointColor[4] = { 0.0, 0.4f, 0.2f, 1.0 };
float dynamicBodyColor[4] = { 0.1f, 0.4f, 0.8f, 1 };
float staticBodyColor[4] = { 0.4f, 0.4f, 0.4f, 1.0 };


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	std::string logPath = FileSystem::normalizePath(FileSystem::getProgramPath() + "/log");
	FileSystem::makeDirs(logPath);
	logger.addSink(unique_ptr<ConsoleSink>(new ConsoleSink(LogLevel::INFO)));
	logger.addSink(unique_ptr<FileSink>(new FileSink(LogLevel::DEBUG, logPath + "/PBD.log")));

	exePath = FileSystem::getProgramPath();
	dataPath = exePath + "/" + std::string(PBD_DATA_PATH);

	// OpenGL
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Rigid body demo");
	MiniGL::initLights ();
	MiniGL::initTexture();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection);
	initShader();

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0, 0.1f, 500.0, Vector3r (0.0, 10.0, 30.0), Vector3r (0.0, 0.0, 0.0));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_REAL, setTimeStep, getTimeStep, 0, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwType enumType2 = TwDefineEnum("SimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "SimulationMethod", enumType2, setSimulationMethod, getSimulationMethod, &simulationMethod, " label='Simulation method' enum='0 {None}, 1 {Distance constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "Stiffness", TW_TYPE_REAL, setStiffness, getStiffness, &model, " label='Stiffness'  min=0.0 step=0.1 precision=4 group='Distance constraints' ");
	TwAddVarCB(MiniGL::getTweakBar(), "XXStiffness", TW_TYPE_REAL, setXXStiffness, getXXStiffness, &model, " label='Stiffness XX'  min=0.0 step=0.1 precision=4 group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "YYStiffness", TW_TYPE_REAL, setYYStiffness, getYYStiffness, &model, " label='Stiffness YY'  min=0.0 step=0.1 precision=4 group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "XYStiffness", TW_TYPE_REAL, setXYStiffness, getXYStiffness, &model, " label='Stiffness XY'  min=0.0 step=0.1 precision=4 group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "XXStiffnessFEM", TW_TYPE_REAL, setXXStiffness, getXXStiffness, &model, " label='Youngs modulus XX'  min=0.0 step=0.1 precision=4 group='FEM based PBD' ");
	TwAddVarCB(MiniGL::getTweakBar(), "YYStiffnessFEM", TW_TYPE_REAL, setYYStiffness, getYYStiffness, &model, " label='Youngs modulus YY'  min=0.0 step=0.1 precision=4 group='FEM based PBD' ");
	TwAddVarCB(MiniGL::getTweakBar(), "XYStiffnessFEM", TW_TYPE_REAL, setXYStiffness, getXYStiffness, &model, " label='Youngs modulus XY'  min=0.0 step=0.1 precision=4 group='FEM based PBD' ");
	TwAddVarCB(MiniGL::getTweakBar(), "XYPoissonRatioFEM", TW_TYPE_REAL, setXYPoissonRatio, getXYPoissonRatio, &model, " label='Poisson ratio XY'  min=0.0 step=0.1 precision=4 group='FEM based PBD' ");
	TwAddVarCB(MiniGL::getTweakBar(), "YXPoissonRatioFEM", TW_TYPE_REAL, setYXPoissonRatio, getYXPoissonRatio, &model, " label='Poisson ratio YX'  min=0.0 step=0.1 precision=4 group='FEM based PBD' ");
	TwAddVarCB(MiniGL::getTweakBar(), "NormalizeStretch", TW_TYPE_BOOL32, setNormalizeStretch, getNormalizeStretch, &model, " label='Normalize stretch' group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "NormalizeShear", TW_TYPE_BOOL32, setNormalizeShear, getNormalizeShear, &model, " label='Normalize shear' group='Strain based dynamics' ");
	TwType enumType3 = TwDefineEnum("BendingMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "BendingMethod", enumType3, setBendingMethod, getBendingMethod, &bendingMethod, " label='Bending method' enum='0 {None}, 1 {Dihedral angle}, 2 {Isometric bending}' group=Bending");
	TwAddVarCB(MiniGL::getTweakBar(), "BendingStiffness", TW_TYPE_REAL, setBendingStiffness, getBendingStiffness, &model, " label='Bending stiffness'  min=0.0 step=0.01 precision=4 group=Bending ");

	glutMainLoop ();	

	cleanup ();
	
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

	vertFile = dataPath + "/shaders/vs_smoothTex.glsl";
	fragFile = dataPath + "/shaders/fs_smoothTex.glsl";
	shaderTex = MiniGL::createShader(vertFile, "", fragFile);

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
	Timing::printAverageTimes();
	Timing::reset();

	model.reset();
	sim.reset();
	TimeManager::getCurrent()->setTime(0.0);

	model.cleanup();
	buildModel();
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
	
	selectedBodies.clear(); 
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	std::vector<Vector3r, Eigen::aligned_allocator<Vector3r> > x;
	x.resize(rb.size());
	for (unsigned int i = 0; i < rb.size(); i++)
	{
		x[i] = rb[i]->getPosition();
	}
 
	Selection::selectRect(start, end, &x[0], &x[rb.size() - 1], selectedBodies);
	if ((selectedBodies.size() > 0) || (selectedParticles.size() > 0))
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

	for (unsigned int i = 0; i < model.getTriangleModels().size(); i++)
		model.getTriangleModels()[i]->updateMeshNormals(model.getParticles());
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.005);

	createClothMesh();
	createRigidBodyModel();	
}

void renderBallJoint(BallJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(2), 0.1f, jointColor);
}

void renderRigidBodyParticleBallJoint(RigidBodyParticleBallJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(1), 0.1f, jointColor);
}

void renderRigidBodyModel()
{
	// Draw simulation model

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();

	float selectionColor[4] = { 0.8f, 0.0f, 0.0f, 1 };
	float surfaceColor[4] = { 0.1f, 0.4f, 0.8f, 1 };

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

		if (rb[i]->getMass() != 0.0)
		{

			const VertexData &vd = rb[i]->getGeometry().getVertexData();
			const IndexedFaceMesh &mesh = rb[i]->getGeometry().getMesh();
			if (!selected)
			{
				if (shader)
					glUniform3fv(shader->getUniform("surface_color"), 1, surfaceColor);
				Visualization::drawMesh(vd, mesh, 0, surfaceColor);
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
		else if (constraints[i]->getTypeId() == RigidBodyParticleBallJoint::TYPE_ID)
		{
			renderRigidBodyParticleBallJoint(*(RigidBodyParticleBallJoint*)constraints[i]);
		}
	}
}

void renderTriangleModels()
{
	// Draw simulation model

	const ParticleData &pd = model.getParticles();
	float surfaceColor[4] = { 0.2f, 0.5, 1.0, 1 };

	if (shaderTex)
	{
		shaderTex->begin();
		glUniform3fv(shaderTex->getUniform("surface_color"), 1, surfaceColor);
		glUniform1f(shaderTex->getUniform("shininess"), 5.0);
		glUniform1f(shaderTex->getUniform("specular_factor"), 0.2f);

		GLfloat matrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
		glUniformMatrix4fv(shaderTex->getUniform("modelview_matrix"), 1, GL_FALSE, matrix);
		GLfloat pmatrix[16];
		glGetFloatv(GL_PROJECTION_MATRIX, pmatrix);
		glUniformMatrix4fv(shaderTex->getUniform("projection_matrix"), 1, GL_FALSE, pmatrix);
	}

	for (unsigned int i = 0; i < model.getTriangleModels().size(); i++)
	{
		// mesh 
		TriangleModel *triModel = model.getTriangleModels()[i];
		const IndexedFaceMesh &mesh = triModel->getParticleMesh();
		Visualization::drawTexturedMesh(pd, mesh, triModel->getIndexOffset(), surfaceColor);
	}
	if (shaderTex)
		shaderTex->end();

	float red[4] = { 0.8f, 0.0, 0.0, 1 };
	for (unsigned int j = 0; j < selectedParticles.size(); j++)
	{
		MiniGL::drawSphere(pd.getPosition(selectedParticles[j]), 0.08f, red);
	}
}


void render ()
{
	MiniGL::coordinateSystem();
	
	renderRigidBodyModel();
	renderTriangleModels();

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

/** Create the model
*/
void createRigidBodyModel()
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();

	string fileName = FileSystem::normalizePath(dataPath + "/models/cube.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	loadObj(fileName, vd, mesh, Vector3r(width, height, depth));

	rb.resize(12);

	//////////////////////////////////////////////////////////////////////////
	// -5, -5
	//////////////////////////////////////////////////////////////////////////
	rb[0] = new RigidBody();
	rb[0]->initBody(0.0,
		Vector3r(-5.0, 0.0, -5.0),
		computeInertiaTensorBox(1.0, 0.5, 0.5, 0.5),
		Quaternionr(1.0, 0.0, 0.0, 0.0), 
		vd, mesh);

	// dynamic body
	rb[1] = new RigidBody();
	rb[1]->initBody(1.0,
		Vector3r(-5.0, 1.0, -5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	// dynamic body
	rb[2] = new RigidBody();
	rb[2]->initBody(1.0,
		Vector3r(-5.0, 3.0, -5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	model.addBallJoint(0, 1, Vector3r(-5.0, 0.0, -5.0));
	model.addBallJoint(1, 2, Vector3r(-5.0, 2.0, -5.0));

	//////////////////////////////////////////////////////////////////////////
	// 5, -5
	//////////////////////////////////////////////////////////////////////////
	rb[3] = new RigidBody();
	rb[3]->initBody(0.0,
		Vector3r(5.0, 0.0, -5.0),
		computeInertiaTensorBox(1.0, 0.5, 0.5, 0.5),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	// dynamic body
	rb[4] = new RigidBody();
	rb[4]->initBody(1.0,
		Vector3r(5.0, 1.0, -5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	// dynamic body
	rb[5] = new RigidBody();
	rb[5]->initBody(1.0,
		Vector3r(5.0, 3.0, -5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	model.addBallJoint(3, 4, Vector3r(5.0, 0.0, -5.0));
	model.addBallJoint(4, 5, Vector3r(5.0, 2.0, -5.0));

	//////////////////////////////////////////////////////////////////////////
	// 5, 5
	//////////////////////////////////////////////////////////////////////////
	rb[6] = new RigidBody();
	rb[6]->initBody(0.0,
		Vector3r(5.0, 0.0, 5.0),
		computeInertiaTensorBox(1.0, 0.5, 0.5, 0.5),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	// dynamic body
	rb[7] = new RigidBody();
	rb[7]->initBody(1.0,
		Vector3r(5.0, 1.0, 5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	// dynamic body
	rb[8] = new RigidBody();
	rb[8]->initBody(1.0,
		Vector3r(5.0, 3.0, 5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	model.addBallJoint(6, 7, Vector3r(5.0, 0.0, 5.0));
	model.addBallJoint(7, 8, Vector3r(5.0, 2.0, 5.0));

	//////////////////////////////////////////////////////////////////////////
	// -5, 5
	//////////////////////////////////////////////////////////////////////////
	rb[9] = new RigidBody();
	rb[9]->initBody(0.0,
		Vector3r(-5.0, 0.0, 5.0),
		computeInertiaTensorBox(1.0, 0.5, 0.5, 0.5),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	// dynamic body
	rb[10] = new RigidBody();
	rb[10]->initBody(1.0,
		Vector3r(-5.0, 1.0, 5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	// dynamic body
	rb[11] = new RigidBody();
	rb[11]->initBody(1.0,
		Vector3r(-5.0, 3.0, 5.0),
		computeInertiaTensorBox(1.0, width, height, depth),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh);

	model.addBallJoint(9, 10, Vector3r(-5.0, 0.0, 5.0));
	model.addBallJoint(10, 11, Vector3r(-5.0, 2.0, 5.0));
	

	model.addRigidBodyParticleBallJoint(2, 0);
	model.addRigidBodyParticleBallJoint(5, (nRows - 1)*nCols);
	model.addRigidBodyParticleBallJoint(8, nRows*nCols - 1);
	model.addRigidBodyParticleBallJoint(11, nCols-1);

}


/** Create a particle model mesh
*/
void createClothMesh()
{
	TriangleModel::ParticleMesh::UVs uvs;
	uvs.resize(nRows*nCols);

	const Real dy = clothWidth / (Real)(nCols - 1);
	const Real dx = clothHeight / (Real)(nRows - 1);

	Vector3r points[nRows*nCols];
	for (int i = 0; i < nRows; i++)
	{
		for (int j = 0; j < nCols; j++)
		{
			const Real y = (Real)dy*j;
			const Real x = (Real)dx*i;
			points[i*nCols + j] = Vector3r(x - 5.0, 4.0, y - 5.0);

			uvs[i*nCols + j][0] = x / clothWidth;
			uvs[i*nCols + j][1] = y / clothHeight;
		}
	}
	const int nIndices = 6 * (nRows - 1)*(nCols - 1);

	TriangleModel::ParticleMesh::UVIndices uvIndices;
	uvIndices.resize(nIndices);

	unsigned int indices[nIndices];
	int index = 0;
	for (int i = 0; i < nRows - 1; i++)
	{
		for (int j = 0; j < nCols - 1; j++)
		{
			int helper = 0;
			if (i % 2 == j % 2)
				helper = 1;

			indices[index] = i*nCols + j;
			indices[index + 1] = i*nCols + j + 1;
			indices[index + 2] = (i + 1)*nCols + j + helper;

			uvIndices[index] = i*nCols + j;
			uvIndices[index + 1] = i*nCols + j + 1;
			uvIndices[index + 2] = (i + 1)*nCols + j + helper;
			index += 3;

			indices[index] = (i + 1)*nCols + j + 1;
			indices[index + 1] = (i + 1)*nCols + j;
			indices[index + 2] = i*nCols + j + 1 - helper;

			uvIndices[index] = (i + 1)*nCols + j + 1;
			uvIndices[index + 1] = (i + 1)*nCols + j;
			uvIndices[index + 2] = i*nCols + j + 1 - helper;
			index += 3;
		}
	}

	model.addTriangleModel(nRows*nCols, nIndices / 3, &points[0], &indices[0], uvIndices, uvs);

	ParticleData &pd = model.getParticles();
	for (unsigned int i = 0; i < pd.getNumberOfParticles(); i++)
	{
		pd.setMass(i, 1.0);
	}

	// init constraints
	for (unsigned int cm = 0; cm < model.getTriangleModels().size(); cm++)
	{
		if (simulationMethod == 1)
		{
			const unsigned int offset = model.getTriangleModels()[cm]->getIndexOffset();
			const unsigned int nEdges = model.getTriangleModels()[cm]->getParticleMesh().numEdges();
			const IndexedFaceMesh::Edge *edges = model.getTriangleModels()[cm]->getParticleMesh().getEdges().data();
			for (unsigned int i = 0; i < nEdges; i++)
			{
				const unsigned int v1 = edges[i].m_vert[0] + offset;
				const unsigned int v2 = edges[i].m_vert[1] + offset;

				model.addDistanceConstraint(v1, v2);
			}
		}
		else if (simulationMethod == 2)
		{
			const unsigned int offset = model.getTriangleModels()[cm]->getIndexOffset();
			TriangleModel::ParticleMesh &mesh = model.getTriangleModels()[cm]->getParticleMesh();
			const unsigned int *tris = mesh.getFaces().data();
			const unsigned int nFaces = mesh.numFaces();
			for (unsigned int i = 0; i < nFaces; i++)
			{
				const unsigned int v1 = tris[3 * i] + offset;
				const unsigned int v2 = tris[3 * i + 1] + offset;
				const unsigned int v3 = tris[3 * i + 2] + offset;
				model.addFEMTriangleConstraint(v1, v2, v3);
			}
		}
		else if (simulationMethod == 3)
		{
			const unsigned int offset = model.getTriangleModels()[cm]->getIndexOffset();
			TriangleModel::ParticleMesh &mesh = model.getTriangleModels()[cm]->getParticleMesh();
			const unsigned int *tris = mesh.getFaces().data();
			const unsigned int nFaces = mesh.numFaces();
			for (unsigned int i = 0; i < nFaces; i++)
			{
				const unsigned int v1 = tris[3 * i] + offset;
				const unsigned int v2 = tris[3 * i + 1] + offset;
				const unsigned int v3 = tris[3 * i + 2] + offset;
				model.addStrainTriangleConstraint(v1, v2, v3);
			}
		}
		if (bendingMethod != 0)
		{
			const unsigned int offset = model.getTriangleModels()[cm]->getIndexOffset();
			TriangleModel::ParticleMesh &mesh = model.getTriangleModels()[cm]->getParticleMesh();
			unsigned int nEdges = mesh.numEdges();
			const TriangleModel::ParticleMesh::Edge *edges = mesh.getEdges().data();
			const unsigned int *tris = mesh.getFaces().data();
			for (unsigned int i = 0; i < nEdges; i++)
			{
				const int tri1 = edges[i].m_face[0];
				const int tri2 = edges[i].m_face[1];
				if ((tri1 != 0xffffffff) && (tri2 != 0xffffffff))
				{
					// Find the triangle points which do not lie on the axis
					const int axisPoint1 = edges[i].m_vert[0];
					const int axisPoint2 = edges[i].m_vert[1];
					int point1 = -1;
					int point2 = -1;
					for (int j = 0; j < 3; j++)
					{
						if ((tris[3 * tri1 + j] != axisPoint1) && (tris[3 * tri1 + j] != axisPoint2))
						{
							point1 = tris[3 * tri1 + j];
							break;
						}
					}
					for (int j = 0; j < 3; j++)
					{
						if ((tris[3 * tri2 + j] != axisPoint1) && (tris[3 * tri2 + j] != axisPoint2))
						{
							point2 = tris[3 * tri2 + j];
							break;
						}
					}
					if ((point1 != -1) && (point2 != -1))
					{
						const unsigned int vertex1 = point1 + offset;
						const unsigned int vertex2 = point2 + offset;
						const unsigned int vertex3 = edges[i].m_vert[0] + offset;
						const unsigned int vertex4 = edges[i].m_vert[1] + offset;
						if (bendingMethod == 1)
							model.addDihedralConstraint(vertex1, vertex2, vertex3, vertex4);
						else if (bendingMethod == 2)
							model.addIsometricBendingConstraint(vertex1, vertex2, vertex3, vertex4);
					}
				}
			}
		}
	}

	LOG_INFO << "Number of triangles: " << nIndices / 3;
	LOG_INFO << "Number of vertices: " << nRows*nCols;

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

void TW_CALL setStiffness(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setClothStiffness(val);
}

void TW_CALL getStiffness(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getClothStiffness();
}

void TW_CALL setXXStiffness(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setClothXXStiffness(val);
}

void TW_CALL getXXStiffness(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getClothXXStiffness();
}

void TW_CALL setYYStiffness(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setClothYYStiffness(val);
}

void TW_CALL getYYStiffness(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getClothYYStiffness();
}

void TW_CALL setXYStiffness(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setClothXYStiffness(val);
}

void TW_CALL getXYStiffness(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getClothXYStiffness();
}

void TW_CALL setYXPoissonRatio(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setClothYXPoissonRatio(val);
}

void TW_CALL getYXPoissonRatio(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getClothYXPoissonRatio();
}

void TW_CALL setXYPoissonRatio(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setClothXYPoissonRatio(val);
}

void TW_CALL getXYPoissonRatio(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getClothXYPoissonRatio();
}

void TW_CALL setNormalizeStretch(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((SimulationModel*)clientData)->setClothNormalizeStretch(val);
}

void TW_CALL getNormalizeStretch(void *value, void *clientData)
{
	*(bool *)(value) = ((SimulationModel*)clientData)->getClothNormalizeStretch();
}

void TW_CALL setNormalizeShear(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((SimulationModel*)clientData)->setClothNormalizeShear(val);
}

void TW_CALL getNormalizeShear(void *value, void *clientData)
{
	*(bool *)(value) = ((SimulationModel*)clientData)->getClothNormalizeShear();
}

void TW_CALL setBendingStiffness(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setClothBendingStiffness(val);
}

void TW_CALL getBendingStiffness(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getClothBendingStiffness();
}

void TW_CALL setBendingMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	*((short*)clientData) = val;
	reset();
}

void TW_CALL getBendingMethod(void *value, void *clientData)
{
	*(short *)(value) = *((short*)clientData);
}

void TW_CALL setSimulationMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	*((short*)clientData) = val;
	reset();
}

void TW_CALL getSimulationMethod(void *value, void *clientData)
{
	*(short *)(value) = *((short*)clientData);
}