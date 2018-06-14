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
void createBunnyRodModel();
void render ();
void reset();
void cleanup();
void initShader();
void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData);
void TW_CALL getVelocityUpdateMethod(void *value, void *clientData);


SimulationModel model;
TimeStepController sim;

// bunny rod scene
const int numberOfBodies = 10;
const Real width = 1.0;
const Real height = 0.1;
const Real depth = 0.1;
const Real youngsModulus = 209e9;
const Real torsionModulus = 79e9;
const Real density = 7800.;

const Real bunnyDensity = 500.;

const Real timeStepSize = 0.01;
bool doPause = true;
std::vector<unsigned int> selectedBodies;
Vector3r oldMousePos;
Shader *shader;
string exePath;
string dataPath;

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
	MiniGL::setViewport (40.0, 0.1f, 500.0, Vector3r (5.0, 10.0, 30.0), Vector3r (5.0, 0.0, 0.0));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_REAL, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");

	glutMainLoop ();	

	cleanup ();

	Timing::printAverageTimes();
	
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
		sim.step(model);
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (timeStepSize);

	createBunnyRodModel();
}

void renderBallJoint(BallJoint &bj)
{
	float jointColor[4] = { 0.0, 0.6f, 0.2f, 1 };
	MiniGL::drawSphere(bj.m_jointInfo.col(2), 1.25f*(float) height, jointColor);
}

void render ()
{
	MiniGL::coordinateSystem();
	
	// Draw sim model
	
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
	if (shader)
		shader->end();

	for (size_t i = 0; i < constraints.size(); i++)
	{
		if (constraints[i]->getTypeId() == BallJoint::TYPE_ID)
		{
			renderBallJoint(*(BallJoint*)constraints[i]);
		}
	}

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
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

/** Create the bunny rod body model
*/
void createBunnyRodModel()
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();

	string fileName = FileSystem::normalizePath(dataPath + "/models/cube.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	loadObj(fileName, vd, mesh, Vector3r(height, width, depth));

	string fileName2 = FileSystem::normalizePath(dataPath + "/models/bunny_10k.obj");
	IndexedFaceMesh mesh2;
	VertexData vd2;
	loadObj(fileName2, vd2, mesh2, Vector3r(2.0, 2.0, 2.0)); 

	rb.resize(numberOfBodies);	
	for (unsigned int i = 0; i < numberOfBodies-1; i++)
	{			
		rb[i] = new RigidBody();

		Real mass(0.25 * M_PI * width * height * depth * density);

		const Real Iy = 1. / 12. * mass*(3.*(0.25*height*height) + width*width);
		const Real Iz = 1. / 12. * mass*(3.*(0.25*depth*depth) + width*width);
		const Real Ix = 0.25 * mass*(0.25*(height*height + depth*depth));
		Vector3r inertia(Iy, Ix, Iz); // rod axis along y-axis
		
		Real angle(M_PI_2);
		Vector3r axis(0., 0., 1.);
		AngleAxisr angleAxis(angle, axis);
		Quaternionr rotation(angleAxis);

		rb[i]->initBody(mass,
			Vector3r((Real)i*width, 0.0, 0.0),
			inertia,
			rotation,
			vd, mesh);
	}
	// Make first body static
	rb[0]->setMass(0.0);

	// bunny
	const Quaternionr q(AngleAxisr((1.0/6.0*M_PI), Vector3r(0.0, 0.0, 1.0)));
	const Vector3r t(0.411 + ((Real)numberOfBodies - 1.0)*width, -1.776, 0.356);
	rb[numberOfBodies - 1] = new RigidBody();
	rb[numberOfBodies - 1]->initBody(bunnyDensity, t, q, vd2, mesh2);

	constraints.reserve(numberOfBodies - 1);
	for (unsigned int i = 0; i < numberOfBodies-2; i++)
	{
		model.addStretchBendingTwistingConstraint(i, i + 1, Vector3r((Real)i*width + 0.5*width, 0.0, 0.0),
			0.25*(height+depth), width, youngsModulus, torsionModulus);
	}
	unsigned int i(numberOfBodies - 2);
	model.addBallJoint(i, i + 1, Vector3r((Real)i*width + 0.5*width, 0.0, 0.0));

	sim.setMaxIterations(200);
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

