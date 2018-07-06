#include "Common/Common.h"
#include "GL/glew.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "FluidModel.h"
#include "TimeStepFluidModel.h"
#include <iostream>
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
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
void createBreakingDam();
void addWall(const Vector3r &minX, const Vector3r &maxX, std::vector<Vector3r> &boundaryParticles);
void initBoundaryData(std::vector<Vector3r> &boundaryParticles);
void render ();
void cleanup();
void reset();
void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end, void *clientData);
void createSphereBuffers(Real radius, int resolution);
void renderSphere(const Vector3r &x, const float color[]);
void releaseSphereBuffers();
void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData);
void TW_CALL getVelocityUpdateMethod(void *value, void *clientData);
void TW_CALL setViscosity(const void *value, void *clientData);
void TW_CALL getViscosity(void *value, void *clientData);



FluidModel model;
TimeStepFluidModel simulation;

const Real particleRadius = static_cast<Real>(0.025);
const unsigned int width = 15;
const unsigned int depth = 15;
const unsigned int height = 20;
const Real containerWidth = (width + 1)*particleRadius*static_cast<Real>(2.0 * 5.0);
const Real containerDepth = (depth + 1)*particleRadius*static_cast<Real>(2.0);
const Real containerHeight = 4.0;
bool doPause = true;
std::vector<unsigned int> selectedParticles;
Vector3r oldMousePos;
// initiate buffers
GLuint elementbuffer;
GLuint normalbuffer;
GLuint vertexbuffer;
int vertexBufferSize = 0;
GLint context_major_version, context_minor_version;
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
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Fluid demo");
	MiniGL::initLights ();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection, nullptr);

	MiniGL::getOpenGLVersion(context_major_version, context_minor_version);

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0, 0.1f, 500.0, Vector3r (0.0, 3.0, 8.0), Vector3r (0.0, 0.0, 0.0));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_REAL, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &simulation, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "Viscosity", TW_TYPE_REAL, setViscosity, getViscosity, &model, " label='Viscosity'  min=0.0 max = 0.5 step=0.001 precision=4 group=Simulation ");

	buildModel();

	if (context_major_version >= 3)
		createSphereBuffers((Real)particleRadius, 8);

	glutMainLoop ();	

	cleanup ();

	Timing::printAverageTimes();
	
	return 0;
}

void cleanup()
{
	delete TimeManager::getCurrent();
	if (context_major_version >= 3)
		releaseSphereBuffers();
}

void reset()
{
	Timing::printAverageTimes();
	Timing::reset();

	model.reset();
	simulation.reset();
	TimeManager::getCurrent()->setTime(0.0);
}

void mouseMove(int x, int y, void *clientData)
{
	Vector3r mousePos;
	MiniGL::unproject(x, y, mousePos);
	const Vector3r diff = mousePos - oldMousePos;

	TimeManager *tm = TimeManager::getCurrent();
	const Real h = tm->getTimeStepSize();

	ParticleData &pd = model.getParticles();
	for (unsigned int j = 0; j < selectedParticles.size(); j++)
	{
		pd.getVelocity(selectedParticles[j]) += 5.0*diff/h;
	}
	oldMousePos = mousePos;
}

void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end, void *clientData)
{
	std::vector<unsigned int> hits;
	selectedParticles.clear();
	ParticleData &pd = model.getParticles();
	Selection::selectRect(start, end, &pd.getPosition(0), &pd.getPosition(pd.size() - 1), selectedParticles);
	if (selectedParticles.size() > 0)
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
		simulation.step(model);
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.0025));

	createBreakingDam();
}

void render ()
{
	float gridColor[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
	MiniGL::drawGrid(gridColor);

	MiniGL::coordinateSystem();

	// Draw simulation model
	
	const ParticleData &pd = model.getParticles();
	const unsigned int nParticles = pd.size();

	float surfaceColor[4] = { 0.2f, 0.6f, 0.8f, 1 };
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, surfaceColor);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, surfaceColor);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(surfaceColor);

	glPointSize(4.0);

	const Real supportRadius = model.getSupportRadius();
	Real vmax = static_cast<Real>(0.4*2.0)*supportRadius / TimeManager::getCurrent()->getTimeStepSize();
	Real vmin = 0.0;

	if (context_major_version > 3)
	{
		for (unsigned int i = 0; i < nParticles; i++)
		{
			Real v = pd.getVelocity(i).norm();
			v = static_cast<Real>(0.5)*((v - vmin) / (vmax - vmin));
			v = min(static_cast<Real>(128.0)*v*v, static_cast<Real>(0.5));
			float fluidColor[4] = { 0.2f, 0.2f, 0.2f, 1.0 };
			MiniGL::hsvToRgb(0.55f, 1.0f, 0.5f + (float)v, fluidColor);
			renderSphere(pd.getPosition(i), fluidColor);
		}

// 		for (unsigned int i = 0; i < model.numBoundaryParticles(); i++)
// 			renderSphere(model.getBoundaryX(i), surfaceColor);
	}
	else
	{
		glDisable(GL_LIGHTING);
		glBegin(GL_POINTS);
		for (unsigned int i = 0; i < nParticles; i++)
		{
			Real v = pd.getVelocity(i).norm();
			v = static_cast<Real>(0.5)*((v - vmin) / (vmax - vmin));
			v = min(static_cast<Real>(128.0)*v*v, static_cast<Real>(0.5));
			float fluidColor[4] = { 0.2f, 0.2f, 0.2f, 1.0 };
			MiniGL::hsvToRgb(0.55f, 1.0f, 0.5f + (float)v, fluidColor);

			glColor3fv(fluidColor);
			glVertex3v(&pd.getPosition(i)[0]);
		}
		glEnd();

		// 	glBegin(GL_POINTS);
		// 	for (unsigned int i = 0; i < model.numBoundaryParticles(); i++)
		// 	{
		// 		glColor3fv(surfaceColor);
		// 		glVertex3fv(&model.getBoundaryX(i)[0]);
		// 	}
		// 	glEnd();

		glEnable(GL_LIGHTING);
	}



	float red[4] = { 0.8f, 0.0f, 0.0f, 1 };
	for (unsigned int j = 0; j < selectedParticles.size(); j++)
	{
		MiniGL::drawSphere(pd.getPosition(selectedParticles[j]), 0.08f, red);
	}

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
}


/** Create a breaking dam scenario
*/
void createBreakingDam()
{
	LOG_INFO << "Initialize fluid particles";
	const Real diam = 2.0*particleRadius;
	const Real startX = -static_cast<Real>(0.5)*containerWidth + diam;
	const Real startY = diam;
	const Real startZ = -static_cast<Real>(0.5)*containerDepth + diam;
	const Real yshift = sqrt(static_cast<Real>(3.0)) * particleRadius;

	std::vector<Vector3r> fluidParticles;
	fluidParticles.resize(width*height*depth);

	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  
		for (int i = 0; i < (int)width; i++)
		{
			for (unsigned int j = 0; j < height; j++)
			{
				for (unsigned int k = 0; k < depth; k++)
				{
					fluidParticles[i*height*depth + j*depth + k] = diam*Vector3r((Real)i, (Real)j, (Real)k) + Vector3r(startX, startY, startZ);
				}
			}
		}
	}

	model.setParticleRadius(particleRadius);

	std::vector<Vector3r> boundaryParticles;
	initBoundaryData(boundaryParticles);

	model.initModel((unsigned int)fluidParticles.size(), fluidParticles.data(), (unsigned int)boundaryParticles.size(), boundaryParticles.data());

	LOG_INFO << "Number of particles: " << width*height*depth;
}


void addWall(const Vector3r &minX, const Vector3r &maxX, std::vector<Vector3r> &boundaryParticles)
{
	const Real particleDistance = static_cast<Real>(2.0)*model.getParticleRadius();

	const Vector3r diff = maxX - minX;
	const unsigned int stepsX = (unsigned int)(diff[0] / particleDistance) + 1u;
	const unsigned int stepsY = (unsigned int)(diff[1] / particleDistance) + 1u;
	const unsigned int stepsZ = (unsigned int)(diff[2] / particleDistance) + 1u;

	const unsigned int startIndex = (unsigned int) boundaryParticles.size();
	boundaryParticles.resize(startIndex + stepsX*stepsY*stepsZ);

	#pragma omp parallel default(shared)
	{
		#pragma omp for schedule(static)  
		for (int j = 0; j < (int)stepsX; j++)
		{
			for (unsigned int k = 0; k < stepsY; k++)
			{
				for (unsigned int l = 0; l < stepsZ; l++)
				{
					const Vector3r currPos = minX + Vector3r(j*particleDistance, k*particleDistance, l*particleDistance);
					boundaryParticles[startIndex + j*stepsY*stepsZ + k*stepsZ + l] = currPos;
				}
			}
		}
	}
}

void initBoundaryData(std::vector<Vector3r> &boundaryParticles)
{
	const Real x1 = -containerWidth / 2.0;
	const Real x2 = containerWidth / 2.0;
	const Real y1 = 0.0;
	const Real y2 = containerHeight;
	const Real z1 = -containerDepth / 2.0;
	const Real z2 = containerDepth / 2.0;

	const Real diam = 2.0*particleRadius;

	// Floor
	addWall(Vector3r(x1, y1, z1), Vector3r(x2, y1, z2), boundaryParticles);
	// Top
	addWall(Vector3r(x1, y2, z1), Vector3r(x2, y2, z2), boundaryParticles);
	// Left
	addWall(Vector3r(x1, y1, z1), Vector3r(x1, y2, z2), boundaryParticles);
	// Right
	addWall(Vector3r(x2, y1, z1), Vector3r(x2, y2, z2), boundaryParticles);
	// Back
	addWall(Vector3r(x1, y1, z1), Vector3r(x2, y2, z1), boundaryParticles);
	// Front
	addWall(Vector3r(x1, y1, z2), Vector3r(x2, y2, z2), boundaryParticles);
}


void createSphereBuffers(Real radius, int resolution)
{
	Real PI = static_cast<Real>(M_PI);
	// vectors to hold our data
	// vertice positions
	std::vector<Vector3r> v;
	// normals
	std::vector<Vector3r> n;
	std::vector<unsigned short> indices;

	// initiate the variable we are going to use
	Real X1, Y1, X2, Y2, Z1, Z2;
	Real inc1, inc2, inc3, inc4, radius1, radius2;

	for (int w = 0; w < resolution; w++)
	{
		for (int h = (-resolution / 2); h < (resolution / 2); h++)
		{
			inc1 = (w / (Real)resolution) * 2 * PI;
			inc2 = ((w + 1) / (Real)resolution) * 2 * PI;
			inc3 = (h / (Real)resolution)*PI;
			inc4 = ((h + 1) / (Real)resolution)*PI;

			X1 = sin(inc1);
			Y1 = cos(inc1);
			X2 = sin(inc2);
			Y2 = cos(inc2);

			// store the upper and lower radius, remember everything is going to be drawn as triangles
			radius1 = radius*cos(inc3);
			radius2 = radius*cos(inc4);

			Z1 = radius*sin(inc3);
			Z2 = radius*sin(inc4);

			// insert the triangle coordinates
			v.push_back(Vector3r(radius1*X1, Z1, radius1*Y1));
			v.push_back(Vector3r(radius1*X2, Z1, radius1*Y2));
			v.push_back(Vector3r(radius2*X2, Z2, radius2*Y2));

			indices.push_back((unsigned short)v.size() - 3);
			indices.push_back((unsigned short)v.size() - 2);
			indices.push_back((unsigned short)v.size() - 1);

			v.push_back(Vector3r(radius1*X1, Z1, radius1*Y1));
			v.push_back(Vector3r(radius2*X2, Z2, radius2*Y2));
			v.push_back(Vector3r(radius2*X1, Z2, radius2*Y1));

			indices.push_back((unsigned short)v.size() - 3);
			indices.push_back((unsigned short)v.size() - 2);
			indices.push_back((unsigned short)v.size() - 1);

			// insert the normal data
			n.push_back(Vector3r(X1, Z1, Y1));
			n.push_back(Vector3r(X2, Z1, Y2));
			n.push_back(Vector3r(X2, Z2, Y2));
			n.push_back(Vector3r(X1, Z1, Y1));
			n.push_back(Vector3r(X2, Z2, Y2));
			n.push_back(Vector3r(X1, Z2, Y1));
		}
	}

	for (unsigned int i = 0; i < n.size(); i++)
		n[i].normalize();


	glGenBuffersARB(1, &vertexbuffer);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, vertexbuffer);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB, v.size() * sizeof(Vector3r), &v[0], GL_STATIC_DRAW);

	glGenBuffersARB(1, &normalbuffer);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, normalbuffer);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB, n.size() * sizeof(Vector3r), &n[0], GL_STATIC_DRAW);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

	// Generate a buffer for the indices as well
	glGenBuffersARB(1, &elementbuffer);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, elementbuffer);
	glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB, indices.size() * sizeof(unsigned short), &indices[0], GL_STATIC_DRAW);
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);

	// store the number of indices for later use
	vertexBufferSize = (unsigned int)indices.size();

	// clean up after us
	indices.clear();
	n.clear();
	v.clear();
}

void renderSphere(const Vector3r &x, const float color[])
{
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);


	glBindBufferARB(GL_ARRAY_BUFFER_ARB, vertexbuffer);
	glVertexPointer(3, GL_REAL, 0, 0);

	glBindBufferARB(GL_ARRAY_BUFFER_ARB, normalbuffer);
	glNormalPointer(GL_REAL, 0, 0);

	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, elementbuffer);

	glPushMatrix();
	glTranslated(x[0], x[1], x[2]);
	glDrawElements(GL_TRIANGLES, (GLsizei)vertexBufferSize, GL_UNSIGNED_SHORT, 0);
	glPopMatrix();
	glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);

	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void releaseSphereBuffers()
{
	if (elementbuffer != 0)
	{
		glDeleteBuffersARB(1, &elementbuffer);
		elementbuffer = 0;
	}
	if (normalbuffer != 0)
	{
		glDeleteBuffersARB(1, &normalbuffer);
		normalbuffer = 0;
	}
	if (vertexbuffer != 0)
	{
		glDeleteBuffersARB(1, &vertexbuffer);
		vertexbuffer = 0;
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
	((TimeStepFluidModel*)clientData)->setVelocityUpdateMethod((unsigned int)val);
}

void TW_CALL getVelocityUpdateMethod(void *value, void *clientData)
{
	*(short *)(value) = (short)((TimeStepFluidModel*)clientData)->getVelocityUpdateMethod();
}

void TW_CALL setViscosity(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((FluidModel*)clientData)->setViscosity(val);
}

void TW_CALL getViscosity(void *value, void *clientData)
{
	*(Real *)(value) = ((FluidModel*)clientData)->getViscosity();
}

