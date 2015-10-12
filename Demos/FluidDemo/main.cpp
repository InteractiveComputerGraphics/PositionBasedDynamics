#include "Demos/Utils/Config.h"
#include "GL/glew.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Demos/Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "FluidModel.h"
#include "TimeStepFluidModel.h"
#include <iostream>

// Enable memory leak detection
#ifdef _DEBUG
	#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;

void timeStep ();
void buildModel ();
void createBreakingDam();
void addWall(const Eigen::Vector3f &minX, const Eigen::Vector3f &maxX, std::vector<Eigen::Vector3f> &boundaryParticles);
void initBoundaryData(std::vector<Eigen::Vector3f> &boundaryParticles);
void render ();
void cleanup();
void reset();
void hsvToRgb(float h, float s, float v, float *rgb);
void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
void createSphereBuffers(float radius, int resolution);
void renderSphere(const Eigen::Vector3f &x, const float color[]);
void releaseSphereBuffers();
void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData);
void TW_CALL getVelocityUpdateMethod(void *value, void *clientData);
void TW_CALL setViscosity(const void *value, void *clientData);
void TW_CALL getViscosity(void *value, void *clientData);



FluidModel model;
TimeStepFluidModel simulation;

const float particleRadius = 0.025f;
const unsigned int width = 15;
const unsigned int depth = 15;
const unsigned int height = 20;
const float containerWidth = (width + 1)*particleRadius*2.0f * 5.0f;
const float containerDepth = (depth + 1)*particleRadius*2.0f;
const float containerHeight = 4.0f;
bool doPause = true;
std::vector<unsigned int> selectedParticles;
Eigen::Vector3f oldMousePos;
// initiate buffers
GLuint elementbuffer;
GLuint normalbuffer;
GLuint vertexbuffer;
int vertexBufferSize = 0;
GLint context_major_version, context_minor_version;


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	// OpenGL
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Fluid demo");
	MiniGL::initLights ();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection);

	MiniGL::getOpenGLVersion(context_major_version, context_minor_version);

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3f (0.0, -3.0, 8.0), Vector3f (0.0, 0.0, 0.0));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_FLOAT, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &simulation, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "Viscosity", TW_TYPE_FLOAT, setViscosity, getViscosity, &model, " label='Viscosity'  min=0.0 max = 0.5 step=0.001 precision=4 group=Simulation ");

	buildModel();

	if (context_major_version >= 3)
		createSphereBuffers((float)particleRadius, 8);

	glutMainLoop ();	

	cleanup ();
	
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
	model.reset();
	simulation.reset();
	TimeManager::getCurrent()->setTime(0.0);
}

void mouseMove(int x, int y)
{
	Eigen::Vector3f mousePos;
	MiniGL::unproject(x, y, mousePos);
	const Eigen::Vector3f diff = mousePos - oldMousePos;

	TimeManager *tm = TimeManager::getCurrent();
	const float h = tm->getTimeStepSize();

	ParticleData &pd = model.getParticles();
	for (unsigned int j = 0; j < selectedParticles.size(); j++)
	{
		pd.getVelocity(selectedParticles[j]) += 5.0*diff/h;
	}
	oldMousePos = mousePos;
}

void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end)
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
	for (unsigned int i = 0; i < 4; i++)
		simulation.step(model);
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.0025f);

	createBreakingDam();
}

void hsvToRgb(float h, float s, float v, float *rgb)
{
	int i = (int)floor(h * 6);
	float f = h * 6 - i;
	float p = v * (1 - s);
	float q = v * (1 - f * s);
	float t = v * (1 - (1 - f) * s);

	switch (i % 6)
	{
	case 0: rgb[0] = v, rgb[1] = t, rgb[2] = p; break;
	case 1: rgb[0] = q, rgb[1] = v, rgb[2] = p; break;
	case 2: rgb[0] = p, rgb[1] = v, rgb[2] = t; break;
	case 3: rgb[0] = p, rgb[1] = q, rgb[2] = v; break;
	case 4: rgb[0] = t, rgb[1] = p, rgb[2] = v; break;
	case 5: rgb[0] = v, rgb[1] = p, rgb[2] = q; break;
	}
}

void render ()
{
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

	glPointSize(4.0f);

	const float supportRadius = model.getSupportRadius();
	float vmax = 0.4f*2.0f*supportRadius / TimeManager::getCurrent()->getTimeStepSize();
	float vmin = 0.0f;

	if (context_major_version > 3)
	{
		for (unsigned int i = 0; i < nParticles; i++)
		{
			float v = pd.getVelocity(i).norm();
			v = 0.5f*((v - vmin) / (vmax - vmin));
			v = min(128.0f*v*v, 0.5f);
			float fluidColor[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
			hsvToRgb(0.55f, 1.0f, 0.5f + (float)v, fluidColor);
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
			float v = pd.getVelocity(i).norm();
			v = 0.5f*((v - vmin) / (vmax - vmin));
			v = min(128.0f*v*v, 0.5f);
			float fluidColor[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
			hsvToRgb(0.55f, 1.0f, 0.5f + (float)v, fluidColor);

			glColor3fv(fluidColor);
			glVertex3fv(&pd.getPosition(i)[0]);
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
	std::cout << "Initialize fluid particles\n";
	const float diam = 2.0f*particleRadius;
	const float startX = -0.5f*containerWidth + diam;
	const float startY = diam;
	const float startZ = -0.5f*containerDepth + diam;
	const float yshift = sqrt(3.0f) * particleRadius;

	std::vector<Eigen::Vector3f> fluidParticles;
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
					fluidParticles[i*height*depth + j*depth + k] = diam*Eigen::Vector3f((float)i, (float)j, (float)k) + Eigen::Vector3f(startX, startY, startZ);
				}
			}
		}
	}

	model.setParticleRadius(particleRadius);

	std::vector<Eigen::Vector3f> boundaryParticles;
	initBoundaryData(boundaryParticles);

	model.initModel((unsigned int)fluidParticles.size(), fluidParticles.data(), (unsigned int)boundaryParticles.size(), boundaryParticles.data());

	std::cout << "Number of particles: " << width*height*depth << "\n";
}


void addWall(const Eigen::Vector3f &minX, const Eigen::Vector3f &maxX, std::vector<Eigen::Vector3f> &boundaryParticles)
{
	const float particleDistance = 2.0f*model.getParticleRadius();

	const Eigen::Vector3f diff = maxX - minX;
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
					const Eigen::Vector3f currPos = minX + Eigen::Vector3f(j*particleDistance, k*particleDistance, l*particleDistance);
					boundaryParticles[startIndex + j*stepsY*stepsZ + k*stepsZ + l] = currPos;
				}
			}
		}
	}
}

void initBoundaryData(std::vector<Eigen::Vector3f> &boundaryParticles)
{
	const float x1 = -containerWidth / 2.0f;
	const float x2 = containerWidth / 2.0f;
	const float y1 = 0.0f;
	const float y2 = containerHeight;
	const float z1 = -containerDepth / 2.0f;
	const float z2 = containerDepth / 2.0f;

	const float diam = 2.0f*particleRadius;

	// Floor
	addWall(Eigen::Vector3f(x1, y1, z1), Eigen::Vector3f(x2, y1, z2), boundaryParticles);
	// Top
	addWall(Eigen::Vector3f(x1, y2, z1), Eigen::Vector3f(x2, y2, z2), boundaryParticles);
	// Left
	addWall(Eigen::Vector3f(x1, y1, z1), Eigen::Vector3f(x1, y2, z2), boundaryParticles);
	// Right
	addWall(Eigen::Vector3f(x2, y1, z1), Eigen::Vector3f(x2, y2, z2), boundaryParticles);
	// Back
	addWall(Eigen::Vector3f(x1, y1, z1), Eigen::Vector3f(x2, y2, z1), boundaryParticles);
	// Front
	addWall(Eigen::Vector3f(x1, y1, z2), Eigen::Vector3f(x2, y2, z2), boundaryParticles);
}


void createSphereBuffers(float radius, int resolution)
{
	float PI = static_cast<float>(M_PI);
	// vectors to hold our data
	// vertice positions
	std::vector<Eigen::Vector3f> v;
	// normals
	std::vector<Eigen::Vector3f> n;
	std::vector<unsigned short> indices;

	// initiate the variable we are going to use
	float X1, Y1, X2, Y2, Z1, Z2;
	float inc1, inc2, inc3, inc4, radius1, radius2;

	for (int w = 0; w < resolution; w++)
	{
		for (int h = (-resolution / 2); h < (resolution / 2); h++)
		{
			inc1 = (w / (float)resolution) * 2 * PI;
			inc2 = ((w + 1) / (float)resolution) * 2 * PI;
			inc3 = (h / (float)resolution)*PI;
			inc4 = ((h + 1) / (float)resolution)*PI;

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
			v.push_back(Eigen::Vector3f(radius1*X1, Z1, radius1*Y1));
			v.push_back(Eigen::Vector3f(radius1*X2, Z1, radius1*Y2));
			v.push_back(Eigen::Vector3f(radius2*X2, Z2, radius2*Y2));

			indices.push_back((unsigned short)v.size() - 3);
			indices.push_back((unsigned short)v.size() - 2);
			indices.push_back((unsigned short)v.size() - 1);

			v.push_back(Eigen::Vector3f(radius1*X1, Z1, radius1*Y1));
			v.push_back(Eigen::Vector3f(radius2*X2, Z2, radius2*Y2));
			v.push_back(Eigen::Vector3f(radius2*X1, Z2, radius2*Y1));

			indices.push_back((unsigned short)v.size() - 3);
			indices.push_back((unsigned short)v.size() - 2);
			indices.push_back((unsigned short)v.size() - 1);

			// insert the normal data
			n.push_back(Eigen::Vector3f(X1, Z1, Y1));
			n.push_back(Eigen::Vector3f(X2, Z1, Y2));
			n.push_back(Eigen::Vector3f(X2, Z2, Y2));
			n.push_back(Eigen::Vector3f(X1, Z1, Y1));
			n.push_back(Eigen::Vector3f(X2, Z2, Y2));
			n.push_back(Eigen::Vector3f(X1, Z2, Y1));
		}
	}

	for (unsigned int i = 0; i < n.size(); i++)
		n[i].normalize();


	glGenBuffersARB(1, &vertexbuffer);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, vertexbuffer);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB, v.size() * sizeof(Eigen::Vector3f), &v[0], GL_STATIC_DRAW);

	glGenBuffersARB(1, &normalbuffer);
	glBindBufferARB(GL_ARRAY_BUFFER_ARB, normalbuffer);
	glBufferDataARB(GL_ARRAY_BUFFER_ARB, n.size() * sizeof(Eigen::Vector3f), &n[0], GL_STATIC_DRAW);
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

void renderSphere(const Eigen::Vector3f &x, const float color[])
{
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);


	glBindBufferARB(GL_ARRAY_BUFFER_ARB, vertexbuffer);
	glVertexPointer(3, GL_FLOAT, 0, 0);

	glBindBufferARB(GL_ARRAY_BUFFER_ARB, normalbuffer);
	glNormalPointer(GL_FLOAT, 0, 0);

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
	((TimeStepFluidModel*)clientData)->setVelocityUpdateMethod((unsigned int)val);
}

void TW_CALL getVelocityUpdateMethod(void *value, void *clientData)
{
	*(short *)(value) = (short)((TimeStepFluidModel*)clientData)->getVelocityUpdateMethod();
}

void TW_CALL setViscosity(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((FluidModel*)clientData)->setViscosity(val);
}

void TW_CALL getViscosity(void *value, void *clientData)
{
	*(float *)(value) = ((FluidModel*)clientData)->getViscosity();
}

