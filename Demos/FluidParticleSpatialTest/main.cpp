#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/TimeManager.h"
#include "Demos/Common/DemoBase.h"
#include <Eigen/Dense>
#include "FluidModel.h"
#include "TimeStepFluidModel.h"
#include "Simulation/Simulation.h"
#include <iostream>
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
#include "../Common/imguiParameters.h"
#define _USE_MATH_DEFINES
#include "math.h"

#include <algorithm>
#include <fstream>
#include <sstream>

#include <limits>
#undef max
#undef min
#define NOMINMAX

// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
	#define new DEBUG_NEW 
#endif

using namespace PBD;
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
void selection(const Vector2i& start, const Vector2i& end, void* clientData);
void createSphereBuffers(Real radius, int resolution);
void renderSphere(const Vector3r& x, const float color[]);
void releaseSphereBuffers();

FluidModel model;
TimeStepFluidModel simulation;
DemoBase* base;

#define MULT * 0//>> 2
Real particleRadius = static_cast<Real>(0.025);
/*const*/ unsigned int width = 15 + (15 MULT);
/*const*/ unsigned int depth = 15 + (15 MULT);
/*const*/ unsigned int height = 20 + (20 MULT);
/*const*/ Real containerWidth = (width + 1) * particleRadius * static_cast<Real>(2.0 * 5.0);
/*const*/ Real containerDepth = (depth + 1) * particleRadius * static_cast<Real>(2.0);
/*const*/ Real containerHeight = (height + 1 - 5) * particleRadius * static_cast<Real>(2.0 * 5.0);
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

std::ofstream Utilities::graphingData;
string dataFilename = "D:\\projects\\master_thesis\\code\\PBDSpatialPartitioningThesis\\bin\\output\\Fluid demo\\log\\graphingData";

// main 
int main(int argc, char** argv)
{
	printf("#Arguments: %d\n", argc);
	printf("Argument 0: %s\n", argv[0]);

	if (argc != 1)
	{
		if (argc != 3)
		{
			return 0;
		}

		printf("Argument 1: %s\n", argv[1]);

		if (argv[1][0] == 'd')
		{
			width = 15;
			depth = 15;
			height = 20;
		}
		else
		{
			int shift = stoi(argv[1]); //Make into an int
			if (shift < 0)
			{
				width = 15 + (double)(15 >> (-1) * shift);
				depth = 15 + (15 >> (-1) * shift);
				height = 20 + (20 >> (-1) * shift);
			}
			else
			{
				 width = 15 + (double)(15 << shift);
				 depth = 15 + (15 << shift);
				 height = 20 + (20 << shift);
			}
		}

		containerWidth = (width + 1) * particleRadius * static_cast<Real>(2.0 * 5.0);
		containerDepth = (depth + 1) * particleRadius * static_cast<Real>(2.0);
		containerHeight = (height + 1 - 5) * particleRadius * static_cast<Real>(2.0 * 5.0);

		printf("Argument 2: %s\n", argv[2]);
		dataFilename += string(argv[2]);
	}

#ifdef TAKETIME
	Utilities::graphingData.open(dataFilename + ".csv", std::ios::out);
	if (Utilities::graphingData.fail())
		std::cerr << "Failed to open file: graphingData.csv\n";
#endif // TAKETIME

	base = new DemoBase();
	base->init(argc, argv, "Fluid demo");

	printf("%d, %d, %d\n", width, depth, height);
	printf("%f, %f, %f\n", containerWidth, containerDepth, containerHeight);

	// we use an own time step controller
	delete PBD::Simulation::getCurrent()->getTimeStep();
	PBD::Simulation::getCurrent()->setTimeStep(nullptr);

	MiniGL::setSelectionFunc(selection, nullptr);
	MiniGL::setClientIdleFunc(timeStep);
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);
	MiniGL::setViewport(40.0, 0.1f, 500.0, Vector3r(0.0, 3.0, 8.0), Vector3r(0.0, 0.0, 0.0));
	buildModel();

	base->createParameterGUI();

	// add additional parameter just for this demo
	imguiParameters::imguiEnumParameter* eparam = new imguiParameters::imguiEnumParameter();
	eparam->description = "Velocity update method";
	eparam->label = "Velocity update method";
	eparam->getFct = [&]() -> int { return simulation.getVelocityUpdateMethod(); };
	eparam->setFct = [&](int i) -> void { simulation.setVelocityUpdateMethod(i); };
	eparam->items.push_back("First Order Update");
	eparam->items.push_back("Second Order Update");
	imguiParameters::addParam("Simulation", "PBD", eparam);

	imguiParameters::imguiNumericParameter<Real>* param = new imguiParameters::imguiNumericParameter<Real>();
	param->description = "Viscosity coefficient";
	param->label = "Viscosity";
	param->getFct = [&]() -> Real { return model.getViscosity(); };
	param->setFct = [&](Real v) -> void { model.setViscosity(v); };
	imguiParameters::addParam("Simulation", "PBD", param);

	MiniGL::getOpenGLVersion(context_major_version, context_minor_version);
	if (context_major_version >= 3)
		createSphereBuffers((Real)particleRadius, 8);

	base->setValue<bool>(DemoBase::PAUSE, false);

	MiniGL::mainLoop();

#ifdef TAKETIME
	Utilities::graphingData.close();
#endif // TAKETIME

	cleanup();
	base->cleanup();

	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();
	delete base;
	delete Simulation::getCurrent();

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

void selection(const Vector2i &start, const Vector2i &end, void *clientData)
{
	std::vector<unsigned int> hits;
	selectedParticles.clear();
	ParticleData &pd = model.getParticles();
	Selection::selectRect(start, end, &pd.getPosition(0), &pd.getPosition(pd.size() - 1), selectedParticles);
	if (selectedParticles.size() > 0)
		MiniGL::setMouseMoveFunc(2, mouseMove);
	else
		MiniGL::setMouseMoveFunc(-1, NULL);

	MiniGL::unproject(end[0], end[1], oldMousePos);
}

#ifdef TAKETIME
int numberOfTimeSteps = 0;
int numResets = 0;
#define STOPATTIMESTEP 512
#define RESETNUM 2
#endif // TAKETIME
void timeStep ()
{
	/*const Real pauseAt = base->getValue<Real>(DemoBase::PAUSE_AT);
	if ((pauseAt > 0.0) && (pauseAt < TimeManager::getCurrent()->getTime()))
		base->setValue(DemoBase::PAUSE, true);*/

#ifdef TAKETIME
	if (numberOfTimeSteps == STOPATTIMESTEP)
	{
		if (numResets < RESETNUM)
		{
			numResets++;
			numberOfTimeSteps = 0;
			//graphingData << "\nHARD RESET TIME BABY,\n";
			Utilities::graphingData.close();
			Utilities::graphingData.open(dataFilename + ".csv", std::ios::out);
			if (Utilities::graphingData.fail())
				std::cerr << "Failed to open file: graphingData.csv\n";
			reset();
		}
		else
		{
			MiniGL::leaveMainLoop();
		}
	}
#endif // TAKETIME

	if (base->getValue<bool>(DemoBase::PAUSE))
	{
		//printf("Simulation is PAUSED\n");
		return;
	}
		

	// Simulation code
	const unsigned int numSteps = base->getValue<unsigned int>(DemoBase::NUM_STEPS_PER_RENDER);
	for (unsigned int i = 0; i < numSteps; i++)
	{
#ifdef TAKETIME
		graphingData << numberOfTimeSteps << ",";
		//if (numberOfTimeSteps == 0) graphingData << "0,0,0,0,";
		START_TIMING("SimStep");
#endif // TAKETIME

		simulation.step(model);

#ifdef TAKETIME
		STOP_TIMING_AVG;
		graphingData << "\n";
		numberOfTimeSteps++;
#endif // TAKETIME

		base->step();
	}
//#ifdef TAKETIME
//	Timing::printAverageTimes();
//#endif // TAKETIME
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.0025));

	createBreakingDam();
}

void render ()
{
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
	base->render();
}

const Real nudge = 0;// particleRadius * 2.1;
/** Create a breaking dam scenario
*/
void createBreakingDam()
{
	LOG_INFO << "Initialize fluid particles";
	const Real diam = 2.0*particleRadius;
	/*const Real startX = -static_cast<Real>(0.5)*containerWidth + diam;
	const Real startY = diam;
	const Real startZ = -static_cast<Real>(0.5)*containerDepth + diam;*/
	/*const Real startX = diam;
	const Real startY = diam;
	const Real startZ = diam;*/
	const Real startX = diam + nudge * 2;
	const Real startY = diam + nudge * 2;
	const Real startZ = diam + nudge * 2;
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

	//Real* temp = fluidParticles[0].data();

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

bool vec3Comp(Vector3r i, Vector3r j) {
	if (i[0] < j[0]) return true;
	else if (i[0] > j[0]) return false;
	else
	{
		if (i[1] < j[1]) return true;
		else if (i[1] > j[1]) return false;
		else
		{
			return i[2] < j[2];
		}
	}
}

void initBoundaryData(std::vector<Vector3r> &boundaryParticles)
{
	/*const Real x1 = -containerWidth / 2.0;
	const Real x2 = containerWidth / 2.0;
	const Real y1 = 0.0;
	const Real y2 = containerHeight;
	const Real z1 = -containerDepth / 2.0;
	const Real z2 = containerDepth / 2.0;*/
	const Real x1 = 0.0 + nudge * 2;// + 0.03125f;
	const Real x2 = containerWidth + nudge * 2;// + 0.03125f;
	const Real y1 = 0.0 + nudge * 2;// + 0.03125f;
	const Real y2 = containerHeight + nudge * 2;// + 0.03125f;
	const Real z1 = 0.0 + nudge * 2;// + 0.03125f;
	const Real z2 = containerDepth + nudge * 2;// + 0.03125f;

	const Real diam = 2.0 * particleRadius;

	// Floor
	addWall(Vector3r(x1, y1 - nudge, z1), Vector3r(x2, y1 - nudge, z2), boundaryParticles);
	// Top
	addWall(Vector3r(x1, y2 + nudge, z1), Vector3r(x2, y2 + nudge, z2), boundaryParticles);
	// Left
	addWall(Vector3r(x1 - nudge, y1, z1), Vector3r(x1 - nudge, y2, z2), boundaryParticles);
	// Right
	addWall(Vector3r(x2 + nudge, y1, z1), Vector3r(x2 + nudge, y2, z2), boundaryParticles);
	// Back
	addWall(Vector3r(x1, y1, z1 - nudge), Vector3r(x2, y2, z1 - nudge), boundaryParticles);
	// Front
	addWall(Vector3r(x1, y1, z2 + nudge), Vector3r(x2, y2, z2 + nudge), boundaryParticles);
	
	//// Floor
	//addWall(Vector3r(x1, y1, z1), Vector3r(x2, y1, z2), boundaryParticles);
	//// Top
	//addWall(Vector3r(x1, y2, z1), Vector3r(x2, y2, z2), boundaryParticles);
	//// Left
	//addWall(Vector3r(x1, y1 + diam, z1), Vector3r(x1, y2 - diam, z2), boundaryParticles);
	//// Right
	//addWall(Vector3r(x2, y1 + diam, z1), Vector3r(x2, y2 - diam, z2), boundaryParticles);
	//// Back												 
	//addWall(Vector3r(x1 + diam, y1 + diam, z1), Vector3r(x2 - diam, y2 - diam, z1), boundaryParticles);
	//// Front											 
	//addWall(Vector3r(x1 + diam, y1 + diam, z2), Vector3r(x2 - diam, y2 - diam, z2), boundaryParticles);

	/*std::sort(boundaryParticles.begin(), boundaryParticles.end(), vec3Comp);

	for (Vector3r p : boundaryParticles)
	{
		printf("(%f, %f, %f)\n", p[0], p[1], p[2]);
	}
	printf("\n");

	bool duplics = false;
	auto point = boundaryParticles.begin();
	while (point != boundaryParticles.end())
	{
		point = std::adjacent_find(point, boundaryParticles.end());
		if (point == boundaryParticles.end()) break;
		printf("There are duplicates of (%f, %f, %f)\n", (*point)[0], (*point)[1], (*point)[2]);
		duplics = true;
	}
	if (!duplics) printf("There are no duplicates!\n");*/
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


	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(Vector3r), &v[0], GL_STATIC_DRAW);

	glGenBuffers(1, &normalbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glBufferData(GL_ARRAY_BUFFER, n.size() * sizeof(Vector3r), &n[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// Generate a buffer for the indices as well
	glGenBuffers(1, &elementbuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned short), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

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


	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glVertexPointer(3, GL_REAL, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glNormalPointer(GL_REAL, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);

	glPushMatrix();
	glTranslated(x[0], x[1], x[2]);
	glDrawElements(GL_TRIANGLES, (GLsizei)vertexBufferSize, GL_UNSIGNED_SHORT, 0);
	glPopMatrix();
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void releaseSphereBuffers()
{
	if (elementbuffer != 0)
	{
		glDeleteBuffers(1, &elementbuffer);
		elementbuffer = 0;
	}
	if (normalbuffer != 0)
	{
		glDeleteBuffers(1, &normalbuffer);
		normalbuffer = 0;
	}
	if (vertexbuffer != 0)
	{
		glDeleteBuffers(1, &vertexbuffer);
		vertexbuffer = 0;
	}
}
