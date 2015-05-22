#include "Demos/Utils/Config.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Demos/Utils/TimeManager.h"
#include <Eigen/Dense>
#include "TetModel.h"
#include "TimeStepTetModel.h"
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
void createMesh();
void render ();
void cleanup();
void reset();
void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
void TW_CALL setStiffness(const void *value, void *clientData);
void TW_CALL getStiffness(void *value, void *clientData);
void TW_CALL setPoissonRatio(const void *value, void *clientData);
void TW_CALL getPoissonRatio(void *value, void *clientData);
void TW_CALL setNormalizeStretch(const void *value, void *clientData);
void TW_CALL getNormalizeStretch(void *value, void *clientData);
void TW_CALL setNormalizeShear(const void *value, void *clientData);
void TW_CALL getNormalizeShear(void *value, void *clientData);
void TW_CALL setSimulationMethod(const void *value, void *clientData);
void TW_CALL getSimulationMethod(void *value, void *clientData);
void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData);
void TW_CALL getVelocityUpdateMethod(void *value, void *clientData);

TetModel model;
TimeStepTetModel simulation;

const unsigned int width = 30;
const unsigned int depth = 5;
const unsigned int height = 5; 
bool pause = true;
std::vector<unsigned int> selectedParticles;
Eigen::Vector3f oldMousePos;

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	// OpenGL
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Bar demo");
	MiniGL::initLights ();
	MiniGL::initTexture();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection);

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3f (5.0, -10.0, 30.0), Vector3f (5.0, 0.0, 0.0));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &pause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_FLOAT, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &simulation, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwType enumType2 = TwDefineEnum("SimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "SimulationMethod", enumType2, setSimulationMethod, getSimulationMethod, &simulation, 
			" label='Simulation method' enum='0 {None}, 1 {Volume constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics (no inversion handling)}, 4 {Shape matching (no inversion handling)}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "Stiffness", TW_TYPE_FLOAT, setStiffness, getStiffness, &model, " label='Stiffness'  min=0.0 step=0.1 precision=4 group='Simulation' ");
	TwAddVarCB(MiniGL::getTweakBar(), "PoissonRatio", TW_TYPE_FLOAT, setPoissonRatio, getPoissonRatio, &model, " label='Poisson ratio XY'  min=0.0 step=0.1 precision=4 group='Simulation' ");
	TwAddVarCB(MiniGL::getTweakBar(), "NormalizeStretch", TW_TYPE_BOOL32, setNormalizeStretch, getNormalizeStretch, &model, " label='Normalize stretch' group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "NormalizeShear", TW_TYPE_BOOL32, setNormalizeShear, getNormalizeShear, &model, " label='Normalize shear' group='Strain based dynamics' ");

	glutMainLoop ();	

	cleanup ();

	return 0;
}

void cleanup()
{
	delete TimeManager::getCurrent();
}

void reset()
{
	model.reset();
	simulation.reset(model);
	TimeManager::getCurrent()->setTime(0.0);
}

void mouseMove(int x, int y)
{
	Eigen::Vector3f mousePos;
	MiniGL::unproject(x, y, mousePos);
	const Eigen::Vector3f diff = mousePos - oldMousePos;

	TimeManager *tm = TimeManager::getCurrent();
	const float h = tm->getTimeStepSize();

	ParticleData &pd = model.getParticleMesh().getVertexData();
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
	ParticleData &pd = model.getParticleMesh().getVertexData();
	Selection::selectRect(start, end, &pd.getPosition(0), &pd.getPosition(pd.size() - 1), selectedParticles);
	if (selectedParticles.size() > 0)
		MiniGL::setMouseMoveFunc(GLUT_MIDDLE_BUTTON, mouseMove);
	else
		MiniGL::setMouseMoveFunc(-1, NULL);

	MiniGL::unproject(end[0], end[1], oldMousePos);
}

void timeStep ()
{
	if (pause)
		return;

	// Simulation code
	for (unsigned int i = 0; i < 4; i++)
		simulation.step(model);

 	IndexedFaceMesh<VertexData> &visMesh = model.getVisMesh();
 	const ParticleData &pd = model.getParticleMesh().getVertexData();
 	VertexData &vd = model.getVisMesh().getVertexData();
 	for (unsigned int i = 0; i < pd.size(); i++)
 		vd.getPosition(i) = pd.getPosition(i);
 	visMesh.updateVertexNormals();
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.005f);

	createMesh();
}


void render ()
{
	MiniGL::coordinateSystem();
	
	// Draw simulation model
	
	// mesh 
	const ParticleData &pd = model.getParticleMesh().getVertexData();
	const IndexedFaceMesh<VertexData> &visMesh = model.getVisMesh();
	const unsigned int *faces = visMesh.getFaces().data();
	const unsigned int nFaces = visMesh.numFaces();
	const Eigen::Vector3f *vertexNormals = visMesh.getVertexNormals().data();

	float surfaceColor[4] = { 0.2f, 0.5f, 1.0f, 1 };
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, surfaceColor);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, surfaceColor);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(surfaceColor);

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, &pd.getPosition(0)[0]);
	glNormalPointer(GL_FLOAT, 0, &vertexNormals[0][0]);
	glDrawElements(GL_TRIANGLES, (GLsizei)3 * visMesh.numFaces(), GL_UNSIGNED_INT, visMesh.getFaces().data());
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
 
 	float red[4] = { 0.8f, 0.0f, 0.0f, 1 };
 	for (unsigned int j = 0; j < selectedParticles.size(); j++)
 	{
 		MiniGL::drawSphere(pd.getPosition(selectedParticles[j]), 0.08f, red);
 	}

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
}


void createMesh()
{
	Eigen::Vector3f points[width*height*depth];
	for (unsigned int i = 0; i < width; i++)
	{
		for (unsigned int j = 0; j < height; j++)
		{
			for (unsigned int k = 0; k < depth; k++)
			{
				points[i*height*depth + j*depth + k] = 0.3f*Eigen::Vector3f((float)i, (float)j, (float)k);
			}
		}
	}

	vector<unsigned int> indices;
	for (unsigned int i = 0; i < width - 1; i++)
	{
		for (unsigned int j = 0; j < height - 1; j++)
		{
			for (unsigned int k = 0; k < depth - 1; k++)
			{
				// For each block, the 8 corners are numerated as:
				//     4*-----*7
				//     /|    /|
				//    / |   / |
				//  5*-----*6 |
				//   | 0*--|--*3
				//   | /   | /
				//   |/    |/
				//  1*-----*2
				unsigned int p0 = i*height*depth + j*depth + k;
				unsigned int p1 = p0 + 1;
				unsigned int p3 = (i + 1)*height*depth + j*depth + k;
				unsigned int p2 = p3 + 1;
				unsigned int p7 = (i + 1)*height*depth + (j + 1)*depth + k;
				unsigned int p6 = p7 + 1;
				unsigned int p4 = i*height*depth + (j + 1)*depth + k;
				unsigned int p5 = p4 + 1;

				// Ensure that neighboring tetras are sharing faces
				if ((i + j + k) % 2 == 1)
				{
					indices.push_back(p2); indices.push_back(p1); indices.push_back(p6); indices.push_back(p3);
					indices.push_back(p6); indices.push_back(p3); indices.push_back(p4); indices.push_back(p7);
					indices.push_back(p4); indices.push_back(p1); indices.push_back(p6); indices.push_back(p5);
					indices.push_back(p3); indices.push_back(p1); indices.push_back(p4); indices.push_back(p0);
					indices.push_back(p6); indices.push_back(p1); indices.push_back(p4); indices.push_back(p3);
				}
				else
				{
					indices.push_back(p0); indices.push_back(p2); indices.push_back(p5); indices.push_back(p1);
					indices.push_back(p7); indices.push_back(p2); indices.push_back(p0); indices.push_back(p3);
					indices.push_back(p5); indices.push_back(p2); indices.push_back(p7); indices.push_back(p6);
					indices.push_back(p7); indices.push_back(p0); indices.push_back(p5); indices.push_back(p4);
					indices.push_back(p0); indices.push_back(p2); indices.push_back(p7); indices.push_back(p5);
				}
			}
		}
	}
	model.setGeometry(width*height*depth, points, (unsigned int) indices.size()/4u, indices.data());

	TetModel::ParticleMesh &mesh = model.getParticleMesh();
	ParticleData &pd = mesh.getVertexData();
	for (unsigned int i = 0; i < pd.getNumberOfParticles(); i++)
	{
		pd.setMass(i, 1.0);
	}
	for (unsigned int i = 0; i < 1; i++)
	{
		for (unsigned int j = 0; j < height; j++)
		{
			for (unsigned int k = 0; k < depth; k++)
				pd.setMass(i*height*depth + j*depth + k, 0.0);
		}
	}
	model.initConstraints();

	std::cout << "Number of tets: " << indices.size() / 4 << "\n";
	std::cout << "Number of vertices: " << width*height*depth << "\n";

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

void TW_CALL setStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((TetModel*) clientData)->setStiffness(val);
}

void TW_CALL getStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((TetModel*)clientData)->getStiffness();
}

void TW_CALL setPoissonRatio(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((TetModel*)clientData)->setPoissonRatio(val);
}

void TW_CALL getPoissonRatio(void *value, void *clientData)
{
	*(float *)(value) = ((TetModel*)clientData)->getPoissonRatio();
}

void TW_CALL setNormalizeStretch(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((TetModel*)clientData)->setNormalizeStretch(val);
}

void TW_CALL getNormalizeStretch(void *value, void *clientData)
{
	*(bool *)(value) = ((TetModel*)clientData)->getNormalizeStretch();
}

void TW_CALL setNormalizeShear(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((TetModel*)clientData)->setNormalizeShear(val);
}

void TW_CALL getNormalizeShear(void *value, void *clientData)
{
	*(bool *)(value) = ((TetModel*)clientData)->getNormalizeShear();
}

void TW_CALL setSimulationMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	((TimeStepTetModel*)clientData)->setSimulationMethod((unsigned int)val);
}

void TW_CALL getSimulationMethod(void *value, void *clientData)
{
	*(short *)(value) = (short)((TimeStepTetModel*)clientData)->getSimulationMethod();
}

void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	((TimeStepTetModel*)clientData)->setVelocityUpdateMethod((unsigned int)val);
}

void TW_CALL getVelocityUpdateMethod(void *value, void *clientData)
{
	*(short *)(value) = (short)((TimeStepTetModel*)clientData)->getVelocityUpdateMethod();
}