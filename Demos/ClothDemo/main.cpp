#include "Demos/Utils/Config.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Demos/Utils/TimeManager.h"
#include <Eigen/Dense>
#include "TriangleModel.h"
#include "TimeStepTriangleModel.h"

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

TriangleModel model;
TimeStepTriangleModel simulation;

const int nRows = 30;
const int nCols = 30;
const float width = 10.0f;
const float height = 10.0f;
bool pause = true;
std::vector<unsigned int> selectedParticles;
Eigen::Vector3f oldMousePos;

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	// OpenGL
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Cloth demo");
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
	TwType enumType = TwDefineEnum("SimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "SimulationMethod", enumType, setSimulationMethod, getSimulationMethod, &simulation, " label='Simulation method' enum='0 {None}, 1 {Distance constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "Stiffness", TW_TYPE_FLOAT, setStiffness, getStiffness, &model, " label='Stiffness'  min=0.0 step=0.1 precision=4 group='Distance constraints' ");
	TwAddVarCB(MiniGL::getTweakBar(), "XXStiffness", TW_TYPE_FLOAT, setXXStiffness, getXXStiffness, &model, " label='Stiffness XX'  min=0.0 step=0.1 precision=4 group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "YYStiffness", TW_TYPE_FLOAT, setYYStiffness, getYYStiffness, &model, " label='Stiffness YY'  min=0.0 step=0.1 precision=4 group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "XYStiffness", TW_TYPE_FLOAT, setXYStiffness, getXYStiffness, &model, " label='Stiffness XY'  min=0.0 step=0.1 precision=4 group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "XXStiffnessFEM", TW_TYPE_FLOAT, setXXStiffness, getXXStiffness, &model, " label='Youngs modulus XX'  min=0.0 step=0.1 precision=4 group='FEM based PBD' ");
	TwAddVarCB(MiniGL::getTweakBar(), "YYStiffnessFEM", TW_TYPE_FLOAT, setYYStiffness, getYYStiffness, &model, " label='Youngs modulus YY'  min=0.0 step=0.1 precision=4 group='FEM based PBD' ");
	TwAddVarCB(MiniGL::getTweakBar(), "XYStiffnessFEM", TW_TYPE_FLOAT, setXYStiffness, getXYStiffness, &model, " label='Youngs modulus XY'  min=0.0 step=0.1 precision=4 group='FEM based PBD' ");
	TwAddVarCB(MiniGL::getTweakBar(), "XYPoissonRatioFEM", TW_TYPE_FLOAT, setXYPoissonRatio, getXYPoissonRatio, &model, " label='Poisson ratio XY'  min=0.0 step=0.1 precision=4 group='FEM based PBD' ");
	TwAddVarCB(MiniGL::getTweakBar(), "YXPoissonRatioFEM", TW_TYPE_FLOAT, setYXPoissonRatio, getYXPoissonRatio, &model, " label='Poisson ratio YX'  min=0.0 step=0.1 precision=4 group='FEM based PBD' ");
	TwAddVarCB(MiniGL::getTweakBar(), "NormalizeStretch", TW_TYPE_BOOL32, setNormalizeStretch, getNormalizeStretch, &model, " label='Normalize stretch' group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "NormalizeShear", TW_TYPE_BOOL32, setNormalizeShear, getNormalizeShear, &model, " label='Normalize shear' group='Strain based dynamics' ");
	TwType enumType2 = TwDefineEnum("BendingMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "BendingMethod", enumType2, setBendingMethod, getBendingMethod, &simulation, " label='Bending method' enum='0 {None}, 1 {Dihedral angle}, 2 {Isometric bending}' group=Bending");
	TwAddVarCB(MiniGL::getTweakBar(), "BendingStiffness", TW_TYPE_FLOAT, setBendingStiffness, getBendingStiffness, &model, " label='Bending stiffness'  min=0.0 step=0.01 precision=4 group=Bending ");

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
	model.getParticleMesh().updateVertexNormals();
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
	const IndexedFaceMesh<ParticleData> &mesh = model.getParticleMesh();
	const unsigned int *faces = mesh.getFaces().data();
	const unsigned int nFaces = mesh.numFaces();
	const Eigen::Vector3f *vertexNormals = mesh.getVertexNormals().data();
	const Eigen::Vector2f *uvs = mesh.getUVs().data();

	float surfaceColor[4] = { 0.2f, 0.5f, 1.0f, 1 };
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, surfaceColor);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, surfaceColor);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(surfaceColor);

	MiniGL::bindTexture();

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, &pd.getPosition(0)[0]);
	glTexCoordPointer(2, GL_FLOAT, 0, &uvs[0][0]);
	glNormalPointer(GL_FLOAT, 0, &vertexNormals[0][0]);
	glDrawElements(GL_TRIANGLES, (GLsizei)3 * mesh.numFaces(), GL_UNSIGNED_INT, mesh.getFaces().data());
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);

	MiniGL::unbindTexture();

	float red[4] = { 0.8f, 0.0f, 0.0f, 1 };
	for (unsigned int j = 0; j < selectedParticles.size(); j++)
	{
		MiniGL::drawSphere(pd.getPosition(selectedParticles[j]), 0.08f, red);
	}

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
}


/** Create a particle model mesh 
*/
void createMesh()
{
	TriangleModel::ParticleMesh::UVs uvs;
	uvs.resize(nRows*nCols);

	const float dy = width / (float)(nCols-1);
	const float dx = height / (float)(nRows-1);

	Eigen::Vector3f points[nRows*nCols];
	for (int i = 0; i < nRows; i++)
	{
		for (int j = 0; j < nCols; j++)
		{
			const float y = (float)dy*j;
			const float x = (float)dx*i;
			points[i*nCols + j] = Eigen::Vector3f(x, 1.0, y);

			uvs[i*nCols + j][0] = x/width;
			uvs[i*nCols + j][1] = y/height;
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
	model.setGeometry(nRows*nCols, &points[0], nIndices / 3, &indices[0], uvIndices, uvs);
	
	TriangleModel::ParticleMesh &mesh = model.getParticleMesh();
	ParticleData &pd = mesh.getVertexData();
	for (unsigned int i = 0; i < pd.getNumberOfParticles(); i++)
	{
		pd.setMass(i, 1.0);
	}

	// Set mass of points to zero => make it static
	pd.setMass(0, 0.0);
	pd.setMass((nRows-1)*nCols, 0.0);
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
	((TriangleModel*) clientData)->setStiffness(val);
}

void TW_CALL getStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((TriangleModel*)clientData)->getStiffness();
}

void TW_CALL setXXStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((TriangleModel*)clientData)->setXXStiffness(val);
}

void TW_CALL getXXStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((TriangleModel*)clientData)->getXXStiffness();
}

void TW_CALL setYYStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((TriangleModel*)clientData)->setYYStiffness(val);
}

void TW_CALL getYYStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((TriangleModel*)clientData)->getYYStiffness();
}

void TW_CALL setXYStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((TriangleModel*)clientData)->setXYStiffness(val);
}

void TW_CALL getXYStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((TriangleModel*)clientData)->getXYStiffness();
}

void TW_CALL setYXPoissonRatio(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((TriangleModel*)clientData)->setYXPoissonRatio(val);
}

void TW_CALL getYXPoissonRatio(void *value, void *clientData)
{
	*(float *)(value) = ((TriangleModel*)clientData)->getYXPoissonRatio();
}

void TW_CALL setXYPoissonRatio(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((TriangleModel*)clientData)->setXYPoissonRatio(val);
}

void TW_CALL getXYPoissonRatio(void *value, void *clientData)
{
	*(float *)(value) = ((TriangleModel*)clientData)->getXYPoissonRatio();
}

void TW_CALL setNormalizeStretch(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((TriangleModel*)clientData)->setNormalizeStretch(val);
}

void TW_CALL getNormalizeStretch(void *value, void *clientData)
{
	*(bool *)(value) = ((TriangleModel*)clientData)->getNormalizeStretch();
}

void TW_CALL setNormalizeShear(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((TriangleModel*)clientData)->setNormalizeShear(val);
}

void TW_CALL getNormalizeShear(void *value, void *clientData)
{
	*(bool *)(value) = ((TriangleModel*)clientData)->getNormalizeShear();
}

void TW_CALL setBendingStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((TriangleModel*)clientData)->setBendingStiffness(val);
}

void TW_CALL getBendingStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((TriangleModel*)clientData)->getBendingStiffness();
}

void TW_CALL setBendingMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	((TimeStepTriangleModel*)clientData)->setBendingMethod((unsigned int) val);
}

void TW_CALL getBendingMethod(void *value, void *clientData)
{
	*(short *)(value) = (short) ((TimeStepTriangleModel*)clientData)->getBendingMethod();
}

void TW_CALL setSimulationMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	((TimeStepTriangleModel*)clientData)->setSimulationMethod((unsigned int)val);
}

void TW_CALL getSimulationMethod(void *value, void *clientData)
{
	*(short *)(value) = (short)((TimeStepTriangleModel*)clientData)->getSimulationMethod();
}
