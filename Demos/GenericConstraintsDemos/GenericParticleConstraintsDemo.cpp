#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Demos/Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "GenericConstraintsModel.h"
#include <iostream>
#include "Demos/Simulation/TimeStepController.h"
#include "Demos/Visualization/Visualization.h"
#include "Demos/Utils/Logger.h"
#include "Demos/Utils/Timing.h"
#include "Demos/Utils/FileSystem.h"

// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
	#define new DEBUG_NEW 
#endif

INIT_TIMING
INIT_LOGGING

using namespace PBD;
using namespace Eigen;
using namespace std;

void timeStep ();
void buildModel ();
void createMesh();
void render ();
void cleanup();
void reset();
void initShader();
void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData);
void TW_CALL getVelocityUpdateMethod(void *value, void *clientData);
void TW_CALL setStiffness(const void *value, void *clientData);
void TW_CALL getStiffness(void *value, void *clientData);
void TW_CALL setBendingStiffness(const void *value, void *clientData);
void TW_CALL getBendingStiffness(void *value, void *clientData);


GenericConstraintsModel model;
TimeStepController simulation;

const int nRows = 30;
const int nCols = 30;
const Real width = 10.0;
const Real height = 10.0;
bool doPause = true;
std::vector<unsigned int> selectedParticles;
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
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Cloth demo");
	MiniGL::initLights ();
	MiniGL::initTexture();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection);
	initShader();

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3r (5.0, 10.0, 30.0), Vector3r (5.0, 0.0, 0.0));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_REAL, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &simulation, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "Stiffness", TW_TYPE_REAL, setStiffness, getStiffness, &model, " label='Stiffness'  min=0.0 step=0.1 precision=4 group='Distance constraints' ");
	TwAddVarCB(MiniGL::getTweakBar(), "BendingStiffness", TW_TYPE_REAL, setBendingStiffness, getBendingStiffness, &model, " label='Bending stiffness'  min=0.0 step=0.01 precision=4 group=Bending ");

	glutMainLoop ();	

	cleanup ();
	
	Timing::printAverageTimes();

	return 0;
}

void initShader()
{
	std::string vertFile = dataPath + "/shaders/vs_smoothTex.glsl";
	std::string fragFile = dataPath + "/shaders/fs_smoothTex.glsl";
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
	simulation.reset();
	TimeManager::getCurrent()->setTime(0.0);
}

void mouseMove(int x, int y)
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
	for (unsigned int i = 0; i < 8; i++)
		simulation.step(model);

	for (unsigned int i = 0; i < model.getTriangleModels().size(); i++)
		model.getTriangleModels()[i]->updateMeshNormals(model.getParticles());
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.005);

	createMesh();
}

void renderTriangleModels()
{
	// Draw simulation model

	const ParticleData &pd = model.getParticles();
	float surfaceColor[4] = { 0.2f, 0.5f, 1.0f, 1 };

	if (shader)
	{
		shader->begin();
		glUniform3fv(shader->getUniform("surface_color"), 1, surfaceColor);
		glUniform1f(shader->getUniform("shininess"), 5.0f);
		glUniform1f(shader->getUniform("specular_factor"), 0.2f);

		GLfloat matrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
		glUniformMatrix4fv(shader->getUniform("modelview_matrix"), 1, GL_FALSE, matrix);
		GLfloat pmatrix[16];
		glGetFloatv(GL_PROJECTION_MATRIX, pmatrix);
		glUniformMatrix4fv(shader->getUniform("projection_matrix"), 1, GL_FALSE, pmatrix);
	}

	for (unsigned int i = 0; i < model.getTriangleModels().size(); i++)
	{
		// mesh 
		TriangleModel *triModel = model.getTriangleModels()[i];
		const IndexedFaceMesh &mesh = triModel->getParticleMesh();
		Visualization::drawTexturedMesh(pd, mesh, triModel->getIndexOffset(), surfaceColor);
	}
	if (shader)
		shader->end();
}

void render ()
{
	MiniGL::coordinateSystem();
	
	renderTriangleModels();

	float red[4] = { 0.8f, 0.0f, 0.0f, 1 };
	const ParticleData &pd = model.getParticles();
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

	const Real dy = width / (Real)(nCols - 1);
	const Real dx = height / (Real)(nRows - 1);

	Vector3r points[nRows*nCols];
	for (int i = 0; i < nRows; i++)
	{
		for (int j = 0; j < nCols; j++)
		{
			const Real y = (Real)dy*j;
			const Real x = (Real)dx*i;
			points[i*nCols + j] = Vector3r(x, 1.0, y);

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
	model.addTriangleModel(nRows*nCols, nIndices / 3, &points[0], &indices[0], uvIndices, uvs);
	
	ParticleData &pd = model.getParticles();
	for (unsigned int i = 0; i < pd.getNumberOfParticles(); i++)
	{
		pd.setMass(i, 1.0);
	}

	// Set mass of points to zero => make it static
	pd.setMass(0, 0.0);
	pd.setMass((nRows-1)*nCols, 0.0);

	// init constraints
	for (unsigned int cm = 0; cm < model.getTriangleModels().size(); cm++)
	{
		const unsigned int offset = model.getTriangleModels()[cm]->getIndexOffset();
		IndexedFaceMesh &mesh = model.getTriangleModels()[cm]->getParticleMesh();
		const unsigned int nEdges = mesh.numEdges();
		const IndexedFaceMesh::Edge *edges = mesh.getEdges().data();

		// distance constraints
		for (unsigned int i = 0; i < nEdges; i++)
		{
			const unsigned int v1 = edges[i].m_vert[0] + offset;
			const unsigned int v2 = edges[i].m_vert[1] + offset;

			model.addGenericDistanceConstraint(v1, v2);
		}

		// bending constraints
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
					model.addGenericIsometricBendingConstraint(vertex1, vertex2, vertex3, vertex4);
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
	((GenericConstraintsModel*)clientData)->setClothStiffness(val);
}

void TW_CALL getStiffness(void *value, void *clientData)
{
	*(Real *)(value) = ((GenericConstraintsModel*)clientData)->getClothStiffness();
}

void TW_CALL setBendingStiffness(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((GenericConstraintsModel*)clientData)->setClothBendingStiffness(val);
}

void TW_CALL getBendingStiffness(void *value, void *clientData)
{
	*(Real *)(value) = ((GenericConstraintsModel*)clientData)->getClothBendingStiffness();
}
