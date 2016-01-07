#include "Demos/Utils/Config.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Demos/Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "GenericConstraintsModel.h"
#include <iostream>
#include "Demos/Simulation/TimeStepController.h"

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
const float width = 10.0f;
const float height = 10.0f;
bool doPause = true;
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

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_FLOAT, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &simulation, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "Stiffness", TW_TYPE_FLOAT, setStiffness, getStiffness, &model, " label='Stiffness'  min=0.0 step=0.1 precision=4 group='Distance constraints' ");
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
	for (unsigned int i = 0; i < 8; i++)
		simulation.step(model);

	for (unsigned int i = 0; i < model.getTriangleModels().size(); i++)
		model.getTriangleModels()[i]->updateMeshNormals(model.getParticles());
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.005f);

	createMesh();
}

void renderTriangleModels()
{
	// Draw simulation model

	const ParticleData &pd = model.getParticles();

	for (unsigned int i = 0; i < model.getTriangleModels().size(); i++)
	{
		// mesh 
		const IndexedFaceMesh &mesh = model.getTriangleModels()[i]->getParticleMesh();
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
		glVertexPointer(3, GL_FLOAT, 0, &pd.getPosition(model.getTriangleModels()[i]->getIndexOffset())[0]);
		glTexCoordPointer(2, GL_FLOAT, 0, &uvs[0][0]);
		glNormalPointer(GL_FLOAT, 0, &vertexNormals[0][0]);
		glDrawElements(GL_TRIANGLES, (GLsizei)3 * mesh.numFaces(), GL_UNSIGNED_INT, mesh.getFaces().data());
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);

		MiniGL::unbindTexture();
	}
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

	std::cout << "Number of triangles: " << nIndices / 3 << "\n";
	std::cout << "Number of vertices: " << nRows*nCols << "\n";

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
	((TimeStepController*)clientData)->setVelocityUpdateMethod((unsigned int)val);
}

void TW_CALL getVelocityUpdateMethod(void *value, void *clientData)
{
	*(short *)(value) = (short)((TimeStepController*)clientData)->getVelocityUpdateMethod();
}

void TW_CALL setStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((GenericConstraintsModel*)clientData)->setClothStiffness(val);
}

void TW_CALL getStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((GenericConstraintsModel*)clientData)->getClothStiffness();
}

void TW_CALL setBendingStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((GenericConstraintsModel*)clientData)->setClothBendingStiffness(val);
}

void TW_CALL getBendingStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((GenericConstraintsModel*)clientData)->getClothBendingStiffness();
}
