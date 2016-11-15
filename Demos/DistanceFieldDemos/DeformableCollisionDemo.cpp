#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Demos/Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Demos/Simulation/SimulationModel.h"
#include "Demos/Simulation/TimeStepController.h"
#include <iostream>
#include "Demos/Visualization/Visualization.h"
#include "Demos/Utils/Utilities.h"
#include "Demos/Simulation/DistanceFieldCollisionDetection.h"
#include "Demos/Utils/OBJLoader.h"
#include "Demos/Utils/Timing.h"

// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
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
void initShader();
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
void TW_CALL setMaxIterations(const void *value, void *clientData);
void TW_CALL getMaxIterations(void *value, void *clientData);
void TW_CALL setMaxIterationsV(const void *value, void *clientData);
void TW_CALL getMaxIterationsV(void *value, void *clientData);
void TW_CALL setContactTolerance(const void *value, void *clientData);
void TW_CALL getContactTolerance(void *value, void *clientData);
void TW_CALL setContactStiffnessRigidBody(const void *value, void *clientData);
void TW_CALL getContactStiffnessRigidBody(void *value, void *clientData);
void TW_CALL setContactStiffnessParticleRigidBody(const void *value, void *clientData);
void TW_CALL getContactStiffnessParticleRigidBody(void *value, void *clientData);



SimulationModel model;
DistanceFieldCollisionDetection cd;
TimeStepController sim;

const unsigned int width = 30;
const unsigned int depth = 5;
const unsigned int height = 5; 
short simulationMethod = 2;
bool doPause = true;
bool renderContacts = false;
std::vector<unsigned int> selectedParticles;
Vector3r oldMousePos;
Shader *shader;
string exePath;
string dataPath;

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	exePath = Utilities::getFilePath(argv[0]);
	dataPath = exePath + "/" + std::string(PBD_DATA_PATH);

	// OpenGL
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Bar collision demo");
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
	TwAddVarRW(MiniGL::getTweakBar(), "RenderContacts", TW_TYPE_BOOLCPP, &renderContacts, " label='Render contacts' group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_REAL, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwType enumType2 = TwDefineEnum("SimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "SimulationMethod", enumType2, setSimulationMethod, getSimulationMethod, &simulationMethod,
			" label='Simulation method' enum='0 {None}, 1 {Volume constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics (no inversion handling)}, 4 {Shape matching (no inversion handling)}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "Stiffness", TW_TYPE_REAL, setStiffness, getStiffness, &model, " label='Stiffness'  min=0.0 step=0.1 precision=4 group='Simulation' ");
	TwAddVarCB(MiniGL::getTweakBar(), "PoissonRatio", TW_TYPE_REAL, setPoissonRatio, getPoissonRatio, &model, " label='Poisson ratio XY'  min=0.0 step=0.1 precision=4 group='Simulation' ");
	TwAddVarCB(MiniGL::getTweakBar(), "NormalizeStretch", TW_TYPE_BOOL32, setNormalizeStretch, getNormalizeStretch, &model, " label='Normalize stretch' group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "NormalizeShear", TW_TYPE_BOOL32, setNormalizeShear, getNormalizeShear, &model, " label='Normalize shear' group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "MaxIter", TW_TYPE_UINT32, setMaxIterations, getMaxIterations, &sim, " label='Max. iterations'  min=1 step=1 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "MaxIterV", TW_TYPE_UINT32, setMaxIterationsV, getMaxIterationsV, &sim, " label='Max. iterations Vel.'  min=1 step=1 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactTolerance", TW_TYPE_REAL, setContactTolerance, getContactTolerance, &cd, " label='Contact tolerance'  min=0.0 step=0.001 precision=3 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessRigidBody", TW_TYPE_REAL, setContactStiffnessRigidBody, getContactStiffnessRigidBody, &model, " label='Contact stiffness RB'  min=0.0 step=0.1 precision=2 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessParticleRigidBody", TW_TYPE_REAL, setContactStiffnessParticleRigidBody, getContactStiffnessParticleRigidBody, &model, " label='Contact stiffness Particle-RB'  min=0.0 step=0.1 precision=2 group=Simulation ");


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
		sim.step(model);

	for (unsigned int i = 0; i < model.getTetModels().size(); i++)
		model.getTetModels()[i]->updateMeshNormals(model.getParticles());
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.005);

	createMesh();

	// create static rigid body
	string fileName = Utilities::normalizePath(dataPath + "/models/cube.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	OBJLoader::loadObj(fileName, vd, mesh);	

	string fileNameTorus = Utilities::normalizePath(dataPath + "/models/torus.obj");
	IndexedFaceMesh meshTorus;
	VertexData vdTorus;
	OBJLoader::loadObj(fileNameTorus, vdTorus, meshTorus);

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	rb.resize(2);

	// floor
	rb[0] = new RigidBody();
	rb[0]->initBody(1.0,
		Vector3r(0.0, -5.5, 0.0),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh,
		Vector3r(100.0, 1.0, 100.0));
	rb[0]->setMass(0.0);

	// torus
	rb[1] = new RigidBody();
	rb[1]->initBody(1.0,
		Vector3r(5.0, -1.5, 0.0),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vdTorus, meshTorus,
		Vector3r(3.0, 3.0, 3.0));
	rb[1]->setMass(0.0);
	rb[1]->setFrictionCoeff(0.1);

	sim.setCollisionDetection(model, &cd);
	cd.setTolerance(0.05);
	
	const std::vector<Vector3r> *vertices1 = rb[0]->getGeometry().getVertexDataLocal().getVertices();
	const unsigned int nVert1 = static_cast<unsigned int>(vertices1->size());
	cd.addCollisionBox(0, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices1)[0], nVert1, Vector3r(100.0, 1.0, 100.0));

	const std::vector<Vector3r> *vertices2 = rb[1]->getGeometry().getVertexDataLocal().getVertices();
	const unsigned int nVert2 = static_cast<unsigned int>(vertices2->size());
	cd.addCollisionTorus(1, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices2)[0], nVert2, Vector2r(3.0, 1.5));

	SimulationModel::TetModelVector &tm = model.getTetModels();
	ParticleData &pd = model.getParticles();
	for (unsigned int i = 0; i < tm.size(); i++)
	{
		const unsigned int nVert = tm[i]->getParticleMesh().numVertices();
		unsigned int offset = tm[i]->getIndexOffset();
		tm[i]->setFrictionCoeff(0.1);
		cd.addCollisionObjectWithoutGeometry(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert);
	}
}

void renderModels()
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

	for (unsigned int i = 0; i < model.getTetModels().size(); i++)
	{
		TetModel *tetModel = model.getTetModels()[i];
		const IndexedFaceMesh &surfaceMesh = tetModel->getSurfaceMesh();
		Visualization::drawMesh(pd, surfaceMesh, tetModel->getIndexOffset(), surfaceColor);
	}

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	float rbColor[4] = { 0.4f, 0.4f, 0.4f, 1 };
	for (size_t i = 0; i < rb.size(); i++)
	{
		const VertexData &vd = rb[i]->getGeometry().getVertexData();
		const IndexedFaceMesh &mesh = rb[i]->getGeometry().getMesh();
		if (shader)
			glUniform3fv(shader->getUniform("surface_color"), 1, rbColor);
		Visualization::drawTexturedMesh(vd, mesh, 0, rbColor);
	}

	if (shader)
		shader->end();
}

void renderContact(ParticleRigidBodyContactConstraint &cc)
{
	float col1[4] = { 0.0f, 0.6f, 0.2f, 1 };
	float col2[4] = { 0.6f, 0.0f, 0.2f, 1 };
	MiniGL::drawPoint(cc.m_constraintInfo.col(0), 5.0f, col1);
	MiniGL::drawPoint(cc.m_constraintInfo.col(1), 5.0f, col2);
	MiniGL::drawVector(cc.m_constraintInfo.col(1), cc.m_constraintInfo.col(1) + cc.m_constraintInfo.col(2), 1.0f, col2);
}

void render ()
{
	MiniGL::coordinateSystem();

	SimulationModel::ParticleRigidBodyContactConstraintVector &contacts = model.getParticleRigidBodyContactConstraints();
	
	// Draw simulation model
	renderModels();
 
	float red[4] = { 0.8f, 0.0f, 0.0f, 1 };
	const ParticleData &pd = model.getParticles();
	for (unsigned int j = 0; j < selectedParticles.size(); j++)
	{
		MiniGL::drawSphere(pd.getPosition(selectedParticles[j]), 0.08f, red);
	}

	if (renderContacts)
	{
		for (unsigned int i = 0; i < contacts.size(); i++)
			renderContact(contacts[i]);
	}

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
}


void createMesh()
{
	Vector3r points[width*height*depth];
	for (unsigned int i = 0; i < width; i++)
	{
		for (unsigned int j = 0; j < height; j++)
		{
			for (unsigned int k = 0; k < depth; k++)
			{
				points[i*height*depth + j*depth + k] = 0.3*Vector3r((Real)i, (Real)j + 3.0, (Real)k);
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
	model.addTetModel(width*height*depth, (unsigned int)indices.size() / 4u, points, indices.data());

	ParticleData &pd = model.getParticles();
	for (unsigned int i = 0; i < pd.getNumberOfParticles(); i++)
	{
		pd.setMass(i, 1.0);
	}

	// init constraints
	for (unsigned int cm = 0; cm < model.getTetModels().size(); cm++)
	{
		const unsigned int nTets = model.getTetModels()[cm]->getParticleMesh().numTets();
		const unsigned int *tets = model.getTetModels()[cm]->getParticleMesh().getTets().data();
		const IndexedTetMesh::VertexTets *vTets = model.getTetModels()[cm]->getParticleMesh().getVertexTets().data();
		if (simulationMethod == 1)
		{
			const unsigned int offset = model.getTetModels()[cm]->getIndexOffset();
			const unsigned int nEdges = model.getTetModels()[cm]->getParticleMesh().numEdges();
			const IndexedTetMesh::Edge *edges = model.getTetModels()[cm]->getParticleMesh().getEdges().data();
			for (unsigned int i = 0; i < nEdges; i++)
			{
				const unsigned int v1 = edges[i].m_vert[0] + offset;
				const unsigned int v2 = edges[i].m_vert[1] + offset;

				model.addDistanceConstraint(v1, v2);
			}
			
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v1 = tets[4 * i];
				const unsigned int v2 = tets[4 * i + 1];
				const unsigned int v3 = tets[4 * i + 2];
				const unsigned int v4 = tets[4 * i + 3];

				model.addVolumeConstraint(v1, v2, v3, v4);
			}
		}
		else if (simulationMethod == 2)
		{
			TetModel::ParticleMesh &mesh = model.getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v1 = tets[4 * i];
				const unsigned int v2 = tets[4 * i + 1];
				const unsigned int v3 = tets[4 * i + 2];
				const unsigned int v4 = tets[4 * i + 3];

				model.addFEMTetConstraint(v1, v2, v3, v4);
			}
		}
		else if (simulationMethod == 3)
		{
			TetModel::ParticleMesh &mesh = model.getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v1 = tets[4 * i];
				const unsigned int v2 = tets[4 * i + 1];
				const unsigned int v3 = tets[4 * i + 2];
				const unsigned int v4 = tets[4 * i + 3];

				model.addStrainTetConstraint(v1, v2, v3, v4);
			}
		}
		else if (simulationMethod == 4)
		{
			TetModel::ParticleMesh &mesh = model.getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v[4] = { tets[4 * i], tets[4 * i + 1], tets[4 * i + 2], tets[4 * i + 3] };
				// Important: Divide position correction by the number of clusters 
				// which contain the vertex.
				const unsigned int nc[4] = { vTets[v[0]].m_numTets, vTets[v[1]].m_numTets, vTets[v[2]].m_numTets, vTets[v[3]].m_numTets };
				model.addShapeMatchingConstraint(4, v, nc);
			}
		}
		model.getTetModels()[cm]->updateMeshNormals(pd);
	}

	std::cout << "Number of tets: " << indices.size() / 4 << "\n";
	std::cout << "Number of vertices: " << width*height*depth << "\n";

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

void TW_CALL setStiffness(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*) clientData)->setSolidStiffness(val);
}

void TW_CALL getStiffness(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getSolidStiffness();
}

void TW_CALL setPoissonRatio(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setSolidPoissonRatio(val);
}

void TW_CALL getPoissonRatio(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getSolidPoissonRatio();
}

void TW_CALL setNormalizeStretch(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((SimulationModel*)clientData)->setSolidNormalizeStretch(val);
}

void TW_CALL getNormalizeStretch(void *value, void *clientData)
{
	*(bool *)(value) = ((SimulationModel*)clientData)->getSolidNormalizeStretch();
}

void TW_CALL setNormalizeShear(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((SimulationModel*)clientData)->setSolidNormalizeShear(val);
}

void TW_CALL getNormalizeShear(void *value, void *clientData)
{
	*(bool *)(value) = ((SimulationModel*)clientData)->getSolidNormalizeShear();
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

void TW_CALL setMaxIterationsV(const void *value, void *clientData)
{
	const unsigned int val = *(const unsigned int *)(value);
	((TimeStepController*)clientData)->setMaxIterationsV(val);
}

void TW_CALL getMaxIterationsV(void *value, void *clientData)
{
	*(unsigned int *)(value) = ((TimeStepController*)clientData)->getMaxIterationsV();
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
