#include "Common/Common.h"
#include "Demos/Simulation/CubicSDFCollisionDetection.h"
#include "Demos/Simulation/DistanceFieldCollisionDetection.h"
#include "Demos/Simulation/TimeManager.h"
#include "Demos/Simulation/SimulationModel.h"
#include "Demos/Simulation/TimeStepController.h"
#include "Demos/Utils/FileSystem.h"
#include "Demos/Utils/Logger.h"
#include "Demos/Utils/OBJLoader.h"
#include "Demos/Utils/TetGenLoader.h"
#include "Demos/Utils/Timing.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Demos/Visualization/Visualization.h"
#include "GL/glut.h"
#include <Eigen/Dense>
#include <iostream>
#include <set>

#include "StiffRodsSceneLoader.h"

#define _USE_MATH_DEFINES
#include "math.h"

INIT_TIMING
INIT_LOGGING

// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

void timeStep();
void buildModel();
void readScene();
void render();
void reset();
void cleanup();
void initShader();
void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
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
void TW_CALL setClothSimulationMethod(const void *value, void *clientData);
void TW_CALL getClothSimulationMethod(void *value, void *clientData);
void TW_CALL setSolidSimulationMethod(const void *value, void *clientData);
void TW_CALL getSolidSimulationMethod(void *value, void *clientData);

void singleStep();


SimulationModel model;
CubicSDFCollisionDetection cd;
TimeStepController sim;

short clothSimulationMethod = 2;
short solidSimulationMethod = 2;
short bendingMethod = 2;
bool doPause = true;
std::vector<unsigned int> selectedBodies;
std::vector<unsigned int> selectedParticles;
Vector3r oldMousePos;
Shader *shader;
Shader *shaderTex;
bool renderContacts = false;
string exePath;
string dataPath;
bool drawAABB = false;
bool drawSDF = false;
int drawBVHDepth = -1;
float jointColor[4] = { 0.0f, 0.6f, 0.2f, 1 };

string sceneFileName = "/scenes/Wilberforce_scene.json";
string sceneName;
Vector3r camPos;
Vector3r camLookat;


// main 
int main(int argc, char **argv)
{
	REPORT_MEMORY_LEAKS;
	
	std::string logPath = FileSystem::normalizePath(FileSystem::getProgramPath() + "/log");
	FileSystem::makeDirs(logPath);
	logger.addSink(unique_ptr<ConsoleSink>(new ConsoleSink(LogLevel::INFO)));
	logger.addSink(unique_ptr<FileSink>(new FileSink(LogLevel::DEBUG, logPath + "/PBD_StiffRods.log")));
	LOG_INFO << "Starting new log entries";

	exePath = FileSystem::getProgramPath();
	dataPath = exePath + "/" + std::string(PBD_DATA_PATH);

	if (argc > 1)
		sceneFileName = string(argv[1]);
	else
		sceneFileName = FileSystem::normalizePath(dataPath + sceneFileName);

	buildModel();

	// OpenGL
	MiniGL::init(argc, argv, 1024, 768, 0, 0, sceneName.c_str());
	MiniGL::initLights();
	MiniGL::initTexture();
	MiniGL::setClientIdleFunc(50, timeStep);
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setKeyFunc(1, 's', singleStep);
	MiniGL::setSelectionFunc(selection);
	initShader();

	MiniGL::setClientSceneFunc(render);
	MiniGL::setViewport(40.0f, 0.1f, 500.0f, camPos, camLookat);

	SimulationModel::TriangleModelVector &triModels = model.getTriangleModels();
	SimulationModel::TetModelVector &tetModels = model.getTetModels();
	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarRW(MiniGL::getTweakBar(), "RenderContacts", TW_TYPE_BOOLCPP, &renderContacts, " label='Render contacts' group=Simulation ");
	TwAddVarRW(MiniGL::getTweakBar(), "RenderAABBs", TW_TYPE_BOOLCPP, &drawAABB, " label='Render AABBs' group=Simulation ");
	TwAddVarRW(MiniGL::getTweakBar(), "RenderSDFs", TW_TYPE_BOOLCPP, &drawSDF, " label='Render SDFs' group=Simulation ");
	TwAddVarRW(MiniGL::getTweakBar(), "RenderBVH", TW_TYPE_INT32, &drawBVHDepth, " label='Render BVH depth' group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_REAL, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "MaxIter", TW_TYPE_UINT32, setMaxIterations, getMaxIterations, &sim, " label='Max. iterations'  min=1 step=1 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "MaxIterV", TW_TYPE_UINT32, setMaxIterationsV, getMaxIterationsV, &sim, " label='Max. iterations Vel.'  min=1 step=1 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactTolerance", TW_TYPE_REAL, setContactTolerance, getContactTolerance, &cd, " label='Contact tolerance'  min=0.0 step=0.001 precision=3 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessRigidBody", TW_TYPE_REAL, setContactStiffnessRigidBody, getContactStiffnessRigidBody, &model, " label='Contact stiffness RB'  min=0.0 step=0.1 precision=2 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessParticleRigidBody", TW_TYPE_REAL, setContactStiffnessParticleRigidBody, getContactStiffnessParticleRigidBody, &model, " label='Contact stiffness Particle-RB'  min=0.0 step=0.1 precision=2 group=Simulation ");
	if (triModels.size() > 0)
	{
		TwType enumType2 = TwDefineEnum("ClothSimulationMethodType", NULL, 0);
		TwAddVarCB(MiniGL::getTweakBar(), "ClothSimulationMethod", enumType2, setClothSimulationMethod, getClothSimulationMethod, &clothSimulationMethod, " label='Cloth sim. method' enum='0 {None}, 1 {Distance constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics}' group=Simulation");
	}
	if (tetModels.size() > 0)
	{
	TwType enumType3 = TwDefineEnum("SolidSimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "SolidSimulationMethod", enumType3, setSolidSimulationMethod, getSolidSimulationMethod, &solidSimulationMethod,
		" label='Solid sim. method' enum='0 {None}, 1 {Volume constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics (no inversion handling)}, 4 {Shape matching (no inversion handling)}' group=Simulation");
	}
	if ((triModels.size() > 0) || (tetModels.size() > 0))
	{
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
	}
	if (triModels.size() > 0)
	{
		TwType enumType4 = TwDefineEnum("BendingMethodType", NULL, 0);
		TwAddVarCB(MiniGL::getTweakBar(), "BendingMethod", enumType4, setBendingMethod, getBendingMethod, &bendingMethod, " label='Bending method' enum='0 {None}, 1 {Dihedral angle}, 2 {Isometric bending}' group=Bending");
		TwAddVarCB(MiniGL::getTweakBar(), "BendingStiffness", TW_TYPE_REAL, setBendingStiffness, getBendingStiffness, &model, " label='Bending stiffness'  min=0.0 step=0.01 precision=4 group=Bending ");
	}

	glutMainLoop();

	cleanup();

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

	std::string vertFileTex = dataPath + "/shaders/vs_smoothTex.glsl";
	std::string fragFileTex = dataPath + "/shaders/fs_smoothTex.glsl";
	shaderTex = MiniGL::createShader(vertFileTex, "", fragFileTex);

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
	sim.getCollisionDetection()->cleanup();
	model.cleanup();
	sim.reset();
	TimeManager::getCurrent()->setTime(0.0);
	readScene();
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
		if (rb[selectedBodies[j]]->getMass() != 0.0)
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
	if (pd.size() > 0)
		Selection::selectRect(start, end, &pd.getPosition(0), &pd.getPosition(pd.size() - 1), selectedParticles);

	selectedBodies.clear();
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	std::vector<Vector3r, Eigen::aligned_allocator<Vector3r> > x;
	x.resize(rb.size());
	for (unsigned int i = 0; i < rb.size(); i++)
	{
		x[i] = rb[i]->getPosition();
	}

	if (rb.size() > 0)
		Selection::selectRect(start, end, &x[0], &x[rb.size() - 1], selectedBodies);
	if ((selectedBodies.size() > 0) || (selectedParticles.size() > 0))
		MiniGL::setMouseMoveFunc(GLUT_MIDDLE_BUTTON, mouseMove);
	else
		MiniGL::setMouseMoveFunc(-1, NULL);

	MiniGL::unproject(end[0], end[1], oldMousePos);
}

void singleStep()
{
	// Simulation code
	sim.step(model);

	// Update visualization models
	const ParticleData &pd = model.getParticles();
	for (unsigned int i = 0; i < model.getTetModels().size(); i++)
	{
		model.getTetModels()[i]->updateMeshNormals(pd);
		model.getTetModels()[i]->updateVisMesh(pd);
	}
	for (unsigned int i = 0; i < model.getTriangleModels().size(); i++)
	{
		model.getTriangleModels()[i]->updateMeshNormals(pd);
	}
}

void timeStep()
{
	if (doPause)
		return;

	// Simulation code
	for (unsigned int i = 0; i < 8; i++)
	{
		sim.step(model);
	}

	// Update visualization models
	const ParticleData &pd = model.getParticles();
	for (unsigned int i = 0; i < model.getTetModels().size(); i++)
	{
		model.getTetModels()[i]->updateMeshNormals(pd);
		model.getTetModels()[i]->updateVisMesh(pd);
	}
	for (unsigned int i = 0; i < model.getTriangleModels().size(); i++)
	{
		model.getTriangleModels()[i]->updateMeshNormals(pd);
	}
}

void buildModel()
{
	TimeManager::getCurrent()->setTimeStepSize(0.005);

	sim.setCollisionDetection(model, &cd);

	readScene();
}

void renderTriangleModels()
{
	const ParticleData &pd = model.getParticles();
	float surfaceColor[4] = { 0.8f, 0.9f, 0.2f, 1 };

	if (shaderTex)
	{
		shaderTex->begin();
		glUniform1f(shaderTex->getUniform("shininess"), 5.0f);
		glUniform1f(shaderTex->getUniform("specular_factor"), 0.2f);

		GLfloat matrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
		glUniformMatrix4fv(shaderTex->getUniform("modelview_matrix"), 1, GL_FALSE, matrix);
		GLfloat pmatrix[16];
		glGetFloatv(GL_PROJECTION_MATRIX, pmatrix);
		glUniformMatrix4fv(shaderTex->getUniform("projection_matrix"), 1, GL_FALSE, pmatrix);

		glUniform3fv(shaderTex->getUniform("surface_color"), 1, surfaceColor);
	}

	for (unsigned int i = 0; i < model.getTriangleModels().size(); i++)
	{
		// mesh 
		const IndexedFaceMesh &mesh = model.getTriangleModels()[i]->getParticleMesh();
		const unsigned int offset = model.getTriangleModels()[i]->getIndexOffset();
		Visualization::drawTexturedMesh(pd, mesh, offset, surfaceColor);
	}

	if (shaderTex)
		shaderTex->end();
}

void renderTetModels()
{
	const ParticleData &pd = model.getParticles();
	float surfaceColor[4] = { 0.8f, 0.4f, 0.7f, 1 };

	if (shader)
	{
		shader->begin();
		glUniform1f(shader->getUniform("shininess"), 5.0f);
		glUniform1f(shader->getUniform("specular_factor"), 0.2f);

		GLfloat matrix[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
		glUniformMatrix4fv(shader->getUniform("modelview_matrix"), 1, GL_FALSE, matrix);
		GLfloat pmatrix[16];
		glGetFloatv(GL_PROJECTION_MATRIX, pmatrix);
		glUniformMatrix4fv(shader->getUniform("projection_matrix"), 1, GL_FALSE, pmatrix);

		glUniform3fv(shader->getUniform("surface_color"), 1, surfaceColor);
	}

	for (unsigned int i = 0; i < model.getTetModels().size(); i++)
	{
		const VertexData &vdVis = model.getTetModels()[i]->getVisVertices();
		if (vdVis.size() > 0)
		{
			const IndexedFaceMesh &visMesh = model.getTetModels()[i]->getVisMesh();
			Visualization::drawMesh(vdVis, visMesh, 0, surfaceColor);
		}
		else
		{
			const IndexedFaceMesh &surfaceMesh = model.getTetModels()[i]->getSurfaceMesh();
			const unsigned int offset = model.getTetModels()[i]->getIndexOffset();
			Visualization::drawMesh(pd, surfaceMesh, offset, surfaceColor);
		}
	}

	if (shader)
		shader->end();
}

void renderAABB(AABB &aabb)
{
	Vector3r p1, p2;
	glBegin(GL_LINES);
	for (unsigned char i = 0; i < 12; i++)
	{
		AABB::getEdge(aabb, i, p1, p2);
		glVertex3d(p1[0], p1[1], p1[2]);
		glVertex3d(p2[0], p2[1], p2[2]);
	}
	glEnd();
}

void renderSDF(CollisionDetection::CollisionObject* co)
{
	if (!cd.isDistanceFieldCollisionObject(co))
		return;

	const SimulationModel::RigidBodyVector &rigidBodies = model.getRigidBodies();
	RigidBody *rb = rigidBodies[co->m_bodyIndex];

	const Vector3r &com = rb->getPosition();
	const Matrix3r &R = rb->getTransformationR();
	const Vector3r &v1 = rb->getTransformationV1();
	const Vector3r &v2 = rb->getTransformationV2();

	DistanceFieldCollisionDetection::DistanceFieldCollisionObject *dfco = (DistanceFieldCollisionDetection::DistanceFieldCollisionObject *)co;
	const Vector3r &startX = co->m_aabb.m_p[0];
	const Vector3r &endX = co->m_aabb.m_p[1];
	Vector3r diff = endX - startX;
	const unsigned int steps = 20;
	Vector3r stepSize = (1.0 / steps) * diff;
	for (Real x = startX[0]; x < endX[0]; x += stepSize[0])
	{
		for (Real y = startX[1]; y < endX[1]; y += stepSize[1])
		{
			for (Real z = startX[2]; z < endX[2]; z += stepSize[2])
			{
				Vector3r pos_w(x, y, z);
				const Vector3r pos = R * (pos_w - com) + v1;
				const Real dist = dfco->distance(pos, 0.0);

				if (dist < 0.0)
				{
					float col[4] = { (float)-dist, 0.0f, 0.0f, 1.0f };
					MiniGL::drawPoint(pos_w, 3.0f, col);
				}
			}
		}
	}
}

void renderBallJoint(BallJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(2), 0.15f, jointColor);
}

void renderRigidBodyParticleBallJoint(RigidBodyParticleBallJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(1), 0.1f, jointColor);
}

void renderBallOnLineJoint(BallOnLineJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(5), 0.1f, jointColor);
	MiniGL::drawCylinder(bj.m_jointInfo.col(5) - bj.m_jointInfo.col(7), bj.m_jointInfo.col(5) + bj.m_jointInfo.col(7), jointColor, 0.05f);
}

void renderHingeJoint(HingeJoint &hj)
{
	MiniGL::drawSphere(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawSphere(hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawCylinder(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), jointColor, 0.05f);
}

void renderUniversalJoint(UniversalJoint &uj)
{
	MiniGL::drawSphere(uj.m_jointInfo.col(4) - 0.5*uj.m_jointInfo.col(6), 0.1f, jointColor);
	MiniGL::drawSphere(uj.m_jointInfo.col(4) + 0.5*uj.m_jointInfo.col(6), 0.1f, jointColor);
	MiniGL::drawSphere(uj.m_jointInfo.col(5) - 0.5*uj.m_jointInfo.col(7), 0.1f, jointColor);
	MiniGL::drawSphere(uj.m_jointInfo.col(5) + 0.5*uj.m_jointInfo.col(7), 0.1f, jointColor);
	MiniGL::drawCylinder(uj.m_jointInfo.col(4) - 0.5*uj.m_jointInfo.col(6), uj.m_jointInfo.col(4) + 0.5*uj.m_jointInfo.col(6), jointColor, 0.05f);
	MiniGL::drawCylinder(uj.m_jointInfo.col(5) - 0.5*uj.m_jointInfo.col(7), uj.m_jointInfo.col(5) + 0.5*uj.m_jointInfo.col(7), jointColor, 0.05f);
}

void renderSliderJoint(SliderJoint &joint)
{
	MiniGL::drawSphere(joint.m_jointInfo.col(6), 0.1f, jointColor);
	MiniGL::drawCylinder(joint.m_jointInfo.col(7) - joint.m_jointInfo.col(8), joint.m_jointInfo.col(7) + joint.m_jointInfo.col(8), jointColor, 0.05f);
}

void renderTargetPositionMotorSliderJoint(TargetPositionMotorSliderJoint &joint)
{
	MiniGL::drawSphere(joint.m_jointInfo.col(6), 0.1f, jointColor);
	MiniGL::drawCylinder(joint.m_jointInfo.col(7) - joint.m_jointInfo.col(8), joint.m_jointInfo.col(7) + joint.m_jointInfo.col(8), jointColor, 0.05f);
}

void renderTargetVelocityMotorSliderJoint(TargetVelocityMotorSliderJoint &joint)
{
	MiniGL::drawSphere(joint.m_jointInfo.col(6), 0.1f, jointColor);
	MiniGL::drawCylinder(joint.m_jointInfo.col(7) - joint.m_jointInfo.col(8), joint.m_jointInfo.col(7) + joint.m_jointInfo.col(8), jointColor, 0.05f);
}

void renderTargetAngleMotorHingeJoint(TargetAngleMotorHingeJoint &hj)
{
	MiniGL::drawSphere(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawSphere(hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawCylinder(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), jointColor, 0.05f);
}

void renderTargetVelocityMotorHingeJoint(TargetVelocityMotorHingeJoint &hj)
{
	MiniGL::drawSphere(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawSphere(hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), 0.1f, jointColor);
	MiniGL::drawCylinder(hj.m_jointInfo.col(6) - 0.5*hj.m_jointInfo.col(8), hj.m_jointInfo.col(6) + 0.5*hj.m_jointInfo.col(8), jointColor, 0.05f);
}

void renderRigidBodyContact(RigidBodyContactConstraint &cc)
{
	float col1[4] = { 0.0f, 0.6f, 0.2f, 1 };
	float col2[4] = { 0.6f, 0.0f, 0.2f, 1 };
	MiniGL::drawPoint(cc.m_constraintInfo.col(0), 5.0f, col1);
	MiniGL::drawPoint(cc.m_constraintInfo.col(1), 5.0f, col2);
	MiniGL::drawVector(cc.m_constraintInfo.col(1), cc.m_constraintInfo.col(1) + cc.m_constraintInfo.col(2), 1.0f, col2);
}

void renderParticleRigidBodyContact(ParticleRigidBodyContactConstraint &cc)
{
	float col1[4] = { 0.0f, 0.6f, 0.2f, 1 };
	float col2[4] = { 0.6f, 0.0f, 0.2f, 1 };
	MiniGL::drawPoint(cc.m_constraintInfo.col(0), 5.0f, col1);
	MiniGL::drawPoint(cc.m_constraintInfo.col(1), 5.0f, col2);
	MiniGL::drawVector(cc.m_constraintInfo.col(1), cc.m_constraintInfo.col(1) + cc.m_constraintInfo.col(2), 1.0f, col2);
}

void render()
{
	MiniGL::coordinateSystem();

	// Draw sim model

	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();
	SimulationModel::RigidBodyContactConstraintVector &rigidBodyContacts = model.getRigidBodyContactConstraints();
	SimulationModel::ParticleRigidBodyContactConstraintVector &particleRigidBodyContacts = model.getParticleRigidBodyContactConstraints();

	float selectionColor[4] = { 0.8f, 0.0f, 0.0f, 1 };
	float surfaceColor[4] = { 0.3f, 0.5f, 0.8f, 1 };
	float staticColor[4] = { 0.5f, 0.5f, 0.5f, 1 };

	if (renderContacts)
	{
		for (unsigned int i = 0; i < rigidBodyContacts.size(); i++)
			renderRigidBodyContact(rigidBodyContacts[i]);
		for (unsigned int i = 0; i < particleRigidBodyContacts.size(); i++)
			renderParticleRigidBodyContact(particleRigidBodyContacts[i]);
	}


	if (shader)
	{
		shader->begin();
		glUniform1f(shader->getUniform("shininess"), 5.0f);
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
			if (rb[i]->getMass() == 0.0)
			{
				if (shader)
					glUniform3fv(shader->getUniform("surface_color"), 1, staticColor);
				Visualization::drawMesh(vd, mesh, 0, staticColor);
			}
			else
			{
				if (shader)
					glUniform3fv(shader->getUniform("surface_color"), 1, surfaceColor);
				Visualization::drawMesh(vd, mesh, 0, surfaceColor);
			}
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



	renderTriangleModels();
	renderTetModels();

	for (size_t i = 0; i < constraints.size(); i++)
	{
		if (constraints[i]->getTypeId() == BallJoint::TYPE_ID)
		{
			renderBallJoint(*(BallJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == BallOnLineJoint::TYPE_ID)
		{
			renderBallOnLineJoint(*(BallOnLineJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == HingeJoint::TYPE_ID)
		{
			renderHingeJoint(*(HingeJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == UniversalJoint::TYPE_ID)
		{
			renderUniversalJoint(*(UniversalJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == SliderJoint::TYPE_ID)
		{
			renderSliderJoint(*(SliderJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == TargetAngleMotorHingeJoint::TYPE_ID)
		{
			renderTargetAngleMotorHingeJoint(*(TargetAngleMotorHingeJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == TargetVelocityMotorHingeJoint::TYPE_ID)
		{
			renderTargetVelocityMotorHingeJoint(*(TargetVelocityMotorHingeJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == TargetPositionMotorSliderJoint::TYPE_ID)
		{
			renderTargetPositionMotorSliderJoint(*(TargetPositionMotorSliderJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == TargetVelocityMotorSliderJoint::TYPE_ID)
		{
			renderTargetVelocityMotorSliderJoint(*(TargetVelocityMotorSliderJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == RigidBodyParticleBallJoint::TYPE_ID)
		{
			renderRigidBodyParticleBallJoint(*(RigidBodyParticleBallJoint*)constraints[i]);
		}
	}



	if (drawSDF || drawAABB || (drawBVHDepth >= 0))
	{
		ObjectArray<CollisionDetection::CollisionObject*> &collisionObjects = cd.getCollisionObjects();
		for (unsigned int k = 0; k < collisionObjects.size(); k++)
		{
			if (drawAABB)
				renderAABB(collisionObjects[k]->m_aabb);

			if (drawSDF)
				renderSDF(collisionObjects[k]);

			if (drawBVHDepth >= 0)
			{
				if (cd.isDistanceFieldCollisionObject(collisionObjects[k]))
				{
					const PointCloudBSH &bvh = ((DistanceFieldCollisionDetection::DistanceFieldCollisionObject*) collisionObjects[k])->m_bvh;

					std::function<bool(unsigned int, unsigned int)> predicate = [&](unsigned int node_index, unsigned int depth) { return (int)depth <= drawBVHDepth; };
					std::function<void(unsigned int, unsigned int)> cb = [&](unsigned int node_index, unsigned int depth)
					{
						if (depth == drawBVHDepth)
						{
							const BoundingSphere &bs = bvh.hull(node_index);
							if (collisionObjects[k]->m_bodyType == CollisionDetection::CollisionObject::RigidBodyCollisionObjectType)
							{
								RigidBody *body = rb[collisionObjects[k]->m_bodyIndex];
								const Vector3r &sphere_x = bs.x();
								const Vector3r sphere_x_w = body->getRotation() * sphere_x + body->getPosition();
								MiniGL::drawSphere(sphere_x_w, std::max((float)bs.r(), 0.05f), staticColor);
							}
							else
								MiniGL::drawSphere(bs.x(), std::max((float)bs.r(), 0.05f), staticColor);
						}
					};

					bvh.traverse_depth_first(predicate, cb);
				}
			}
		}
	}

	float red[4] = { 0.8f, 0.0f, 0.0f, 1 };
	const ParticleData &pd = model.getParticles();
	for (unsigned int j = 0; j < selectedParticles.size(); j++)
	{
		MiniGL::drawSphere(pd.getPosition(selectedParticles[j]), 0.08f, red);
	}

	MiniGL::drawTime(TimeManager::getCurrent()->getTime());
}

void initTriangleModelConstraints()
{
	// init constraints
	for (unsigned int cm = 0; cm < model.getTriangleModels().size(); cm++)
	{
		const unsigned int offset = model.getTriangleModels()[cm]->getIndexOffset();
		if (clothSimulationMethod == 1)
		{
			const unsigned int nEdges = model.getTriangleModels()[cm]->getParticleMesh().numEdges();
			const IndexedFaceMesh::Edge *edges = model.getTriangleModels()[cm]->getParticleMesh().getEdges().data();
			for (unsigned int i = 0; i < nEdges; i++)
			{
				const unsigned int v1 = edges[i].m_vert[0] + offset;
				const unsigned int v2 = edges[i].m_vert[1] + offset;

				model.addDistanceConstraint(v1, v2);
			}
		}
		else if (clothSimulationMethod == 2)
		{

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
		else if (clothSimulationMethod == 3)
		{
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
}

void initTetModelConstraints()
{
	// init constraints
	for (unsigned int cm = 0; cm < model.getTetModels().size(); cm++)
	{
		const unsigned int offset = model.getTetModels()[cm]->getIndexOffset();
		const unsigned int nTets = model.getTetModels()[cm]->getParticleMesh().numTets();
		const unsigned int *tets = model.getTetModels()[cm]->getParticleMesh().getTets().data();
		const IndexedTetMesh::VertexTets *vTets = model.getTetModels()[cm]->getParticleMesh().getVertexTets().data();
		if (solidSimulationMethod == 1)
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
				const unsigned int v1 = tets[4 * i] + offset;
				const unsigned int v2 = tets[4 * i + 1] + offset;
				const unsigned int v3 = tets[4 * i + 2] + offset;
				const unsigned int v4 = tets[4 * i + 3] + offset;

				model.addVolumeConstraint(v1, v2, v3, v4);
			}
		}
		else if (solidSimulationMethod == 2)
		{
			TetModel::ParticleMesh &mesh = model.getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v1 = tets[4 * i] + offset;
				const unsigned int v2 = tets[4 * i + 1] + offset;
				const unsigned int v3 = tets[4 * i + 2] + offset;
				const unsigned int v4 = tets[4 * i + 3] + offset;

				model.addFEMTetConstraint(v1, v2, v3, v4);
			}
		}
		else if (solidSimulationMethod == 3)
		{
			TetModel::ParticleMesh &mesh = model.getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v1 = tets[4 * i] + offset;
				const unsigned int v2 = tets[4 * i + 1] + offset;
				const unsigned int v3 = tets[4 * i + 2] + offset;
				const unsigned int v4 = tets[4 * i + 3] + offset;

				model.addStrainTetConstraint(v1, v2, v3, v4);
			}
		}
		else if (solidSimulationMethod == 4)
		{
			TetModel::ParticleMesh &mesh = model.getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v[4] = { tets[4 * i] + offset,
					tets[4 * i + 1] + offset,
					tets[4 * i + 2] + offset,
					tets[4 * i + 3] + offset };
				// Important: Divide position correction by the number of clusters 
				// which contain the vertex.
				const unsigned int nc[4] = { vTets[v[0]].m_numTets, vTets[v[1]].m_numTets, vTets[v[2]].m_numTets, vTets[v[3]].m_numTets };
				model.addShapeMatchingConstraint(4, v, nc);
			}
		}
	}
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


/** Create the rigid body model
*/
void readScene()
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::TriangleModelVector &triModels = model.getTriangleModels();
	SimulationModel::TetModelVector &tetModels = model.getTetModels();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();

	StiffRodsSceneLoader::RodSceneData data;
	StiffRodsSceneLoader loader;
	loader.readStiffRodsScene(sceneFileName, data);
	LOG_INFO << "Scene: " << sceneFileName;

	camPos = data.m_camPosition;
	camLookat = data.m_camLookat;

	TimeManager::getCurrent()->setTimeStepSize(data.m_timeStepSize);

	sceneName = data.m_sceneName;

	sim.setGravity(data.m_gravity);
	sim.setMaxIterations(data.m_maxIter);
	sim.setMaxIterationsV(data.m_maxIterVel);
	sim.setVelocityUpdateMethod(data.m_velocityUpdateMethod);
	if (data.m_triangleModelSimulationMethod != -1)
		clothSimulationMethod = data.m_triangleModelSimulationMethod;
	if (data.m_tetModelSimulationMethod != -1)
		solidSimulationMethod = data.m_tetModelSimulationMethod;
	if (data.m_triangleModelBendingMethod != -1)
		bendingMethod = data.m_triangleModelBendingMethod;
	cd.setTolerance(data.m_contactTolerance);
	model.setContactStiffnessRigidBody(data.m_contactStiffnessRigidBody);
	model.setContactStiffnessParticleRigidBody(data.m_contactStiffnessParticleRigidBody);

	model.setClothStiffness(data.m_cloth_stiffness);
	model.setClothBendingStiffness(data.m_cloth_bendingStiffness);
	model.setClothXXStiffness(data.m_cloth_xxStiffness);
	model.setClothYYStiffness(data.m_cloth_yyStiffness);
	model.setClothXYStiffness(data.m_cloth_xyStiffness);
	model.setClothXYPoissonRatio(data.m_cloth_xyPoissonRatio);
	model.setClothYXPoissonRatio(data.m_cloth_yxPoissonRatio);
	model.setClothNormalizeStretch(data.m_cloth_normalizeStretch);
	model.setClothNormalizeShear(data.m_cloth_normalizeShear);

	//////////////////////////////////////////////////////////////////////////
	// rigid bodies
	//////////////////////////////////////////////////////////////////////////

	// map file names to loaded geometry to prevent multiple imports of same files
	std::map<std::string, pair<VertexData, IndexedFaceMesh>> objFiles;
	std::map<std::string, CubicSDFCollisionDetection::GridPtr> distanceFields;
	for (unsigned int rbIndex = 0; rbIndex < data.m_rigidBodyData.size(); rbIndex++)
	{
		SceneLoader::RigidBodyData &rbd = data.m_rigidBodyData[rbIndex];

		// Check if already loaded
		rbd.m_modelFile = FileSystem::normalizePath(rbd.m_modelFile);
		if (objFiles.find(rbd.m_modelFile) == objFiles.end())
		{
			IndexedFaceMesh mesh;
			VertexData vd;
			loadObj(rbd.m_modelFile, vd, mesh, Vector3r::Ones());
			objFiles[rbd.m_modelFile] = { vd, mesh };
		}

		const std::string basePath = FileSystem::getFilePath(sceneFileName);
		const string cachePath = basePath + "/Cache";
		const string resStr = to_string(rbd.m_resolutionSDF[0]) + "_" + to_string(rbd.m_resolutionSDF[1]) + "_" + to_string(rbd.m_resolutionSDF[2]);
		const std::string modelFileName = FileSystem::getFileNameWithExt(rbd.m_modelFile);
		const string sdfFileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + "_" + resStr + ".csdf");

		std::string sdfKey = rbd.m_collisionObjectFileName;
		if (sdfKey == "")
		{
			sdfKey = sdfFileName;
		}
		if (distanceFields.find(sdfKey) == distanceFields.end())
		{
			// Generate SDF
			if (rbd.m_collisionObjectType == SceneLoader::RigidBodyData::SDF)
			{
				if (rbd.m_collisionObjectFileName == "")
				{
					std::string md5FileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + ".md5");
					string md5Str = FileSystem::getFileMD5(rbd.m_modelFile);
					bool md5 = false;
					if (FileSystem::fileExists(md5FileName))
						md5 = FileSystem::checkMD5(md5Str, md5FileName);

					// check MD5 if cache file is available
					const string resStr = to_string(rbd.m_resolutionSDF[0]) + "_" + to_string(rbd.m_resolutionSDF[1]) + "_" + to_string(rbd.m_resolutionSDF[2]);
					const string sdfFileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + "_" + resStr + ".csdf");
					bool foundCacheFile = FileSystem::fileExists(sdfFileName);

					if (foundCacheFile && md5)
					{
						LOG_INFO << "Load cached SDF: " << sdfFileName;
						distanceFields[sdfFileName] = std::make_shared<CubicSDFCollisionDetection::Grid>(sdfFileName);
					}
					else
					{
						VertexData &vd = objFiles[rbd.m_modelFile].first;
						IndexedFaceMesh &mesh = objFiles[rbd.m_modelFile].second;

						std::vector<unsigned int> &faces = mesh.getFaces();
						const unsigned int nFaces = mesh.numFaces();

						Discregrid::TriangleMesh sdfMesh(&vd.getPosition(0)[0], faces.data(), vd.size(), nFaces);
						Discregrid::MeshDistance md(sdfMesh);
						Eigen::AlignedBox3d domain;
						for (auto const& x : sdfMesh.vertices())
						{
							domain.extend(x);
						}
						domain.max() += 1.0e-3 * domain.diagonal().norm() * Eigen::Vector3d::Ones();
						domain.min() -= 1.0e-3 * domain.diagonal().norm() * Eigen::Vector3d::Ones();

						LOG_INFO << "Set SDF resolution: " << rbd.m_resolutionSDF[0] << ", " << rbd.m_resolutionSDF[1] << ", " << rbd.m_resolutionSDF[2];
						distanceFields[sdfFileName] = std::make_shared<CubicSDFCollisionDetection::Grid>(domain, std::array<unsigned int, 3>({ rbd.m_resolutionSDF[0], rbd.m_resolutionSDF[1], rbd.m_resolutionSDF[2] }));
						auto func = Discregrid::DiscreteGrid::ContinuousFunction{};
						func = [&md](Eigen::Vector3d const& xi) {return md.signedDistanceCached(xi); };
						LOG_INFO << "Generate SDF for " << rbd.m_modelFile;
						distanceFields[sdfFileName]->addFunction(func, true);
						if (FileSystem::makeDir(cachePath) == 0)
						{
							LOG_INFO << "Save SDF: " << sdfFileName;
							distanceFields[sdfFileName]->save(sdfFileName);
							FileSystem::writeMD5File(rbd.m_modelFile, md5FileName);
						}
					}
				}
				else
				{
					std::string fileName = rbd.m_collisionObjectFileName;
					if (FileSystem::isRelativePath(fileName))
					{
						fileName = FileSystem::normalizePath(basePath + "/" + fileName);
					}
					LOG_INFO << "Load SDF: " << fileName;
					distanceFields[rbd.m_collisionObjectFileName] = std::make_shared<CubicSDFCollisionDetection::Grid>(fileName);
				}
			}
		}
	}

	for (unsigned int rbIndex = 0; rbIndex < data.m_tetModelData.size(); rbIndex++)
	{
		const SceneLoader::TetModelData &tmd = data.m_tetModelData[rbIndex];

		// Check if already loaded
		if ((tmd.m_modelFileVis != "") &&
			(objFiles.find(tmd.m_modelFileVis) == objFiles.end()))
		{
			IndexedFaceMesh mesh;
			VertexData vd;
			loadObj(FileSystem::normalizePath(tmd.m_modelFileVis), vd, mesh, Vector3r::Ones());
			objFiles[tmd.m_modelFileVis] = { vd, mesh };
		}
	}

	// Stiff rods: gather rigid body id of bodies belonging to a tree
	std::set<unsigned int> treeSegments;
	for (auto &tree : data.m_TreeModels)
	{
		treeSegments.insert(tree.m_SegmentIds.begin(), tree.m_SegmentIds.end());
	}
	// end stiff rods


	rb.resize(data.m_rigidBodyData.size());
	std::map<unsigned int, unsigned int> id_index;
	unsigned int rbIndex = 0;
	for (unsigned int idx(0); idx < data.m_rigidBodyData.size(); ++idx)
	{
		const SceneLoader::RigidBodyData &rbd = data.m_rigidBodyData[idx];

		// Stiff rods: do not handle segments of rods here
		if (treeSegments.find(rbd.m_id) != treeSegments.end())
			continue;
		// end stiff rods

		if (objFiles.find(rbd.m_modelFile) == objFiles.end())
			continue;

		id_index[rbd.m_id] = rbIndex;

		VertexData &vd = objFiles[rbd.m_modelFile].first;
		IndexedFaceMesh &mesh = objFiles[rbd.m_modelFile].second;

		rb[rbIndex] = new RigidBody();
		rb[rbIndex]->initBody(rbd.m_density,
			rbd.m_x,
			rbd.m_q,
			vd, mesh,
			rbd.m_scale);

		if (!rbd.m_isDynamic)
			rb[rbIndex]->setMass(0.0);
		else
		{
			rb[rbIndex]->setVelocity(rbd.m_v);
			rb[rbIndex]->setAngularVelocity(rbd.m_omega);
		}
		rb[rbIndex]->setRestitutionCoeff(rbd.m_restitutionCoeff);
		rb[rbIndex]->setFrictionCoeff(rbd.m_frictionCoeff);

		const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
		const unsigned int nVert = static_cast<unsigned int>(vertices->size());

		switch (rbd.m_collisionObjectType)
		{
		case SceneLoader::RigidBodyData::No_Collision_Object: break;
		case SceneLoader::RigidBodyData::Sphere:
			cd.addCollisionSphere(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale[0], rbd.m_testMesh, rbd.m_invertSDF);
			break;
		case SceneLoader::RigidBodyData::Box:
			cd.addCollisionBox(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale, rbd.m_testMesh, rbd.m_invertSDF);
			break;
		case SceneLoader::RigidBodyData::Cylinder:
			cd.addCollisionCylinder(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale.head<2>(), rbd.m_testMesh, rbd.m_invertSDF);
			break;
		case SceneLoader::RigidBodyData::Torus:
			cd.addCollisionTorus(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale.head<2>(), rbd.m_testMesh, rbd.m_invertSDF);
			break;
		case SceneLoader::RigidBodyData::HollowSphere:
			cd.addCollisionHollowSphere(rbIndex, PBD::CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale[0], rbd.m_thicknessSDF, rbd.m_testMesh, rbd.m_invertSDF);
			break;
		case SceneLoader::RigidBodyData::HollowBox:
			cd.addCollisionHollowBox(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale, rbd.m_thicknessSDF, rbd.m_testMesh, rbd.m_invertSDF);
			break;
		case SceneLoader::RigidBodyData::SDF:
		{
												if (rbd.m_collisionObjectFileName == "")
												{
													const std::string basePath = FileSystem::getFilePath(sceneFileName);
													const string cachePath = basePath + "/Cache";
													const string resStr = to_string(rbd.m_resolutionSDF[0]) + "_" + to_string(rbd.m_resolutionSDF[1]) + "_" + to_string(rbd.m_resolutionSDF[2]);
													const std::string modelFileName = FileSystem::getFileNameWithExt(rbd.m_modelFile);
													const string sdfFileName = FileSystem::normalizePath(cachePath + "/" + modelFileName + "_" + resStr + ".csdf");
													cd.addCubicSDFCollisionObject(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, distanceFields[sdfFileName], rbd.m_collisionObjectScale, rbd.m_testMesh, rbd.m_invertSDF);
												}
												else
													cd.addCubicSDFCollisionObject(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, distanceFields[rbd.m_collisionObjectFileName], rbd.m_collisionObjectScale, rbd.m_testMesh, rbd.m_invertSDF);
												break;
		}
		}
		// stiff rods
		++rbIndex;
		// end stiff rods
	}

	//////////////////////////////////////////////////////////////////////////
	// stiff rod models
	//////////////////////////////////////////////////////////////////////////

	if (data.m_TreeModels.size())
		LOG_INFO << "Loading " << data.m_TreeModels.size() << " stiff rods" ;

	std::map<unsigned int, unsigned int> index_id;

	std::map<unsigned int, StiffRodsSceneLoader::StretchBendingTwistingConstraintData*> cIDTocData;
	for (auto &constraint : data.m_StretchBendingTwistingConstraints)
	{
		cIDTocData[constraint.m_Id] = &constraint;
	}

	unsigned int lengthIdx(1), widthIdx(0), heightIdx(2); // y-axis is used as the length axis to be compatible to Blender's armatures (Blender 2.78)
	std::map<unsigned int, Vector3r> segmentScale;
	std::map<unsigned int, Real> segmentDensity;

	for (auto &treeModel : data.m_TreeModels)
	{
		START_TIMING("Load tree");

		std::vector<RigidBody*> treeSegments;
		treeSegments.reserve(treeModel.m_SegmentIds.size());
		
		const Vector3r &modelPosition(treeModel.m_X);
		const Quaternionr &modelRotationQuat(treeModel.m_Q);
		Matrix3r modelRotationMat(modelRotationQuat.toRotationMatrix());

		std::cout << "Loading " << treeModel.m_SegmentIds.size() << " stiff rod segments" << std::endl;
		for (size_t idx(0); idx < data.m_rigidBodyData.size(); ++idx)
		{
			auto &segmentData(data.m_rigidBodyData[static_cast<unsigned int>(idx)]);
			if (treeModel.m_SegmentIds.find(segmentData.m_id) == treeModel.m_SegmentIds.end())
				continue;

			if (objFiles.find(segmentData.m_modelFile) == objFiles.end())
				continue;

			// continue if segment has already been created because it is also a member of another tree
			if (id_index.find(segmentData.m_id) != id_index.end())
				continue;

			VertexData &vd = objFiles[segmentData.m_modelFile].first;
			IndexedFaceMesh &mesh = objFiles[segmentData.m_modelFile].second;

			//compute mass and inertia of a cylinder
			Vector3r & scale(segmentData.m_scale);
			Real length(scale[lengthIdx]), width(scale[widthIdx]), height(scale[heightIdx]);
			Real density(segmentData.m_density);
			segmentDensity[segmentData.m_id] = density;

			Real mass(M_PI_4 * width * height * length * density);

			const Real Iy = 1. / 12. * mass*(3.*(0.25*height*height) + length*length);
			const Real Iz = 1. / 12. * mass*(3.*(0.25*width*width) + length*length);
			const Real Ix = 0.25 * mass*(0.25*(height*height + width*width));

			Vector3r inertiaTensor;
			inertiaTensor(lengthIdx) = Ix;
			inertiaTensor(widthIdx) = Iy;
			inertiaTensor(heightIdx) = Iz;

			rb[rbIndex] = new RigidBody();
			rb[rbIndex]->initBody(mass,
				modelPosition + modelRotationMat * segmentData.m_x,
				inertiaTensor,
				modelRotationQuat * segmentData.m_q,
				vd, mesh, scale);

			if (!segmentData.m_isDynamic)
				rb[rbIndex]->setMass(0.0);
			else
			{
				rb[rbIndex]->setVelocity(segmentData.m_v);
				rb[rbIndex]->setAngularVelocity(segmentData.m_omega);
			}
			rb[rbIndex]->setRestitutionCoeff(segmentData.m_restitutionCoeff);
			rb[rbIndex]->setFrictionCoeff(segmentData.m_frictionCoeff);
			
			segmentScale[segmentData.m_id] = scale;

			id_index[segmentData.m_id] = rbIndex;
			index_id[rbIndex] = segmentData.m_id;
			treeSegments.push_back(rb[rbIndex]);

			++rbIndex;
		}

		// Make roots static;
		for (auto id : treeModel.m_StaticSegments)
		{
			std::map<unsigned int, unsigned int>::iterator it(id_index.find(id));
			if (it != id_index.end())
			{
				rb[it->second]->setMass(0.);
			}
		}


		// create zero-stretch, bending and twisting constraints

		std::vector<std::pair<unsigned int, unsigned int>>  constraintSegmentIndices;
		std::vector<Vector3r> constraintPositions;
		std::vector<Real> averageRadii;
		std::vector<Real> averageSegmentLengths;
		std::vector<Real> youngsModuli(treeModel.m_StretchBendingTwistingConstraintIds.size(), treeModel.m_YoungsModulus);
		std::vector<Real> torsionModuli(treeModel.m_StretchBendingTwistingConstraintIds.size(), treeModel.m_TorsionModulus);

		constraintSegmentIndices.reserve(treeModel.m_StretchBendingTwistingConstraintIds.size());
		constraintPositions.reserve(treeModel.m_StretchBendingTwistingConstraintIds.size());
		averageRadii.reserve(treeModel.m_StretchBendingTwistingConstraintIds.size());
		averageSegmentLengths.reserve(treeModel.m_StretchBendingTwistingConstraintIds.size());

		for (unsigned int cID : treeModel.m_StretchBendingTwistingConstraintIds)
		{
			if (cIDTocData.find(cID) == cIDTocData.end())
				continue;

			auto &sbtConstraint = *cIDTocData[cID];

			std::map<unsigned int, unsigned int>::iterator itFirstIdx(id_index.find(sbtConstraint.m_SegmentID[0])),
				itSecondIdx(id_index.find(sbtConstraint.m_SegmentID[1]));

			if (itFirstIdx == id_index.end() || itSecondIdx == id_index.end())
				continue;

			constraintSegmentIndices.push_back(std::pair<unsigned int, unsigned int>(itFirstIdx->second, itSecondIdx->second));			

			const Vector3r &localPosition(sbtConstraint.m_Position);
			Vector3r x(modelPosition + modelRotationMat * localPosition);
			constraintPositions.push_back(x);

			// compute average length
			Real avgLength(0.5*(segmentScale[sbtConstraint.m_SegmentID[0]][lengthIdx] + segmentScale[sbtConstraint.m_SegmentID[1]][lengthIdx]));
			averageSegmentLengths.push_back(avgLength);
			
			// compute average radius
			averageRadii.push_back(0.125*(segmentScale[sbtConstraint.m_SegmentID[0]][widthIdx]
				+ segmentScale[sbtConstraint.m_SegmentID[0]][heightIdx]
				+ segmentScale[sbtConstraint.m_SegmentID[1]][widthIdx] + segmentScale[sbtConstraint.m_SegmentID[1]][heightIdx]));
		}

		model.addDirectPositionBasedSolverForStiffRodsConstraint(constraintSegmentIndices,
			constraintPositions, averageRadii, averageSegmentLengths, youngsModuli, torsionModuli);		

		STOP_TIMING_PRINT;// load tree
	}


	//////////////////////////////////////////////////////////////////////////
	// triangle models
	//////////////////////////////////////////////////////////////////////////

	// map file names to loaded geometry to prevent multiple imports of same files
	std::map<std::string, pair<VertexData, IndexedFaceMesh>> triFiles;
	for (unsigned int i = 0; i < data.m_triangleModelData.size(); i++)
	{
		const SceneLoader::TriangleModelData &tmd = data.m_triangleModelData[i];

		// Check if already loaded
		if (triFiles.find(tmd.m_modelFile) == triFiles.end())
		{
			IndexedFaceMesh mesh;
			VertexData vd;
			loadObj(FileSystem::normalizePath(tmd.m_modelFile), vd, mesh, Vector3r::Ones());
			triFiles[tmd.m_modelFile] = { vd, mesh };
		}
	}

	triModels.reserve(data.m_triangleModelData.size());
	std::map<unsigned int, unsigned int> tm_id_index;
	for (unsigned int i = 0; i < data.m_triangleModelData.size(); i++)
	{
		const SceneLoader::TriangleModelData &tmd = data.m_triangleModelData[i];

		if (triFiles.find(tmd.m_modelFile) == triFiles.end())
			continue;

		tm_id_index[tmd.m_id] = i;

		VertexData vd = triFiles[tmd.m_modelFile].first;
		IndexedFaceMesh &mesh = triFiles[tmd.m_modelFile].second;

		const Matrix3r R = tmd.m_q.matrix();
		for (unsigned int j = 0; j < vd.size(); j++)
		{
			vd.getPosition(j) = R * (vd.getPosition(j).cwiseProduct(tmd.m_scale)) + tmd.m_x;
		}

		model.addTriangleModel(vd.size(), mesh.numFaces(), &vd.getPosition(0), mesh.getFaces().data(), mesh.getUVIndices(), mesh.getUVs());

		TriangleModel *tm = triModels[triModels.size() - 1];
		ParticleData &pd = model.getParticles();
		unsigned int offset = tm->getIndexOffset();

		for (unsigned int j = 0; j < tmd.m_staticParticles.size(); j++)
		{
			const unsigned int index = tmd.m_staticParticles[j] + offset;
			pd.setMass(index, 0.0);
		}

		tm->setRestitutionCoeff(tmd.m_restitutionCoeff);
		tm->setFrictionCoeff(tmd.m_frictionCoeff);
	}

	initTriangleModelConstraints();

	//////////////////////////////////////////////////////////////////////////
	// tet models
	//////////////////////////////////////////////////////////////////////////

	// map file names to loaded geometry to prevent multiple imports of same files
	std::map<pair<string, string>, pair<vector<Vector3r>, vector<unsigned int>>> tetFiles;
	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		const SceneLoader::TetModelData &tmd = data.m_tetModelData[i];

		// Check if already loaded
		pair<string, string> fileNames = { tmd.m_modelFileNodes, tmd.m_modelFileElements };
		if (tetFiles.find(fileNames) == tetFiles.end())
		{
			vector<Vector3r> vertices;
			vector<unsigned int> tets;
			TetGenLoader::loadTetgenModel(FileSystem::normalizePath(tmd.m_modelFileNodes), FileSystem::normalizePath(tmd.m_modelFileElements), vertices, tets);
			tetFiles[fileNames] = { vertices, tets };
		}
	}

	tetModels.reserve(data.m_tetModelData.size());
	std::map<unsigned int, unsigned int> tm_id_index2;
	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		const SceneLoader::TetModelData &tmd = data.m_tetModelData[i];

		pair<string, string> fileNames = { tmd.m_modelFileNodes, tmd.m_modelFileElements };
		auto geo = tetFiles.find(fileNames);
		if (geo == tetFiles.end())
			continue;

		tm_id_index2[tmd.m_id] = i;

		vector<Vector3r> vertices = geo->second.first;
		vector<unsigned int> &tets = geo->second.second;

		const Matrix3r R = tmd.m_q.matrix();
		for (unsigned int j = 0; j < vertices.size(); j++)
		{
			vertices[j] = R * (vertices[j].cwiseProduct(tmd.m_scale)) + tmd.m_x;
		}

		model.addTetModel((unsigned int)vertices.size(), (unsigned int)tets.size() / 4, vertices.data(), tets.data());

		TetModel *tm = tetModels[tetModels.size() - 1];
		ParticleData &pd = model.getParticles();
		unsigned int offset = tm->getIndexOffset();

		for (unsigned int j = 0; j < tmd.m_staticParticles.size(); j++)
		{
			const unsigned int index = tmd.m_staticParticles[j] + offset;
			pd.setMass(index, 0.0);
		}

		// read visualization mesh
		if (tmd.m_modelFileVis != "")
		{
			if (objFiles.find(tmd.m_modelFileVis) != objFiles.end())
			{
				IndexedFaceMesh &visMesh = tm->getVisMesh();
				VertexData &vdVis = tm->getVisVertices();
				vdVis = objFiles[tmd.m_modelFileVis].first;
				visMesh = objFiles[tmd.m_modelFileVis].second;

				for (unsigned int j = 0; j < vdVis.size(); j++)
					vdVis.getPosition(j) = R * (vdVis.getPosition(j).cwiseProduct(tmd.m_scale)) + tmd.m_x;

				tm->updateMeshNormals(pd);
				tm->attachVisMesh(pd);
				tm->updateVisMesh(pd);
			}
		}

		tm->setRestitutionCoeff(tmd.m_restitutionCoeff);
		tm->setFrictionCoeff(tmd.m_frictionCoeff);

		tm->updateMeshNormals(pd);
	}

	initTetModelConstraints();

	// init collision objects for deformable models
	ParticleData &pd = model.getParticles();
	for (unsigned int i = 0; i < data.m_triangleModelData.size(); i++)
	{
		TriangleModel *tm = triModels[i];
		unsigned int offset = tm->getIndexOffset();
		const unsigned int nVert = tm->getParticleMesh().numVertices();
		cd.addCollisionObjectWithoutGeometry(i, CollisionDetection::CollisionObject::TriangleModelCollisionObjectType, &pd.getPosition(offset), nVert);

	}
	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		TetModel *tm = tetModels[i];
		unsigned int offset = tm->getIndexOffset();
		const unsigned int nVert = tm->getParticleMesh().numVertices();
		cd.addCollisionObjectWithoutGeometry(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert);
	}

	//////////////////////////////////////////////////////////////////////////
	// joints
	//////////////////////////////////////////////////////////////////////////

	for (unsigned int i = 0; i < data.m_ballJointData.size(); i++)
	{
		const SceneLoader::BallJointData &jd = data.m_ballJointData[i];
		model.addBallJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position);
	}

	for (unsigned int i = 0; i < data.m_ballOnLineJointData.size(); i++)
	{
		const SceneLoader::BallOnLineJointData &jd = data.m_ballOnLineJointData[i];
		model.addBallOnLineJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
	}

	for (unsigned int i = 0; i < data.m_hingeJointData.size(); i++)
	{
		const SceneLoader::HingeJointData &jd = data.m_hingeJointData[i];
		model.addHingeJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
	}

	for (unsigned int i = 0; i < data.m_universalJointData.size(); i++)
	{
		const SceneLoader::UniversalJointData &jd = data.m_universalJointData[i];
		model.addUniversalJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis[0], jd.m_axis[1]);
	}

	for (unsigned int i = 0; i < data.m_sliderJointData.size(); i++)
	{
		const SceneLoader::SliderJointData &jd = data.m_sliderJointData[i];
		model.addSliderJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
	}

	for (unsigned int i = 0; i < data.m_rigidBodyParticleBallJointData.size(); i++)
	{
		const SceneLoader::RigidBodyParticleBallJointData &jd = data.m_rigidBodyParticleBallJointData[i];
		model.addRigidBodyParticleBallJoint(id_index[jd.m_bodyID[0]], jd.m_bodyID[1]);
	}

	for (unsigned int i = 0; i < data.m_targetAngleMotorHingeJointData.size(); i++)
	{
		const SceneLoader::TargetAngleMotorHingeJointData &jd = data.m_targetAngleMotorHingeJointData[i];
		model.addTargetAngleMotorHingeJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((MotorJoint*)constraints[constraints.size() - 1])->setTarget(jd.m_target);
		((MotorJoint*)constraints[constraints.size() - 1])->setTargetSequence(jd.m_targetSequence);
		((MotorJoint*)constraints[constraints.size() - 1])->setRepeatSequence(jd.m_repeat);
	}

	for (unsigned int i = 0; i < data.m_targetVelocityMotorHingeJointData.size(); i++)
	{
		const SceneLoader::TargetVelocityMotorHingeJointData &jd = data.m_targetVelocityMotorHingeJointData[i];
		model.addTargetVelocityMotorHingeJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((MotorJoint*)constraints[constraints.size() - 1])->setTarget(jd.m_target);
		((MotorJoint*)constraints[constraints.size() - 1])->setTargetSequence(jd.m_targetSequence);
		((MotorJoint*)constraints[constraints.size() - 1])->setRepeatSequence(jd.m_repeat);
	}

	for (unsigned int i = 0; i < data.m_targetPositionMotorSliderJointData.size(); i++)
	{
		const SceneLoader::TargetPositionMotorSliderJointData &jd = data.m_targetPositionMotorSliderJointData[i];
		model.addTargetPositionMotorSliderJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((MotorJoint*)constraints[constraints.size() - 1])->setTarget(jd.m_target);
		((MotorJoint*)constraints[constraints.size() - 1])->setTargetSequence(jd.m_targetSequence);
		((MotorJoint*)constraints[constraints.size() - 1])->setRepeatSequence(jd.m_repeat);
	}

	for (unsigned int i = 0; i < data.m_targetVelocityMotorSliderJointData.size(); i++)
	{
		const SceneLoader::TargetVelocityMotorSliderJointData &jd = data.m_targetVelocityMotorSliderJointData[i];
		model.addTargetVelocityMotorSliderJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((MotorJoint*)constraints[constraints.size() - 1])->setTarget(jd.m_target);
		((MotorJoint*)constraints[constraints.size() - 1])->setTargetSequence(jd.m_targetSequence);
		((MotorJoint*)constraints[constraints.size() - 1])->setRepeatSequence(jd.m_repeat);
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

void TW_CALL setContactTolerance(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((DistanceFieldCollisionDetection*)clientData)->setTolerance(val);
}

void TW_CALL getContactTolerance(void *value, void *clientData)
{
	*(Real *)(value) = ((DistanceFieldCollisionDetection*)clientData)->getTolerance();
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

void TW_CALL setClothSimulationMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	*((short*)clientData) = val;
	reset();
}

void TW_CALL getClothSimulationMethod(void *value, void *clientData)
{
	*(short *)(value) = *((short*)clientData);
}

void TW_CALL setSolidSimulationMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	*((short*)clientData) = val;
	reset();
}

void TW_CALL getSolidSimulationMethod(void *value, void *clientData)
{
	*(short *)(value) = *((short*)clientData);
}

