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
#include "Demos/Utils/Utilities.h"
#include "Demos/Simulation/DistanceFieldCollisionDetection.h"
#include "Demos/Utils/SceneLoader.h"
#include "Demos/Utils/TetGenLoader.h"
#include "Demos/Utils/Timing.h"

#define _USE_MATH_DEFINES
#include "math.h"


// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
	#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;

void timeStep ();
void buildModel ();
void readScene();
void render ();
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


SimulationModel model;
DistanceFieldCollisionDetection cd;
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
int drawBVHDepth = -1;
float jointColor[4] = { 0.0f, 0.6f, 0.2f, 1 };
string sceneFileName = "/scenes/DeformableSolidCollisionScene.json";
string sceneName;
Vector3r camPos;
Vector3r camLookat;


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	exePath = Utilities::getFilePath(argv[0]);
	dataPath = exePath + "/" + std::string(PBD_DATA_PATH);

	if (argc > 1)
		sceneFileName = string(argv[1]);
	else
		sceneFileName = Utilities::normalizePath(dataPath + sceneFileName);

	buildModel();

	// OpenGL
	MiniGL::init(argc, argv, 1024, 768, 0, 0, sceneName.c_str());
	MiniGL::initLights ();
	MiniGL::initTexture();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection);
	initShader();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport(40.0f, 0.1f, 500.0f, camPos, camLookat);

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarRW(MiniGL::getTweakBar(), "RenderContacts", TW_TYPE_BOOLCPP, &renderContacts, " label='Render contacts' group=Simulation ");
	TwAddVarRW(MiniGL::getTweakBar(), "RenderAABBs", TW_TYPE_BOOLCPP, &drawAABB, " label='Render AABBs' group=Simulation ");
	TwAddVarRW(MiniGL::getTweakBar(), "RenderBVH", TW_TYPE_INT32, &drawBVHDepth, " label='Render BVH depth' group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_REAL, setTimeStep, getTimeStep, &model, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "MaxIter", TW_TYPE_UINT32, setMaxIterations, getMaxIterations, &sim, " label='Max. iterations'  min=1 step=1 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "MaxIterV", TW_TYPE_UINT32, setMaxIterationsV, getMaxIterationsV, &sim, " label='Max. iterations Vel.'  min=1 step=1 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactTolerance", TW_TYPE_REAL, setContactTolerance, getContactTolerance, &cd, " label='Contact tolerance'  min=0.0 step=0.001 precision=3 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessRigidBody", TW_TYPE_REAL, setContactStiffnessRigidBody, getContactStiffnessRigidBody, &model, " label='Contact stiffness RB'  min=0.0 step=0.1 precision=2 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessParticleRigidBody", TW_TYPE_REAL, setContactStiffnessParticleRigidBody, getContactStiffnessParticleRigidBody, &model, " label='Contact stiffness Particle-RB'  min=0.0 step=0.1 precision=2 group=Simulation ");
	TwType enumType2 = TwDefineEnum("ClothSimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "ClothSimulationMethod", enumType2, setClothSimulationMethod, getClothSimulationMethod, &clothSimulationMethod, " label='Cloth sim. method' enum='0 {None}, 1 {Distance constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics}' group=Simulation");
	TwType enumType3 = TwDefineEnum("SolidSimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "SolidSimulationMethod", enumType3, setSolidSimulationMethod, getSolidSimulationMethod, &solidSimulationMethod,
		" label='Solid sim. method' enum='0 {None}, 1 {Volume constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics (no inversion handling)}, 4 {Shape matching (no inversion handling)}' group=Simulation");
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
	TwType enumType4 = TwDefineEnum("BendingMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "BendingMethod", enumType4, setBendingMethod, getBendingMethod, &bendingMethod, " label='Bending method' enum='0 {None}, 1 {Dihedral angle}, 2 {Isometric bending}' group=Bending");
	TwAddVarCB(MiniGL::getTweakBar(), "BendingStiffness", TW_TYPE_REAL, setBendingStiffness, getBendingStiffness, &model, " label='Bending stiffness'  min=0.0 step=0.01 precision=4 group=Bending ");

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

void timeStep ()
{
	if (doPause)
		return;

	// Simulation code
	for (unsigned int i = 0; i < 8; i++)
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

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.005);

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

void render ()
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



	if (drawAABB || (drawBVHDepth >= 0))
	{
		ObjectArray<CollisionDetection::CollisionObject*> &collisionObjects = cd.getCollisionObjects();
		for (unsigned int k = 0; k < collisionObjects.size(); k++)
		{
			if (drawAABB)
				renderAABB(collisionObjects[k]->m_aabb);

			if (drawBVHDepth >= 0)
			{
				if (cd.isDistanceFieldCollisionObject(collisionObjects[k]))
				{
					const PointCloudBSH &bvh = ((DistanceFieldCollisionDetection::DistanceFieldCollisionObject*) collisionObjects[k])->m_bvh;

					std::function<bool(unsigned int, unsigned int)> predicate = [&](unsigned int node_index, unsigned int depth) { return (int) depth <= drawBVHDepth; };
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

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
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


/** Create the rigid body model
*/
void readScene()
{
	SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
	SimulationModel::TriangleModelVector &triModels = model.getTriangleModels();
	SimulationModel::TetModelVector &tetModels = model.getTetModels();
	SimulationModel::ConstraintVector &constraints = model.getConstraints();

	SceneLoader::SceneData data;
	SceneLoader loader;
	loader.readScene(sceneFileName, data);
	std::cout << "Scene: " << sceneFileName << "\n";

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
	for (unsigned int i = 0; i < data.m_rigidBodyData.size(); i++)
	{
		const SceneLoader::RigidBodyData &rbd = data.m_rigidBodyData[i];

		// Check if already loaded
		if (objFiles.find(rbd.m_modelFile) == objFiles.end())
		{
			IndexedFaceMesh mesh;
			VertexData vd;
			OBJLoader::loadObj(Utilities::normalizePath(rbd.m_modelFile), vd, mesh);
			objFiles[rbd.m_modelFile] = { vd, mesh };
		}
	}

	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		const SceneLoader::TetModelData &tmd = data.m_tetModelData[i];

		// Check if already loaded
		if ((tmd.m_modelFileVis != "") &&
			(objFiles.find(tmd.m_modelFileVis) == objFiles.end()))
		{
			IndexedFaceMesh mesh;
			VertexData vd;
			OBJLoader::loadObj(Utilities::normalizePath(tmd.m_modelFileVis), vd, mesh);
			objFiles[tmd.m_modelFileVis] = { vd, mesh };
		}
	}


	rb.resize(data.m_rigidBodyData.size());
	std::map<unsigned int, unsigned int> id_index;
	for (unsigned int i = 0; i < data.m_rigidBodyData.size(); i++)
	{
		const SceneLoader::RigidBodyData &rbd = data.m_rigidBodyData[i];

		if (objFiles.find(rbd.m_modelFile) == objFiles.end())
			continue;

		id_index[rbd.m_id] = i;

		VertexData &vd = objFiles[rbd.m_modelFile].first;
		IndexedFaceMesh &mesh = objFiles[rbd.m_modelFile].second;

		rb[i] = new RigidBody();
		rb[i]->initBody(rbd.m_density,
			rbd.m_x,
			rbd.m_q,
			vd, mesh,
			rbd.m_scale);

		if (!rbd.m_isDynamic)
			rb[i]->setMass(0.0);
		else
		{
			rb[i]->setVelocity(rbd.m_v);
			rb[i]->setAngularVelocity(rbd.m_omega);
		}
		rb[i]->setRestitutionCoeff(rbd.m_restitutionCoeff);
		rb[i]->setFrictionCoeff(rbd.m_frictionCoeff);

		const std::vector<Vector3r> *vertices = rb[i]->getGeometry().getVertexDataLocal().getVertices();
		const unsigned int nVert = static_cast<unsigned int>(vertices->size());

		switch (rbd.m_collisionObjectType)
		{
			case SceneLoader::RigidBodyData::No_Collision_Object: break;
			case SceneLoader::RigidBodyData::Sphere: 
				cd.addCollisionSphere(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale[0], rbd.m_testMesh, rbd.m_invertSDF);
				break;
			case SceneLoader::RigidBodyData::Box:
				cd.addCollisionBox(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale, rbd.m_testMesh, rbd.m_invertSDF);
				break;
			case SceneLoader::RigidBodyData::Cylinder:
				cd.addCollisionCylinder(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale.head<2>(), rbd.m_testMesh, rbd.m_invertSDF);
				break;
			case SceneLoader::RigidBodyData::Torus:
				cd.addCollisionTorus(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale.head<2>(), rbd.m_testMesh, rbd.m_invertSDF);
				break;
			case SceneLoader::RigidBodyData::HollowSphere:
				cd.addCollisionHollowSphere(i, PBD::CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale[0], rbd.m_thicknessSDF, rbd.m_testMesh, rbd.m_invertSDF);
				break;
			case SceneLoader::RigidBodyData::HollowBox:
				cd.addCollisionHollowBox(i, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale, rbd.m_thicknessSDF, rbd.m_testMesh, rbd.m_invertSDF);
				break;
		}
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
			OBJLoader::loadObj(Utilities::normalizePath(tmd.m_modelFile), vd, mesh);
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
			TetGenLoader::loadTetgenModel(Utilities::normalizePath(tmd.m_modelFileNodes), Utilities::normalizePath(tmd.m_modelFileElements), vertices, tets);
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
		((TargetAngleMotorHingeJoint*)constraints[constraints.size() - 1])->setTargetAngle(jd.m_target);
	}

	for (unsigned int i = 0; i < data.m_targetVelocityMotorHingeJointData.size(); i++)
	{
		const SceneLoader::TargetVelocityMotorHingeJointData &jd = data.m_targetVelocityMotorHingeJointData[i];
		model.addTargetVelocityMotorHingeJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((TargetVelocityMotorHingeJoint*)constraints[constraints.size() - 1])->setTargetAngularVelocity(jd.m_target);
	}

	for (unsigned int i = 0; i < data.m_targetPositionMotorSliderJointData.size(); i++)
	{
		const SceneLoader::TargetPositionMotorSliderJointData &jd = data.m_targetPositionMotorSliderJointData[i];
		model.addTargetPositionMotorSliderJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((TargetPositionMotorSliderJoint*)constraints[constraints.size() - 1])->setTargetPosition(jd.m_target);
	}

	for (unsigned int i = 0; i < data.m_targetVelocityMotorSliderJointData.size(); i++)
	{
		const SceneLoader::TargetVelocityMotorSliderJointData &jd = data.m_targetVelocityMotorSliderJointData[i];
		model.addTargetVelocityMotorSliderJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((TargetVelocityMotorSliderJoint*)constraints[constraints.size() - 1])->setTargetVelocity(jd.m_target);
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
