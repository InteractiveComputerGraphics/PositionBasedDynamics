#include "Demos/Utils/Config.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Demos/Utils/TimeManager.h"
#include <Eigen/Dense>
#include "RigidBodyParticleModel.h"
#include "TimeStepRigidBodyParticleModel.h"
#include <iostream>

#define _USE_MATH_DEFINES
#include "math.h"

// Enable memory leak detection
#ifdef _DEBUG
	#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;

void timeStep ();
void buildModel ();
void createRigidBodyModel();
void createClothMesh();
void render ();
void reset();
void cleanup();
void selection(const Eigen::Vector2i &start, const Eigen::Vector2i &end);
void TW_CALL setTimeStep(const void *value, void *clientData);
void TW_CALL getTimeStep(void *value, void *clientData);
void TW_CALL setVelocityUpdateMethod(const void *value, void *clientData);
void TW_CALL getVelocityUpdateMethod(void *value, void *clientData);
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


RigidBodyParticleModel model;
TimeStepRigidBodyParticleModel sim;

const int nRows = 20;
const int nCols = 20;
const float clothWidth = 10.0f;
const float clothHeight = 10.0f;
const float width = 0.2f;
const float height = 2.0f;
const float depth = 0.2f;
bool doPause = true;
std::vector<unsigned int> selectedBodies;
std::vector<unsigned int> selectedParticles;
Eigen::Vector3f oldMousePos;
float jointColor[4] = { 0.0f, 0.4f, 0.2f, 1.0f };
float dynamicBodyColor[4] = { 0.1f, 0.4f, 0.8f, 1 };
float staticBodyColor[4] = { 0.4f, 0.4f, 0.4f, 1.0f };


// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	// OpenGL
	MiniGL::init (argc, argv, 1024, 768, 0, 0, "Rigid body demo");
	MiniGL::initLights ();
	MiniGL::initTexture();
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setSelectionFunc(selection);

	buildModel ();

	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3f (0.0, -10.0, 30.0), Vector3f (0.0, 0.0, 0.0));

	TwAddVarRW(MiniGL::getTweakBar(), "Pause", TW_TYPE_BOOLCPP, &doPause, " label='Pause' group=Simulation key=SPACE ");
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_FLOAT, setTimeStep, getTimeStep, 0, " label='Time step size'  min=0.0 max = 0.1 step=0.001 precision=4 group=Simulation ");
	TwType enumType = TwDefineEnum("VelocityUpdateMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "VelocityUpdateMethod", enumType, setVelocityUpdateMethod, getVelocityUpdateMethod, &sim, " label='Velocity update method' enum='0 {First Order Update}, 1 {Second Order Update}' group=Simulation");
	TwType enumType2 = TwDefineEnum("SimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "SimulationMethod", enumType2, setSimulationMethod, getSimulationMethod, &sim, " label='Simulation method' enum='0 {None}, 1 {Distance constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics}' group=Simulation");
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
	TwType enumType3 = TwDefineEnum("BendingMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "BendingMethod", enumType3, setBendingMethod, getBendingMethod, &sim, " label='Bending method' enum='0 {None}, 1 {Dihedral angle}, 2 {Isometric bending}' group=Bending");
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
	sim.reset();
	TimeManager::getCurrent()->setTime(0.0);
}

void mouseMove(int x, int y)
{
	Eigen::Vector3f mousePos;
	MiniGL::unproject(x, y, mousePos);
	const Eigen::Vector3f diff = mousePos - oldMousePos;

	TimeManager *tm = TimeManager::getCurrent();
	const float h = tm->getTimeStepSize();

	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
	for (size_t j = 0; j < selectedBodies.size(); j++)
	{
		rb[selectedBodies[j]]->getVelocity() += 1.0f / h * diff;
	}
	ParticleData &pd = model.getParticleMesh().getVertexData();
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
	ParticleData &pd = model.getParticleMesh().getVertexData();
	Selection::selectRect(start, end, &pd.getPosition(0), &pd.getPosition(pd.size() - 1), selectedParticles);
	
	selectedBodies.clear(); 
	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
	std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > x;
	x.resize(rb.size());
 	for (unsigned int i = 0; i < rb.size(); i++)
 	{
 		x[i] = rb[i]->getPosition();
 	}
 
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
	for (unsigned int i = 0; i < 4; i++)
	{
		TimeManager *tm = TimeManager::getCurrent();
		const float h = tm->getTimeStepSize();
		const float currentTime = tm->getTime();

		sim.step(model);

		tm->setTime(currentTime + h);
	}
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (0.005f);

	createClothMesh();
	createRigidBodyModel();	
}

void renderBallJoint(RigidBodyParticleModel::BallJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(2), 0.1f, jointColor);
}

void renderRigidBodyParticleBallJoint(RigidBodyParticleModel::RigidBodyParticleBallJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(1), 0.1f, jointColor);
}

void renderRigidBodyModel()
{
	// Draw simulation model

	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyParticleModel::JointVector &joints = model.getJoints();

	float selectionColor[4] = { 0.8f, 0.0f, 0.0f, 1 };
	float surfaceColor[4] = { 0.1f, 0.4f, 0.8f, 1 };

	for (size_t i = 0; i < rb.size(); i++)
	{
		bool selected = false;
		for (unsigned int j = 0; j < selectedBodies.size(); j++)
		{
			if (selectedBodies[j] == i)
				selected = true;
		}

// 		if (rb[i]->getMass() == 0.0f)
// 			MiniGL::drawCube(rb[i]->getPosition(), rb[i]->getRotationMatrix().transpose(), 0.175f, 0.175f, 0.175f, staticBodyColor);
		if (rb[i]->getMass() != 0.0f)
		{
			if (!selected)
				MiniGL::drawCube(rb[i]->getPosition(), rb[i]->getRotationMatrix().transpose(), width, height, depth, dynamicBodyColor);
			else
				MiniGL::drawCube(rb[i]->getPosition(), rb[i]->getRotationMatrix().transpose(), width, height, depth, selectionColor);
		}

	}

	for (size_t i = 0; i < joints.size(); i++)
	{
		if (joints[i]->getTypeId() == RigidBodyParticleModel::BallJoint::TYPE_ID)
		{
			renderBallJoint(*(RigidBodyParticleModel::BallJoint*) joints[i]);
		}
		else if (joints[i]->getTypeId() == RigidBodyParticleModel::RigidBodyParticleBallJoint::TYPE_ID)
		{
			renderRigidBodyParticleBallJoint(*(RigidBodyParticleModel::RigidBodyParticleBallJoint*) joints[i]);
		}
	}
}

void renderClothModel()
{
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

	MiniGL::drawTime(TimeManager::getCurrent()->getTime());
}


void render ()
{
	MiniGL::coordinateSystem();
	
	renderRigidBodyModel();
	renderClothModel();

	MiniGL::drawTime( TimeManager::getCurrent ()->getTime ());
}

// Compute diagonal inertia tensor
Eigen::Vector3f computeInertiaTensorBox(const float mass, const float width, const float height, const float depth)
{
	const float Ix = (mass / 12.0f) * (height*height + depth*depth);
	const float Iy = (mass / 12.0f) * (width*width + depth*depth);
	const float Iz = (mass / 12.0f) * (width*width + height*height);
	return Eigen::Vector3f(Ix, Iy, Iz);
}

/** Create the model
*/
void createRigidBodyModel()
{
	RigidBodyParticleModel::RigidBodyVector &rb = model.getRigidBodies();
	RigidBodyParticleModel::JointVector &joints = model.getJoints();

	rb.resize(12);

	//////////////////////////////////////////////////////////////////////////
	// -5, -5
	//////////////////////////////////////////////////////////////////////////
	rb[0] = new RigidBody();
	rb[0]->initBody(0.0f,
		Eigen::Vector3f(-5.0, 0.0f, -5.0),
		computeInertiaTensorBox(1.0f, 0.5f, 0.5f, 0.5f),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	// dynamic body
	rb[1] = new RigidBody();
	rb[1]->initBody(1.0f,
		Eigen::Vector3f(-5.0f, 1.0f, -5.0f),
		computeInertiaTensorBox(1.0f, width, height, depth),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	// dynamic body
	rb[2] = new RigidBody();
	rb[2]->initBody(1.0f,
		Eigen::Vector3f(-5.0f, 3.0f, -5.0f),
		computeInertiaTensorBox(1.0f, width, height, depth),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	model.addBallJoint(0, 1, Eigen::Vector3f(-5.0f, 0.0f, -5.0f));
	model.addBallJoint(1, 2, Eigen::Vector3f(-5.0f, 2.0f, -5.0f));

	//////////////////////////////////////////////////////////////////////////
	// 5, -5
	//////////////////////////////////////////////////////////////////////////
	rb[3] = new RigidBody();
	rb[3]->initBody(0.0f,
		Eigen::Vector3f(5.0, 0.0f, -5.0),
		computeInertiaTensorBox(1.0f, 0.5f, 0.5f, 0.5f),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	// dynamic body
	rb[4] = new RigidBody();
	rb[4]->initBody(1.0f,
		Eigen::Vector3f(5.0f, 1.0f, -5.0f),
		computeInertiaTensorBox(1.0f, width, height, depth),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	// dynamic body
	rb[5] = new RigidBody();
	rb[5]->initBody(1.0f,
		Eigen::Vector3f(5.0f, 3.0f, -5.0f),
		computeInertiaTensorBox(1.0f, width, height, depth),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	model.addBallJoint(3, 4, Eigen::Vector3f(5.0f, 0.0f, -5.0f));
	model.addBallJoint(4, 5, Eigen::Vector3f(5.0f, 2.0f, -5.0f));

	//////////////////////////////////////////////////////////////////////////
	// 5, 5
	//////////////////////////////////////////////////////////////////////////
	rb[6] = new RigidBody();
	rb[6]->initBody(0.0f,
		Eigen::Vector3f(5.0, 0.0f, 5.0),
		computeInertiaTensorBox(1.0f, 0.5f, 0.5f, 0.5f),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	// dynamic body
	rb[7] = new RigidBody();
	rb[7]->initBody(1.0f,
		Eigen::Vector3f(5.0f, 1.0f, 5.0f),
		computeInertiaTensorBox(1.0f, width, height, depth),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	// dynamic body
	rb[8] = new RigidBody();
	rb[8]->initBody(1.0f,
		Eigen::Vector3f(5.0f, 3.0f, 5.0f),
		computeInertiaTensorBox(1.0f, width, height, depth),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	model.addBallJoint(6, 7, Eigen::Vector3f(5.0f, 0.0f, 5.0f));
	model.addBallJoint(7, 8, Eigen::Vector3f(5.0f, 2.0f, 5.0f));

	//////////////////////////////////////////////////////////////////////////
	// -5, 5
	//////////////////////////////////////////////////////////////////////////
	rb[9] = new RigidBody();
	rb[9]->initBody(0.0f,
		Eigen::Vector3f(-5.0, 0.0f, 5.0),
		computeInertiaTensorBox(1.0f, 0.5f, 0.5f, 0.5f),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	// dynamic body
	rb[10] = new RigidBody();
	rb[10]->initBody(1.0f,
		Eigen::Vector3f(-5.0f, 1.0f, 5.0f),
		computeInertiaTensorBox(1.0f, width, height, depth),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	// dynamic body
	rb[11] = new RigidBody();
	rb[11]->initBody(1.0f,
		Eigen::Vector3f(-5.0f, 3.0f, 5.0f),
		computeInertiaTensorBox(1.0f, width, height, depth),
		Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f));

	model.addBallJoint(9, 10, Eigen::Vector3f(-5.0f, 0.0f, 5.0f));
	model.addBallJoint(10, 11, Eigen::Vector3f(-5.0f, 2.0f, 5.0f));
	

	model.addRigidBodyParticleBallJoint(2, 0);
	model.addRigidBodyParticleBallJoint(5, (nRows - 1)*nCols);
	model.addRigidBodyParticleBallJoint(8, nRows*nCols - 1);
	model.addRigidBodyParticleBallJoint(11, nCols-1);

}


/** Create a particle model mesh
*/
void createClothMesh()
{
	RigidBodyParticleModel::ParticleMesh::UVs uvs;
	uvs.resize(nRows*nCols);

	const float dy = clothWidth / (float)(nCols - 1);
	const float dx = clothHeight / (float)(nRows - 1);

	Eigen::Vector3f points[nRows*nCols];
	for (int i = 0; i < nRows; i++)
	{
		for (int j = 0; j < nCols; j++)
		{
			const float y = (float)dy*j;
			const float x = (float)dx*i;
			points[i*nCols + j] = Eigen::Vector3f(x - 5.0f, 4.0f, y - 5.0f);

			uvs[i*nCols + j][0] = x / clothWidth;
			uvs[i*nCols + j][1] = y / clothHeight;
		}
	}
	const int nIndices = 6 * (nRows - 1)*(nCols - 1);

	RigidBodyParticleModel::ParticleMesh::UVIndices uvIndices;
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

	RigidBodyParticleModel::ParticleMesh &mesh = model.getParticleMesh();
	ParticleData &pd = mesh.getVertexData();
	for (unsigned int i = 0; i < pd.getNumberOfParticles(); i++)
	{
		pd.setMass(i, 1.0);
	}

	model.initConstraints();

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
	((TimeStepRigidBodyParticleModel*)clientData)->setVelocityUpdateMethod((unsigned int)val);
}

void TW_CALL getVelocityUpdateMethod(void *value, void *clientData)
{
	*(short *)(value) = (short)((TimeStepRigidBodyParticleModel*)clientData)->getVelocityUpdateMethod();
}

void TW_CALL setStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((RigidBodyParticleModel*)clientData)->setStiffness(val);
}

void TW_CALL getStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((RigidBodyParticleModel*)clientData)->getStiffness();
}

void TW_CALL setXXStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((RigidBodyParticleModel*)clientData)->setXXStiffness(val);
}

void TW_CALL getXXStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((RigidBodyParticleModel*)clientData)->getXXStiffness();
}

void TW_CALL setYYStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((RigidBodyParticleModel*)clientData)->setYYStiffness(val);
}

void TW_CALL getYYStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((RigidBodyParticleModel*)clientData)->getYYStiffness();
}

void TW_CALL setXYStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((RigidBodyParticleModel*)clientData)->setXYStiffness(val);
}

void TW_CALL getXYStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((RigidBodyParticleModel*)clientData)->getXYStiffness();
}

void TW_CALL setYXPoissonRatio(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((RigidBodyParticleModel*)clientData)->setYXPoissonRatio(val);
}

void TW_CALL getYXPoissonRatio(void *value, void *clientData)
{
	*(float *)(value) = ((RigidBodyParticleModel*)clientData)->getYXPoissonRatio();
}

void TW_CALL setXYPoissonRatio(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((RigidBodyParticleModel*)clientData)->setXYPoissonRatio(val);
}

void TW_CALL getXYPoissonRatio(void *value, void *clientData)
{
	*(float *)(value) = ((RigidBodyParticleModel*)clientData)->getXYPoissonRatio();
}

void TW_CALL setNormalizeStretch(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((RigidBodyParticleModel*)clientData)->setNormalizeStretch(val);
}

void TW_CALL getNormalizeStretch(void *value, void *clientData)
{
	*(bool *)(value) = ((RigidBodyParticleModel*)clientData)->getNormalizeStretch();
}

void TW_CALL setNormalizeShear(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((RigidBodyParticleModel*)clientData)->setNormalizeShear(val);
}

void TW_CALL getNormalizeShear(void *value, void *clientData)
{
	*(bool *)(value) = ((RigidBodyParticleModel*)clientData)->getNormalizeShear();
}

void TW_CALL setBendingStiffness(const void *value, void *clientData)
{
	const float val = *(const float *)(value);
	((RigidBodyParticleModel*)clientData)->setBendingStiffness(val);
}

void TW_CALL getBendingStiffness(void *value, void *clientData)
{
	*(float *)(value) = ((RigidBodyParticleModel*)clientData)->getBendingStiffness();
}

void TW_CALL setBendingMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	((TimeStepRigidBodyParticleModel*)clientData)->setBendingMethod((unsigned int)val);
}

void TW_CALL getBendingMethod(void *value, void *clientData)
{
	*(short *)(value) = (short)((TimeStepRigidBodyParticleModel*)clientData)->getBendingMethod();
}

void TW_CALL setSimulationMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	((TimeStepRigidBodyParticleModel*)clientData)->setSimulationMethod((unsigned int)val);
}

void TW_CALL getSimulationMethod(void *value, void *clientData)
{
	*(short *)(value) = (short)((TimeStepRigidBodyParticleModel*)clientData)->getSimulationMethod();
}
