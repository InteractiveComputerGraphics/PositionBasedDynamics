#include "DemoBase.h"
#include "Demos/Visualization/MiniGL.h"
#include "Utils/SceneLoader.h"
#include "Utils/FileSystem.h"
#include "Simulation/TimeManager.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/Simulation.h"
#include "NumericParameter.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/Version.h"
#include "Utils/SystemInfo.h"
#include "../Visualization/Visualization.h"
#include "Simulation/DistanceFieldCollisionDetection.h"
#include "Utils/OBJLoader.h"
#include "Utils/PLYLoader.h"


INIT_LOGGING
INIT_TIMING

using namespace PBD;
using namespace std;
using namespace GenParam;
using namespace Utilities;

int DemoBase::PAUSE = -1;
int DemoBase::PAUSE_AT = -1;
int DemoBase::NUM_STEPS_PER_RENDER = -1;
int DemoBase::RENDER_TETS = -1;
int DemoBase::RENDER_TETS0 = -1;
int DemoBase::RENDER_CONTACTS = -1;
int DemoBase::RENDER_AABB = -1;
int DemoBase::RENDER_SDF = -1;
int DemoBase::RENDER_BVH = -1;
int DemoBase::RENDER_BVH_TETS = -1;
int DemoBase::EXPORT_OBJ = -1;
int DemoBase::EXPORT_PLY = -1;
int DemoBase::EXPORT_FPS = -1;

 
DemoBase::DemoBase()
{
	Utilities::logger.addSink(unique_ptr<Utilities::ConsoleSink>(new Utilities::ConsoleSink(Utilities::LogLevel::INFO)));
	Utilities::logger.addSink(shared_ptr<Utilities::BufferSink>(new Utilities::BufferSink(Utilities::LogLevel::DEBUG)));

	m_sceneLoader = nullptr;
	m_numberOfStepsPerRenderUpdate = 8;
	m_renderContacts = false;
	m_renderAABB = false;
	m_renderSDF = false;
	m_renderBVHDepth = -1;
	m_renderBVHDepthTets = -1;
	m_renderRefTets = false;
	m_renderTets = false;
	m_sceneFile = "";
	m_sceneName = "";
	m_doPause = true;
	m_pauseAt = -1.0;
	m_useCache = true;
	m_oldMousePos.setZero();
	m_enableExportOBJ = false;
	m_enableExportPLY = false;
	m_exportFPS = 25;
	m_nextFrameTime = 0.0;
	m_frameCounter = 1;

	m_gui = new Simulator_GUI_imgui(this);
}

DemoBase::~DemoBase()
{
	delete m_sceneLoader;
	delete m_gui;
}

void DemoBase::initParameters()
{
	ParameterObject::initParameters();

	PAUSE = createBoolParameter("pause", "Pause", &m_doPause);
	setGroup(PAUSE, "Simulation|General");
	setDescription(PAUSE, "Pause simulation.");
	setHotKey(PAUSE, "space");

	PAUSE_AT = createNumericParameter("pauseAt", "Pause simulation at", &m_pauseAt);
	setGroup(PAUSE_AT, "Simulation|General");
	setDescription(PAUSE_AT, "Pause simulation at the given time. When the value is negative, the simulation is not paused.");

	NUM_STEPS_PER_RENDER = createNumericParameter("numberOfStepsPerRenderUpdate", "# time steps / update", &m_numberOfStepsPerRenderUpdate);
	setGroup(NUM_STEPS_PER_RENDER, "Visualization|General");
	setDescription(NUM_STEPS_PER_RENDER, "Pause simulation at the given time. When the value is negative, the simulation is not paused.");
	static_cast<NumericParameter<unsigned int>*>(getParameter(NUM_STEPS_PER_RENDER))->setMinValue(1);

	RENDER_TETS = createBoolParameter("renderTets", "Render tet model", &m_renderTets);
	setGroup(RENDER_TETS, "Visualization|Solids");
	setDescription(RENDER_TETS, "Render tet model.");

	RENDER_TETS0 = createBoolParameter("renderTets0", "Render ref. tet model", &m_renderRefTets);
	setGroup(RENDER_TETS0, "Visualization|Solids");
	setDescription(RENDER_TETS0, "Render ref. tet model.");

	RENDER_CONTACTS = createBoolParameter("renderContacts", "Render contacts", &m_renderContacts);
	setGroup(RENDER_CONTACTS, "Visualization|Contact");
	setDescription(RENDER_CONTACTS, "Render contact points and normals.");

	RENDER_AABB = createBoolParameter("renderAABB", "Render AABBs", &m_renderAABB);
	setGroup(RENDER_AABB, "Visualization|Contact");
	setDescription(RENDER_AABB, "Render AABBs.");

	RENDER_SDF = createBoolParameter("renderSDF", "Render SDFs", &m_renderSDF);
	setGroup(RENDER_SDF, "Visualization|Contact");
	setDescription(RENDER_SDF, "Render SDFs.");

	RENDER_BVH = createNumericParameter("renderBVHDepth", "Render BVH depth", &m_renderBVHDepth);
	setGroup(RENDER_BVH, "Visualization|Contact");
	setDescription(RENDER_BVH, "Render BVH until given depth.");
	static_cast<NumericParameter<int>*>(getParameter(RENDER_BVH))->setMinValue(-1);

	RENDER_BVH_TETS = createNumericParameter("renderBVHDepthTets", "Render BVH depth (tets)", &m_renderBVHDepthTets);
	setGroup(RENDER_BVH_TETS, "Visualization|Contact");
	setDescription(RENDER_BVH_TETS, "Render BVH (tets) until given depth.");
	static_cast<NumericParameter<int>*>(getParameter(RENDER_BVH_TETS))->setMinValue(-1);

	EXPORT_OBJ = createBoolParameter("exportOBJ", "Export OBJ", &m_enableExportOBJ);
	setGroup(EXPORT_OBJ, "Simulation|Export");
	setDescription(EXPORT_OBJ, "Export meshes in OBJ files.");

	EXPORT_PLY = createBoolParameter("exportPLY", "Export PLY", &m_enableExportPLY);
	setGroup(EXPORT_PLY, "Simulation|Export");
	setDescription(EXPORT_PLY, "Export meshes in PLY files.");

	EXPORT_FPS = createNumericParameter("exportFPS", "Export FPS", &m_exportFPS);
	setGroup(EXPORT_FPS, "Simulation|Export");
	setDescription(EXPORT_FPS, "Frame rate for export.");
	static_cast<NumericParameter<int>*>(getParameter(EXPORT_FPS))->setMinValue(0);
}

void DemoBase::createParameterGUI()
{
	m_gui->initSimulationParameterGUI();
}

void DemoBase::cleanup()
{	
	m_scene.m_rigidBodyData.clear();
	m_scene.m_rigidBodyData.clear();
	m_scene.m_triangleModelData.clear();
	m_scene.m_tetModelData.clear();
	m_scene.m_ballJointData.clear();
	m_scene.m_ballOnLineJointData.clear();
	m_scene.m_hingeJointData.clear();
	m_scene.m_universalJointData.clear();
	m_scene.m_sliderJointData.clear();
	m_scene.m_rigidBodyParticleBallJointData.clear();
	m_scene.m_targetAngleMotorHingeJointData.clear();
	m_scene.m_targetVelocityMotorHingeJointData.clear();
	m_scene.m_targetPositionMotorSliderJointData.clear();
	m_scene.m_targetVelocityMotorSliderJointData.clear();
	m_scene.m_rigidBodySpringData.clear();
	m_scene.m_distanceJointData.clear();
	m_scene.m_damperJointData.clear();
}

void DemoBase::init(int argc, char **argv, const char *demoName)
{
	initParameters();
	m_exePath = FileSystem::getProgramPath();

	m_sceneFile = "";
	setUseCache(true);
	for (int i = 1; i < argc; i++)
	{
		string argStr = argv[i];
		if (argStr == "--no-cache")
			setUseCache(false);
		else
		{
			m_sceneFile = string(argv[i]);
			if (FileSystem::isRelativePath(m_sceneFile))
				m_sceneFile = FileSystem::normalizePath(m_exePath + "/" + m_sceneFile);
		}
	}

	if (m_sceneFile != "")
		m_outputPath = FileSystem::normalizePath(getExePath() + "/output/" + FileSystem::getFileName(m_sceneFile));
	else
		m_outputPath = FileSystem::normalizePath(getExePath() + "/output/" + std::string(demoName));

#ifdef DL_OUTPUT
	std::string sceneFilePath = FileSystem::normalizePath(m_outputPath + "/scene");
	FileSystem::makeDirs(sceneFilePath);
	FileSystem::copyFile(m_sceneFile, sceneFilePath + "/" + FileSystem::getFileNameWithExt(m_sceneFile));

	std::string progFilePath = FileSystem::normalizePath(m_outputPath + "/program");
	FileSystem::makeDirs(progFilePath);
	FileSystem::copyFile(argv[0], progFilePath + "/" + FileSystem::getFileNameWithExt(argv[0]));
#endif

	std::string logPath = FileSystem::normalizePath(m_outputPath + "/log");
	FileSystem::makeDirs(logPath);
	Utilities::logger.addSink(unique_ptr<Utilities::FileSink>(new Utilities::FileSink(Utilities::LogLevel::DEBUG, logPath + "/PBD_log.txt")));

	LOG_DEBUG << "Git refspec: " << GIT_REFSPEC;
	LOG_DEBUG << "Git SHA1:    " << GIT_SHA1;
	LOG_DEBUG << "Git status:  " << GIT_LOCAL_STATUS;
	LOG_DEBUG << "Host name:   " << SystemInfo::getHostName();
	LOG_INFO << "PositionBasedDynamics " << PBD_VERSION;

	m_gui->init();

	// OpenGL
	MiniGL::init(argc, argv, 1280, 1024, demoName, m_gui->getVSync(), m_gui->getMaximized());
	MiniGL::initLights();
	MiniGL::initTexture();
	MiniGL::getOpenGLVersion(m_context_major_version, m_context_minor_version);
	MiniGL::setViewport(40.0, 0.1f, 500.0, Vector3r(0.0, 3.0, 8.0), Vector3r(0.0, 0.0, 0.0));
	MiniGL::setSelectionFunc(selection, this);

	if (MiniGL::checkOpenGLVersion(3, 3))
		initShaders();

	m_gui->initImgui();
}

void DemoBase::readScene()
{
	if (m_sceneLoader == nullptr)
		m_sceneLoader = new SceneLoader();
	if (m_sceneFile != "")
		m_sceneLoader->readScene(m_sceneFile.c_str(), m_scene);
	else
		return;

	m_sceneName = m_scene.m_sceneName;
}

void DemoBase::initShaders()
{
	std::string vertFile = m_exePath + "/resources/shaders/vs_smooth.glsl";
	std::string fragFile = m_exePath + "/resources/shaders/fs_smooth.glsl";
	m_shader.compileShaderFile(GL_VERTEX_SHADER, vertFile);
	m_shader.compileShaderFile(GL_FRAGMENT_SHADER, fragFile);
	m_shader.createAndLinkProgram();
	m_shader.begin();
	m_shader.addUniform("modelview_matrix");
	m_shader.addUniform("projection_matrix");
	m_shader.addUniform("surface_color");
	m_shader.addUniform("shininess");
	m_shader.addUniform("specular_factor");
	m_shader.end();

	vertFile = m_exePath + "/resources/shaders/vs_smoothTex.glsl";
	fragFile = m_exePath + "/resources/shaders/fs_smoothTex.glsl";
	m_shaderTex.compileShaderFile(GL_VERTEX_SHADER, vertFile);
	m_shaderTex.compileShaderFile(GL_FRAGMENT_SHADER, fragFile);
	m_shaderTex.createAndLinkProgram();
	m_shaderTex.begin();
	m_shaderTex.addUniform("modelview_matrix");
	m_shaderTex.addUniform("projection_matrix");
	m_shaderTex.addUniform("surface_color");
	m_shaderTex.addUniform("shininess");
	m_shaderTex.addUniform("specular_factor");
	m_shaderTex.end();

	vertFile = m_exePath + "/resources/shaders/vs_flat.glsl";
	std::string geomFile = m_exePath + "/resources/shaders/gs_flat.glsl";
	fragFile = m_exePath + "/resources/shaders/fs_flat.glsl";
	m_shaderFlat.compileShaderFile(GL_VERTEX_SHADER, vertFile);
	m_shaderFlat.compileShaderFile(GL_GEOMETRY_SHADER, geomFile);
	m_shaderFlat.compileShaderFile(GL_FRAGMENT_SHADER, fragFile);
	m_shaderFlat.createAndLinkProgram();
	m_shaderFlat.begin();
	m_shaderFlat.addUniform("modelview_matrix");
	m_shaderFlat.addUniform("projection_matrix");
	m_shaderFlat.addUniform("surface_color");
	m_shaderFlat.addUniform("shininess");
	m_shaderFlat.addUniform("specular_factor");
	m_shaderFlat.end();
}


void DemoBase::shaderTexBegin(const float *col)
{
	m_shaderTex.begin();
	glUniform1f(m_shaderTex.getUniform("shininess"), 5.0f);
	glUniform1f(m_shaderTex.getUniform("specular_factor"), 0.2f);

	GLfloat matrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
	glUniformMatrix4fv(m_shaderTex.getUniform("modelview_matrix"), 1, GL_FALSE, matrix);
	GLfloat pmatrix[16];
	glGetFloatv(GL_PROJECTION_MATRIX, pmatrix);
	glUniformMatrix4fv(m_shaderTex.getUniform("projection_matrix"), 1, GL_FALSE, pmatrix);
	glUniform3fv(m_shaderTex.getUniform("surface_color"), 1, col);
}

void DemoBase::shaderTexEnd()
{
	m_shaderTex.end();
}

void DemoBase::shaderBegin(const float *col)
{
	m_shader.begin();
	glUniform1f(m_shader.getUniform("shininess"), 5.0f);
	glUniform1f(m_shader.getUniform("specular_factor"), 0.2f);

	GLfloat matrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
	glUniformMatrix4fv(m_shader.getUniform("modelview_matrix"), 1, GL_FALSE, matrix);
	GLfloat pmatrix[16];
	glGetFloatv(GL_PROJECTION_MATRIX, pmatrix);
	glUniformMatrix4fv(m_shader.getUniform("projection_matrix"), 1, GL_FALSE, pmatrix);
	glUniform3fv(m_shader.getUniform("surface_color"), 1, col);
}

void DemoBase::shaderEnd()
{
	m_shader.end();
}

void DemoBase::shaderFlatBegin(const float* col)
{
	m_shaderFlat.begin();
	glUniform1f(m_shaderFlat.getUniform("shininess"), 5.0f);
	glUniform1f(m_shaderFlat.getUniform("specular_factor"), 0.2f);

	GLfloat matrix[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
	glUniformMatrix4fv(m_shaderFlat.getUniform("modelview_matrix"), 1, GL_FALSE, matrix);
	GLfloat pmatrix[16];
	glGetFloatv(GL_PROJECTION_MATRIX, pmatrix);
	glUniformMatrix4fv(m_shaderFlat.getUniform("projection_matrix"), 1, GL_FALSE, pmatrix);
	glUniform3fv(m_shaderFlat.getUniform("surface_color"), 1, col);
}

void DemoBase::shaderFlatEnd()
{
	m_shaderFlat.end();
}

void DemoBase::readParameters()
{
	Simulation* sim = Simulation::getCurrent();
	m_sceneLoader->readParameterObject(this);
	m_sceneLoader->readParameterObject(sim);
	m_sceneLoader->readParameterObject(sim->getModel());
	m_sceneLoader->readParameterObject(sim->getTimeStep());
	if (sim->getTimeStep()->getCollisionDetection() != nullptr)
		m_sceneLoader->readParameterObject(sim->getTimeStep()->getCollisionDetection());
}

void DemoBase::render()
{
	float gridColor[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
	MiniGL::drawGrid_xz(gridColor);

	MiniGL::coordinateSystem();

	// Draw sim model	
	SimulationModel *model = Simulation::getCurrent()->getModel();
	if (model == nullptr)
	{
		m_gui->update();
		return;
	}
	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
	SimulationModel::ConstraintVector &constraints = model->getConstraints();
	SimulationModel::RigidBodyContactConstraintVector &rigidBodyContacts = model->getRigidBodyContactConstraints();
	SimulationModel::ParticleRigidBodyContactConstraintVector &particleRigidBodyContacts = model->getParticleRigidBodyContactConstraints();

	float selectionColor[4] = { 0.8f, 0.0f, 0.0f, 1 };
	float surfaceColor[4] = { 0.1f, 0.4f, 0.7f, 1 };
	float staticColor[4] = { 0.5f, 0.5f, 0.5f, 1 };

	if (m_renderContacts)
	{
		for (unsigned int i = 0; i < rigidBodyContacts.size(); i++)
			renderRigidBodyContact(rigidBodyContacts[i]);
		for (unsigned int i = 0; i < particleRigidBodyContacts.size(); i++)
			renderParticleRigidBodyContact(particleRigidBodyContacts[i]);
	}


	for (size_t i = 0; i < rb.size(); i++)
	{
		bool selected = false;
		for (unsigned int j = 0; j < m_selectedBodies.size(); j++)
		{
			if (m_selectedBodies[j] == i)
				selected = true;
		}
		std::cout << "AH" << std::endl;
		const VertexData &vd = rb[i]->getGeometry().getVertexData();
		const IndexedFaceMesh &mesh = rb[i]->getGeometry().getMesh();

		if (mesh.getFlatShading())
			shaderFlatBegin(staticColor);
		else
			shaderBegin(staticColor);

		if (!selected)
		{
			if (rb[i]->getMass() == 0.0)
			{
				glUniform3fv(m_shader.getUniform("surface_color"), 1, staticColor);
				Visualization::drawMesh(vd, mesh, 0, staticColor);
			}
			else
			{
				glUniform3fv(m_shader.getUniform("surface_color"), 1, surfaceColor);
				Visualization::drawMesh(vd, mesh, 0, surfaceColor);
			}
		}
		else
		{
			glUniform3fv(m_shader.getUniform("surface_color"), 1, selectionColor);
			Visualization::drawMesh(vd, mesh, 0, selectionColor);
		}

		if (mesh.getFlatShading())
			shaderFlatEnd();
		else
			shaderEnd();
	}

	

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
		else if (constraints[i]->getTypeId() == RigidBodySpring::TYPE_ID)
		{
			renderSpring(*(RigidBodySpring*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == DistanceJoint::TYPE_ID)
		{
			renderDistanceJoint(*(DistanceJoint*)constraints[i]);
		}
		else if (constraints[i]->getTypeId() == DamperJoint::TYPE_ID)
		{
			renderDamperJoint(*(DamperJoint*)constraints[i]);
		}
	}


	DistanceFieldCollisionDetection *cd = (DistanceFieldCollisionDetection*)Simulation::getCurrent()->getTimeStep()->getCollisionDetection();
	if (cd && (m_renderSDF || m_renderAABB || (m_renderBVHDepth >= 0) || (m_renderBVHDepthTets >= 0)))
	{
		std::vector<CollisionDetection::CollisionObject*> &collisionObjects = cd->getCollisionObjects();
		for (unsigned int k = 0; k < collisionObjects.size(); k++)
		{
			if (m_renderAABB)
				renderAABB(collisionObjects[k]->m_aabb);

			if (m_renderSDF)
				renderSDF(collisionObjects[k]);

			if (m_renderBVHDepth >= 0)
			{
				if (cd->isDistanceFieldCollisionObject(collisionObjects[k]))
				{
					const PointCloudBSH &bvh = ((DistanceFieldCollisionDetection::DistanceFieldCollisionObject*) collisionObjects[k])->m_bvh;

					std::function<bool(unsigned int, unsigned int)> predicate = [&](unsigned int node_index, unsigned int depth) { return (int)depth <= m_renderBVHDepth; };
					std::function<void(unsigned int, unsigned int)> cb = [&](unsigned int node_index, unsigned int depth)
					{
						if (depth == m_renderBVHDepth)
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

			if (m_renderBVHDepthTets >= 0)
			{
				if (cd->isDistanceFieldCollisionObject(collisionObjects[k]) && (collisionObjects[k]->m_bodyType == CollisionDetection::CollisionObject::TetModelCollisionObjectType))
				{

					TetMeshBSH &bvh = ((DistanceFieldCollisionDetection::DistanceFieldCollisionObject*) collisionObjects[k])->m_bvhTets;

					std::function<bool(unsigned int, unsigned int)> predicate = [&](unsigned int node_index, unsigned int depth) { return (int)depth <= m_renderBVHDepthTets; };
					std::function<void(unsigned int, unsigned int)> cb = [&](unsigned int node_index, unsigned int depth)
					{
						if (depth == m_renderBVHDepthTets)
						{
							const BoundingSphere &bs = bvh.hull(node_index);
							const Vector3r &sphere_x = bs.x();
							MiniGL::drawSphere(sphere_x, std::max((float)bs.r(), 0.05f), staticColor);
						}
					};

					bvh.traverse_depth_first(predicate, cb);
				}
			}
		}
	}

	const Vector3r refOffset(0, 0, 0);
	const ParticleData &pd = model->getParticles();
	if (m_renderRefTets || m_renderTets)
	{
		shaderBegin(surfaceColor);

		for (unsigned int i = 0; i < model->getTetModels().size(); i++)
		{
			const IndexedTetMesh &mesh = model->getTetModels()[i]->getParticleMesh();
			const unsigned int nTets = mesh.numTets();
			const unsigned int *indices = mesh.getTets().data();
			const unsigned int offset = model->getTetModels()[i]->getIndexOffset();

			const Vector3r &ix = model->getTetModels()[i]->getInitialX();
			const Matrix3r &R = model->getTetModels()[i]->getInitialR();

			for (unsigned int j = 0; j < nTets; j++)
			{
				if (m_renderTets)
				{
					const Vector3r &x0 = pd.getPosition(indices[4 * j] + offset);
					const Vector3r &x1 = pd.getPosition(indices[4 * j + 1] + offset);
					const Vector3r &x2 = pd.getPosition(indices[4 * j + 2] + offset);
					const Vector3r &x3 = pd.getPosition(indices[4 * j + 3] + offset);
					MiniGL::drawTetrahedron(x0, x1, x2, x3, surfaceColor);
				}
				if (m_renderRefTets)
				{
// 					const Vector3r &x0 = R.transpose() * (pd.getPosition0(indices[4 * j + 0] + offset) - ix);
// 					const Vector3r &x1 = R.transpose() * (pd.getPosition0(indices[4 * j + 1] + offset) - ix);
// 					const Vector3r &x2 = R.transpose() * (pd.getPosition0(indices[4 * j + 2] + offset) - ix);
// 					const Vector3r &x3 = R.transpose() * (pd.getPosition0(indices[4 * j + 3] + offset) - ix);
					const Vector3r &x0 = pd.getPosition0(indices[4 * j] + offset) + refOffset;
					const Vector3r &x1 = pd.getPosition0(indices[4 * j + 1] + offset) + refOffset;
					const Vector3r &x2 = pd.getPosition0(indices[4 * j + 2] + offset) + refOffset;
					const Vector3r &x3 = pd.getPosition0(indices[4 * j + 3] + offset) + refOffset;

					MiniGL::drawTetrahedron(x0, x1, x2, x3, staticColor);
				}
			}
		}
		shaderEnd();
	}

	float red[4] = { 0.8f, 0.0f, 0.0f, 1 };
	for (unsigned int j = 0; j < m_selectedParticles.size(); j++)
	{
		MiniGL::drawSphere(pd.getPosition(m_selectedParticles[j]), 0.08f, red);
	}

	m_gui->update();
}

void DemoBase::renderTriangleModels()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const ParticleData &pd = model->getParticles();
	float surfaceColor[4] = { 0.8f, 0.9f, 0.2f, 1 };

	shaderTexBegin(surfaceColor);

	for (unsigned int i = 0; i < model->getTriangleModels().size(); i++)
	{
		// mesh 
		const IndexedFaceMesh &mesh = model->getTriangleModels()[i]->getParticleMesh();
		const unsigned int offset = model->getTriangleModels()[i]->getIndexOffset();
		Visualization::drawTexturedMesh(pd, mesh, offset, surfaceColor);
	}

	shaderTexEnd();
}

void DemoBase::renderTetModels()
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const ParticleData &pd = model->getParticles();
	float surfaceColor[4] = { 0.1f, 0.4f, 0.7f, 1 };

	shaderBegin(surfaceColor);

	for (unsigned int i = 0; i < model->getTetModels().size(); i++)
	{
		const VertexData &vdVis = model->getTetModels()[i]->getVisVertices();
		if (vdVis.size() > 0)
		{
			const IndexedFaceMesh &visMesh = model->getTetModels()[i]->getVisMesh();
			Visualization::drawMesh(vdVis, visMesh, 0, surfaceColor);
		}
		else
		{
			const IndexedFaceMesh &surfaceMesh = model->getTetModels()[i]->getSurfaceMesh();
			const unsigned int offset = model->getTetModels()[i]->getIndexOffset();
			Visualization::drawMesh(pd, surfaceMesh, offset, surfaceColor);
		}
	}

	shaderEnd();
}

void DemoBase::renderAABB(AABB &aabb)
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

void DemoBase::renderSDF(CollisionDetection::CollisionObject* co)
{
	DistanceFieldCollisionDetection *cd = (DistanceFieldCollisionDetection*)Simulation::getCurrent()->getTimeStep()->getCollisionDetection();
	if ((!cd->isDistanceFieldCollisionObject(co)) || (co->m_bodyType != CollisionDetection::CollisionObject::RigidBodyCollisionObjectType))
		return;

	SimulationModel *model = Simulation::getCurrent()->getModel();
	const SimulationModel::RigidBodyVector &rigidBodies = model->getRigidBodies();
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
				const double dist = dfco->distance(pos.template cast<double>(), 0.0);

				if (dist < 0.0)
				{
					float col[4] = { (float)-dist, 0.0f, 0.0f, 1.0f };
					MiniGL::drawPoint(pos_w, 3.0f, col);
				}
			}
		}
	}
}

void DemoBase::renderBallJoint(BallJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(2), 0.15f, m_jointColor);
}

void DemoBase::renderRigidBodyParticleBallJoint(RigidBodyParticleBallJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(1), 0.1f, m_jointColor);
}

void DemoBase::renderBallOnLineJoint(BallOnLineJoint &bj)
{
	MiniGL::drawSphere(bj.m_jointInfo.col(5), 0.1f, m_jointColor);
	MiniGL::drawCylinder(bj.m_jointInfo.col(5) - bj.m_jointInfo.col(7), bj.m_jointInfo.col(5) + bj.m_jointInfo.col(7), m_jointColor, 0.05f);
}

void DemoBase::renderHingeJoint(HingeJoint &joint)
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const SimulationModel::RigidBodyVector &rigidBodies = model->getRigidBodies();
	RigidBody *rb = rigidBodies[joint.m_bodies[0]];

	const Vector3r &c = joint.m_jointInfo.block<3, 1>(0, 4);
	const Vector3r &axis_local = joint.m_jointInfo.block<3, 1>(0, 6);
	const Vector3r axis = rb->getRotation().matrix() * axis_local;

	MiniGL::drawSphere(c - 0.5*axis, 0.1f, m_jointColor);
	MiniGL::drawSphere(c + 0.5*axis, 0.1f, m_jointColor);
	MiniGL::drawCylinder(c - 0.5*axis, c + 0.5*axis, m_jointColor, 0.05f);
}

void DemoBase::renderUniversalJoint(UniversalJoint &uj)
{
	MiniGL::drawSphere(uj.m_jointInfo.col(4) - 0.5*uj.m_jointInfo.col(6), 0.1f, m_jointColor);
	MiniGL::drawSphere(uj.m_jointInfo.col(4) + 0.5*uj.m_jointInfo.col(6), 0.1f, m_jointColor);
	MiniGL::drawSphere(uj.m_jointInfo.col(5) - 0.5*uj.m_jointInfo.col(7), 0.1f, m_jointColor);
	MiniGL::drawSphere(uj.m_jointInfo.col(5) + 0.5*uj.m_jointInfo.col(7), 0.1f, m_jointColor);
	MiniGL::drawCylinder(uj.m_jointInfo.col(4) - 0.5*uj.m_jointInfo.col(6), uj.m_jointInfo.col(4) + 0.5*uj.m_jointInfo.col(6), m_jointColor, 0.05f);
	MiniGL::drawCylinder(uj.m_jointInfo.col(5) - 0.5*uj.m_jointInfo.col(7), uj.m_jointInfo.col(5) + 0.5*uj.m_jointInfo.col(7), m_jointColor, 0.05f);
}

void DemoBase::renderSliderJoint(SliderJoint &joint)
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const SimulationModel::RigidBodyVector &rigidBodies = model->getRigidBodies();
	RigidBody *rb = rigidBodies[joint.m_bodies[0]];

	Quaternionr qR0;
	qR0.coeffs() = joint.m_jointInfo.col(1);
	const Vector3r &c = rb->getPosition();
	Vector3r axis = qR0.matrix().col(0);
	MiniGL::drawSphere(c, 0.1f, m_jointColor);
	MiniGL::drawCylinder(c - axis, c + axis, m_jointColor, 0.05f);
}

void DemoBase::renderTargetPositionMotorSliderJoint(TargetPositionMotorSliderJoint &joint)
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const SimulationModel::RigidBodyVector &rigidBodies = model->getRigidBodies();
	RigidBody *rb = rigidBodies[joint.m_bodies[0]];

	const Vector3r &c = rb->getPosition();
	Vector3r axis = joint.m_jointInfo.block<3, 1>(0, 1);
	MiniGL::drawSphere(c, 0.1f, m_jointColor);
	MiniGL::drawCylinder(c - axis, c + axis, m_jointColor, 0.05f);
}

void DemoBase::renderTargetVelocityMotorSliderJoint(TargetVelocityMotorSliderJoint &joint)
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const SimulationModel::RigidBodyVector &rigidBodies = model->getRigidBodies();
	RigidBody *rb = rigidBodies[joint.m_bodies[0]];

	Quaternionr qR0;
	qR0.coeffs() = joint.m_jointInfo.col(1);
	const Vector3r &c = rb->getPosition();
	Vector3r axis = qR0.matrix().col(0);
	MiniGL::drawSphere(c, 0.1f, m_jointColor);
	MiniGL::drawCylinder(c - axis, c + axis, m_jointColor, 0.05f);
}

void DemoBase::renderTargetAngleMotorHingeJoint(TargetAngleMotorHingeJoint &joint)
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const SimulationModel::RigidBodyVector &rigidBodies = model->getRigidBodies();
	RigidBody *rb = rigidBodies[joint.m_bodies[0]];

	const Vector3r &c = joint.m_jointInfo.block<3, 1>(0, 5);
	const Vector3r &axis_local = joint.m_jointInfo.block<3, 1>(0, 7);
	const Vector3r axis = rb->getRotation().matrix() * axis_local;

	MiniGL::drawSphere(c - 0.5*axis, 0.1f, m_jointColor);
	MiniGL::drawSphere(c + 0.5*axis, 0.1f, m_jointColor);
	MiniGL::drawCylinder(c - 0.5*axis, c + 0.5*axis, m_jointColor, 0.05f);
}

void DemoBase::renderTargetVelocityMotorHingeJoint(TargetVelocityMotorHingeJoint &joint)
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const SimulationModel::RigidBodyVector &rigidBodies = model->getRigidBodies();
	RigidBody *rb = rigidBodies[joint.m_bodies[0]];

	const Vector3r &c = joint.m_jointInfo.block<3, 1>(0, 5);
	const Vector3r axis = joint.m_jointInfo.block<3, 1>(0, 7);

	MiniGL::drawSphere(c - 0.5*axis, 0.1f, m_jointColor);
	MiniGL::drawSphere(c + 0.5*axis, 0.1f, m_jointColor);
	MiniGL::drawCylinder(c - 0.5*axis, c + 0.5*axis, m_jointColor, 0.05f);
}

void DemoBase::renderRigidBodyContact(RigidBodyContactConstraint &cc)
{
	float col1[4] = { 0.0f, 0.6f, 0.2f, 1 };
	float col2[4] = { 0.6f, 0.0f, 0.2f, 1 };
	MiniGL::drawPoint(cc.m_constraintInfo.col(0), 5.0f, col1);
	MiniGL::drawPoint(cc.m_constraintInfo.col(1), 5.0f, col2);
	MiniGL::drawVector(cc.m_constraintInfo.col(1), cc.m_constraintInfo.col(1) + cc.m_constraintInfo.col(2), 1.0f, col2);
}

void DemoBase::renderParticleRigidBodyContact(ParticleRigidBodyContactConstraint &cc)
{
	float col1[4] = { 0.0f, 0.6f, 0.2f, 1 };
	float col2[4] = { 0.6f, 0.0f, 0.2f, 1 };
	MiniGL::drawPoint(cc.m_constraintInfo.col(0), 5.0f, col1);
	MiniGL::drawPoint(cc.m_constraintInfo.col(1), 5.0f, col2);
	MiniGL::drawVector(cc.m_constraintInfo.col(1), cc.m_constraintInfo.col(1) + cc.m_constraintInfo.col(2), 1.0f, col2);
}

void DemoBase::renderSpring(RigidBodySpring &s)
{
	MiniGL::drawSphere(s.m_jointInfo.col(2), 0.1f, m_jointColor);
	MiniGL::drawSphere(s.m_jointInfo.col(3), 0.1f, m_jointColor);
	MiniGL::drawCylinder(s.m_jointInfo.col(2), s.m_jointInfo.col(3), m_jointColor, 0.05f);
}

void DemoBase::renderDistanceJoint(DistanceJoint &j)
{
	MiniGL::drawSphere(j.m_jointInfo.col(2), 0.1f, m_jointColor);
	MiniGL::drawSphere(j.m_jointInfo.col(3), 0.1f, m_jointColor);
	MiniGL::drawCylinder(j.m_jointInfo.col(2), j.m_jointInfo.col(3), m_jointColor, 0.05f);
}

void DemoBase::renderDamperJoint(DamperJoint &joint)
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const SimulationModel::RigidBodyVector &rigidBodies = model->getRigidBodies();
	RigidBody *rb = rigidBodies[joint.m_bodies[0]];

	Quaternionr qR0;
	qR0.coeffs() = joint.m_jointInfo.col(1);
	const Vector3r &c = rb->getPosition();
	Vector3r axis = qR0.matrix().col(0);
	MiniGL::drawSphere(c, 0.1f, m_jointColor);
	MiniGL::drawCylinder(c - axis, c + axis, m_jointColor, 0.05f);
}

void DemoBase::mouseMove(int x, int y, void *clientData)
{
	DemoBase *base = (DemoBase*)clientData;
	SimulationModel *model = Simulation::getCurrent()->getModel();

	Vector3r mousePos;
	MiniGL::unproject(x, y, mousePos);
	const Vector3r diff = mousePos - base->m_oldMousePos;

	TimeManager *tm = TimeManager::getCurrent();
	const Real h = tm->getTimeStepSize();

	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
	for (size_t j = 0; j < base->m_selectedBodies.size(); j++)
	{
		const Real mass = rb[base->m_selectedBodies[j]]->getMass();
		if (mass != 0.0)
			rb[base->m_selectedBodies[j]]->getVelocity() += 3.0 / h * diff;
	}
	ParticleData &pd = model->getParticles();
	for (unsigned int j = 0; j < base->m_selectedParticles.size(); j++)
	{
		const Real mass = pd.getMass(base->m_selectedParticles[j]);
		if (mass != 0.0)
			pd.getVelocity(base->m_selectedParticles[j]) += 5.0*diff / h;
	}
	base->m_oldMousePos = mousePos;
}

void DemoBase::selection(const Vector2i &start, const Vector2i &end, void *clientData)
{
	SimulationModel *model = Simulation::getCurrent()->getModel();
	DemoBase *base = (DemoBase*)clientData;

	std::vector<unsigned int> hits;

	base->m_selectedParticles.clear();
	ParticleData &pd = model->getParticles();
	if (pd.size() > 0)
		Selection::selectRect(start, end, &pd.getPosition(0), &pd.getPosition(pd.size() - 1), base->m_selectedParticles);

	base->m_selectedBodies.clear();
	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
	std::vector<Vector3r, Eigen::aligned_allocator<Vector3r> > x;
	x.resize(rb.size());
	for (unsigned int i = 0; i < rb.size(); i++)
	{
		x[i] = rb[i]->getPosition();
	}

	if (rb.size() > 0)
		Selection::selectRect(start, end, &x[0], &x[rb.size() - 1], base->m_selectedBodies);
	if ((base->m_selectedBodies.size() > 0) || (base->m_selectedParticles.size() > 0))
		MiniGL::setMouseMoveFunc(2, mouseMove);
	else
		MiniGL::setMouseMoveFunc(-1, NULL);

	MiniGL::unproject(end[0], end[1], base->m_oldMousePos);
}

void DemoBase::reset()
{
	m_nextFrameTime = 0.0;
	m_frameCounter = 1;
}

void DemoBase::loadMesh(const std::string& filename, VertexData& vd, Utilities::IndexedFaceMesh& mesh, const Vector3r& translation,
	const Matrix3r& rotation, const Vector3r& scale)
{
	string ext = FileSystem::getFileExt(filename);
	transform(ext.begin(), ext.end(), ext.begin(), ::toupper);

	std::vector<std::array<float, 3>> x;
	if (ext == "OBJ")
	{
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
			int posIndices[3];
			int texIndices[3];
			for (int j = 0; j < 3; j++)
			{
				posIndices[j] = faces[i].posIndices[j];
				if (nTexCoords > 0)
				{
					texIndices[j] = faces[i].texIndices[j];
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
	else if (ext == "PLY")
	{
		std::vector<std::array<int, 3>> faces;
		OBJLoader::Vec3f s = { (float)scale[0], (float)scale[1], (float)scale[2] };
		PLYLoader::loadPly(filename, x, faces, s);

		mesh.release();
		const unsigned int nPoints = (unsigned int)x.size();
		const unsigned int nFaces = (unsigned int)faces.size();
		mesh.initMesh(nPoints, nFaces * 2, nFaces);
		vd.reserve(nPoints);
		for (unsigned int i = 0; i < nPoints; i++)
		{
			vd.addVertex(Vector3r(x[i][0], x[i][1], x[i][2]));
		}
		for (unsigned int i = 0; i < nFaces; i++)
		{
			int posIndices[3];
			for (int j = 0; j < 3; j++)
				posIndices[j] = faces[i][j];

			mesh.addFace(&posIndices[0]);
		}

		LOG_INFO << "Number of triangles: " << nFaces;
		LOG_INFO << "Number of vertices: " << nPoints;

	}
	else
		LOG_ERR << "File " << filename << " has a unknown file type.";
}

void DemoBase::exportMeshOBJ(const std::string& exportFileName, const unsigned int nVert, const Vector3r* pos, const unsigned int nTri, const unsigned int* faces)
{
	// Open the file
	std::ofstream outfile(exportFileName);
	if (!outfile)
	{
		LOG_WARN << "Cannot open a file to save OBJ mesh.";
		return;
	}

	// Header
	outfile << "# Created by the PositionBasedDynamics library\n";
	outfile << "g default\n";

	// Vertices
	{
		for (auto j = 0u; j < nVert; j++)
		{
			const Vector3r& x = pos[j];
			outfile << "v " << x[0] << " " << x[1] << " " << x[2] << "\n";
		}
	}

	// faces
	{
		for (auto j = 0u; j < nTri; j++)
		{
			outfile << "f " << faces[3 * j + 0] + 1 << " " << faces[3 * j + 1] + 1 << " " << faces[3 * j + 2] + 1 << "\n";
		}
	}
	outfile.close();
}


void DemoBase::exportOBJ()
{
	if (!m_enableExportOBJ)
		return;

	if (TimeManager::getCurrent()->getTime() < m_nextFrameTime)
		return;

	m_nextFrameTime += 1.0 / (Real)m_exportFPS;

	//////////////////////////////////////////////////////////////////////////
	// rigid bodies
	//////////////////////////////////////////////////////////////////////////

	std::string exportPath = getOutputPath() + "/export";
	FileSystem::makeDirs(exportPath);

	SimulationModel* model = Simulation::getCurrent()->getModel();
	const ParticleData& pd = model->getParticles();
	for (unsigned int i = 0; i < model->getTriangleModels().size(); i++)
	{
		const IndexedFaceMesh& mesh = model->getTriangleModels()[i]->getParticleMesh();
		const unsigned int offset = model->getTriangleModels()[i]->getIndexOffset();
		const Vector3r* x = model->getParticles().getVertices().data();

		std::string fileName = "triangle_model";
		fileName = fileName + std::to_string(i) + "_" + std::to_string(m_frameCounter) + ".obj";
		std::string exportFileName = FileSystem::normalizePath(exportPath + "/" + fileName);

		exportMeshOBJ(exportFileName, mesh.numVertices(), &x[offset], mesh.numFaces(), mesh.getFaces().data());
	}

	for (unsigned int i = 0; i < model->getTetModels().size(); i++)
	{
		const IndexedFaceMesh& mesh = model->getTetModels()[i]->getVisMesh();
		// has a vis mesh
		if (mesh.numVertices() > 0)
		{
			const Vector3r* x = model->getTetModels()[i]->getVisVertices().getVertices().data();

			std::string fileName = "tet_model";
			fileName = fileName + std::to_string(i) + "_" + std::to_string(m_frameCounter) + ".obj";
			std::string exportFileName = FileSystem::normalizePath(exportPath + "/" + fileName);

			exportMeshOBJ(exportFileName, mesh.numVertices(), x, mesh.numFaces(), mesh.getFaces().data());
		}
		else
		{
			const IndexedFaceMesh& mesh = model->getTetModels()[i]->getSurfaceMesh();
			const unsigned int offset = model->getTetModels()[i]->getIndexOffset();
			const Vector3r* x = model->getParticles().getVertices().data();

			std::string fileName = "tet_model";
			fileName = fileName + std::to_string(i) + "_" + std::to_string(m_frameCounter) + ".obj";
			std::string exportFileName = FileSystem::normalizePath(exportPath + "/" + fileName);

			exportMeshOBJ(exportFileName, mesh.numVertices(), &x[offset], mesh.numFaces(), mesh.getFaces().data());
		}
	}

	for (unsigned int i = 0; i < model->getRigidBodies().size(); i++)
	{
		const IndexedFaceMesh& mesh = model->getRigidBodies()[i]->getGeometry().getMesh();
		const Vector3r* x = model->getRigidBodies()[i]->getGeometry().getVertexData().getVertices().data();

		std::string fileName = "rigid_body";
		fileName = fileName + std::to_string(i) + "_" + std::to_string(m_frameCounter) + ".obj";
		std::string exportFileName = FileSystem::normalizePath(exportPath + "/" + fileName);

		exportMeshOBJ(exportFileName, mesh.numVertices(), x, mesh.numFaces(), mesh.getFaces().data());
	}


	m_frameCounter++;
}

void DemoBase::exportMeshPLY(const std::string& exportFileName, const unsigned int nVert, const Vector3r* pos, const unsigned int nTri, const unsigned int* faces)
{
	// Suppose these hold your data
	std::vector<std::array<double, 3>> meshVertexPositions;
	std::vector<std::vector<size_t>> meshFaceIndices;

	// vertices
	meshVertexPositions.resize(nVert);
	for (auto j = 0u; j < nVert; j++)
		meshVertexPositions[j] = { pos[j][0], pos[j][1], pos[j][2] };

	// faces
	meshFaceIndices.resize(nTri);
	for (auto j = 0u; j < nTri; j++)
	{
		meshFaceIndices[j].resize(3);
		meshFaceIndices[j] = { faces[3 * j], faces[3 * j + 1], faces[3 * j + 2] };
	}

	// Create an empty object
	happly::PLYData plyOut;

	// Add mesh data (elements are created automatically)
	plyOut.addVertexPositions(meshVertexPositions);
	plyOut.addFaceIndices(meshFaceIndices);


	// Write the object to file
	plyOut.write(exportFileName, happly::DataFormat::Binary);
}

void DemoBase::exportPLY()
{
	if (!m_enableExportPLY)
		return;

	if (TimeManager::getCurrent()->getTime() < m_nextFrameTime)
		return;

	m_nextFrameTime += 1.0 / (Real)m_exportFPS;

	//////////////////////////////////////////////////////////////////////////
	// rigid bodies
	//////////////////////////////////////////////////////////////////////////

	std::string exportPath = getOutputPath() + "/export";
	FileSystem::makeDirs(exportPath);

	SimulationModel* model = Simulation::getCurrent()->getModel();
	const ParticleData& pd = model->getParticles();
	for (unsigned int i = 0; i < model->getTriangleModels().size(); i++)
	{
		const IndexedFaceMesh& mesh = model->getTriangleModels()[i]->getParticleMesh();
		const unsigned int offset = model->getTriangleModels()[i]->getIndexOffset();
		const Vector3r* x = model->getParticles().getVertices().data();

		std::string fileName = "triangle_model";
		fileName = fileName + std::to_string(i) + "_" + std::to_string(m_frameCounter) + ".ply";
		std::string exportFileName = FileSystem::normalizePath(exportPath + "/" + fileName);

		exportMeshPLY(exportFileName, mesh.numVertices(), &x[offset], mesh.numFaces(), mesh.getFaces().data());
	}

	for (unsigned int i = 0; i < model->getTetModels().size(); i++)
	{
		const IndexedFaceMesh& mesh = model->getTetModels()[i]->getVisMesh();
		// has a vis mesh
		if (mesh.numVertices() > 0)
		{
			const Vector3r* x = model->getTetModels()[i]->getVisVertices().getVertices().data();

			std::string fileName = "tet_model";
			fileName = fileName + std::to_string(i) + "_" + std::to_string(m_frameCounter) + ".ply";
			std::string exportFileName = FileSystem::normalizePath(exportPath + "/" + fileName);

			exportMeshPLY(exportFileName, mesh.numVertices(), x, mesh.numFaces(), mesh.getFaces().data());
		}
		else
		{
			const IndexedFaceMesh& mesh = model->getTetModels()[i]->getSurfaceMesh();
			const unsigned int offset = model->getTetModels()[i]->getIndexOffset();
			const Vector3r* x = model->getParticles().getVertices().data();

			std::string fileName = "tet_model";
			fileName = fileName + std::to_string(i) + "_" + std::to_string(m_frameCounter) + ".ply";
			std::string exportFileName = FileSystem::normalizePath(exportPath + "/" + fileName);

			exportMeshPLY(exportFileName, mesh.numVertices(), &x[offset], mesh.numFaces(), mesh.getFaces().data());
		}
	}

	for (unsigned int i = 0; i < model->getRigidBodies().size(); i++)
	{
		const IndexedFaceMesh& mesh = model->getRigidBodies()[i]->getGeometry().getMesh();
		const Vector3r* x = model->getRigidBodies()[i]->getGeometry().getVertexData().getVertices().data();

		std::string fileName = "rigid_body";
		fileName = fileName + std::to_string(i) + "_" + std::to_string(m_frameCounter) + ".ply";
		std::string exportFileName = FileSystem::normalizePath(exportPath + "/" + fileName);

		exportMeshPLY(exportFileName, mesh.numVertices(), x, mesh.numFaces(), mesh.getFaces().data());
	}


	m_frameCounter++;
}


void DemoBase::step()
{
	exportOBJ();
	exportPLY();
}

