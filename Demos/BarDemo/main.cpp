#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "GL/glut.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeStepController.h"
#include <iostream>
#include "Demos/Visualization/Visualization.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
#include "Demos/Common/DemoBase.h"
#include "Demos/Common/TweakBarParameters.h"
#include "Simulation/Simulation.h"


// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
	#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

void initParameters();
void timeStep ();
void buildModel ();
void createMesh();
void render ();
void reset();
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

DemoBase *base;
const unsigned int width = 30;
const unsigned int depth = 5;
const unsigned int height = 5; 
short simulationMethod = 2;

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Bar demo");

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	buildModel();

	initParameters();

	Simulation::getCurrent()->setSimulationMethodChangedCallback([&]() { reset(); initParameters(); base->getSceneLoader()->readParameterObject(Simulation::getCurrent()->getTimeStep()); });

	// OpenGL
	MiniGL::setClientIdleFunc (50, timeStep);		
	MiniGL::setKeyFunc(0, 'r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3r (5.0, 10.0, 30.0), Vector3r (5.0, 0.0, 0.0));

	TwType enumType2 = TwDefineEnum("SimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "SimulationMethod", enumType2, setSimulationMethod, getSimulationMethod, &simulationMethod,
			" label='Simulation method' enum='0 {None}, 1 {Volume constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics (no inversion handling)}, 4 {Shape matching (no inversion handling)}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "Stiffness", TW_TYPE_REAL, setStiffness, getStiffness, model, " label='Stiffness'  min=0.0 step=0.1 precision=4 group='Simulation' ");
	TwAddVarCB(MiniGL::getTweakBar(), "PoissonRatio", TW_TYPE_REAL, setPoissonRatio, getPoissonRatio, model, " label='Poisson ratio XY'  min=0.0 step=0.1 precision=4 group='Simulation' ");
	TwAddVarCB(MiniGL::getTweakBar(), "NormalizeStretch", TW_TYPE_BOOL32, setNormalizeStretch, getNormalizeStretch, model, " label='Normalize stretch' group='Strain based dynamics' ");
	TwAddVarCB(MiniGL::getTweakBar(), "NormalizeShear", TW_TYPE_BOOL32, setNormalizeShear, getNormalizeShear, model, " label='Normalize shear' group='Strain based dynamics' ");

	glutMainLoop ();	

	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	delete Simulation::getCurrent();
	delete base;
	delete model;

	return 0;
}

void initParameters()
{
	TwRemoveAllVars(MiniGL::getTweakBar());
	TweakBarParameters::cleanup();

	MiniGL::initTweakBarParameters();

	TweakBarParameters::createParameterGUI();
	TweakBarParameters::createParameterObjectGUI(base);
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent());
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent()->getModel());
	TweakBarParameters::createParameterObjectGUI(Simulation::getCurrent()->getTimeStep());
}

void reset()
{
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Simulation::getCurrent()->reset();
	base->getSelectedParticles().clear();
	
	Simulation::getCurrent()->getModel()->cleanup();
	buildModel();
}

void timeStep ()
{
	const Real pauseAt = base->getValue<Real>(DemoBase::PAUSE_AT);
	if ((pauseAt > 0.0) && (pauseAt < TimeManager::getCurrent()->getTime()))
		base->setValue(DemoBase::PAUSE, true);

	if (base->getValue<bool>(DemoBase::PAUSE))
		return;

	// Simulation code
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const unsigned int numSteps = base->getValue<unsigned int>(DemoBase::NUM_STEPS_PER_RENDER);
	for (unsigned int i = 0; i < numSteps; i++)
	{
		START_TIMING("SimStep");
		Simulation::getCurrent()->getTimeStep()->step(*model);
		STOP_TIMING_AVG;
	}

	for (unsigned int i = 0; i < model->getTetModels().size(); i++)
	{
		model->getTetModels()[i]->updateMeshNormals(model->getParticles());
	} 	
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.005));

	createMesh();
}


void render ()
{
	base->render();
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
				points[i*height*depth + j*depth + k] = 0.3*Vector3r((Real)i, (Real)j, (Real)k);
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
	SimulationModel *model = Simulation::getCurrent()->getModel();
	model->addTetModel(width*height*depth, (unsigned int)indices.size() / 4u, points, indices.data());

	ParticleData &pd = model->getParticles();
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

	// init constraints
	for (unsigned int cm = 0; cm < model->getTetModels().size(); cm++)
	{
		const unsigned int nTets = model->getTetModels()[cm]->getParticleMesh().numTets();
		const unsigned int *tets = model->getTetModels()[cm]->getParticleMesh().getTets().data();
		const IndexedTetMesh::VertexTets *vTets = model->getTetModels()[cm]->getParticleMesh().getVertexTets().data();
		if (simulationMethod == 1)
		{
			const unsigned int offset = model->getTetModels()[cm]->getIndexOffset();
			const unsigned int nEdges = model->getTetModels()[cm]->getParticleMesh().numEdges();
			const IndexedTetMesh::Edge *edges = model->getTetModels()[cm]->getParticleMesh().getEdges().data();
			for (unsigned int i = 0; i < nEdges; i++)
			{
				const unsigned int v1 = edges[i].m_vert[0] + offset;
				const unsigned int v2 = edges[i].m_vert[1] + offset;

				model->addDistanceConstraint(v1, v2);
			}
			
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v1 = tets[4 * i];
				const unsigned int v2 = tets[4 * i + 1];
				const unsigned int v3 = tets[4 * i + 2];
				const unsigned int v4 = tets[4 * i + 3];

				model->addVolumeConstraint(v1, v2, v3, v4);
			}
		}
		else if (simulationMethod == 2)
		{
			TetModel::ParticleMesh &mesh = model->getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v1 = tets[4 * i];
				const unsigned int v2 = tets[4 * i + 1];
				const unsigned int v3 = tets[4 * i + 2];
				const unsigned int v4 = tets[4 * i + 3];

				model->addFEMTetConstraint(v1, v2, v3, v4);
			}
		}
		else if (simulationMethod == 3)
		{
			TetModel::ParticleMesh &mesh = model->getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v1 = tets[4 * i];
				const unsigned int v2 = tets[4 * i + 1];
				const unsigned int v3 = tets[4 * i + 2];
				const unsigned int v4 = tets[4 * i + 3];

				model->addStrainTetConstraint(v1, v2, v3, v4);
			}
		}
		else if (simulationMethod == 4)
		{
			TetModel::ParticleMesh &mesh = model->getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v[4] = { tets[4 * i], tets[4 * i + 1], tets[4 * i + 2], tets[4 * i + 3] };
				// Important: Divide position correction by the number of clusters 
				// which contain the vertex.
				const unsigned int nc[4] = { vTets[v[0]].m_numTets, vTets[v[1]].m_numTets, vTets[v[2]].m_numTets, vTets[v[3]].m_numTets };
				model->addShapeMatchingConstraint(4, v, nc);
			}
		}
		model->getTetModels()[cm]->updateMeshNormals(pd);
	}

	LOG_INFO << "Number of tets: " << indices.size() / 4;
	LOG_INFO << "Number of vertices: " << width*height*depth;

}

void TW_CALL setStiffness(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*) clientData)->setValue<Real>(SimulationModel::SOLID_STIFFNESS, val);
}

void TW_CALL getStiffness(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getValue<Real>(SimulationModel::SOLID_STIFFNESS);
}

void TW_CALL setPoissonRatio(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setValue<Real>(SimulationModel::SOLID_POISSON_RATIO, val);
}

void TW_CALL getPoissonRatio(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getValue<Real>(SimulationModel::SOLID_POISSON_RATIO);
}

void TW_CALL setNormalizeStretch(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((SimulationModel*)clientData)->setValue<Real>(SimulationModel::SOLID_NORMALIZE_STRETCH, val);
}

void TW_CALL getNormalizeStretch(void *value, void *clientData)
{
	*(bool *)(value) = ((SimulationModel*)clientData)->getValue<Real>(SimulationModel::SOLID_NORMALIZE_STRETCH);
}

void TW_CALL setNormalizeShear(const void *value, void *clientData)
{
	const bool val = *(const bool *)(value);
	((SimulationModel*)clientData)->setValue<Real>(SimulationModel::SOLID_NORMALIZE_SHEAR, val);
}

void TW_CALL getNormalizeShear(void *value, void *clientData)
{
	*(bool *)(value) = ((SimulationModel*)clientData)->getValue<Real>(SimulationModel::SOLID_NORMALIZE_SHEAR);
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
