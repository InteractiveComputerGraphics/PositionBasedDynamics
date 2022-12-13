#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "GenericConstraintsModel.h"
#include <iostream>
#include "Simulation/TimeStepController.h"
#include "Demos/Visualization/Visualization.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
#include "Demos/Common/DemoBase.h"
#include "Simulation/Simulation.h"
#include "GenericConstraints.h"

// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
	#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

void timeStep ();
void buildModel ();
void createMesh();
void render ();
void reset();

DemoBase *base;

const int nRows = 30;
const int nCols = 30;
const Real width = 10.0;
const Real height = 10.0;
Real bendingStiffness = 0.01;
Real distanceStiffness = 1.0;

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Generic cloth demo");

	GenericConstraintsModel *model = new GenericConstraintsModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	buildModel();

	base->createParameterGUI();

	// reset simulation when cloth simulation/bending method has changed
	model->setClothSimulationMethodChangedCallback([&]() { reset(); });
	model->setClothBendingMethodChangedCallback([&]() { reset(); });

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3r (5.0, 10.0, 30.0), Vector3r (5.0, 0.0, 0.0));
	MiniGL::mainLoop();

	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	delete Simulation::getCurrent();
	delete base;
	delete model;

	return 0;
}

void reset()
{
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Simulation::getCurrent()->reset();
	base->getSelectedParticles().clear();
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

		base->step();
	}

	for (unsigned int i = 0; i < model->getTriangleModels().size(); i++)
		model->getTriangleModels()[i]->updateMeshNormals(model->getParticles());
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


/** Create a particle model mesh 
*/
void createMesh()
{
	GenericConstraintsModel* model = (GenericConstraintsModel*)Simulation::getCurrent()->getModel();
	model->addRegularTriangleModel(nCols, nRows,
		Vector3r(0, 1, 0), AngleAxisr(M_PI * 0.5, Vector3r(1, 0, 0)).matrix(), Vector2r(width, height));

	// Set mass of points to zero => make it static
	ParticleData& pd = model->getParticles();
	pd.setMass(0, 0.0);
	pd.setMass(nRows-1, 0.0);

	// init constraints
	for (unsigned int cm = 0; cm < model->getTriangleModels().size(); cm++)
	{
		const unsigned int offset = model->getTriangleModels()[cm]->getIndexOffset();
		IndexedFaceMesh &mesh = model->getTriangleModels()[cm]->getParticleMesh();
		const unsigned int nEdges = mesh.numEdges();
		const IndexedFaceMesh::Edge *edges = mesh.getEdges().data();

		// distance constraints
		for (unsigned int i = 0; i < nEdges; i++)
		{
			const unsigned int v1 = edges[i].m_vert[0] + offset;
			const unsigned int v2 = edges[i].m_vert[1] + offset;

			model->addGenericDistanceConstraint(v1, v2, distanceStiffness);
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
					model->addGenericIsometricBendingConstraint(vertex1, vertex2, vertex3, vertex4, bendingStiffness);
				}
			}
		}
	}

	LOG_INFO << "Number of triangles: " << model->getTriangleModels()[0]->getParticleMesh().numFaces();
	LOG_INFO << "Number of vertices: " << nRows*nCols;

}
