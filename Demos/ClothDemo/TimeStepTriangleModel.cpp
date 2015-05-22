#include "TimeStepTriangleModel.h"
#include "Demos/Utils/TimeManager.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"
#include "PositionBasedDynamics/TimeIntegration.h"

using namespace PBD;
using namespace std;

TimeStepTriangleModel::TimeStepTriangleModel()
{
	m_simulationMethod = 1;
	m_bendingMethod = 1;
}

TimeStepTriangleModel::~TimeStepTriangleModel(void)
{
}

void TimeStepTriangleModel::step(TriangleModel &model)
{
	TimeManager *tm = TimeManager::getCurrent ();
	const float h = tm->getTimeStepSize();
	ParticleData &pd = model.getParticleMesh().getVertexData();

	clearAccelerations(model);
	for (unsigned int i = 0; i < pd.size(); i++)
	{ 
		pd.getLastPosition(i) = pd.getOldPosition(i);
		pd.getOldPosition(i) = pd.getPosition(i);
		TimeIntegration::semiImplicitEuler(h, pd.getMass(i), pd.getPosition(i), pd.getVelocity(i), pd.getAcceleration(i));
	}

	constraintProjection(model);

	// Update velocities	
	for (unsigned int i = 0; i < pd.size(); i++)
	{
		if (m_velocityUpdateMethod == 0)
			TimeIntegration::velocityUpdateFirstOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getVelocity(i));
		else
			TimeIntegration::velocityUpdateSecondOrder(h, pd.getMass(i), pd.getPosition(i), pd.getOldPosition(i), pd.getLastPosition(i), pd.getVelocity(i));
	}

	// compute new time	
	tm->setTime (tm->getTime () + h);
}

/** Clear accelerations and add gravitation.
 */
void TimeStepTriangleModel::clearAccelerations(TriangleModel &model)
{
	ParticleData &pd = model.getParticleMesh().getVertexData();
	const unsigned int count = pd.size();
	const Eigen::Vector3f grav(0.0f, -9.81f, 0.0f);
	for (unsigned int i=0; i < count; i++)
	{
		// Clear accelerations of dynamic particles
		if (pd.getMass(i) != 0.0)
		{
			Eigen::Vector3f &a = pd.getAcceleration(i);
			a = grav;
		}
	}
}

void TimeStepTriangleModel::reset(TriangleModel &model)
{

}

void TimeStepTriangleModel::constraintProjection(TriangleModel &model)
{
	const unsigned int maxIter = 5;
	unsigned int iter = 0;


	ParticleData &pd = model.getParticleMesh().getVertexData();

	while (iter < maxIter)
	{
		if (m_simulationMethod == 1)		// Distance constraints
		{
			const unsigned int nEdges = model.getParticleMesh().numEdges();
			const IndexedFaceMesh<ParticleData>::Edge *edges = model.getParticleMesh().getEdges().data();
			for (unsigned int i = 0; i < nEdges; i++)
			{
				const unsigned int v1 = edges[i].m_vert[0];
				const unsigned int v2 = edges[i].m_vert[1];

				const Eigen::Vector3f &x1_0 = pd.getPosition0(v1);
				const Eigen::Vector3f &x2_0 = pd.getPosition0(v2);

				Eigen::Vector3f &x1 = pd.getPosition(v1);
				Eigen::Vector3f &x2 = pd.getPosition(v2);

				const float invMass1 = pd.getInvMass(v1);
				const float invMass2 = pd.getInvMass(v2);

				float restLength = (x2_0 - x1_0).norm();

				Eigen::Vector3f corr1, corr2;
				const bool res = PositionBasedDynamics::solveDistanceConstraint(x1, invMass1, x2, invMass2, restLength, model.getStiffness(), model.getStiffness(), corr1, corr2);

				if (res)
				{
					if (invMass1 != 0.0f)
						x1 += corr1;
					if (invMass2 != 0.0f)
						x2 += corr2;
				}
			}
		}
		else if (m_simulationMethod == 2)		// strain energy constraint
		{
			std::vector<TriangleModel::TriangleConstraint> &triangleConstraints = model.getTriangleConstraints();
			const unsigned int *tris = model.getParticleMesh().getFaces().data();
			const unsigned int nFaces = model.getParticleMesh().numFaces();
			for (unsigned int i = 0; i < nFaces; i++)
			{
				Eigen::Vector3f &x1 = pd.getPosition(tris[3 * i]);
				Eigen::Vector3f &x2 = pd.getPosition(tris[3 * i + 1]);
				Eigen::Vector3f &x3 = pd.getPosition(tris[3 * i + 2]);

				const float invMass1 = pd.getInvMass(tris[3 * i]);
				const float invMass2 = pd.getInvMass(tris[3 * i + 1]);
				const float invMass3 = pd.getInvMass(tris[3 * i + 2]);

				Eigen::Vector3f corr1, corr2, corr3;
				const bool res = PositionBasedDynamics::solveFEMTriangleConstraint(
					x1, invMass1,
					x2, invMass2,
					x3, invMass3,
					triangleConstraints[i].triangleArea,
					triangleConstraints[i].invRestMat_FEM,
					model.getXXStiffness(),
					model.getYYStiffness(),
					model.getXYStiffness(),
					model.getXYPoissonRatio(),
					model.getYXPoissonRatio(),
					corr1, corr2, corr3);

				if (res)
				{
					if (invMass1 != 0.0f)
						x1 += corr1;
					if (invMass2 != 0.0f)
						x2 += corr2;
					if (invMass3 != 0.0f)
						x3 += corr3;
				}
			}
		}
		else if (m_simulationMethod == 3)		// strain based dynamics
		{
			std::vector<TriangleModel::TriangleConstraint> &triangleConstraints = model.getTriangleConstraints();
			const unsigned int *tris = model.getParticleMesh().getFaces().data();
			const unsigned int nFaces = model.getParticleMesh().numFaces();
			for (unsigned int i = 0; i < nFaces; i++)
			{
				Eigen::Vector3f &x1 = pd.getPosition(tris[3*i]);
				Eigen::Vector3f &x2 = pd.getPosition(tris[3*i+1]);
				Eigen::Vector3f &x3 = pd.getPosition(tris[3*i+2]);

				const float invMass1 = pd.getInvMass(tris[3*i]);
				const float invMass2 = pd.getInvMass(tris[3*i+1]);
				const float invMass3 = pd.getInvMass(tris[3*i+2]);

				Eigen::Vector3f corr1, corr2, corr3;
				const bool res = PositionBasedDynamics::solveStrainTriangleConstraint(
					x1, invMass1,
					x2, invMass2,
					x3, invMass3,
					triangleConstraints[i].invRestMat_SBD,
					model.getXXStiffness(), 
					model.getYYStiffness(), 
					model.getXYStiffness(), 
					model.getNormalizeStretch(), 
					model.getNormalizeShear(), 
					corr1, corr2, corr3);

				if (res)
				{
					if (invMass1 != 0.0f)
						x1 += corr1;
					if (invMass2 != 0.0f)
						x2 += corr2;
					if (invMass3 != 0.0f)
						x3 += corr3;
				}
			}
		}

		if (m_bendingMethod != 0)
		{
			std::vector<TriangleModel::BendingConstraint> &bendingConstraints = model.getBendingConstraints();

			for (unsigned int i = 0; i < (unsigned int)bendingConstraints.size(); i++)
			{
				Eigen::Vector3f &x1 = pd.getPosition(bendingConstraints[i].vertex1);
				Eigen::Vector3f &x2 = pd.getPosition(bendingConstraints[i].vertex2);
				Eigen::Vector3f &x3 = pd.getPosition(bendingConstraints[i].vertex3);
				Eigen::Vector3f &x4 = pd.getPosition(bendingConstraints[i].vertex4);

				const float invMass1 = pd.getInvMass(bendingConstraints[i].vertex1);
				const float invMass2 = pd.getInvMass(bendingConstraints[i].vertex2);
				const float invMass3 = pd.getInvMass(bendingConstraints[i].vertex3);
				const float invMass4 = pd.getInvMass(bendingConstraints[i].vertex4);

				Eigen::Vector3f corr1, corr2, corr3, corr4;
				bool res;
				if (m_bendingMethod == 1)
					res = PositionBasedDynamics::solveDihedralConstraint(x1, invMass1, x2, invMass2, x3, invMass3, x4, invMass4, bendingConstraints[i].restAngle, model.getBendingStiffness(), corr1, corr2, corr3, corr4);
				else if (m_bendingMethod == 2)
					res = PositionBasedDynamics::solveIsometricBendingConstraint(x1, invMass1, x2, invMass2, x3, invMass3, x4, invMass4, bendingConstraints[i].Q, model.getBendingStiffness(), corr1, corr2, corr3, corr4);

				if (res)
				{
					x1 += corr1;
					x2 += corr2;
					x3 += corr3;
					x4 += corr4;
				}
			}
		}

		iter++;
	}
}
