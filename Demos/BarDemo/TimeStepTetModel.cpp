#include "TimeStepTetModel.h"
#include "Demos/Utils/TimeManager.h"
#include "PositionBasedDynamics/PositionBasedDynamics.h"
#include "PositionBasedDynamics/TimeIntegration.h"

using namespace PBD;
using namespace std;

TimeStepTetModel::TimeStepTetModel()
{
	m_simulationMethod = 1;
}

TimeStepTetModel::~TimeStepTetModel(void)
{
}

void TimeStepTetModel::step(TetModel &model)
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
void TimeStepTetModel::clearAccelerations(TetModel &model)
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

void TimeStepTetModel::reset(TetModel &model)
{

}

void TimeStepTetModel::constraintProjection(TetModel &model)
{
	const unsigned int maxIter = 5;
	unsigned int iter = 0;

	ParticleData &pd = model.getParticleMesh().getVertexData();
	std::vector<TetModel::TetConstraint> &tetConstraints = model.getTetConstraints();
	const IndexedTetMesh<ParticleData>::VertexTets *vTets = model.getParticleMesh().getVertexTets().data();

	while (iter < maxIter)
	{
		const unsigned int nTets = model.getParticleMesh().numTets();
		const unsigned int *tets = model.getParticleMesh().getTets().data();

		if (m_simulationMethod == 1)		// Distance constraints
		{
			const unsigned int nEdges = model.getParticleMesh().numEdges();
			const IndexedTetMesh<ParticleData>::Edge *edges = model.getParticleMesh().getEdges().data();
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
				const bool res = PositionBasedDynamics::solveDistanceConstraint(x1, invMass1, x2, invMass2, restLength, 0.0f, model.getStiffness(), corr1, corr2);

				if (res)
				{
					if (invMass1 != 0.0f)
						x1 += corr1;
					if (invMass2 != 0.0f)
						x2 += corr2;
				}
			}
		}

		for (unsigned int i = 0; i < nTets; i++)
		{
			const unsigned int v1 = tets[4 * i];
			const unsigned int v2 = tets[4 * i + 1];
			const unsigned int v3 = tets[4 * i + 2];
			const unsigned int v4 = tets[4 * i + 3];

			Eigen::Vector3f &x1 = pd.getPosition(v1);
			Eigen::Vector3f &x2 = pd.getPosition(v2);
			Eigen::Vector3f &x3 = pd.getPosition(v3);
			Eigen::Vector3f &x4 = pd.getPosition(v4);

			const float invMass1 = pd.getInvMass(v1);
			const float invMass2 = pd.getInvMass(v2);
			const float invMass3 = pd.getInvMass(v3);
			const float invMass4 = pd.getInvMass(v4);
		
			Eigen::Vector3f corr1, corr2, corr3, corr4;
			bool res = false;
			if (m_simulationMethod == 1)		// Distance constraints
			{ 				
 				res = PositionBasedDynamics::solveVolumeConstraint(	x1, invMass1, 
																	x2, invMass2, 
																	x3, invMass3,
																	x4, invMass4,
																	tetConstraints[i].tetVolume, 
																	model.getStiffness(),
																	model.getStiffness(), 
																	corr1, corr2, corr3, corr4);
			}
			else if (m_simulationMethod == 2)		// strain energy constraint
			{
				float currentVolume = -(1.0f / 6.0f) * (x4 - x1).dot((x3 - x1).cross(x2 - x1));
				bool handleInversion = false;
				if (currentVolume / tetConstraints[i].tetVolume < 0.2)		// Only 20% of initial volume left
					handleInversion = true;

				res = PositionBasedDynamics::solveFEMTetraConstraint(
					x1, invMass1,
					x2, invMass2,
					x3, invMass3,
					x4, invMass4,
					tetConstraints[i].tetVolume,
					tetConstraints[i].invRestMat_FEM,
					model.getStiffness(),
					model.getPoissonRatio(), handleInversion,
					corr1, corr2, corr3, corr4);
			}
			else if (m_simulationMethod == 3)		// strain based dynamics
			{
				Eigen::Vector3f stiffness(model.getStiffness(), model.getStiffness(), model.getStiffness());
				res = PositionBasedDynamics::solveStrainTetraConstraint(
					x1, invMass1,
					x2, invMass2,
					x3, invMass3,
					x4, invMass4,
					tetConstraints[i].invRestMat_SBD,
					stiffness,
					stiffness,
					model.getNormalizeStretch(),
					model.getNormalizeShear(),
					corr1, corr2, corr3, corr4);
			}
			else if (m_simulationMethod == 4)		// shape matching
			{
				Eigen::Vector3f &x1_0 = pd.getPosition0(v1);
				Eigen::Vector3f &x2_0 = pd.getPosition0(v2);
				Eigen::Vector3f &x3_0 = pd.getPosition0(v3);
				Eigen::Vector3f &x4_0 = pd.getPosition0(v4);

				Eigen::Vector3f x[4] = { x1, x2, x3, x4 };
				Eigen::Vector3f x0[4] = { x1_0, x2_0, x3_0, x4_0 };
				float w[4] = { invMass1, invMass2, invMass3, invMass4 };

				Eigen::Vector3f corr[4];
				res = PositionBasedDynamics::solveShapeMatchingConstraint(
					x0, x, w, 4,
					tetConstraints[i].restCm_SM,
					tetConstraints[i].invRestMat_SM,
					model.getStiffness(),
					false, 
					corr);

				// Important: Divide position correction by the number of clusters 
				// which contain the vertex. 
				corr1 = (1.0f / vTets[v1].m_numTets) * corr[0];
				corr2 = (1.0f / vTets[v2].m_numTets) * corr[1];
				corr3 = (1.0f / vTets[v3].m_numTets) * corr[2];
				corr4 = (1.0f / vTets[v4].m_numTets) * corr[3];
			}

			if (res)
 			{
				if (invMass1 != 0.0f)
 					x1 += corr1;
				if (invMass2 != 0.0f)
 					x2 += corr2;
				if (invMass3 != 0.0f)
					x3 += corr3;
				if (invMass4 != 0.0f)
					x4 += corr4;
 			}
		}	

		iter++;
	}
}
