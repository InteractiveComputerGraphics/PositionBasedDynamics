#include "DistanceFieldCollisionDetection.h"
#include "Demos/Simulation/IDFactory.h"
#include "omp.h"

using namespace PBD;

int DistanceFieldCollisionDetection::DistanceFieldCollisionBox::TYPE_ID = IDFactory::getId();
int DistanceFieldCollisionDetection::DistanceFieldCollisionSphere::TYPE_ID = IDFactory::getId();
int DistanceFieldCollisionDetection::DistanceFieldCollisionTorus::TYPE_ID = IDFactory::getId();
int DistanceFieldCollisionDetection::DistanceFieldCollisionCylinder::TYPE_ID = IDFactory::getId();
int DistanceFieldCollisionDetection::DistanceFieldCollisionHollowSphere::TYPE_ID = IDFactory::getId();
int DistanceFieldCollisionDetection::DistanceFieldCollisionHollowBox::TYPE_ID = IDFactory::getId();
int DistanceFieldCollisionDetection::DistanceFieldCollisionObjectWithoutGeometry::TYPE_ID = IDFactory::getId();


DistanceFieldCollisionDetection::DistanceFieldCollisionDetection() :
	CollisionDetection()
{
}

DistanceFieldCollisionDetection::~DistanceFieldCollisionDetection()
{
}

void DistanceFieldCollisionDetection::collisionDetection(SimulationModel &model)
{
	model.resetContacts();
	const SimulationModel::RigidBodyVector &rigidBodies = model.getRigidBodies();
	const SimulationModel::TriangleModelVector &triModels = model.getTriangleModels();
	const SimulationModel::TetModelVector &tetModels = model.getTetModels();
	const ParticleData &pd = model.getParticles();

	std::vector < std::pair<unsigned int, unsigned int>> coPairs;
	for (unsigned int i = 0; i < m_collisionObjects.size(); i++)
	{
		CollisionDetection::CollisionObject *co1 = m_collisionObjects[i];
		for (unsigned int k = 0; k < m_collisionObjects.size(); k++)
		{
			CollisionDetection::CollisionObject *co2 = m_collisionObjects[k];
			if ((i != k))
			{
				coPairs.push_back({ i, k });
			}
		}
	}

	//omp_set_num_threads(1);
	std::vector<std::vector<ContactData> > contacts_mt;
#ifdef _DEBUG
	const unsigned int maxThreads = 1;
#else
	const unsigned int maxThreads = omp_get_max_threads();
#endif
	contacts_mt.resize(maxThreads);

	#pragma omp parallel default(shared)
	{
		// Update BVHs
		#pragma omp for schedule(static)
		for (int i = 0; i < (int)m_collisionObjects.size(); i++)
		{
			CollisionDetection::CollisionObject *co = m_collisionObjects[i];
			updateAABB(model, co);
			if (isDistanceFieldCollisionObject(co))
			{
				if ((co->m_bodyType == CollisionDetection::CollisionObject::TriangleModelCollisionObjectType) ||
					(co->m_bodyType == CollisionDetection::CollisionObject::TetModelCollisionObjectType))
				{
					DistanceFieldCollisionObject *sco = (DistanceFieldCollisionObject*)co;
					sco->m_bvh.update();
				}
			}
		}

		#pragma omp for schedule(static)
		for (int i = 0; i < (int)coPairs.size(); i++)
		{
			std::pair<unsigned int, unsigned int> &coPair = coPairs[i];
			CollisionDetection::CollisionObject *co1 = m_collisionObjects[coPair.first];
			CollisionDetection::CollisionObject *co2 = m_collisionObjects[coPair.second];

			if ((co2->m_bodyType != CollisionDetection::CollisionObject::RigidBodyCollisionObjectType) ||
				!isDistanceFieldCollisionObject(co1) ||
				!isDistanceFieldCollisionObject(co2) ||
				!AABB::intersection(co1->m_aabb, co2->m_aabb))
				continue;


			RigidBody *rb2 = rigidBodies[co2->m_bodyIndex];
			if ((co1->m_bodyType == CollisionDetection::CollisionObject::RigidBodyCollisionObjectType) &&
				((DistanceFieldCollisionObject*) co1)->m_testMesh)
			{
				RigidBody *rb1 = rigidBodies[co1->m_bodyIndex];
				const Real restitutionCoeff = rb1->getRestitutionCoeff() * rb2->getRestitutionCoeff();
				const Real frictionCoeff = rb1->getFrictionCoeff() + rb2->getFrictionCoeff();
				collisionDetectionRigidBodies(rb1, (DistanceFieldCollisionObject*)co1, rb2, (DistanceFieldCollisionObject*)co2,
					restitutionCoeff, frictionCoeff
					, contacts_mt
					);
			}
			else if (co1->m_bodyType == CollisionDetection::CollisionObject::TriangleModelCollisionObjectType)
			{
				TriangleModel *tm = triModels[co1->m_bodyIndex];
				const unsigned int offset = tm->getIndexOffset();
				const IndexedFaceMesh &mesh = tm->getParticleMesh();
				const unsigned int numVert = mesh.numVertices();
				const Real restitutionCoeff = tm->getRestitutionCoeff() * rb2->getRestitutionCoeff();
				const Real frictionCoeff = tm->getFrictionCoeff() + rb2->getFrictionCoeff();
				collisionDetectionRBSolid(pd, offset, numVert, (DistanceFieldCollisionObject*)co1, rb2, (DistanceFieldCollisionObject*)co2,
					restitutionCoeff, frictionCoeff
					, contacts_mt
					);
			}
			else if (co1->m_bodyType == CollisionDetection::CollisionObject::TetModelCollisionObjectType)
			{
				TetModel *tm = tetModels[co1->m_bodyIndex];
				const unsigned int offset = tm->getIndexOffset();
				const IndexedTetMesh &mesh = tm->getParticleMesh();
				const unsigned int numVert = mesh.numVertices();
				const Real restitutionCoeff = tm->getRestitutionCoeff() * rb2->getRestitutionCoeff();
				const Real frictionCoeff = tm->getFrictionCoeff() + rb2->getFrictionCoeff();
				collisionDetectionRBSolid(pd, offset, numVert, (DistanceFieldCollisionObject*)co1, rb2, (DistanceFieldCollisionObject*)co2,
					restitutionCoeff, frictionCoeff
					, contacts_mt
					);
			}
		}
	}
	for (unsigned int i = 0; i < contacts_mt.size(); i++)
	{
		for (unsigned int j = 0; j < contacts_mt[i].size(); j++)
		{
			if (contacts_mt[i][j].m_type == 1)
				addParticleRigidBodyContact(contacts_mt[i][j].m_index1, contacts_mt[i][j].m_index2,
				contacts_mt[i][j].m_cp1, contacts_mt[i][j].m_cp2, contacts_mt[i][j].m_normal,
				contacts_mt[i][j].m_dist, contacts_mt[i][j].m_restitution, contacts_mt[i][j].m_friction);
			else
				addRigidBodyContact(contacts_mt[i][j].m_index1, contacts_mt[i][j].m_index2,
				contacts_mt[i][j].m_cp1, contacts_mt[i][j].m_cp2, contacts_mt[i][j].m_normal,
				contacts_mt[i][j].m_dist, contacts_mt[i][j].m_restitution, contacts_mt[i][j].m_friction);
		}
	}
}

void DistanceFieldCollisionDetection::collisionDetectionRigidBodies(RigidBody *rb1, DistanceFieldCollisionObject *co1, RigidBody *rb2, DistanceFieldCollisionObject *co2,
	const Real restitutionCoeff, const Real frictionCoeff
	, std::vector<std::vector<ContactData> > &contacts_mt
	)
{
	if ((rb1->getMass() == 0.0) && (rb2->getMass() == 0.0))
		return;

	const VertexData &vd = rb1->getGeometry().getVertexData();

	const Vector3r &com2 = rb2->getPosition();

	// remove the rotation of the main axis transformation that is performed
	// to get a diagonal inertia tensor since the distance function is
	// evaluated in local coordinates
	//
	// transformation world to local:
	// p_local = R_initial^T ( R_MAT R^T (p_world - x) - x_initial + x_MAT)
	//
	// transformation local to:
	// p_world = R R_MAT^T (R_initial p_local + x_initial - x_MAT) + x
	//
	const Matrix3r &R = rb2->getTransformationR();
	const Vector3r &v1 = rb2->getTransformationV1();
	const Vector3r &v2 = rb2->getTransformationV2();

	const PointCloudBSH &bvh = ((DistanceFieldCollisionDetection::DistanceFieldCollisionObject*) co1)->m_bvh;
	std::function<bool(unsigned int, unsigned int)> predicate = [&](unsigned int node_index, unsigned int depth)
	{
		const BoundingSphere &bs = bvh.hull(node_index);
		const Vector3r &sphere_x = bs.x();
		const Vector3r sphere_x_w = rb1->getRotation() * sphere_x + rb1->getPosition();

		AlignedBox3r box3f;
		box3f.extend(co2->m_aabb.m_p[0]);
		box3f.extend(co2->m_aabb.m_p[1]);
		const Real dist = box3f.exteriorDistance(sphere_x_w);

		// Test if center of bounding sphere intersects AABB
		if (dist < bs.r())
		{
			// Test if distance of center of bounding sphere to collision object is smaller than the radius
			const Vector3r x = R * (sphere_x_w - com2) + v1;
			const Real dist2 = co2->distance(x.template cast<Real>(), static_cast<Real>(m_tolerance));
			if (dist2 == std::numeric_limits<Real>::max())
				return true;
			if (dist2 < bs.r())
				return true;
		}
		return false;
	};
	std::function<void(unsigned int, unsigned int)> cb = [&](unsigned int node_index, unsigned int depth)
	{
		auto const& node = bvh.node(node_index);
		if (!node.is_leaf())
			return;

		for (auto i = node.begin; i < node.begin + node.n; ++i)
		{
			unsigned int index = bvh.entity(i);
			const Vector3r &x_w = vd.getPosition(index);
			const Vector3r x = R * (x_w - com2) + v1;
			Vector3r cp, n;
			Real dist;
			if (co2->collisionTest(x, m_tolerance, cp, n, dist))
			{
				const Vector3r cp_w = R.transpose() * cp + v2;
				const Vector3r n_w = R.transpose() * n;

#ifdef _DEBUG
				int tid = 0;
#else
				int tid = omp_get_thread_num();
#endif
				contacts_mt[tid].push_back({ 0, co1->m_bodyIndex, co2->m_bodyIndex, x_w, cp_w, n_w, dist, restitutionCoeff, frictionCoeff });
			}
		}
	};
	bvh.traverse_depth_first(predicate, cb);
}


void DistanceFieldCollisionDetection::collisionDetectionRBSolid(const ParticleData &pd, const unsigned int offset, const unsigned int numVert,
	DistanceFieldCollisionObject *co1, RigidBody *rb2, DistanceFieldCollisionObject *co2,
	const Real restitutionCoeff, const Real frictionCoeff
	, std::vector<std::vector<ContactData> > &contacts_mt
	)
{
	const Vector3r &com2 = rb2->getPosition();

	// remove the rotation of the main axis transformation that is performed
	// to get a diagonal inertia tensor since the distance function is
	// evaluated in local coordinates
	//
	// transformation world to local:
	// p_local = R_initial^T ( R_MAT R^T (p_world - x) - x_initial + x_MAT)
	//
	// transformation local to:
	// p_world = R R_MAT^T (R_initial p_local + x_initial - x_MAT) + x
	//
	const Matrix3r &R = rb2->getTransformationR();
	const Vector3r &v1 = rb2->getTransformationV1();
	const Vector3r &v2 = rb2->getTransformationV2();

	const PointCloudBSH &bvh = ((DistanceFieldCollisionDetection::DistanceFieldCollisionObject*) co1)->m_bvh;

	std::function<bool(unsigned int, unsigned int)> predicate = [&](unsigned int node_index, unsigned int depth)
	{
		const BoundingSphere &bs = bvh.hull(node_index);
		const Vector3r &sphere_x_w = bs.x();

		AlignedBox3r box3f;
		box3f.extend(co2->m_aabb.m_p[0]);
		box3f.extend(co2->m_aabb.m_p[1]);
		const Real dist = box3f.exteriorDistance(sphere_x_w);

		// Test if center of bounding sphere intersects AABB
		if (dist < bs.r())
		{
			// Test if distance of center of bounding sphere to collision object is smaller than the radius
			const Vector3r x = R * (sphere_x_w - com2) + v1;
			const Real dist2 = co2->distance(x.template cast<Real>(), static_cast<Real>(m_tolerance));
			if (dist2 == std::numeric_limits<Real>::max())
				return true;
			if (dist2 < bs.r())
				return true;
		}
		return false;
	};

	std::function<void(unsigned int, unsigned int)> cb = [&](unsigned int node_index, unsigned int depth)
	{
		auto const& node = bvh.node(node_index);
		if (!node.is_leaf())
			return;

		for (auto i = node.begin; i < node.begin + node.n; ++i)
		{
			unsigned int index = bvh.entity(i) + offset;
			const Vector3r &x_w = pd.getPosition(index);
			const Vector3r x = R * (x_w - com2) + v1;
			Vector3r cp, n;
			Real dist;
			if (co2->collisionTest(x, m_tolerance, cp, n, dist))
			{
				const Vector3r cp_w = R.transpose() * cp + v2;
				const Vector3r n_w = R.transpose() * n;

#ifdef _DEBUG
				int tid = 0;
#else
				int tid = omp_get_thread_num();
#endif
				contacts_mt[tid].push_back({ 1, index, co2->m_bodyIndex, x_w, cp_w, n_w, dist, restitutionCoeff, frictionCoeff });
			}
		}
	};

	bvh.traverse_depth_first(predicate, cb);
}

bool DistanceFieldCollisionDetection::isDistanceFieldCollisionObject(CollisionObject *co) const
{
	return (co->getTypeId() == DistanceFieldCollisionDetection::DistanceFieldCollisionBox::TYPE_ID) ||
		(co->getTypeId() == DistanceFieldCollisionDetection::DistanceFieldCollisionSphere::TYPE_ID) ||
		(co->getTypeId() == DistanceFieldCollisionDetection::DistanceFieldCollisionTorus::TYPE_ID) ||
		(co->getTypeId() == DistanceFieldCollisionDetection::DistanceFieldCollisionCylinder::TYPE_ID) ||
		(co->getTypeId() == DistanceFieldCollisionDetection::DistanceFieldCollisionHollowSphere::TYPE_ID) ||
		(co->getTypeId() == DistanceFieldCollisionDetection::DistanceFieldCollisionHollowBox::TYPE_ID) ||
		(co->getTypeId() == DistanceFieldCollisionDetection::DistanceFieldCollisionObjectWithoutGeometry::TYPE_ID);
}

void DistanceFieldCollisionDetection::addCollisionBox(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices, const Vector3r &box, const bool testMesh, const bool invertSDF)
{
	DistanceFieldCollisionDetection::DistanceFieldCollisionBox *cf = new DistanceFieldCollisionDetection::DistanceFieldCollisionBox();
	cf->m_bodyIndex = bodyIndex;
	cf->m_bodyType = bodyType;
	// distance function requires 0.5*box
	cf->m_box = 0.5*box;
	cf->m_bvh.init(vertices, numVertices);
	cf->m_bvh.construct();
	cf->m_testMesh = testMesh;
	if (invertSDF)
		cf->m_invertSDF = -1.0;
	m_collisionObjects.push_back(cf);
}

void DistanceFieldCollisionDetection::addCollisionSphere(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices, const Real radius, const bool testMesh, const bool invertSDF)
{
	DistanceFieldCollisionDetection::DistanceFieldCollisionSphere *cs = new DistanceFieldCollisionDetection::DistanceFieldCollisionSphere();
	cs->m_bodyIndex = bodyIndex;
	cs->m_bodyType = bodyType;
	cs->m_radius = radius;
	cs->m_bvh.init(vertices, numVertices);
	cs->m_bvh.construct();
	cs->m_testMesh = testMesh;
	if (invertSDF)
		cs->m_invertSDF = -1.0;
	m_collisionObjects.push_back(cs);
}

void DistanceFieldCollisionDetection::addCollisionTorus(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices, const Vector2r &radii, const bool testMesh, const bool invertSDF)
{
	DistanceFieldCollisionDetection::DistanceFieldCollisionTorus *ct = new DistanceFieldCollisionDetection::DistanceFieldCollisionTorus();
	ct->m_bodyIndex = bodyIndex;
	ct->m_bodyType = bodyType;
	ct->m_radii = radii;
	ct->m_bvh.init(vertices, numVertices);
	ct->m_bvh.construct();
	ct->m_testMesh = testMesh;
	if (invertSDF)
		ct->m_invertSDF = -1.0;
	m_collisionObjects.push_back(ct);
}

void DistanceFieldCollisionDetection::addCollisionCylinder(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices, const Vector2r &dim, const bool testMesh, const bool invertSDF)
{
	DistanceFieldCollisionDetection::DistanceFieldCollisionCylinder *ct = new DistanceFieldCollisionDetection::DistanceFieldCollisionCylinder();
	ct->m_bodyIndex = bodyIndex;
	ct->m_bodyType = bodyType;
	ct->m_dim = dim;
	// distance function uses height/2
	ct->m_dim[1] *= 0.5;
	ct->m_bvh.init(vertices, numVertices);
	ct->m_bvh.construct();
	ct->m_testMesh = testMesh;
	if (invertSDF)
		ct->m_invertSDF = -1.0;
	m_collisionObjects.push_back(ct);
}

void DistanceFieldCollisionDetection::addCollisionHollowSphere(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices, const Real radius, const Real thickness, const bool testMesh, const bool invertSDF)
{
	DistanceFieldCollisionDetection::DistanceFieldCollisionHollowSphere *cs = new DistanceFieldCollisionDetection::DistanceFieldCollisionHollowSphere();
	cs->m_bodyIndex = bodyIndex;
	cs->m_bodyType = bodyType;
	cs->m_radius = radius;
	cs->m_thickness = thickness;
	cs->m_bvh.init(vertices, numVertices);
	cs->m_bvh.construct();
	cs->m_testMesh = testMesh;
	if (invertSDF)
		cs->m_invertSDF = -1.0;
	m_collisionObjects.push_back(cs);
}

void DistanceFieldCollisionDetection::addCollisionHollowBox(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices, const Vector3r &box, const Real thickness, const bool testMesh, const bool invertSDF)
{
	DistanceFieldCollisionDetection::DistanceFieldCollisionHollowBox *cf = new DistanceFieldCollisionDetection::DistanceFieldCollisionHollowBox();
	cf->m_bodyIndex = bodyIndex;
	cf->m_bodyType = bodyType;
	// distance function requires 0.5*box
	cf->m_box = 0.5*box;
	cf->m_thickness = thickness;
	cf->m_bvh.init(vertices, numVertices);
	cf->m_bvh.construct();
	cf->m_testMesh = testMesh;
	if (invertSDF)
		cf->m_invertSDF = -1.0;
	m_collisionObjects.push_back(cf);
}

void DistanceFieldCollisionDetection::addCollisionObjectWithoutGeometry(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices)
{
	DistanceFieldCollisionObjectWithoutGeometry *co = new DistanceFieldCollisionObjectWithoutGeometry();
	co->m_bodyIndex = bodyIndex;
	co->m_bodyType = bodyType;
	co->m_bvh.init(vertices, numVertices);
	co->m_bvh.construct();
	co->m_testMesh = false;
	co->m_invertSDF = 1.0;
	m_collisionObjects.push_back(co);
}

Real DistanceFieldCollisionDetection::DistanceFieldCollisionBox::distance(const Vector3r &x, const Real tolerance)
{
	const Vector3r box_d = m_box.template cast<Real>();
	const Vector3r x_d = x.template cast<Real>();
	const Vector3r d(fabs(x_d.x()) - box_d.x(), fabs(x_d.y()) - box_d.y(), fabs(x_d.z()) - box_d.z());
	const Vector3r max_d(std::max<Real>(d.x(), 0.0), std::max<Real>(d.y(), 0.0), std::max<Real>(d.z(), 0.0));
	return m_invertSDF*(std::min<Real>(std::max<Real>(d.x(), std::max<Real>(d.y(), d.z())), 0.0) + max_d.norm()) - tolerance;
}

Real DistanceFieldCollisionDetection::DistanceFieldCollisionSphere::distance(const Vector3r &x, const Real tolerance)
{
	const Vector3r d = x.template cast<Real>();
	const Real dl = d.norm();
	return m_invertSDF*(dl - static_cast<Real>(m_radius)) - tolerance;
}

bool DistanceFieldCollisionDetection::DistanceFieldCollisionSphere::collisionTest(const Vector3r &x, const Real tolerance, Vector3r &cp, Vector3r &n, Real &dist, const Real maxDist)
{
	const Vector3r d = x;
	const Real dl = d.norm();
	dist = m_invertSDF*(dl - m_radius) - tolerance;
	if (dist < maxDist)
	{
		if (dl < 1.e-6)
			n.setZero();
		else
			n = m_invertSDF * d / dl;

		cp = ((m_radius+tolerance) * n);
		return true;
	}
	return false;
}

Real DistanceFieldCollisionDetection::DistanceFieldCollisionTorus::distance(const Vector3r &x, const Real tolerance)
{
	const Vector2r radii_d = m_radii.template cast<Real>();
	const Vector2r q(Vector2r(x.x(), x.z()).norm() - radii_d.x(), x.y());
	return m_invertSDF*(q.norm() - radii_d.y()) - tolerance;
}

Real DistanceFieldCollisionDetection::DistanceFieldCollisionCylinder::distance(const Vector3r &x, const Real tolerance)
{
	const Real l = sqrt(x.x()*x.x() + x.z()*x.z());
	const Vector2r d = Vector2r(std::abs(l), std::abs(x.y())) - m_dim.template cast<Real>();
	const Vector2r max_d(std::max<Real>(d.x(), 0.0), std::max<Real>(d.y(), 0.0));
	return m_invertSDF*(std::min<Real>(std::max<Real>(d.x(), d.y()), 0.0) + max_d.norm()) - tolerance;
}


Real DistanceFieldCollisionDetection::DistanceFieldCollisionHollowSphere::distance(const Vector3r &x, const Real tolerance)
{
	const Vector3r d = x.template cast<Real>();
	const Real dl = d.norm();
	return m_invertSDF*(fabs(dl - m_radius) - m_thickness) - tolerance;
}

bool DistanceFieldCollisionDetection::DistanceFieldCollisionHollowSphere::collisionTest(const Vector3r &x, const Real tolerance, Vector3r &cp, Vector3r &n, Real &dist, const Real maxDist)
{
	const Vector3r d = x;
	const Real dl = d.norm();
	dist = m_invertSDF*(fabs(dl - m_radius) - m_thickness) - tolerance;
	if (dist < maxDist)
	{
		if (dl < 1.e-6)
			n.setZero();
		else if (dl < m_radius)
			n = -m_invertSDF*d / dl;
		else
			n = m_invertSDF*d / dl;

		cp = x - dist * n;
		return true;
	}
	return false;
}

Real DistanceFieldCollisionDetection::DistanceFieldCollisionHollowBox::distance(const Vector3r &x, const Real tolerance)
{
	const Vector3r box_d = m_box.template cast<Real>();
	const Vector3r x_d = x.template cast<Real>();
	const Vector3r d = x_d.cwiseAbs() - box_d;
	const Vector3r max_d = d.cwiseMax(Vector3r(0.0, 0.0, 0.0));
	return m_invertSDF * (std::abs(std::min<Real>(d.maxCoeff(), 0.0) + max_d.norm()) - m_thickness) - tolerance;
}

void DistanceFieldCollisionDetection::DistanceFieldCollisionObject::approximateNormal(const Vector3r &x, const Real tolerance, Vector3r &n)
{
	// approximate gradient
	Real eps = 1.e-6;
	n.setZero();
	Vector3r xTmp = x;
	for (unsigned int j = 0; j < 3; j++)
	{
		xTmp[j] += eps;

		Real e_p, e_m;
		e_p = distance(xTmp, tolerance);
		xTmp[j] = x[j] - eps;
		e_m = distance(xTmp, tolerance);
		xTmp[j] = x[j];

		Real res = (e_p - e_m) * (1.0 / (2.0*eps));

		n[j] = static_cast<Real>(res);
	}

	const Real norm2 = n.squaredNorm();
	if (norm2 < 1.e-6)
		n.setZero();
	else
		n = n / sqrt(norm2);
}


bool DistanceFieldCollisionDetection::DistanceFieldCollisionObject::collisionTest(const Vector3r &x, const Real tolerance, Vector3r &cp, Vector3r &n, Real &dist, const Real maxDist)
{
	const Real t_d = static_cast<Real>(tolerance);
	dist = (Real)distance(x.template cast<Real>(), t_d);
	if (dist < maxDist)
	{
		// approximate gradient
		const Vector3r x_d = x.template cast<Real>();

		approximateNormal(x_d, t_d, n);

		cp = (x - dist * n);
		return true;
	}
	return false;
}
