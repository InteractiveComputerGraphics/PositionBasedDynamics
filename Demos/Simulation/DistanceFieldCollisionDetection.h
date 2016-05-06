#ifndef _SIMPLECOLLISIONDETECTION_H
#define _SIMPLECOLLISIONDETECTION_H

#include "Common/Common.h"
#include "Demos/Simulation/CollisionDetection.h"
#include "AABB.h"
#include "BoundingSphereHierarchy.h"

namespace PBD
{
	/** Distance field collision detection. */
	class DistanceFieldCollisionDetection : public CollisionDetection
	{
	public:
		struct DistanceFieldCollisionObject : public CollisionObject
		{		
			bool m_testMesh;
			PointCloudBSH m_bvh;

			DistanceFieldCollisionObject() { m_testMesh = true; }
			virtual ~DistanceFieldCollisionObject() {}
			virtual bool collisionTest(const Vector3r &x, const Real tolerance, Vector3r &cp, Vector3r &n, Real &dist);
			virtual void approximateNormal(const Eigen::Vector3d &x, const Real tolerance, Vector3r &n);

			virtual Real distance(const Eigen::Vector3d &x, const Real tolerance) = 0;
		};

		struct DistanceFieldCollisionObjectWithoutGeometry : public DistanceFieldCollisionObject
		{
			static int TYPE_ID;

			virtual ~DistanceFieldCollisionObjectWithoutGeometry() {}
			virtual int &getTypeId() const { return TYPE_ID; }
			virtual bool collisionTest(const Vector3r &x, const Real tolerance, Vector3r &cp, Vector3r &n, Real &dist) { return false; }
			virtual Real distance(const Eigen::Vector3d &x, const Real tolerance) { return 0.0; }
		};

		struct DistanceFieldCollisionBox : public DistanceFieldCollisionObject
		{
			Vector3r m_box;
			static int TYPE_ID;

			virtual ~DistanceFieldCollisionBox() {}
			virtual int &getTypeId() const { return TYPE_ID; }
			virtual Real distance(const Eigen::Vector3d &x, const Real tolerance);
		};

		struct DistanceFieldCollisionSphere : public DistanceFieldCollisionObject
		{
			Real m_radius;
			static int TYPE_ID;

			virtual ~DistanceFieldCollisionSphere() {}
			virtual int &getTypeId() const { return TYPE_ID; }
			virtual bool collisionTest(const Vector3r &x, const Real tolerance, Vector3r &cp, Vector3r &n, Real &dist);
			virtual Real distance(const Eigen::Vector3d &x, const Real tolerance);
		};

		struct DistanceFieldCollisionTorus : public DistanceFieldCollisionObject
		{
			Vector2r m_radii;
			static int TYPE_ID;

			virtual ~DistanceFieldCollisionTorus() {}
			virtual int &getTypeId() const { return TYPE_ID; }
			virtual Real distance(const Eigen::Vector3d &x, const Real tolerance);
		};

		struct DistanceFieldCollisionCylinder : public DistanceFieldCollisionObject
		{
			Vector2r m_dim;
			static int TYPE_ID;

			virtual ~DistanceFieldCollisionCylinder() {}
			virtual int &getTypeId() const { return TYPE_ID; }
			virtual Real distance(const Eigen::Vector3d &x, const Real tolerance);
		};

		struct ContactData
		{
			char m_type;
			unsigned int m_index1;
			unsigned int m_index2;
			Vector3r m_cp1;
			Vector3r m_cp2;
			Vector3r m_normal;
			Real m_dist;
			Real m_restitution;
			Real m_friction;
		};

	protected:
		void collisionDetectionRigidBodies(RigidBody *rb1, DistanceFieldCollisionObject *co1, RigidBody *rb2, DistanceFieldCollisionObject *co2,
			const Real restitutionCoeff, const Real frictionCoeff
			, std::vector<std::vector<ContactData> > &contacts_mt
			);
		void collisionDetectionRBSolid(const ParticleData &pd, const unsigned int offset, const unsigned int numVert, 
			DistanceFieldCollisionObject *co1, RigidBody *rb2, DistanceFieldCollisionObject *co2, 
			const Real restitutionCoeff, const Real frictionCoeff
			, std::vector<std::vector<ContactData> > &contacts_mt
			);


	public:
		DistanceFieldCollisionDetection();
		virtual ~DistanceFieldCollisionDetection();

		virtual void collisionDetection(SimulationModel &model);

		virtual bool isDistanceFieldCollisionObject(CollisionObject *co) const;

		void addCollisionBox(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices, const Vector3r &box, const bool testMesh = true);
		void addCollisionSphere(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices, const Real radius, const bool testMesh = true);
		void addCollisionTorus(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices, const Vector2r &radii, const bool testMesh = true);
		void addCollisionObjectWithoutGeometry(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices);

		/** Add collision cylinder
		 *
		 * @param  bodyIndex index of corresponding body
		 * @param  bodyType type of corresponding body
		 * @param  dim (radius, height) of cylinder
		 */		
		void addCollisionCylinder(const unsigned int bodyIndex, const unsigned int bodyType, const Vector3r *vertices, const unsigned int numVertices, const Vector2r &dim, const bool testMesh = true);
	};
}

#endif
