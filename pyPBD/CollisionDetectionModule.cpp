#include "common.h"

#include <Simulation/CollisionDetection.h>
#include <Simulation/DistanceFieldCollisionDetection.h>
#include <Simulation/CubicSDFCollisionDetection.h>

#include <pybind11/pybind11.h>

namespace py = pybind11;

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

void CollisionDetectionModule(py::module m_sub) 
{
    py::class_<PBD::CollisionDetection::CollisionObject>(m_sub, "CollisionObject")
        .def_readonly_static("RigidBodyCollisionObjectType", &PBD::CollisionDetection::CollisionObject::RigidBodyCollisionObjectType)
        .def_readonly_static("TriangleModelCollisionObjectType", &PBD::CollisionDetection::CollisionObject::TriangleModelCollisionObjectType)
        .def_readonly_static("TetModelCollisionObjectType", &PBD::CollisionDetection::CollisionObject::TetModelCollisionObjectType)
        .def_readwrite("aabb", &PBD::CollisionDetection::CollisionObject::m_aabb)
        .def_readwrite("bodyIndex", &PBD::CollisionDetection::CollisionObject::m_bodyIndex)
        .def_readwrite("bodyType", &PBD::CollisionDetection::CollisionObject::m_bodyType);

    py::class_<PBD::CollisionDetection::CollisionObjectWithoutGeometry, PBD::CollisionDetection::CollisionObject>(m_sub, "CollisionObjectWithoutGeometry")
        .def(py::init<>())
        .def_readwrite_static("TYPE_ID", &PBD::CollisionDetection::CollisionObjectWithoutGeometry::TYPE_ID)
        .def("getTypeId", &PBD::CollisionDetection::CollisionObjectWithoutGeometry::getTypeId);

    py::class_<PBD::CollisionDetection>(m_sub, "CollisionDetection")
        .def_readonly_static("RigidBodyContactType", &PBD::CollisionDetection::RigidBodyContactType)
        .def_readonly_static("ParticleContactType", &PBD::CollisionDetection::ParticleContactType)
        .def_readonly_static("ParticleRigidBodyContactType", &PBD::CollisionDetection::ParticleRigidBodyContactType)
        .def_readonly_static("ParticleSolidContactType", &PBD::CollisionDetection::ParticleSolidContactType)

        .def("cleanup", &PBD::CollisionDetection::cleanup)
        .def("getTolerance", &PBD::CollisionDetection::getTolerance)
        .def("setTolerance", &PBD::CollisionDetection::setTolerance)
        .def("addRigidBodyContact", &PBD::CollisionDetection::addRigidBodyContact)
        .def("addParticleRigidBodyContact", &PBD::CollisionDetection::addParticleRigidBodyContact)
        .def("addParticleSolidContact", &PBD::CollisionDetection::addParticleSolidContact)
        .def("addCollisionObject", &PBD::CollisionDetection::addCollisionObject)
        .def("getCollisionObjects", &PBD::CollisionDetection::getCollisionObjects)
        .def("collisionDetection", &PBD::CollisionDetection::collisionDetection)
        //.def("setContactCallback", &PBD::CollisionDetection::setContactCallback)
        //.def("setSolidContactCallback", &PBD::CollisionDetection::setSolidContactCallback)
        .def("updateAABBs", &PBD::CollisionDetection::updateAABBs)
        .def("updateAABB", overload_cast_<PBD::SimulationModel&, PBD::CollisionDetection::CollisionObject*>()(&PBD::CollisionDetection::updateAABB));

    py::class_<PBD::DistanceFieldCollisionDetection, PBD::CollisionDetection>(m_sub, "DistanceFieldCollisionDetection")
        .def(py::init<>())
        .def("addCollisionBox", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::ParticleData& pd, const unsigned int offset, const unsigned int numVertices, const Vector3r& box, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionBox(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, box, testMesh, invertSDF);
            })
        .def("addCollisionBox", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::VertexData& pd, const unsigned int offset, const unsigned int numVertices, const Vector3r& box, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionBox(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, box, testMesh, invertSDF);
            })

        .def("addCollisionSphere", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::ParticleData& pd, const unsigned int offset, const unsigned int numVertices, const Real radius, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionSphere(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, radius, testMesh, invertSDF);
            })
        .def("addCollisionSphere", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::VertexData& pd, const unsigned int offset, const unsigned int numVertices, const Real radius, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionSphere(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, radius, testMesh, invertSDF);
            })

        .def("addCollisionTorus", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::ParticleData& pd, const unsigned int offset, const unsigned int numVertices, const Vector2r& radii, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionTorus(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, radii, testMesh, invertSDF);
            })
        .def("addCollisionTorus", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::VertexData& pd, const unsigned int offset, const unsigned int numVertices, const Vector2r& radii, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionTorus(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, radii, testMesh, invertSDF);
            })

        .def("addCollisionCylinder", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::ParticleData& pd, const unsigned int offset, const unsigned int numVertices, const Vector2r& dim, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionCylinder(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, dim, testMesh, invertSDF);
            })
        .def("addCollisionCylinder", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::VertexData& pd, const unsigned int offset, const unsigned int numVertices, const Vector2r& dim, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionCylinder(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, dim, testMesh, invertSDF);
            })

        .def("addCollisionHollowSphere", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::ParticleData& pd, const unsigned int offset, const unsigned int numVertices, const Real radius, const Real thickness, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionHollowSphere(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, radius, thickness, testMesh, invertSDF);
            })
        .def("addCollisionHollowSphere", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::VertexData& pd, const unsigned int offset, const unsigned int numVertices, const Real radius, const Real thickness, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionHollowSphere(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, radius, thickness, testMesh, invertSDF);
            })

        .def("addCollisionHollowBox", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::ParticleData& pd, const unsigned int offset, const unsigned int numVertices, const Vector3r& box, const Real thickness, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionHollowBox(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, box, thickness, testMesh, invertSDF);
            })
        .def("addCollisionHollowBox", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::VertexData& pd, const unsigned int offset, const unsigned int numVertices, const Vector3r& box, const Real thickness, const bool testMesh, const bool invertSDF)
            {
                cd.addCollisionHollowBox(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, box, thickness, testMesh, invertSDF);
            })

        .def("addCollisionObjectWithoutGeometry", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::ParticleData &pd, const unsigned int offset, const unsigned int numVertices, const bool testMesh)
            {
                cd.addCollisionObjectWithoutGeometry(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, testMesh);
            })
        .def("addCollisionObjectWithoutGeometry", [](PBD::DistanceFieldCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::VertexData& vd, const unsigned int offset, const unsigned int numVertices, const bool testMesh)
            {
                cd.addCollisionObjectWithoutGeometry(bodyIndex, bodyType, &vd.getPosition(offset), numVertices, testMesh);
            })

         .def("isDistanceFieldCollisionObject", &PBD::DistanceFieldCollisionDetection::isDistanceFieldCollisionObject);

    py::class_<PBD::CubicSDFCollisionDetection::Grid, std::shared_ptr<PBD::CubicSDFCollisionDetection::Grid>>(m_sub, "CubicSDFCollisionDetectionGridPtr")
        ;

    py::class_<PBD::CubicSDFCollisionDetection, PBD::DistanceFieldCollisionDetection>(m_sub, "CubicSDFCollisionDetection")
        .def(py::init<>())
        .def("addCubicSDFCollisionObject", [](PBD::CubicSDFCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::ParticleData& pd, const unsigned int offset, const unsigned int numVertices, const std::string& sdfFile, const Vector3r& scale, const bool testMesh, const bool invertSDF)
            {
                cd.addCubicSDFCollisionObject(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, sdfFile, scale, testMesh, invertSDF);
            })
        .def("addCubicSDFCollisionObject", [](PBD::CubicSDFCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::VertexData& pd, const unsigned int offset, const unsigned int numVertices, const std::string& sdfFile, const Vector3r& scale, const bool testMesh, const bool invertSDF)
            {
                cd.addCubicSDFCollisionObject(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, sdfFile, scale, testMesh, invertSDF);
            })
        .def("addCubicSDFCollisionObject", [](PBD::CubicSDFCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::ParticleData& pd, const unsigned int offset, const unsigned int numVertices, PBD::CubicSDFCollisionDetection::GridPtr sdf, const Vector3r& scale, const bool testMesh, const bool invertSDF)
            {
                cd.addCubicSDFCollisionObject(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, sdf, scale, testMesh, invertSDF);
            })
        .def("addCubicSDFCollisionObject", [](PBD::CubicSDFCollisionDetection& cd, const unsigned int bodyIndex, const unsigned int bodyType,
            const PBD::VertexData& pd, const unsigned int offset, const unsigned int numVertices, const PBD::CubicSDFCollisionDetection::GridPtr sdf, const Vector3r& scale, const bool testMesh, const bool invertSDF)
            {
                cd.addCubicSDFCollisionObject(bodyIndex, bodyType, &pd.getPosition(offset), numVertices, sdf, scale, testMesh, invertSDF);
            })
        .def_static("generateSDF", [](const PBD::VertexData &vd, const Utilities::IndexedFaceMesh &mesh, const Eigen::Matrix<unsigned int, 3, 1> &resolution) -> PBD::CubicSDFCollisionDetection::GridPtr
            {
                const std::vector<unsigned int>& faces = mesh.getFaces();
                const unsigned int nFaces = mesh.numFaces();

#ifdef USE_DOUBLE
                Discregrid::TriangleMesh sdfMesh(&vd.getPosition(0)[0], faces.data(), vd.size(), nFaces);
#else
                // if type is float, copy vector to double vector
                std::vector<double> doubleVec;
                doubleVec.resize(3 * vd.size());
                for (unsigned int i = 0; i < vd.size(); i++)
                    for (unsigned int j = 0; j < 3; j++)
                        doubleVec[3 * i + j] = vd.getPosition(i)[j];
                Discregrid::TriangleMesh sdfMesh(&doubleVec[0], faces.data(), vd.size(), nFaces);
#endif
                Discregrid::MeshDistance md(sdfMesh);
                Eigen::AlignedBox3d domain;
                for (auto const& x : sdfMesh.vertices())
                {
                    domain.extend(x);
                }
                domain.max() += 0.1 * Eigen::Vector3d::Ones();
                domain.min() -= 0.1 * Eigen::Vector3d::Ones();

                std::cout << "Set SDF resolution: " << resolution[0] << ", " << resolution[1] << ", " << resolution[2] << std::endl;
                //PBD::CubicSDFCollisionDetection::Grid *sdf = new PBD::CubicSDFCollisionDetection::Grid(domain, std::array<unsigned int, 3>({ resolution[0], resolution[1], resolution[2] }));
                auto sdf = std::make_shared<PBD::CubicSDFCollisionDetection::Grid>(domain, std::array<unsigned int, 3>({ resolution[0], resolution[1], resolution[2] }));
                auto func = Discregrid::DiscreteGrid::ContinuousFunction{};
                func = [&md](Eigen::Vector3d const& xi) {return md.signedDistanceCached(xi); };
                std::cout << "Generate SDF\n";
                sdf->addFunction(func, true);
                return sdf;
            })
        ;
}