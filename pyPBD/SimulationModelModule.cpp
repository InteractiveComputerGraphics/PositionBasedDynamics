#include "common.h"

#include <Simulation/Simulation.h>
#include <Simulation/SimulationModel.h>
#include <Simulation/CubicSDFCollisionDetection.h>
#include <pyPBD/bind_pointer_vector.h>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

void SimulationModelModule(py::module m_sub) 
{
    py::class_<PBD::TriangleModel>(m_sub, "TriangleModel")
        .def(py::init<>())
        .def("getParticleMesh", (const PBD::TriangleModel::ParticleMesh& (PBD::TriangleModel::*)()const)(&PBD::TriangleModel::getParticleMesh))
        .def("cleanupModel", &PBD::TriangleModel::cleanupModel)
        .def("getIndexOffset", &PBD::TriangleModel::getIndexOffset)
        .def("initMesh", &PBD::TriangleModel::initMesh)
        .def("updateMeshNormals", &PBD::TriangleModel::updateMeshNormals)
        .def("getRestitutionCoeff", &PBD::TriangleModel::getRestitutionCoeff)
        .def("setRestitutionCoeff", &PBD::TriangleModel::setRestitutionCoeff)
        .def("getFrictionCoeff", &PBD::TriangleModel::getFrictionCoeff)
        .def("setFrictionCoeff", &PBD::TriangleModel::setFrictionCoeff);

    py::class_<PBD::TetModel>(m_sub, "TetModel")
        .def(py::init<>())
        .def("getInitialX", &PBD::TetModel::getInitialX)
        .def("setInitialX", &PBD::TetModel::setInitialX)
        .def("getInitialR", &PBD::TetModel::getInitialR)
        .def("setInitialR", &PBD::TetModel::setInitialR)
        .def("getInitialScale", &PBD::TetModel::getInitialScale)
        .def("setInitialScale", &PBD::TetModel::setInitialScale)

        .def("getSurfaceMesh", &PBD::TetModel::getSurfaceMesh)
        .def("getVisVertices", &PBD::TetModel::getVisVertices)
        .def("getVisMesh", &PBD::TetModel::getVisMesh)
        .def("getParticleMesh", (const PBD::TetModel::ParticleMesh & (PBD::TetModel::*)()const)(&PBD::TetModel::getParticleMesh))
        .def("cleanupModel", &PBD::TetModel::cleanupModel)
        .def("getIndexOffset", &PBD::TetModel::getIndexOffset)
        .def("initMesh", &PBD::TetModel::initMesh)
        .def("updateMeshNormals", &PBD::TetModel::updateMeshNormals)
        .def("attachVisMesh", &PBD::TetModel::attachVisMesh)
        .def("updateVisMesh", &PBD::TetModel::updateVisMesh)
        .def("getRestitutionCoeff", &PBD::TetModel::getRestitutionCoeff)
        .def("setRestitutionCoeff", &PBD::TetModel::setRestitutionCoeff)
        .def("getFrictionCoeff", &PBD::TetModel::getFrictionCoeff)
        .def("setFrictionCoeff", &PBD::TetModel::setFrictionCoeff);
 
    py::bind_pointer_vector<std::vector<PBD::TriangleModel*>>(m_sub, "VecTriangleModels");
    py::bind_pointer_vector<std::vector<PBD::TetModel*>>(m_sub, "VecTetModels");
    py::bind_pointer_vector<std::vector<PBD::RigidBody*>>(m_sub, "VecRigidBodies");
    py::bind_vector<std::vector<PBD::Constraint*>>(m_sub, "VecConstraints");

    py::class_<PBD::SimulationModel, GenParam::ParameterObject>(m_sub, "SimulationModel")
        .def(py::init<>())
        .def("init", &PBD::SimulationModel::init)
        .def("reset", &PBD::SimulationModel::reset)
        .def("cleanup", &PBD::SimulationModel::cleanup)
        .def("resetContacts", &PBD::SimulationModel::resetContacts)
        .def("updateConstraints", &PBD::SimulationModel::updateConstraints)
        .def("initConstraintGroups", &PBD::SimulationModel::initConstraintGroups)

        .def("addTriangleModel", [](    
            PBD::SimulationModel &model,
            const unsigned int nPoints,
            const unsigned int nFaces,
            std::vector<Vector3r> &points,
            std::vector<unsigned int> &indices,
            const PBD::TriangleModel::ParticleMesh::UVIndices& uvIndices,
            const PBD::TriangleModel::ParticleMesh::UVs& uvs)
            {
                model.addTriangleModel(nPoints, nFaces, points.data(), indices.data(), uvIndices, uvs);
            })
        .def("addTriangleModelCollision", [](
            PBD::SimulationModel& model,
            const unsigned int nPoints,
            const unsigned int nFaces,
            std::vector<Vector3r>& points,
            std::vector<unsigned int>& indices,
            const PBD::TriangleModel::ParticleMesh::UVIndices& uvIndices,
            const PBD::TriangleModel::ParticleMesh::UVs& uvs)
            {
                auto& triModels = model.getTriangleModels();
                int i = triModels.size();
                model.addTriangleModel(nPoints, nFaces, points.data(), indices.data(), uvIndices, uvs);
                PBD::ParticleData& pd = model.getParticles();
                const unsigned int nVert = triModels[i]->getParticleMesh().numVertices();
                unsigned int offset = triModels[i]->getIndexOffset();
                PBD::Simulation* sim = PBD::Simulation::getCurrent();
                PBD::CubicSDFCollisionDetection* cd = dynamic_cast<PBD::CubicSDFCollisionDetection*>(sim->getTimeStep()->getCollisionDetection());
                if (cd != nullptr)
                    cd->addCollisionObjectWithoutGeometry(i, PBD::CollisionDetection::CollisionObject::TriangleModelCollisionObjectType, &pd.getPosition(offset), nVert, true);
            })
        .def("addRegularTriangleModel", &PBD::SimulationModel::addRegularTriangleModel)
        .def("addRegularTriangleModelCollision", [](PBD::SimulationModel &model, 
            const int width, const int height,
            const Vector3r& translation,
            const Matrix3r& rotation,
            const Vector2r& scale)
            {
                auto &triModels = model.getTriangleModels();
                int i = triModels.size();
                model.addRegularTriangleModel(width, height, translation, rotation, scale);
                PBD::ParticleData& pd = model.getParticles();
                const unsigned int nVert = triModels[i]->getParticleMesh().numVertices();
                unsigned int offset = triModels[i]->getIndexOffset();
                PBD::Simulation* sim = PBD::Simulation::getCurrent();
                PBD::CubicSDFCollisionDetection* cd = dynamic_cast<PBD::CubicSDFCollisionDetection*>(sim->getTimeStep()->getCollisionDetection());
                if (cd != nullptr)
                    cd->addCollisionObjectWithoutGeometry(i, PBD::CollisionDetection::CollisionObject::TriangleModelCollisionObjectType, &pd.getPosition(offset), nVert, true);
            })
        .def("addTetModel", [](
            PBD::SimulationModel& model,
            const unsigned int nPoints,
            const unsigned int nTets,
            std::vector<Vector3r>& points,
            std::vector<unsigned int>& indices)
            {
                model.addTetModel(nPoints, nTets, points.data(), indices.data());
            })
        .def("addTetModelCollision", [](
            PBD::SimulationModel& model,
            const unsigned int nPoints,
            const unsigned int nTets,
            std::vector<Vector3r>& points,
            std::vector<unsigned int>& indices)
            {
                auto& tetModels = model.getTetModels();
                int i = tetModels.size();
                model.addTetModel(nPoints, nTets, points.data(), indices.data());
                PBD::ParticleData& pd = model.getParticles();
                const unsigned int nVert = tetModels[i]->getParticleMesh().numVertices();
                unsigned int offset = tetModels[i]->getIndexOffset();
                PBD::Simulation* sim = PBD::Simulation::getCurrent();
                PBD::CubicSDFCollisionDetection* cd = dynamic_cast<PBD::CubicSDFCollisionDetection*>(sim->getTimeStep()->getCollisionDetection());
                if (cd != nullptr)
                    cd->addCollisionObjectWithoutGeometry(i, PBD::CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, true);
            })
        .def("addRegularTetModel", &PBD::SimulationModel::addRegularTetModel)
        .def("addRegularTetModelCollision", [](PBD::SimulationModel &model, 
            const int width, const int height, const int depth,
            const Vector3r& translation,
            const Matrix3r& rotation,
            const Vector3r& scale)
            {
                auto &tetModels = model.getTetModels();
                int i = tetModels.size();
                model.addRegularTetModel(width, height, depth, translation, rotation, scale);
                PBD::ParticleData& pd = model.getParticles();
                const unsigned int nVert = tetModels[i]->getParticleMesh().numVertices();
                unsigned int offset = tetModels[i]->getIndexOffset();
                PBD::Simulation* sim = PBD::Simulation::getCurrent();
                PBD::CubicSDFCollisionDetection* cd = dynamic_cast<PBD::CubicSDFCollisionDetection*>(sim->getTimeStep()->getCollisionDetection());
                if (cd != nullptr)
                    cd->addCollisionObjectWithoutGeometry(i, PBD::CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, true);
            })
        .def("addLineModel", [](
            PBD::SimulationModel& model,
            const unsigned int nPoints,
            const unsigned int nQuaternions,
            std::vector<Vector3r>& points,
            std::vector<Quaternionr>& quaternions, 
            std::vector<unsigned int>& indices,
            std::vector<unsigned int>& indicesQuaternions)
            {
                model.addLineModel(nPoints, nQuaternions, points.data(), quaternions.data(), indices.data(), indicesQuaternions.data());
            })
        .def("addBallJoint", &PBD::SimulationModel::addBallJoint)
        .def("addBallOnLineJoint", &PBD::SimulationModel::addBallOnLineJoint)
        .def("addHingeJoint", &PBD::SimulationModel::addHingeJoint)
        .def("addTargetAngleMotorHingeJoint", &PBD::SimulationModel::addTargetAngleMotorHingeJoint)
        .def("addTargetVelocityMotorHingeJoint", &PBD::SimulationModel::addTargetVelocityMotorHingeJoint)
        .def("addUniversalJoint", &PBD::SimulationModel::addUniversalJoint)
        .def("addSliderJoint", &PBD::SimulationModel::addSliderJoint)
        .def("addTargetPositionMotorSliderJoint", &PBD::SimulationModel::addTargetPositionMotorSliderJoint)
        .def("addTargetVelocityMotorSliderJoint", &PBD::SimulationModel::addTargetVelocityMotorSliderJoint)
        .def("addRigidBodyParticleBallJoint", &PBD::SimulationModel::addRigidBodyParticleBallJoint)
        .def("addRigidBodySpring", &PBD::SimulationModel::addRigidBodySpring)
        .def("addDistanceJoint", &PBD::SimulationModel::addDistanceJoint)
        .def("addDamperJoint", &PBD::SimulationModel::addDamperJoint)
        .def("addRigidBodyContactConstraint", &PBD::SimulationModel::addRigidBodyContactConstraint)
        .def("addParticleRigidBodyContactConstraint", &PBD::SimulationModel::addParticleRigidBodyContactConstraint)
        .def("addParticleSolidContactConstraint", &PBD::SimulationModel::addParticleSolidContactConstraint)
        .def("addDistanceConstraint", &PBD::SimulationModel::addDistanceConstraint)
        .def("addDistanceConstraint_XPBD", &PBD::SimulationModel::addDistanceConstraint_XPBD)
        .def("addDihedralConstraint", &PBD::SimulationModel::addDihedralConstraint)
        .def("addIsometricBendingConstraint", &PBD::SimulationModel::addIsometricBendingConstraint)
        .def("addIsometricBendingConstraint_XPBD", &PBD::SimulationModel::addIsometricBendingConstraint_XPBD)
        .def("addFEMTriangleConstraint", &PBD::SimulationModel::addFEMTriangleConstraint)
        .def("addStrainTriangleConstraint", &PBD::SimulationModel::addStrainTriangleConstraint)
        .def("addVolumeConstraint", &PBD::SimulationModel::addVolumeConstraint)
        .def("addVolumeConstraint_XPBD", &PBD::SimulationModel::addVolumeConstraint_XPBD)
        .def("addFEMTetConstraint", &PBD::SimulationModel::addFEMTetConstraint)
        .def("addStrainTetConstraint", &PBD::SimulationModel::addStrainTetConstraint)
        .def("addShapeMatchingConstraint", [](
            PBD::SimulationModel& model,
            const unsigned int numberOfParticles, 
            const std::vector<unsigned int>& particleIndices,
            const std::vector<unsigned int>& numClusters,
            const Real stiffness)
            {
                model.addShapeMatchingConstraint(numberOfParticles, particleIndices.data(), numClusters.data(), stiffness);
            })


        .def("addStretchShearConstraint", &PBD::SimulationModel::addStretchShearConstraint)
        .def("addBendTwistConstraint", &PBD::SimulationModel::addBendTwistConstraint)
        .def("addStretchBendingTwistingConstraint", &PBD::SimulationModel::addStretchBendingTwistingConstraint)
        .def("addDirectPositionBasedSolverForStiffRodsConstraint", &PBD::SimulationModel::addDirectPositionBasedSolverForStiffRodsConstraint)
        
        .def("getParticles", &PBD::SimulationModel::getParticles, py::return_value_policy::reference)
        .def("getRigidBodies", &PBD::SimulationModel::getRigidBodies, py::return_value_policy::reference)
        .def("getTriangleModels", &PBD::SimulationModel::getTriangleModels, py::return_value_policy::reference)
        .def("getTetModels", &PBD::SimulationModel::getTetModels, py::return_value_policy::reference)
        .def("getLineModels", &PBD::SimulationModel::getLineModels, py::return_value_policy::reference)
        .def("getConstraints", &PBD::SimulationModel::getConstraints, py::return_value_policy::reference)
        .def("getOrientations", &PBD::SimulationModel::getOrientations, py::return_value_policy::reference)
        .def("getRigidBodyContactConstraints", &PBD::SimulationModel::getRigidBodyContactConstraints, py::return_value_policy::reference)
        .def("getParticleRigidBodyContactConstraints", &PBD::SimulationModel::getParticleRigidBodyContactConstraints, py::return_value_policy::reference)
        .def("getParticleSolidContactConstraints", &PBD::SimulationModel::getParticleSolidContactConstraints, py::return_value_policy::reference)
        .def("getConstraintGroups", &PBD::SimulationModel::getConstraintGroups, py::return_value_policy::reference)
        .def("resetContacts", &PBD::SimulationModel::resetContacts)

        .def("addClothConstraints", &PBD::SimulationModel::addClothConstraints)
        .def("addBendingConstraints", &PBD::SimulationModel::addBendingConstraints)
        .def("addSolidConstraints", &PBD::SimulationModel::addSolidConstraints)

        .def("getContactStiffnessRigidBody", &PBD::SimulationModel::getContactStiffnessRigidBody)
        .def("setContactStiffnessRigidBody", &PBD::SimulationModel::setContactStiffnessRigidBody)
        .def("getContactStiffnessParticleRigidBody", &PBD::SimulationModel::getContactStiffnessParticleRigidBody)
        .def("setContactStiffnessParticleRigidBody", &PBD::SimulationModel::setContactStiffnessParticleRigidBody)

        .def("addRigidBody", [](PBD::SimulationModel &model, const Real density, 
            const PBD::VertexData& vertices, 
            const Utilities::IndexedFaceMesh& mesh, 
            const Vector3r& x, const Real angle, const Vector3r &rotationAxis, 
            const Vector3r& scale, 
            const PBD::CubicSDFCollisionDetection::GridPtr sdf)
            {
                PBD::SimulationModel::RigidBodyVector& rbs = model.getRigidBodies();
                PBD::RigidBody *rb = new PBD::RigidBody();
                rb->initBody(density, x, Quaternionr(AngleAxisr(angle, rotationAxis)), vertices, mesh, scale);
                rbs.push_back(rb);
                if (sdf != nullptr)
                {
                    PBD::Simulation* sim = PBD::Simulation::getCurrent();
                    PBD::CubicSDFCollisionDetection* cd = dynamic_cast<PBD::CubicSDFCollisionDetection*>(sim->getTimeStep()->getCollisionDetection());
                    if (cd != nullptr)
                    {
                        const std::vector<Vector3r>* vertices = rb->getGeometry().getVertexDataLocal().getVertices();
                        const unsigned int nVert = static_cast<unsigned int>(vertices->size());
                        cd->addCubicSDFCollisionObject(rbs.size() - 1,
                            PBD::CollisionDetection::CollisionObject::RigidBodyCollisionObjectType,
                            vertices->data(), nVert, sdf, scale, true, false);
                    }
                }
                return rb;
            }, py::return_value_policy::reference)
        ;

}