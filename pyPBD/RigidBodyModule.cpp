#include "common.h"

#include <Simulation/RigidBody.h>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

#define GET_QUAT_FCT_CONVERT(name) .def(#name, [](PBD::RigidBody& obj) { return obj.name().coeffs(); })
#define SET_QUAT_FCT_CONVERT(name) \
    .def(#name, [](PBD::RigidBody& obj, const Vector4r& qVec) \
        { \
            Quaternionr q; \
            q.coeffs() = qVec; \
            obj.name(q); \
        })

void RigidBodyModule(py::module m_sub)
{
    py::class_<PBD::RigidBodyGeometry>(m_sub, "RigidBodyGeometry")
        .def(py::init<>())
        .def("getMesh", &PBD::RigidBodyGeometry::getMesh)
        .def("getVertexData", (const PBD::VertexData & (PBD::RigidBodyGeometry::*)()const)(&PBD::RigidBodyGeometry::getVertexData))
        .def("getVertexDataLocal", (const PBD::VertexData & (PBD::RigidBodyGeometry::*)()const)(&PBD::RigidBodyGeometry::getVertexDataLocal))
        .def("initMesh", &PBD::RigidBodyGeometry::initMesh)
        .def("updateMeshTransformation", &PBD::RigidBodyGeometry::updateMeshTransformation)
        .def("updateMeshNormals", &PBD::RigidBodyGeometry::updateMeshNormals)
        ;

    py::class_<PBD::RigidBody>(m_sub, "RigidBody")
        .def(py::init<>())
        .def("initBody", [](PBD::RigidBody& obj, const Real density, 
            const Vector3r& x, const Vector4r& qVec,
            const PBD::VertexData& vertices, const Utilities::IndexedFaceMesh& mesh, 
            const Vector3r& scale) 
            {
                Quaternionr q;
                q.coeffs() = qVec;
                obj.initBody(density, x, q, vertices, mesh, scale);
            })
        .def("initBody", [](PBD::RigidBody& obj, const Real mass,
            const Vector3r& x, const Vector3r& inertiaTensor, const Vector4r& qVec,
            const PBD::VertexData& vertices, const Utilities::IndexedFaceMesh& mesh,
            const Vector3r& scale)
            {
                Quaternionr q;
                q.coeffs() = qVec;
                obj.initBody(mass, x, inertiaTensor, q, vertices, mesh, scale);
            })
        .def("reset", &PBD::RigidBody::reset)
        .def("updateInverseTransformation", &PBD::RigidBody::updateInverseTransformation)
        .def("rotationUpdated", &PBD::RigidBody::rotationUpdated)
        .def("updateInertiaW", &PBD::RigidBody::updateInertiaW)
        .def("determineMassProperties", &PBD::RigidBody::determineMassProperties)
        .def("getTransformationR", &PBD::RigidBody::getTransformationR)
        .def("getTransformationV1", &PBD::RigidBody::getTransformationV1)
        .def("getTransformationV2", &PBD::RigidBody::getTransformationV2)
        .def("getTransformationRXV1", &PBD::RigidBody::getTransformationRXV1)
        .def("getMass", (const Real&(PBD::RigidBody::*)()const)(&PBD::RigidBody::getMass))
        .def("setMass", &PBD::RigidBody::setMass)
        .def("getInvMass", &PBD::RigidBody::getInvMass)
        .def("getPosition", (const Vector3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getPosition))
        .def("setPosition", &PBD::RigidBody::setPosition)
        .def("getLastPosition", (const Vector3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getLastPosition))
        .def("setLastPosition", &PBD::RigidBody::setLastPosition)
        .def("getOldPosition", (const Vector3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getOldPosition))
        .def("setOldPosition", &PBD::RigidBody::setOldPosition)
        .def("getPosition0", (const Vector3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getPosition0))
        .def("setPosition0", &PBD::RigidBody::setPosition0)
        .def("getPositionInitial_MAT", (const Vector3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getPositionInitial_MAT))
        .def("setPositionInitial_MAT", &PBD::RigidBody::setPositionInitial_MAT)
        .def("getVelocity", (const Vector3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getVelocity))
        .def("setVelocity", &PBD::RigidBody::setVelocity)
        .def("getVelocity0", (const Vector3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getVelocity0))
        .def("setVelocity0", &PBD::RigidBody::setVelocity0)
        .def("getAcceleration", (const Vector3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getAcceleration))
        .def("setAcceleration", &PBD::RigidBody::setAcceleration)
        .def("getInertiaTensor", &PBD::RigidBody::getInertiaTensor)
        .def("setInertiaTensor", &PBD::RigidBody::setInertiaTensor)
        .def("getInertiaTensorW", (const Matrix3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getInertiaTensorW))
        .def("getInertiaTensorInverse", &PBD::RigidBody::getInertiaTensorInverse)
        .def("getInertiaTensorInverseW", (const Matrix3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getInertiaTensorInverseW))
        .def("setInertiaTensorInverseW", &PBD::RigidBody::setInertiaTensorInverseW)
        GET_QUAT_FCT_CONVERT(getRotation)
        SET_QUAT_FCT_CONVERT(setRotation)
        GET_QUAT_FCT_CONVERT(getLastRotation)
        SET_QUAT_FCT_CONVERT(setLastRotation)
        GET_QUAT_FCT_CONVERT(getOldRotation)
        SET_QUAT_FCT_CONVERT(setOldRotation)
        GET_QUAT_FCT_CONVERT(getRotation0)
        SET_QUAT_FCT_CONVERT(setRotation0)
        GET_QUAT_FCT_CONVERT(getRotationMAT)
        SET_QUAT_FCT_CONVERT(setRotationMAT)
        GET_QUAT_FCT_CONVERT(getRotationInitial)
        SET_QUAT_FCT_CONVERT(setRotationInitial)
        .def("getRotationMatrix", (const Matrix3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getRotationMatrix))
        .def("setRotationMatrix", &PBD::RigidBody::setRotationMatrix)
        .def("getAngularVelocity", (const Vector3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getAngularVelocity))
        .def("setAngularVelocity", &PBD::RigidBody::setAngularVelocity)
        .def("getAngularVelocity0", (const Vector3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getAngularVelocity0))
        .def("setAngularVelocity0", &PBD::RigidBody::setAngularVelocity0)
        .def("getTorque", (const Vector3r & (PBD::RigidBody::*)()const)(&PBD::RigidBody::getTorque))
        .def("setTorque", &PBD::RigidBody::setTorque)
        .def("getRestitutionCoeff", &PBD::RigidBody::getRestitutionCoeff)
        .def("setRestitutionCoeff", &PBD::RigidBody::setRestitutionCoeff)
        .def("getFrictionCoeff", &PBD::RigidBody::getFrictionCoeff)
        .def("setFrictionCoeff", &PBD::RigidBody::setFrictionCoeff)
        .def("getGeometry", &PBD::RigidBody::getGeometry)
    ;

}