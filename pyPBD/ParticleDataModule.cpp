#include "common.h"

#include <Simulation/SimulationModel.h>
#include "Simulation/TimeStepController.h"
#include "Simulation/TimeManager.h"
#include "PositionBasedDynamics/TimeIntegration.h"

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>

namespace py = pybind11;


void ParticleDataModule(py::module m_sub) 
{
    py::class_<PBD::VertexData>(m_sub, "VertexData")
        .def(py::init<>())
        .def("addVertex", &PBD::VertexData::addVertex)
        .def("getPosition", (const Vector3r & (PBD::VertexData::*)(const unsigned int)const)(&PBD::VertexData::getPosition))
        .def("setPosition", &PBD::VertexData::setPosition)
        .def("resize", &PBD::VertexData::resize)
        .def("reserve", &PBD::VertexData::reserve)
        .def("release", &PBD::VertexData::release)
        .def("size", &PBD::VertexData::size)
        //.def("getVertices", &PBD::VertexData::getVertices, py::return_value_policy::reference);
        .def("getVertices", [](PBD::VertexData& vd) -> py::memoryview {
            void* base_ptr = const_cast<Real*>(&(*vd.getVertices())[0][0]);
            int num_vert = vd.size();
            return py::memoryview::from_buffer((Real*)base_ptr, { num_vert, 3 }, { sizeof(Real) * 3, sizeof(Real) }, true);
            });

    py::class_<PBD::ParticleData>(m_sub, "ParticleData")
        .def(py::init<>())
        .def("addVertex", &PBD::ParticleData::addVertex)
        .def("getPosition", (const Vector3r & (PBD::ParticleData::*)(const unsigned int)const)(&PBD::ParticleData::getPosition))
        .def("setPosition", &PBD::ParticleData::setPosition)
        .def("getPosition0", (const Vector3r & (PBD::ParticleData::*)(const unsigned int)const)(&PBD::ParticleData::getPosition0))
        .def("setPosition0", &PBD::ParticleData::setPosition0)
        .def("getMass", (const Real(PBD::ParticleData::*)(const unsigned int)const)(&PBD::ParticleData::getMass))
        .def("getInvMass", &PBD::ParticleData::getInvMass)
        .def("setMass", &PBD::ParticleData::setMass)
        .def("getVelocity", (const Vector3r & (PBD::ParticleData::*)(const unsigned int)const)(&PBD::ParticleData::getVelocity))
        .def("setVelocity", &PBD::ParticleData::setVelocity)
        .def("getAcceleration", (const Vector3r & (PBD::ParticleData::*)(const unsigned int)const)(&PBD::ParticleData::getAcceleration))
        .def("setAcceleration", &PBD::ParticleData::setAcceleration)
        .def("getNumberOfParticles", &PBD::ParticleData::getNumberOfParticles)
        .def("resize", &PBD::ParticleData::resize)
        .def("reserve", &PBD::ParticleData::reserve)
        .def("release", &PBD::ParticleData::release)
        .def("size", &PBD::ParticleData::size)
        //.def("getVertices", &PBD::ParticleData::getVertices, py::return_value_policy::reference);
        .def("getVertices", [](PBD::ParticleData &pd) -> py::memoryview {
            void* base_ptr = const_cast<Real*>(&(*pd.getVertices())[0][0]);
            int num_vert = pd.getNumberOfParticles();
            return py::memoryview::from_buffer((Real*)base_ptr, { num_vert, 3 }, { sizeof(Real) * 3, sizeof(Real) }, true);
            });
}