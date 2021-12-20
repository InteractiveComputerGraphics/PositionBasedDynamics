#include "common.h"

#include <Simulation/Simulation.h>
#include <Simulation/CubicSDFCollisionDetection.h>

#include <pybind11/pybind11.h>

namespace py = pybind11;

void SimulationModule(py::module m_sub) {
    // ---------------------------------------
    // Class Simulation
    // ---------------------------------------
    py::class_<PBD::Simulation, GenParam::ParameterObject>(m_sub, "Simulation")
        .def(py::init<>())
        .def_static("getCurrent", &PBD::Simulation::getCurrent, py::return_value_policy::reference)
        .def_static("setCurrent", &PBD::Simulation::setCurrent)
        .def_static("hasCurrent", &PBD::Simulation::hasCurrent)

        .def("init", &PBD::Simulation::init)
        .def("reset", &PBD::Simulation::reset)
        .def("getModel", &PBD::Simulation::getModel, py::return_value_policy::reference_internal)
        .def("setModel", &PBD::Simulation::setModel)
        .def("getTimeStep", &PBD::Simulation::getTimeStep, py::return_value_policy::reference_internal)
        .def("setTimeStep", &PBD::Simulation::setTimeStep)
        .def("initDefault", [](PBD::Simulation &sim)
            {
                sim.setModel(new PBD::SimulationModel());
                sim.getModel()->init();
                PBD::CubicSDFCollisionDetection* cd = new PBD::CubicSDFCollisionDetection();
                sim.getTimeStep()->setCollisionDetection(*sim.getModel(), cd);
            })
        ;
}