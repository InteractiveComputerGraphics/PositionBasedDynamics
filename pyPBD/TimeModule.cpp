#include "common.h"

#include <Simulation/TimeManager.h>

#include <pybind11/pybind11.h>

namespace py = pybind11;

void TimeModule(py::module m_sub) {
    // ---------------------------------------
    // Class Time Manager
    // ---------------------------------------
    py::class_<PBD::TimeManager>(m_sub, "TimeManager")
            .def(py::init<>())
            .def_static("getCurrent", &PBD::TimeManager::getCurrent, py::return_value_policy::reference)
            .def_static("setCurrent", &PBD::TimeManager::setCurrent)
            .def_static("hasCurrent", &PBD::TimeManager::hasCurrent)

            .def("getTime", &PBD::TimeManager::getTime)
            .def("setTime", &PBD::TimeManager::setTime)
            .def("getTimeStepSize", &PBD::TimeManager::getTimeStepSize)
            .def("setTimeStepSize", &PBD::TimeManager::setTimeStepSize);
}