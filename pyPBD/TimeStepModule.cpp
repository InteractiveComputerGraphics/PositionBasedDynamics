#include "common.h"

#include <Simulation/TimeStep.h>
#include <Simulation/TimeStepController.h>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

void TimeStepModule(py::module m_sub)
{
    py::class_<PBD::TimeStep, GenParam::ParameterObject>(m_sub, "TimeStep")
        //.def(py::init<>())
        .def("step", &PBD::TimeStep::step)
        .def("reset", &PBD::TimeStep::reset)
        .def("init", &PBD::TimeStep::init)
        .def("setCollisionDetection", &PBD::TimeStep::setCollisionDetection)
        .def("getCollisionDetection", &PBD::TimeStep::getCollisionDetection, py::return_value_policy::reference_internal);

    py::class_<PBD::TimeStepController, PBD::TimeStep>(m_sub, "TimeStepController")
        .def_readwrite_static("NUM_SUB_STEPS", &PBD::TimeStepController::NUM_SUB_STEPS)
        .def_readwrite_static("MAX_ITERATIONS", &PBD::TimeStepController::MAX_ITERATIONS)
        .def_readwrite_static("MAX_ITERATIONS_V", &PBD::TimeStepController::MAX_ITERATIONS_V)
        .def_readwrite_static("VELOCITY_UPDATE_METHOD", &PBD::TimeStepController::VELOCITY_UPDATE_METHOD)
        .def_readwrite_static("ENUM_VUPDATE_FIRST_ORDER", &PBD::TimeStepController::ENUM_VUPDATE_FIRST_ORDER)
        .def_readwrite_static("ENUM_VUPDATE_SECOND_ORDER", &PBD::TimeStepController::ENUM_VUPDATE_SECOND_ORDER)

        .def(py::init<>());
}