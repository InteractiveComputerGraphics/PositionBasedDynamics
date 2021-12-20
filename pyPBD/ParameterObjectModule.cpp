//
// Created by sjeske on 1/22/20.
//
#include "common.h"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>

#include <Common/Common.h>
#include <ParameterObject.h>

namespace py = pybind11;

void ParameterObjectModule(py::module m_sub){
    //auto m_sub = m.def_submodule("Common");
    py::class_<GenParam::ParameterObject>(m_sub, "ParameterObject")
            // .def(py::init<>()) TODO: no constructor for now because this object does not need to be constructable
            .def("getValueBool", &GenParam::ParameterObject::getValue<bool>)
            .def("getValueInt", &GenParam::ParameterObject::getValue<int>)
            .def("getValueUInt", &GenParam::ParameterObject::getValue<unsigned int>)
            .def("getValueFloat", &GenParam::ParameterObject::getValue<Real>)
            .def("getValueString", &GenParam::ParameterObject::getValue<std::string>)

            .def("setValueBool", &GenParam::ParameterObject::setValue<bool>)
            .def("setValueInt", &GenParam::ParameterObject::setValue<int>)
            .def("setValueUInt", &GenParam::ParameterObject::setValue<unsigned int>)
            .def("setValueFloat", &GenParam::ParameterObject::setValue<Real>)
            .def("setValueString", &GenParam::ParameterObject::setValue<std::string>);
}
