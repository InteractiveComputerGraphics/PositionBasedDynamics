#ifndef PBD_COMMON_H
#define PBD_COMMON_H

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "Common/Common.h"
#include "Simulation/SimulationModel.h"

//PYBIND11_MAKE_OPAQUE(std::vector<Vector3r>)
PYBIND11_MAKE_OPAQUE(std::vector<PBD::TriangleModel*>)
PYBIND11_MAKE_OPAQUE(std::vector<PBD::TetModel*>)
PYBIND11_MAKE_OPAQUE(std::vector<PBD::Constraint*>)
PYBIND11_MAKE_OPAQUE(std::vector<PBD::RigidBody*>)

#endif //PBD_COMMON_H
