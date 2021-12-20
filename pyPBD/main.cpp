#include <pybind11/pybind11.h>

// Put this here for now
#include "common.h"

#include <Common/Common.h>
#include <Utils/Logger.h>
#include <Utils/Timing.h>
INIT_LOGGING
INIT_TIMING

namespace py = pybind11;

void CollisionDetectionModule(py::module);
void ConstraintsModule(py::module);
void ParameterObjectModule(py::module);
void ParticleDataModule(py::module);
void RigidBodyModule(py::module);
void SimulationModelModule(py::module);
void SimulationModule(py::module);
void TimeStepModule(py::module);
void TimeModule(py::module);
void UtilitiesModule(py::module);

PYBIND11_MODULE(pypbd, m) 
{
    CollisionDetectionModule(m);
    ParticleDataModule(m);
    RigidBodyModule(m);
    ParameterObjectModule(m); 
    SimulationModelModule(m);
    SimulationModule(m);
    ConstraintsModule(m);
    TimeStepModule(m);
    TimeModule(m);
    UtilitiesModule(m);
}