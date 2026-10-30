# CMake Options

This page should give you a short overview over the CMake options of PositionBasedDynamics.

## USE_DOUBLE_PRECISION

If this flag is enabled, then all computations with floating point values are performed using double precision (double). Otherwise single precision (float) is used.

## USE_OpenMP

Enable the OpenMP parallelization which lets the simulation run in parallel on all available cores of the CPU. 

## USE_PYTHON_BINDINGS

Generate a shared library object which can be imported into python scripts and exposes C++ functionality to the python interpreter.
*Default:On*
*Options:<On|Off>*

## PBD_BUILD_TESTS

Build regression tests and register them with CTest. Disabled by default.
To run the tests without building the demos:

```sh
mkdir build-tests
cd build-tests
cmake .. -DPBD_LIBS_ONLY=ON -DPBD_BUILD_TESTS=ON
cmake --build . --target StiffRodsStaticSegmentsTest --config Release
ctest -C Release --output-on-failure
```
