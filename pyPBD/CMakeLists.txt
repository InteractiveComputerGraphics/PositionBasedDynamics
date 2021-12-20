pybind11_add_module(pypbd
  main.cpp

  bind_pointer_vector.h
  common.h
  CollisionDetectionModule.cpp
  ConstraintsModule.cpp
  ParameterObjectModule.cpp
  ParticleDataModule.cpp
  RigidBodyModule.cpp
  SimulationModelModule.cpp
  SimulationModule.cpp
  TimeModule.cpp
  TimeStepModule.cpp
  UtilitiesModule.cpp
)

# All the same postfix, otherwise the module name ist not the same as the exported name and python gets confused
set_target_properties(pypbd PROPERTIES
        DEBUG_POSTFIX ""
        RELWITHDEBINFO_POSTFIX ""
        MINSIZEREL_POSTFIX "")

target_link_libraries(pypbd PRIVATE Simulation PositionBasedDynamics Utils ${Discregrid_LIBRARIES})
add_dependencies(pypbd Simulation PositionBasedDynamics Utils Ext_GenericParameters Ext_Discregrid)

add_custom_target(pypbd_install
        ${PYTHON_EXECUTABLE} setup.py bdist_wheel
        COMMAND ${PYTHON_EXECUTABLE} -m pip install -I build/dist/py*.whl
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
		
