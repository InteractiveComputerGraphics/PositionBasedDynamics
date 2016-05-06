include(Visualization/CMakeLists.txt)
add_definitions(-DPBD_DATA_PATH="../data")

subdirs(Simulation BarDemo ClothDemo DistanceFieldDemos FluidDemo RigidBodyDemos CouplingDemos GenericConstraintsDemos SceneLoaderDemo)

