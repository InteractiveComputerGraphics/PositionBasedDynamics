include(Visualization/CMakeLists.txt)
add_definitions(-DPBD_DATA_PATH="../data")

subdirs(Simulation BarDemo ClothDemo ElasticRodDemos DistanceFieldDemos FluidDemo RigidBodyDemos CouplingDemos GenericConstraintsDemos SceneLoaderDemo)

