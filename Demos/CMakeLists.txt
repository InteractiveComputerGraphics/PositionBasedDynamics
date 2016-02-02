include(Visualization/CMakeLists.txt)
add_definitions(-DPBD_DATA_PATH="../data")

subdirs(BarDemo ClothDemo ElasticRodDemos FluidDemo RigidBodyDemos CouplingDemos GenericConstraintsDemos)

