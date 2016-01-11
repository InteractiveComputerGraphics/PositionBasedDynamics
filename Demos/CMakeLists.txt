include(Visualization/CMakeLists.txt)
add_definitions(-DPBD_DATA_PATH="../data")

subdirs(BarDemo ClothDemo FluidDemo RigidBodyDemos CouplingDemos GenericConstraintsDemos)

