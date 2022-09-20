# Getting started

This page should give you a short overview of PositionBasedDynamics.

## Demos

After building the PositionBasedDynamics library, there are several demos which can by executed in the binary folder. To execute a demo, you can call, e.g.:

```
cd ../bin
./BarDemo
```

## SceneLoaderDemo

This special demo reads a PositionBasedDynamics scene file and performs a simulation of the scene. You can start this simulator, e.g., by calling:

```
cd ../bin
./SceneLoaderDemo ../data/Scenes/CarScene.json
```

The scene file format is explained [here.](file_format.md)

##### Hotkeys

* Space: pause/contiunue simulation
* r: reset simulation
* w: wireframe rendering of meshes
* ESC: exit

## Python bindings 

PositionBasedDynamics implements bindings for python using [pybind11](https://github.com/pybind/pybind11).
See the [getting started guide](./py_getting_started.md).

### Impatient installation guide

In order to install, simply clone the repository and run pip install on the repository.
It is recommended, that you set up a **virtual environment** for this, because cache files will be stored in the directory of the python installation along with models and scene files.

```shell script
git clone https://github.com/InteractiveComputerGraphics/PositionBasedDynamics.git
pip install PositionBasedDynamics/
```





