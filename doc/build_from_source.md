# Installation Instructions - Linux

## Ubuntu Fresh Install

### Installation List

```bash
sudo apt install git cmake xorg-dev freeglut3-dev build-essential
```

### Python Bindings

If you plan on using the python bindings by specifying `-DUSE_PYTHON_BINDINGS=On`, then you should also have a working python installation in your path. This installs an additional tool `pipx`, which allows the installation of packages as executables in virtualized environments.

```bash
sudo apt install python3-dev python3-pip python3-venv
python3 -m pip install pipx
python3 -m pipx ensurepath
```

Alternatively to this you may also install other Python Distributions such as Anaconda (personal preference).

### Building Instructions

```bash
git clone https://github.com/InteractiveComputerGraphics/PositionBasedDynamics.git
cd PositionBasedDynamics
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DUSE_PYTHON_BINDINGS=<On|Off> ..
make -j 4
```

### Run Executable

Run any demo in the folder `bin`, e.g.: 
```
cd ../bin
./SceneLoaderDemo ../data/Scenes/CarScene.json
```

On some systems it may be necessary to define an OpenGL override like so

```bash
cd ../bin
MESA_GL_VERSION_OVERRIDE=3.3 ./SceneLoaderDemo ../data/Scenes/CarScene.json
```

The command loads the selected scene. To start the simulation disable the pause mode by clicking the checkbox or pressing [Space]. More hotkeys are listed [here](getting_started.md).

### Using Bindings

Assuming that the python bindings were generated in the default location `Project Root/build/lib/pypbd.cpython-38-x86_64-linux-gnu.so`, you can use the bindings by adding this path to `sys.path` within your python script, or by calling your scripts within the directory containing the `.so` file. You can test that the bindings work using the following command.

```
cd lib
python3 -c "import pypbd"
```

### Installing Bindings

If you followed the above instructions for building PositionBasedDynamics using CMake and generated the python bindings, then these commands should work automatically. 

**Note:** You don't have to clone the repository again. This only shows, that the command should be run in the project root directory. It is also recommended, that you create and activate a virtual environment before installing, so that your base python installation is not affected by any new generated files. 

```bash
git clone https://github.com/InteractiveComputerGraphics/PositionBasedDynamics.git
cd PositionBasedDynamics
python setup.py bdist_wheel
pip install build/dist/*.whl
```

If you specified any additional CMake variables in the form of `-DVAR_NAME=Value`, you can just append them after `bdist_wheel`

Alternatively you may also run the following command, which essentially combines all of the above commands into a single command. 

```bash
pip install git+https://github.com/InteractiveComputerGraphics/PositionBasedDynamics.git
```

**Drawbacks:** You lose the ability for incremental rebuilds, i.e. if you want to modify the source code and build the bindings anew, you would have to build the entire project every time.

# Installation Instructions - Windows

## Visual Studio 

### Dependencies

To build PositionBasedDynamics on Windows you need to install [CMake](https://cmake.org) and [git](https://git-scm.com/).

### Python Bindings

If you plan on using the python bindings by specifying `-DUSE_PYTHON_BINDINGS=On`, then you should also have a working Python installation in your path. Moreover, you require the Python Package Installer (pip).

### Building Instructions

First, clone the repository by

```bash
git clone https://github.com/InteractiveComputerGraphics/PositionBasedDynamics.git
```

Then run cmake-gui and set "Where is the source code:" to the [PositionBasedDynamics-dir] and "Where to build the binaries:" to [PositionBasedDynamics-dir]/build.

Now run Configure and select the correct Visual Studio version. Ensure that you choose a x64 build on a 64bit system. Finally, run Generate and open the project. Now you can build the project in Visual Studio. Note that you have to select the "Release" build, if you want to have an optimized executable.


### Run Executable

Execute any demo in the folder "bin" to start the simulator. Alternatively, you can start the simulation by loading a json scene file in the command line:

```
./SceneLoaderDemo ../data/Scenes/CarScene.json
```

The command loads the selected scene. To start the simulation disable the pause mode by clicking the checkbox or pressing [Space]. More hotkeys are listed [here](getting_started.md).

### Using Bindings

Assuming that the python bindings were generated in the default location [PositionBasedDynamics-dir]/build/lib/pypbd.cp37-win_amd64.pyd, you can use the bindings by adding this path to `sys.path` within your python script, or by calling your scripts within the directory containing the `.pyd` file. You can test that the bindings work using the following command.

```
cd lib
python3 -c "import pypbd"
```

### Installing Bindings

If you followed the above instructions for building PositionBasedDynamics using CMake and generated the python bindings, then these commands should work automatically. 

**Note:** You don't have to clone the repository again. This only shows, that the command should be run in the project root directory. It is also recommended, that you create and activate a virtual environment before installing, so that your base python installation is not affected by any new generated files. 

```bash
git clone https://github.com/InteractiveComputerGraphics/PositionBasedDynamics.git
cd PositionBasedDynamics
python setup.py bdist_wheel
pip install build/dist/pyPBD-2.1.0-cp38-cp38-win_amd64.whl
```

If you specified any additional CMake variables in the form of `-DVAR_NAME=Value`, you can just append them after `bdist_wheel`

Alternatively you may also run the following command, which essentially combines all of the above commands into a single command. 

```bash
pip install git+https://github.com/InteractiveComputerGraphics/PositionBasedDynamics.git
```

**Drawbacks:** You lose the ability for incremental rebuilds, i.e. if you want to modify the source code and build the bindings anew, you would have to build the entire project every time.
