# pyPBD

## Python bindings for the PositionBasedDynamics library

## Requirements

Currently the generation of python bindings is only tested on 

- Linux Debian, gcc 8.3, Python 3.7/3.8 (Anaconda), CMake 3.13
- Windows 10, Visual Studio 15/17/19, Python 3.7/3.8 (Anaconda), CMake 3.13

Note that the compiler, the python installation as well as cmake have to be available from the command line for the installation process to work. MacOS builds should work but have not been tested.

## Installation

In order to install it is advised that you create a new virtual environment so that any faults during installation can not mess up your python installation. This is done as follows for 

**conda**

```shell script
conda create --name venv python=3.7
conda activate venv
```

**virtualenv**

```shell script
python3 -m virtualenv venv --python=python3.7
source venv/bin/activate
```

Now you can clone the repository by

```shell script
git clone https://github.com/InteractiveComputerGraphics/PositionBasedDynamics.git
```

And finally you should be able to install PositionBasedDynamics using pip. 
**The trailing slash is important** otherwise pip will try to download the package, which is not supported yet at least.
Also note, that `pip install PositionBasedDynamics` should be called from **one directory above** the cloned source directory and **not within** the directory itself.

```shell script
pip install PositionBasedDynamics/
```

While `pip install` is useful if PositionBasedDynamics should only be installed once, for development purposes it might be more sensible to build differently.
Change into the PositionBasedDynamics directory and build a python wheel file as follows

```shell script
cd PositionBasedDynamics
python setup.py bdist_wheel
pip install -I build/dist/*.whl
```

When building a new version of PositionBasedDynamics simply run these commands again and the installation will be updated.
The compile times will be lower, because the build files from previous installations remain. 
If you are getting compile errors please try to compile the pypbd target of the CMake project separately.

Now check your installation by running

```shell script
python -c "import pypbd"
```

**Note**: You may have to install numpy. 
Future releases may already contain numpy as a dependency.

```shell script
pip install numpy
```

## Examples

In the folder "pyPBD/examples" you can find several examples which use the Python interface. 

