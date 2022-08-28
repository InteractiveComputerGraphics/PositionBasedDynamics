import os
import re
import sys
import platform
import subprocess
import multiprocessing as mp
import argparse

from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion

# Extract cmake arguments
parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("-D", action='append', dest='cmake',
                    help="CMake Options")
parser.add_argument("--manylinux-build", action='store_true', dest='manylinux_build')
args, other_args = parser.parse_known_args(sys.argv)
cmake_clargs = args.cmake
sys.argv = other_args

# Project binding name
name = "pyPBD"
internal_name = "pypbd"


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        bin_dir_windows = os.path.join(os.path.abspath(self.build_temp), "bin")
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        # Add cmake command line arguments
        if cmake_clargs is not None:
            cmake_args += ['-D{}'.format(arg) for arg in cmake_clargs]

        if platform.system() == "Windows":
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir),
                           '-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=' + bin_dir_windows,
                           '-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), bin_dir_windows)]
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j{}'.format(mp.cpu_count())]

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())

        # Add position independent code flags if using gcc on linux probably
        if platform.system() == "Linux":
            cmake_args += ['-DCMAKE_CXX_FLAGS=-fPIC', '-DCMAKE_C_FLAGS=-fPIC']

            # Using relative rpath messes up repairing the wheel file. The relative rpath is only necessary when
            # building locally from source
            if not args.manylinux_build:
                cmake_args += ['-DCMAKE_INSTALL_RPATH={}'.format("$ORIGIN"),
                               '-DCMAKE_BUILD_WITH_INSTALL_RPATH:BOOL=ON',
                               '-DCMAKE_INSTALL_RPATH_USE_LINK_PATH:BOOL=OFF']

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        if not args.manylinux_build:
            subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
            subprocess.check_call(['cmake', '--build', '.', '--target', internal_name] + build_args, cwd=self.build_temp)
        else:
            subprocess.check_call(['cmake3', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
            subprocess.check_call(['cmake3', '--build', '.', '--target', internal_name] + build_args, cwd=self.build_temp)


# Get Readme text for long description
cur_dir = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(cur_dir, "README.md"), 'r') as f:
    long_description = f.read()
	
# read version	
f = open("version.txt", "r")
pbd_version = f.readline().strip()
f.close() 

setup(
    name=name,
    version=pbd_version,
    author='Interactive Computer Graphics',
    author_email='',
    description='PBD Project Python Bindings',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/InteractiveComputerGraphics/PositionBasedDynamics',
    license="MIT",
    keywords="simulation rigid-bodies rigid-body-dynamics position-based-dynamics deformable-solids",
    ext_modules=[CMakeExtension(name)],
    cmdclass=dict(build_ext=CMakeBuild),
    packages=find_packages(),
    zip_safe=False,
    install_requires=['numpy']
)