# PositionBasedDynamics

<p align=center><img src="https://github.com/InteractiveComputerGraphics/PositionBasedDynamics/workflows/build-linux/badge.svg">&nbsp;&nbsp; <img src="https://github.com/InteractiveComputerGraphics/PositionBasedDynamics/workflows/build-windows/badge.svg"></p>

This library supports the physically-based simulation of mechanical effects. In the last years position-based simulation methods have become popular in the graphics community. In contrast to classical simulation approaches these methods compute the position changes in each simulation step directly, based on the solution of a quasi-static problem. Therefore, position-based approaches are fast, stable and controllable which make them well-suited for use in interactive environments. However, these methods are generally not as accurate as force-based methods but still provide visual plausibility. Hence, the main application areas of position-based simulation are virtual reality, computer games and special effects in movies and commercials.

The PositionBasedDynamics library allows the position-based handling of many types of constraints in a physically-based simulation. The library uses [CMake](http://www.cmake.org/), [Eigen](http://eigen.tuxfamily.org/), [json](https://github.com/nlohmann/json/) and [AntTweakBar](http://anttweakbar.sourceforge.net/) (only for the demos). All external dependencies are included. 

Furthermore we use our own library:
- [Discregrid](https://github.com/InteractiveComputerGraphics/Discregrid/) to generate cubic signed distance fields for the collision detection


**Author**: [Jan Bender](http://www.interactive-graphics.de), **License**: MIT

## News

* Our new [paper](https://animation.rwth-aachen.de/publication/0557/) about a Direct Position-Based Solver for Stiff Rods uses the PositionBasedDynamics library. You can watch the video  [here](https://www.youtube.com/watch?v=EFH9xt4omls).
* PBD now has a collision detection based on cubic signed distance fields
* SPlisHSPlasH is our new open-source fluid simulator which uses the PositionBasedDynamics library to handle rigid-fluid coupling. It can be downloaded here:
[https://github.com/InteractiveComputerGraphics/SPlisHSPlasH](https://github.com/InteractiveComputerGraphics/SPlisHSPlasH)
* Our new [paper](http://interactive-graphics.de/index.php/research/98-hierarchical-hp-adaptive-signed-distance-fields) about adaptive signed distance fields uses the PositionBasedDynamics library. You can watch the video  [here](https://www.youtube.com/watch?v=x_Iq2yM4FcA).

## Build Instructions

This project is based on [CMake](https://cmake.org/). Simply generate project, Makefiles, etc. using [CMake](https://cmake.org/) and compile the project with the compiler of your choice. The code was tested with the following configurations:
- Windows 10 64-bit, CMake 3.9.5, Visual Studio 2017
- Debian 9 64-bit, CMake 3.12.3, GCC 6.3.0.

Note: Please use a 64-bit target on a 64-bit operating system. 32-bit builds on a 64-bit OS are not supported.

## Documentation

The API documentation can be found here: 

http://www.interactive-graphics.de/PositionBasedDynamics/doc/html

## Latest Important Changes

* added DamperJoint
* improved implementation of slider and hinge joints/motors
* Crispin Deul added the implementation of his paper Deul, Kugelstadt, Weiler, Bender, "Direct Position-Based Solver for Stiff Rods", Computer Graphics Forum 2018 and a corresponding demo
* added collision detection for arbitrary meshes based on cubic signed distance fields
* added implementation of the paper Kugelstadt, Schoemer, "Position and Orientation Based Cosserat Rods", SCA 2016
* removed Boost dependency
* added SceneGenerator.py to generate new scenarios easily by simple Python scripting
* added scene loader based on json 
* added collision detection based on distance functions
* added collision handling for rigid and deformable bodies
* high resolution visualization mesh can be attached to a deformable body
* added support for Mac OS X
* added automatic computation of inertia tensor for arbitrary triangle meshes
* added OBJ file loader
* parallelized unified solver using graph coloring
* implemented unified solver for rigid bodies and deformable solids 



## Features

* Physically-based simulation with position-based constraint handling.
* Simple interface
* Demos 
* Library is free even for commercial applications.
* Collision detection based on cubic signed distance fields
* Library supports many constraints: 
	- Elastic rods:
		- bend-twist constraint
		- stretch-shear constraint
		- Cosserat constraint
	- Deformable solids:		
		- point-point distance constraint
		- point-edge distance constraint
		- point-triangle distance constraint
		- edge-edge distance constraint
		- dihedral bending constraint
		- isometric bending constraint
		- volume constraint
		- shape matching
		- FEM-based PBD (2D & 3D)
		- strain-based dynamics (2D & 3D)
	- Fluids:
		- position-based fluids 
	- Rigid bodies:
		- contact constraints
		- ball joint
		- ball-on-line-joint
		- hinge joint
		- target angle motor hinge joint
		- target velocity motor hinge joint
		- universal joint
		- slider joint
		- target position motor slider joint
		- target velocity motor slider joint
		- ball joint between rigid body and particle
		- distance joint
		- damper joint
		- implicit spring
	- Generic constraints

## Videos

The following videos were generated using the PositionBasedDynamics library:

*Hierarchical hp-Adaptive Signed Distance Fields* | *Direct Position-Based Solver for Stiff Rods*
:---:|:---:
[![Video](https://img.youtube.com/vi/x_Iq2yM4FcA/0.jpg)](https://www.youtube.com/watch?v=x_Iq2yM4FcA) | [![Video](https://img.youtube.com/vi/EFH9xt4omls/0.jpg)](https://www.youtube.com/watch?v=EFH9xt4omls)


## Screenshots
		
![Cloth demo](http://www.interactive-graphics.de/j_images/PositionBasedDynamics.jpg "Cloth demo")	

## References

* J. Bender, M. Müller and M. Macklin, "Position-Based Simulation Methods in Computer Graphics", In Tutorial Proceedings of Eurographics, 2015
* J. Bender, D. Koschier, P. Charrier and D. Weber, ""Position-based simulation of continuous materials", Computers & Graphics 44, 2014
* J. Bender, M. Müller, M. A. Otaduy, M. Teschner and M. Macklin, "A Survey on Position-Based Simulation Methods in Computer Graphics", Computer Graphics Forum 33, 6, 2014
* C. Deul, T. Kugelstadt, M. Weiler, J. Bender, "Direct Position-Based Solver for Stiff Rods", Computer Graphics Forum, 2018
* C. Deul, P. Charrier and J. Bender, "Position-Based Rigid Body Dynamics", Computer Animation and Virtual Worlds, 2014
* D. Koschier, C. Deul, M. Brand and J. Bender, "An hp-Adaptive Discretization Algorithm for Signed Distance Field Generation", IEEE Transactions on Visualization and Computer Graphics 23, 2017
* M. Macklin, M. Müller, N. Chentanez and T.Y. Kim, "Unified particle physics for real-time applications", ACM Trans. Graph. 33, 4, 2014
* M. Müller, N. Chentanez, T.Y. Kim, M. Macklin, "Strain based dynamics", In Proceedings of the 2014 ACM
SIGGRAPH/Eurographics Symposium on Computer Animation, 2014
* J. Bender, D. Weber and R. Diziol, "Fast and stable cloth simulation based on multi-resolution shape matching", Computers & Graphics 37, 8, 2013
* R. Diziol, J. Bender and D. Bayer, "Robust Real-Time Deformation of Incompressible Surface Meshes", In Proceedings of ACM SIGGRAPH / EUROGRAPHICS Symposium on Computer Animation (SCA), 2011
* M. Müller and N. Chentanez, "Solid simulation with oriented particles", ACM Trans. Graph. 30, 4, 2011
* M. Müller, "Hierarchical Position Based Dynamics", In VRIPHYS 08: Fifth Workshop in Virtual Reality Interactions and Physical Simulations, 2008 
* M. Müller, B. Heidelberger, M. Hennix and J. Ratcliff, "Position based dynamics", Journal of Visual Communication and Image Representation 18, 2, 2007
* M. Müller, B. Heidelberger, M. Teschner and M. Gross, "Meshless deformations based on shape matching", ACM Trans. Graph. 24, 3, 2005
* M. Macklin and M. Müller, "Position based fluids", ACM Trans. Graph. 32, 4, 2013
* Dan Koschier, Crispin Deul and Jan Bender, "Hierarchical hp-Adaptive Signed Distance Fields", In Proceedings of ACM SIGGRAPH / EUROGRAPHICS Symposium on Computer Animation (SCA), 2016
* Tassilo Kugelstadt, Elmar Schoemer, "Position and Orientation Based Cosserat Rods", In Proceedings of ACM SIGGRAPH / EUROGRAPHICS Symposium on Computer Animation (SCA), 2016



