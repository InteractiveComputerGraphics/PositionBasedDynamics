# PositionBasedDynamics

This library supports the physically-based simulation of mechanical effects. In the last years position-based simulation methods have become popular in the graphics community. In contrast to classical simulation approaches these methods compute the position changes in each simulation step directly, based on the solution of a quasi-static problem. Therefore, position-based approaches are fast, stable and controllable which make them well-suited for use in interactive environments. However, these methods are generally not as accurate as force-based methods but still provide visual plausibility. Hence, the main application areas of position-based simulation are virtual reality, computer games and special effects in movies and commercials.

The PositionBasedDynamics library allows the position-based handling of many types of constraints in a physically-based simulation. The library uses [CMake](http://www.cmake.org/), [Eigen](http://eigen.tuxfamily.org/) and [AntTweakBar](http://anttweakbar.sourceforge.net/) (only for the demos). All external dependencies are included. 

The library was tested on Windows 7 and OpenSuse Linux 13.2. However, it should also be possible to use it on Mac OS X systems. 

**Author**: [Jan Bender](http://www.interactive-graphics.de), **License**: MIT

## Documentation

The API documentation can be found here: 

http://www.interactive-graphics.de/PositionBasedDynamics/doc/html

## Latest Important Changes

* added target velocity motor slider joint
* added target position motor slider joint
* added slider joint
* added target velocity motor hinge joint 
* added target angle motor hinge joint
* parallelized unified solver using graph coloring
* implemented unified solver for rigid bodies and deformable solids 
* added generic constraint demo
* added generic constraint
* added coupling demo
* added ball joint which links a rigid body and a particle
* added new joint demo

## Features

* Physically-based simulation with position-based constraint handling.
* Simple interface
* Demos 
* Library is free even for commercial applications.
* Library supports many constraints: 
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
	- Generic constraints

## Screenshots		
		
![Cloth demo](http://www.interactive-graphics.de/j_images/PositionBasedDynamics.jpg "Cloth demo")	

## References

* J. Bender, M. Müller and M. Macklin, "Position-Based Simulation Methods in Computer Graphics", In Tutorial Proceedings of Eurographics, 2015
* J. Bender, D. Koschier, P. Charrier and D. Weber, ""Position-based simulation of continuous materials", Computers & Graphics 44, 2014
* J. Bender, M. Müller, M. A. Otaduy, M. Teschner and M. Macklin, "A Survey on Position-Based Simulation Methods in Computer Graphics", Computer Graphics Forum 33, 6, 2014
* C. Deul, P. Charrier and J. Bender, "Position-Based Rigid Body Dynamics", Computer Animation and Virtual Worlds, 2014
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



