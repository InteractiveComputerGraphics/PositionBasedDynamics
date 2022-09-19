# About PositionBasedDynamics

PositionBasedDynamics is an open-source library which enables the physically-based simulation of mechanical effects. In the last years position-based simulation methods have become popular in the graphics community. In contrast to classical simulation approaches these methods compute the position changes in each simulation step directly, based on the solution of a quasi-static problem. Therefore, position-based approaches are fast, stable and controllable which make them well-suited for use in interactive environments. However, these methods are generally not as accurate as force-based methods but still provide visual plausibility. Hence, the main application areas of position-based simulation are virtual reality, computer games and special effects in movies and commercials.


## Main features

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
		- point-point distance constraint (PBD & XPBD)
		- point-edge distance constraint
		- point-triangle distance constraint
		- edge-edge distance constraint
		- dihedral bending constraint
		- isometric bending constraint (PBD & XPBD)
		- volume constraint (PBD & XPBD)
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
- 
## License

The MIT License (MIT)

Copyright (c) 2015-present, PositionBasedDynamics contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.