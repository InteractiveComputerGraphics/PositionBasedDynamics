# PositionBasedDynamics Scene Files

A PositionBasedDynamics scene file is a json file which can contain the following blocks:

* Name
* Camera parameters
* Simulation
* TriangleModels
* TetModels
* RigidBodies
* Joints


## Name 

Each scene has a name.

Example code:
```json
"Name": "ExampleScene"
```

## Camera parameters

Here parameters for the initial camera position can be defined.

* cameraPosition (vec3): Initial position of the camera.
* cameraLookat (vec3): Lookat point of the camera.

## Simulation

This part contains the general settings of the simulation. 

Example code:
```json
"Simulation": 
	{
		"pause": false,
		"pauseAt": 10.0,
    "timeStepSize": 0.01,
		"numberOfStepsPerRenderUpdate": 4,
		"subSteps": 5,
		"maxIterations" : 1,
		"maxIterationsV" : 5,
		"velocityUpdateMethod" : 0,
		"contactTolerance": 0.0,
		"tetModelSimulationMethod": 2,
		"clothSimulationMethod": 2,
		"clothBendingMethod": 2,
		"contactStiffnessRigidBody" : 1.0,
		"contactStiffnessParticleRigidBody": 100.0,
		"cloth_stiffness": 1.0,
		"cloth_bendingStiffness": 0.005,
		"cloth_xxStiffness": 1.0,
		"cloth_yyStiffness": 1.0,
		"cloth_xyStiffness": 1.0,
		"cloth_xyPoissonRatio": 0.3,
		"cloth_yxPoissonRatio": 0.3,
		"cloth_normalizeStretch": 0,
		"cloth_normalizeShear": 0, 
		"solid_stiffness": 1.0,
		"solid_poissonRatio": 0.2,
		"solid_normalizeStretch": 0,
		"solid_normalizeShear": 0
	}
```

##### General:

* pause (bool): Pause simulation at beginning (default: true).
* pauseAt (float): Pause simulation at the given time. When the value is negative, the simulation is not paused (default: -1).

##### Simulation:

* timeStepSize (float): The time step size used for the time integration. (default: 0.005).
* gravitation (vec3): Vector to define the gravitational acceleration (default: [0,-9.81,0]).
* velocityUpdateMethod (int) (default: 1): 
  - 0: First Order Update
  - 1: Second Order Update
* maxIterations (int): Number of iterations of the PBD solver (default: 1).
* maxIterationsV (int): Number of iterations of the velocity solver (default: 5).
* subSteps (int): Number of sub steps of the PBD solver (default: 5).


##### Cloth Simulation

* clothSimulationMethod (int): Cloth simulation method (default: 2):
  - 0: None
	- 1: Distance constraints
	- 2: FEM based PBD
	- 3: Strain based dynamics
	- 4: XPBD distance constraints
*	clothBendingMethod (int): Cloth bending method (default: 2):
  - 0: None
  - 1: Dihedral angle
  - 2: Isometric bending
  - 3: Isometric bending (XPBD)
* cloth_stiffness (float): Stiffness coefficient for the distance constraints (if there are any in the simulation) (default: 1.0).
* cloth_xxStiffness (float): XX stiffness of orthotropic cloth models (only used for FEM based PBD) (default: 1.0).
* cloth_yyStiffness (float): YY stiffness of orthotropic cloth models (only used for FEM based PBD) (default: 1.0).
* cloth_xyPoissonRatio (float): XY stiffness of orthotropic cloth models (only used for FEM based PBD) (default: 1.0).
* cloth_xyStiffness (float): XY Poisson ratio of orthotropic cloth models (only used for FEM based PBD) (default: 0.3).
* cloth_yyStiffness (float): YX Poisson ratio of orthotropic cloth models (only used for FEM based PBD) (default: 0.3).
* cloth_bendingStiffness (float): Bending stiffness of cloth models (default: 0.01).
* cloth_normalizeStretch (bool): Normalize stretch (only used for strain based dynamics) (default: false).
* cloth_normalizeShear (bool): Normalize shear (only used for strain based dynamics) (default: false).

##### Solid Simulation

* solidSimulationMethod (int): Solid simulation method (default: 2):
  - 0: None
	- 1: Distance & volume constraints
	- 2: FEM based PBD
  - 3: FEM based XPBD
	- 4: Strain based dynamics (no inversion handling)
  - 5: Shape Matching (no inversion handling)
	- 6: XPBD distance constraints
* solid_stiffness (float): Stiffness/Young's modulus of solid models (default: 1.0).
* solid_poissonRatio (float): Poisson ratio of solid models (only used for FEM based PBD/XPBD) (default: 0.3).
* solid_volumeStiffness (float): Stiffness coefficient for the volume constraints (if there are any in the simulation) (default: 1.0).
* solid_normalizeStretch (bool): Normalize stretch (only used for strain based dynamics) (default: false).
* solid_normalizeShear (bool): Normalize shear (only used for strain based dynamics) (default: false).

##### Rod Simulation

* rod_stretchingStiffness (float): Stretching stiffness/Youngs modulus of elastic rod models (default: 1.0).
* rod_shearingStiffnessX (float): X shearing stiffness of elastic rod models (default: 1.0).
* rod_shearingStiffnessY (float): Y Shearing stiffness of elastic rod models (default: 1.0).
* rod_bendingStiffnessX (float): X bending stiffness of elastic rod models (default: 0.5).
* rod_bendingStiffnessY (float): Y bending stiffness of elastic rod models (default: 0.5).
* rod_twistingStiffness (float): Twisting stiffness of elastic rod models (default: 0.5).

##### Contact Handling

* contactTolerance (float): Tolerance of the collision detection (default: 0.01).
* contactStiffnessRigidBody (float): Stiffness coefficient for rigid-rigid contact resolution (default: 1.0).
* contactStiffnessParticleRigidBody (float): Stiffness coefficient for particle-rigid contact resolution (default: 100.0).

##### Visualization:

* numberOfStepsPerRenderUpdate (int): Number of simulation steps per rendered frame
* renderTets (bool): Render tet models (default: false).
* renderTets0 (bool): Render initial state of tet models (default: false).
* renderContacts (bool): Render contact points and normals (default: false).
* renderAABB (bool): Render axis-aligned bounding boxes (default: false).
* renderSDF (bool): Render signed distance fields (SDFs) (default: false).
* renderBVH (int): Render bounding volume hierarchies (BVHs) until given depth (default: -1).
* renderBVHDepthTets (int): Render bounding volume hierarchies (BVHs) of tets until given depth (default: -1).

##### Export

* exportOBJ (bool): Enable/disable OBJ file export (default: false).
* exportPLY (bool): Enable/disable PLY file export (default: false).
* exportFPS (float): Frame rate of file export (default: 25).



## Triangle Models

The simulation library supports triangle models to simulate cloth. In the following example a triangle mesh is loaded to simulate a quadratic piece of cloth where two particles are fixed. 

Example code:
```json
"TriangleModels": [
  {
    "id": 0,
    "geometryFile": "../models/plane_50x50.obj",
    "translation": [5,8,0],
    "rotationAxis": [1, 0, 0],
    "rotationAngle": 0.0,
    "scale": [10, 10, 10],
    "staticParticles": [0,49],
    "restitution" : 0.1,
    "friction" : 0.1
  }
]
```

* id (int): ID of the triangle model (default: 0).
* geometryFile (string): File path of a triangle mesh file which should be loaded (default: "").
* translation (vec3): Translation vector of the triangle model (default: [0,0,0]).
* scale (vec3): Scaling vector of the triangle model (default: [1,1,1]).
* rotationAxis (vec3): Axis used to rotate the triangle model after loading (default: [0,0,0]).
* rotationAngle (float): Rotation angle for the initial rotation of the triangle model (default: 0.0).
* staticParticles (array): List of particle indices to define static particles that do not move during the simulation (default: []).
* restitution (float): Resitution coefficient of the triangle model (default: 0.1).
* friction (float): Friction coefficient of the triangle model (default: 0.2).


## Tet Models

The simulation library supports tet models to simulate deformable solids. In the following example a tet mesh is loaded to simulate an elastic Armadillo model.

Example code:
```json
"TetModels": [
  {
    "id": 0,
    "nodeFile": "../models/armadillo_4k.node",
    "eleFile": "../models/armadillo_4k.ele",
    "visFile": "../models/armadillo.obj",
    "translation": [0,8,0],
    "rotationAxis": [1, 0, 0],
    "rotationAngle": 1.57,
    "scale": [2, 2, 2],
    "staticParticles": [],
    "restitution" : 0.1,
    "friction" : 0.1,
    "collisionObjectType": 5,
    "collisionObjectFileName": "",
    "collisionObjectScale": [
        1,
        1,
        1
    ],
    "testMesh": 1
  }
]
```

* id (int): ID of the tet model (default: 0).
* nodeFile (string): File path of a tet nodes file which should be loaded. Such a tet file can be generated from a surface mesh using TetGen (default: "").
* eleFile (string): File path of a tet elements file which should be loaded. Such a tet file can be generated from a surface mesh using TetGen (default: "").
* visFile (string): File path of a visualization triangle mesh file which should be loaded. This mesh is used only for the visualization and the mesh export (default: "").
* translation (vec3): Translation vector of the tet model (default: [0,0,0]).
* scale (vec3): Scaling vector of the tet model (default: [1,1,1]).
* rotationAxis (vec3): Axis used to rotate the tet model after loading (default: [0,0,0]).
* rotationAngle (float): Rotation angle for the initial rotation of the tet model (default: 0.0).
* staticParticles (array): List of particle indices to define static particles that do not move during the simulation (default: []).
* restitution (float): Resitution coefficient of the tet model (default: 0.1).
* friction (float): Friction coefficient of the tet model (default: 0.2).
* collisionObjectType (int): The PBD simulator uses a collision detection based on signed distance functions or signed distance fields (SDF). If simple shapes like spheres and boxes are simulated an analytic signed distance function can be used for the detection. For arbitrary geometries a signed distance field is precomputed. Therefore, the correct type of geometry has to be chosen here (default: 0):
  - 0: no collision object
  - 1: sphere
  - 2: box
  - 3: cylinder
  - 4: torus
  - 5: Signed Distance Field (SDF)
  - 6: hollow sphere
  - 7: hollow box
* collisionObjectFileName (string): File path of a geometry file which contains the collision geometry of the solid. This is used to generate an SDF if collisionObjectType 5 is chosen (default: "").
* testMesh (bool): Defines if the vertices of the solid geometry are tested against the signed distance fields of other objects for collisions (default: true).
* collisionObjectScale (vec3): Scaling vector of the collision object of the solid. In most cases this should be the same value as for "scale" (default: [1,1,1]).
* invertSDF (bool): Invert the signed distance field, flips inside/outside (default: false) 
* thicknessSDF (float): Additional thickness of a signed distance field. The geometry is extended by this distance (default: 0.1).
* resolutionSDF (vec3): Resolution of the signed distance field. Using a higher resolution means that the original geometry can be represented more accurately. Since the field is precomputed, a higher resolution does not directly affect the simulation performance (defaut: [10,10,10]). 
	


## RigidBodies

Here, the static and dynamic rigid bodies are defined for the simulation. If you want to connect two rigid bodies by a joint, make sure to give them an ID. The ID can then be usd for the joint definition. 

Example code:
```json
"RigidBodies": [
    {
        "angularVelocity": [0, 0, 0],
        "collisionObjectFileName": "",
        "collisionObjectScale": [500, 1, 500],
        "collisionObjectType": 2,
        "density": 500,
        "friction": 0.3,
        "geometryFile": "../models/cube.obj",
        "flatShading": true,
        "id": 1,
        "isDynamic": false,
        "restitution": 0.5,
        "rotationAngle": 0.0,
        "rotationAxis": [1, 0, 0],
        "scale": [500, 1, 500],
        "testMesh": 1,
        "translation": [0, -0.5, 0],
        "velocity": [0, 0, 0]
    }
]
```

* id (int): ID of the rigid body which is required for joint definitions (default: 0).
* translation (vec3): Translation vector of the rigid body (default: [0,0,0]).
* scale (vec3): Scaling vector of the rigid body (default: [1,1,1]).
* rotationAxis (vec3): Axis used to rotate the rigid body after loading (default: [0,0,0]).
* rotationAngle (float): Rotation angle for the initial rotation of the rigid body (default: 0.0).
* isDynamic (bool): Defines if the body is static or dynamic.
* density (float): Density of the rigid body (default: 1.0).
* flatShading (bool): Enables/disables flat shading (default: false).
* velocity (vec3): Initial velocity of the rigid body (default: [0,0,0]).
* angularVelocity (vec3): Initial angular velocity of the rigid body (default: [0,0,0]).
* restitution (float): Resitution coefficient of the rigid body (default: 0.6).
* friction (float): Friction coefficient of the rigid body (default: 0.2).
* collisionObjectType (int): The PBD simulator uses a collision detection based on signed distance functions or signed distance fields (SDF). If simple shapes like spheres and boxes are simulated an analytic signed distance function can be used for the detection. For arbitrary geometries a signed distance field is precomputed. Therefore, the correct type of geometry has to be chosen here (default: 0):
  - 0: no collision object
  - 1: sphere
  - 2: box
  - 3: cylinder
  - 4: torus
  - 5: Signed Distance Field (SDF)
  - 6: hollow sphere
  - 7: hollow box
* collisionObjectFileName (string): File path of a geometry file which contains the collision geometry of the rigid body. This is used to generate an SDF if collisionObjectType 5 is chosen (default: "").
* testMesh (bool): Defines if the vertices of the rigid body geometry are tested against the signed distance fields of other objects for collisions (default: true).
* collisionObjectScale (vec3): Scaling vector of the collision object of the rigid body. In most cases this should be the same value as for "scale" (default: [1,1,1]).
* invertSDF (bool): Invert the signed distance field, flips inside/outside (default: false) 
* thicknessSDF (float): Additional thickness of a signed distance field. The geometry is extended by this distance (default: 0.1).
* resolutionSDF (vec3): Resolution of the signed distance field. Using a higher resolution means that the original geometry can be represented more accurately. Since the field is precomputed, a higher resolution does not directly affect the simulation performance (defaut: [10,10,10]). 

## Joints

### BallJoints

This joint links two rigid bodies in a common point so that both can rotate around this point. 

Example code:
```json
"BallJoints": [
  {
      "bodyID1": 1,
      "bodyID2": 2,
      "position": [0,1,0]
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* position (vec3): Position of the common connector point.

### BallOnLineJoints

This joint is like a ball joint that can move on a given line. 

Example code:
```json
"BallOnLineJoints": [
  {
      "bodyID1": 1,
      "bodyID2": 2,
      "position": [0,1,0],
      "axis": [1,0,0]
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* position (vec3): Point on the line.
* axis (vec3): Direction vector of the line.

### HingeJoints

This joint links two rigid bodies so that both can rotate around a common axis. 

Example code:
```json
"HingeJoints": [
  {
      "bodyID1": 1,
      "bodyID2": 2,
      "position": [0,1,0],
      "axis": [1,0,0]
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* position (vec3): Point on the rotation axis.
* axis (vec3): Direction vector of the rotation axis.

### UniversalJoints

This joint links two rigid bodies so that both can rotate around two common axes. 

Example code:
```json
"HingeJoints": [
  {
      "bodyID1": 1,
      "bodyID2": 2,
      "position": [0,1,0],
      "axis1": [1,0,0],
      "axis2": [0,1,0]
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* position (vec3): Point on both rotation axes.
* axis1 (vec3): Direction vector of the first rotation axis.
* axis2 (vec3): Direction vector of the second rotation axis.

### SliderJoints

This joint links two rigid bodies so that they can slide on a common axis.

Example code:
```json
"SliderJoints": [
  {
      "bodyID1": 1,
      "bodyID2": 2,
      "axis": [1,0,0]
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* axis (vec3): Direction vector of the slider axis.

### RigidBodyParticleBallJoints

This joint links a rigid body and a particle by a ball joint so that the rigid body can rotate around the particle. 

Example code:
```json
"RigidBodyParticleBallJoints": [
  {
      "rbID": 1,
      "particleID": 2
  }
]
```

* rbID (int): ID of the rigid body of the joint.
* particleID (int): ID of the particle of the joint.
* position (vec3): Position of the common connector point.

### RigidBodySprings

This joint links two rigid bodies by a spring.

Example code:
```json
"RigidBodySprings": [
  {
      "bodyID1": 1,
      "bodyID2": 2,
      "position1": [1,0,0],
      "position2": [2,0,0],
      "stiffness": 10000.0
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* position1 (vec3): Position of connector point in body 1.
* position2 (vec3): Position of connector point in body 2.
* stiffness (float): Stiffness of the spring.

### DistanceJoints

This joint links two rigid bodies by a distance constraint so that the connector points in both bodies have a constant distance during the simulation.

Example code:
```json
"DistanceJoints": [
  {
      "bodyID1": 1,
      "bodyID2": 2,
      "position1": [1,0,0],
      "position2": [2,0,0]
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* position1 (vec3): Position of connector point in body 1.
* position2 (vec3): Position of connector point in body 2.

### DamperJoints

This joint links two rigid bodies so that they can slide on a common axis while a spring force is also acting on this axis. 

Example code:
```json
"DamperJoints": [
  {
      "bodyID1": 2,
      "bodyID2": 3,
      "axis": [0,1,0],
      "stiffness": 500000.0
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* axis (vec3): Direction vector of the slider axis.
* stiffness (float): Stiffness of the spring.

### TargetAngleMotorHingeJoints

This joint links two rigid bodies by a angular motor so that both can rotate around a common axis. Moreover, a target angle can be defined.

Example code:
```json
"TargetAngleMotorHingeJoints": [
  {
      "bodyID1": 1,
      "bodyID2": 2,
      "position": [0,1,0],
      "axis": [1,0,0],
      "repeatSequence": true,
      "targetSequence": [0,0,2,0.707,8,0.707,12,-0.707,18,-0.707,20,0]
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* position (vec3): Point on the rotation axis.
* axis (vec3): Direction vector of the rotation axis.
* target (float): Target angle of the motor. Either you can define a target angle or a target angle sequence. If both are defined, only the target angle is considered. 
* targetSequence (array): This array defines pairs (time,target angle). So in the example above at time 0 the target angle is 0, at time 2 the angle is 0.707 and so on. In this way a sequence of angles for an animation can be defined (see also the CarScene.json example).
* repeatSequence (bool): Defines if the targetSequence is repeated in a loop.


### TargetVelocityMotorHingeJoints

This joint links two rigid bodies by a angular motor so that both can rotate around a common axis. Moreover, a target angular velocity can be defined.

Example code:
```json
"TargetVelocityMotorHingeJoints": [
  {
      "bodyID1": 1,
      "bodyID2": 2,
      "position": [0,1,0],
      "axis": [1,0,0],
      "repeatSequence": false,
      "targetSequence": [0,0,2,0.5,3,2]
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* position (vec3): Point on the rotation axis.
* axis (vec3): Direction vector of the rotation axis.
* target (float): Target angular velocity of the motor. Either you can define a target velocity or a target velocity sequence. If both are defined, only the target velocity is considered. 
* targetSequence (array): This array defines pairs (time,target angular velocity). So in the example above at time 0 the target velocity is 0, at time 2 the angle is 0.5 and so on. In this way a sequence of velocities for an animation can be defined.
* repeatSequence (bool): Defines if the targetSequence is repeated in a loop.

### TargetPositionMotorSliderJoints

This joint links two rigid bodies by a slider motor so that both can slide on a common axis. Moreover, a target position can be defined.

Example code:
```json
"TargetPositionMotorSliderJoints": [
  {
      "bodyID1": 1,
      "bodyID2": 2,
      "axis": [1,0,0],
      "repeatSequence": true,
      "targetSequence": [0,0,2,0.5,3,2]
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* axis (vec3): Direction vector of the slider axis.
* target (float): Target position of the motor. Either you can define a target position or a target position sequence. If both are defined, only the target position is considered. 
* targetSequence (array): This array defines pairs (time,target position). So in the example above at time 0 the target position is 0, at time 2 the position is 0.5 and so on. In this way a sequence of positions for an animation can be defined.
* repeatSequence (bool): Defines if the targetSequence is repeated in a loop.


### TargetVelocityMotorSliderJoints

This joint links two rigid bodies by a slider motor so that both can slide on a common axis. Moreover, a target velocity can be defined.

Example code:
```json
"TargetPositionMotorSliderJoints": [
  {
      "bodyID1": 1,
      "bodyID2": 2,
      "axis": [1,0,0],
      "repeatSequence": true,
      "targetSequence": [0,0,2,0.5,3,2]
  }
]
```

* bodyID1 (int): ID of the first rigid body of the joint.
* bodyID2 (int): ID of the second rigid body of the joint.
* axis (vec3): Direction vector of the slider axis.
* target (float): Target velocity of the motor. Either you can define a target velocity or a target velocity sequence. If both are defined, only the target velocity is considered. 
* targetSequence (array): This array defines pairs (time,target velocity). So in the example above at time 0 the target velocity is 0, at time 2 the velocity is 0.5 and so on. In this way a sequence of velocities for an animation can be defined.
* repeatSequence (bool): Defines if the targetSequence is repeated in a loop.