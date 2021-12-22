import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

from render_tools import *
from math_tools import *

import pypbd as pbd
import numpy as np

nRows = 50
nCols = 50
width = 10.0
height = 10.0

# 1 = distance constraints (PBD)
# 2 = FEM triangle constraints (PBD)
# 3 = strain triangle constraints (PBD)
# 4 = distance constraints (XPBD)
simModel = 2

# 1 = dihedral angle (PBD)
# 2 = isometric bending (PBD)
# 3 = isometric bending  (XPBD)   
bendingModel = 2

def buildModel():   
    sim = pbd.Simulation.getCurrent()
    
    # Init simulation and collision detection with standard settings
    sim.initDefault()
    model = sim.getModel()
    
    # Create the mesh for the cloth model
    createMesh(simModel, bendingModel)
    
    # Load geometry from OBJ files 
    fileName = "../../data/models/cube.obj"
    (vd, mesh) = pbd.OBJLoader.loadObjToMesh(fileName, [1, 1, 1])
    
    fileName2 = "../../data/models/torus.obj"
    (vdTorus, meshTorus) = pbd.OBJLoader.loadObjToMesh(fileName2, [1, 1, 1])
    
    ## There are two ways to define a rigid body with a corresponding collision object:
    #
    # Floor: You can first generate a signed distance field 
    floorSDF = pbd.CubicSDFCollisionDetection.generateSDF(vd, mesh, [30, 30, 30])
    # and use it as parameter for addRigidBody(). In this way the SDF can
    # be directly used for multiple bodies and it is stored only once.
    rb = model.addRigidBody(1.0,  # density
        vd, mesh,                 # vertices, mesh
        [0.0, -2.5, 0.0],         # position 
        np.identity(3),           # rotation matrix
        [100.0, 1.0, 100.0],      # scaling
        True,                     # test mesh vertices against other signed distance fields
        floorSDF)                 # signed distance field for collision detection
    rb.setMass(0.0)     # make body static
    
    # Torus: Alternatively, addRigidBody() automatically generates the SDF for you
    # by setting generateCollisionObject= True. 
    rb = model.addRigidBody(1.0,            # density
        vdTorus, meshTorus,                 # vertices, mesh
        [0.0, 1.5, 0.0],                    # position 
        np.identity(3),                     # rotation matrix
        [2.0, 2.0, 2.0],                    # scaling
        testMesh=True,                      # test mesh vertices against other signed distance fields
        generateCollisionObject= True,      # automatically generate an SDF and a corresponding collision object
        resolution=[30, 30, 30])            # resolution of the signed distance field
    rb.setMass(0.0)             # make body static
    rb.setFrictionCoeff(0.1)    # set friction coefficient

    # Set collision tolerance (distance threshold)
    cd = sim.getTimeStep().getCollisionDetection()      
    cd.setTolerance(0.05)
    
    ts = sim.getTimeStep()
    ts.setValueUInt(pbd.TimeStepController.NUM_SUB_STEPS, 3)

     
# Create a particle model mesh 
def createMesh(simModel, bendingModel):
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    
    # Generate a regular triangle model with nCols*nRows vertices
    triModel = model.addRegularTriangleModel(nCols, nRows, 
        translation=[-5.0, 4.0, -5.0],                              # position
        rotation=rotation_matrix(math.pi*0.5, [1.0, 0.0, 0.0]),     # rotation matrix
        scale=[width, height],                                      # scaling
        testMesh=True)                                              # test mesh vertices against other signed distance fields

    # init constraints
    stiffness = 1.0
    if (simModel == 4):
        stiffness = 100000
    poissonRatio = 0.3
    
    # Generate a default set of constraints for the cloth model to simulate stretching and shearing resistance
    # simModel: 
    # 1 = distance constraints (PBD)
    # 2 = FEM triangle constraints (PBD)
    # 3 = strain triangle constraints (PBD)
    # 4 = distance constraints (XPBD)
    model.addClothConstraints(triModel, simModel, stiffness, stiffness, stiffness, stiffness, 
        poissonRatio, poissonRatio, False, False)

    bending_stiffness = 0.01
    if (bendingModel == 3):
        bending_stiffness = 100.0
        
    # Generate a default set of bending constraints for the cloth model
    # bendingModel:
    # 1 = dihedral angle (PBD)
    # 2 = isometric bending (PBD)
    # 3 = isometric bending  (XPBD)   
    model.addBendingConstraints(triModel, bendingModel, bending_stiffness)
    
    print("Number of triangles: " + str(triModel.getParticleMesh().numFaces()))
    print("Number of vertices: " + str(nRows*nCols))
    
# Render all bodies    
def render():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    
    # render meshes of rigid bodies
    rbs = model.getRigidBodies()   
    for rb in rbs:
        vd = rb.getGeometry().getVertexData()
        mesh = rb.getGeometry().getMesh()
        drawMesh(vd, mesh, 0, [0,0.2,0.7])
     
    # render mesh of cloth model     
    pd = model.getParticles()   
    triModel = model.getTriangleModels()[0]
    offset = triModel.getIndexOffset()
    drawMesh(pd, triModel.getParticleMesh(), offset, [0,0.3,0.8])
    
    # render time
    glPushMatrix()
    glLoadIdentity()
    drawText([-0.95,0.9], "Time: {:.2f}".format(pbd.TimeManager.getCurrent().getTime()))
    glPopMatrix()
    
# Perform simulation steps
def timeStep():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    
    # We perform 8 simulation steps per render step
    for i in range(8):
        sim.getTimeStep().step(model)
        
    # Update mesh normals of cloth model for rendering
    for triModel in model.getTriangleModels():
        triModel.updateMeshNormals(model.getParticles())
     
# Reset the simulation
def reset():
    sim = pbd.Simulation.getCurrent()
    sim.reset()
    sim.getModel().cleanup()
    cd = sim.getTimeStep().getCollisionDetection()      
    cd.cleanup()
    buildModel()
    
def main():
    # Activate logger to output info on the console
    pbd.Logger.addConsoleSink(pbd.LogLevel.INFO)

    # init OpenGL
    initGL(1280, 1024)  
    gluLookAt (0, 10, -20, 0, 0, 0, 0, 1, 0)
    glRotatef(20.0, 0, 1, 0)
    
    # build simulation model 
    buildModel()

    # start event loop 
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    reset()
                if event.key == pygame.K_ESCAPE:
                    running = False
        
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        glClearColor(0.3, 0.3, 0.3, 1.0)
        timeStep()
        render()
        pygame.display.flip()
        
    pygame.quit()
    pbd.Timing.printAverageTimes()

if __name__ == "__main__":
    main()