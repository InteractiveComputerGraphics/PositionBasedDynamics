import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

from math_tools import *
from render_tools import *

import pypbd as pbd
import numpy as np

nRows = 30
nCols = 30
clothWidth = 10.0
clothHeight = 10.0
width = 0.2
height = 2.0
depth = 0.2

def buildModel():
    sim = pbd.Simulation.getCurrent()
    
    # Init simulation and collision detection with standard settings
    sim.initDefault()
    model = sim.getModel()
    
    # 1 = distance constraints (PBD)
    # 2 = FEM triangle constraints (PBD)
    # 3 = strain triangle constraints (PBD)
    # 4 = distance constraints (XPBD)
    simModel = 2 

    # 1 = dihedral angle (PBD)
    # 2 = isometric bending (PBD)
    # 3 = isometric bending  (XPBD)   
    bendingModel = 2
    
    # Create the mesh for the cloth model
    createMesh(simModel, bendingModel)
    # Create rigid bodies
    createRigidBodyModel()
    
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
        scale=[clothWidth, clothHeight],                            # scaling
        testMesh=False)                                             # test mesh vertices against other signed distance fields

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
        bending_stiffness = 50.0
        
    # Generate a default set of bending constraints for the cloth model
    # bendingModel:
    # 1 = dihedral angle (PBD)
    # 2 = isometric bending (PBD)
    # 3 = isometric bending  (XPBD)   
    model.addBendingConstraints(triModel, bendingModel, bending_stiffness)
    
    print("Number of triangles: " + str(triModel.getParticleMesh().numFaces()))
    print("Number of vertices: " + str(nRows*nCols))
      
   
def createRigidBodyModel():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    rb = model.getRigidBodies()

    # Load geometry from OBJ files 
    fileName = "../../data/models/cube.obj"
    (vd, mesh) = pbd.OBJLoader.loadObjToMesh(fileName, [width, height, depth])
    (vd_static, mesh_static) = pbd.OBJLoader.loadObjToMesh(fileName, [0.5, 0.5, 0.5])

    # -5, -5
    body = model.addRigidBody(1.0,      # density
        vd_static, mesh_static,         # vertices, mesh
        translation=[-5.0, 0.0, -5.0],  # position 
        testMesh=False,                 # no collision handling
        generateCollisionObject=False)  
    body.setMass(0.0)

    # dynamic body
    body = model.addRigidBody(1.0,  
        vd, mesh,     
        [-5.0, 1.0, -5.0],           
        testMesh=False,                 # no collision handling
        generateCollisionObject=False)    
    
    body = model.addRigidBody(1.0,  
        vd, mesh,     
        [-5.0, 3.0, -5.0],           
        testMesh=False,                 # no collision handling
        generateCollisionObject=False)  

    model.addBallJoint(0, 1, [-5.0, 0.0, -5.0])
    model.addBallJoint(1, 2, [-5.0, 2.0, -5.0])

    # 5, -5
    body = model.addRigidBody(1.0,  # density
        vd_static, mesh_static,     # vertices, mesh
        [5.0, 0.0, -5.0],           # position 
        testMesh=False,             # no collision handling
        generateCollisionObject=False)  
    body.setMass(0.0)

    # dynamic body
    body = model.addRigidBody(1.0,  
        vd, mesh,     
        [5.0, 1.0, -5.0],           
        testMesh=False,             # no collision handling
        generateCollisionObject=False)  
    
    body = model.addRigidBody(1.0,  
        vd, mesh,     
        [5.0, 3.0, -5.0],           
        testMesh=False,             # no collision handling
        generateCollisionObject=False)  

    model.addBallJoint(3, 4, [5.0, 0.0, -5.0])
    model.addBallJoint(4, 5, [5.0, 2.0, -5.0])

    # 5, 5
    body = model.addRigidBody(1.0,  # density
        vd_static, mesh_static,     # vertices, mesh
        [5.0, 0.0, 5.0],            # position 
        testMesh=False,             # no collision handling
        generateCollisionObject=False)  
    body.setMass(0.0)

    # dynamic body
    body = model.addRigidBody(1.0,  
        vd, mesh,     
        [5.0, 1.0, 5.0],           
        testMesh=False,             # no collision handling
        generateCollisionObject=False)  
    
    body = model.addRigidBody(1.0,  
        vd, mesh,     
        [5.0, 3.0, 5.0],           
        testMesh=False,             # no collision handling
        generateCollisionObject=False)  

    model.addBallJoint(6, 7, [5.0, 0.0, 5.0])
    model.addBallJoint(7, 8, [5.0, 2.0, 5.0])
    
    # -5, 5
    body = model.addRigidBody(1.0,  # density
        vd_static, mesh_static,     # vertices, mesh
        [-5.0, 0.0, 5.0],           # position 
        testMesh=False,             # no collision handling
        generateCollisionObject=False)  
    body.setMass(0.0)

    # dynamic body
    body = model.addRigidBody(1.0,  
        vd, mesh,     
        [-5.0, 1.0, 5.0],           
        testMesh=False,             # no collision handling
        generateCollisionObject=False)  
    
    body = model.addRigidBody(1.0,  
        vd, mesh,     
        [-5.0, 3.0, 5.0],           
        testMesh=False,             # no collision handling
        generateCollisionObject=False)  

    model.addBallJoint(9, 10, [-5.0, 0.0, 5.0])
    model.addBallJoint(10, 11, [-5.0, 2.0, 5.0])

    model.addRigidBodyParticleBallJoint(2, 0)
    model.addRigidBodyParticleBallJoint(5, nCols - 1)
    model.addRigidBodyParticleBallJoint(8, nRows*nCols - 1)
    model.addRigidBodyParticleBallJoint(11, (nRows-1)*nCols)
      
def render():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    pd = model.getParticles()   
    triModel = model.getTriangleModels()[0]
    offset = triModel.getIndexOffset()
    drawMesh(pd, triModel.getParticleMesh(), offset, [0,0.3,0.8])
    
    rbs = model.getRigidBodies()
    for rb in rbs:
        vd = rb.getGeometry().getVertexData()
        mesh = rb.getGeometry().getMesh()
        drawMesh(vd, mesh, 0, [0.5,0.5,0.5])
    
    glPushMatrix()
    glLoadIdentity()
    drawText([-0.95,0.9], "Time: {:.2f}".format(pbd.TimeManager.getCurrent().getTime()))
    glPopMatrix()
    
def timeStep():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    for i in range(8):
        sim.getTimeStep().step(model)
        
    for triModel in model.getTriangleModels():
        triModel.updateMeshNormals(model.getParticles())
     
def reset():
	pbd.Simulation.getCurrent().reset()
	pbd.Simulation.getCurrent().getModel().cleanup()
	buildModel()
    
def main():
    pbd.Logger.addConsoleSink(pbd.LogLevel.INFO)
    
    initGL(1280, 1024)  
    #initLights()
    gluLookAt (0, 10, -30, 0, 0, 0, 0, 1, 0)
    glRotatef(20.0, 0, 1, 0)
    
    buildModel()

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