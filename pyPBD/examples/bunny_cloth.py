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
bendingModel = 3

def buildModel():   
    sim = pbd.Simulation.getCurrent()
    sim.initDefault()
    model = sim.getModel()
    
    rbs = model.getRigidBodies()
    
    createMesh(simModel, bendingModel)
    
    fileName = "../../data/models/cube.obj"
    (vd, mesh) = pbd.OBJLoader.loadObjToMesh(fileName, [1, 1, 1])
    
    fileName2 = "../../data/models/torus.obj"
    (vdTorus, meshTorus) = pbd.OBJLoader.loadObjToMesh(fileName2, [1, 1, 1])
    
    # floor
    floorSDF = pbd.CubicSDFCollisionDetection.generateSDF(vd, mesh, [30, 30, 30])
    rb = model.addRigidBody(1.0,  # density
        vd, mesh,                 # vertices, mesh
        [0.0, -2.5, 0.0],         # position 
        0.0, [1,0,0],             # angle, rotation axis
        [100.0, 1.0, 100.0],      # scaling
        floorSDF)                 # signed distance field for collision detection
    rb.setMass(0.0)
    
    # torus
    cd = sim.getTimeStep().getCollisionDetection()
    torusSDF = pbd.CubicSDFCollisionDetection.generateSDF(vdTorus, meshTorus, [30, 30, 30])
    rb = model.addRigidBody(1.0,  # density
        vdTorus, meshTorus,       # vertices, mesh
        [0.0, 1.5, 0.0],          # position 
        0.0, [1,0,0],             # angle, rotation axis
        [2.0, 2.0, 2.0],          # scaling
        torusSDF)                 # signed distance field for collision detection
    rb.setMass(0.0)
    rb.setFrictionCoeff(0.1)
       
    cd.setTolerance(0.05)
    
    ts = sim.getTimeStep()
    ts.setValueUInt(pbd.TimeStepController.NUM_SUB_STEPS, 5)

     
# Create a particle model mesh 
def createMesh(simModel, bendingModel):
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    
    fileName2 = "../../data/models/bunny_10k.obj"
    (vd, mesh) = pbd.OBJLoader.loadObjToMesh(fileName2, [5,5,5])

    # move up
    pos = np.array(vd.getVertices())
    for x in pos:
        x[1] += 5
    model.addTriangleModelCollision(vd.size(), mesh.numFaces(), pos, np.array(mesh.getFaces()), [], []) 

    # Set mass of points to zero => make it static
    #pd.setMass(0, 0.0)
    #pd.setMass((nRows-1)*nCols, 0.0)

    # init constraints
    triModels = model.getTriangleModels()
    for triModel in triModels:
        stiffness = 1.0
        if (simModel == 4):
            stiffness = 100000
        poissonRatio = 0.3
        model.addClothConstraints(triModel, simModel, stiffness, stiffness, stiffness, stiffness, 
            poissonRatio, poissonRatio, False, False)

        bending_stiffness = 0.5
        if (bendingModel == 3):
            bending_stiffness = 1000.0
        model.addBendingConstraints(triModel, bendingModel, bending_stiffness)
        
        print("Number of triangles: " + str(triModel.getParticleMesh().numFaces()))
        print("Number of vertices: " + str(nRows*nCols))
        
def render():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    
    rbs = model.getRigidBodies()
    
    for rb in rbs:
        vd = rb.getGeometry().getVertexData()
        mesh = rb.getGeometry().getMesh()
        drawMesh(vd, mesh, 0, [0,0.2,0.7])
        
    pd = model.getParticles()   
    triModel = model.getTriangleModels()[0]
    offset = triModel.getIndexOffset()
    drawMesh(pd, triModel.getParticleMesh(), offset, [0,0.3,0.8])
    
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
    cd.cleanup()
    buildModel()
    
def main():
    pbd.Logger.addConsoleSink(pbd.LogLevel.INFO)
    
    initGL(1280, 1024)  
    #initLights()
    gluLookAt (0, 10, -20, 0, 0, 0, 0, 1, 0)
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