import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

from render_tools import *
from math_tools import *

import pypbd as pbd
import numpy as np


# 1 = distance constraints (PBD)
# 2 = FEM tet constraints (PBD)
# 3 = strain tet constraints (PBD)
# 4 = shape matching
# 5 = distance constraints (XPBD)
simModel = 2


def buildModel():   
    sim = pbd.Simulation.getCurrent()
    sim.initDefault()
    model = sim.getModel()
    
    rbs = model.getRigidBodies()
    
    createMesh(simModel)
       
    fileName2 = "../../data/models/torus.obj"
    (vdTorus, meshTorus) = pbd.OBJLoader.loadObjToMesh(fileName2, [1, 1, 1])
       
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
    rb.setFrictionCoeff(0.3)
       
    cd.setTolerance(0.05)
    
    ts = sim.getTimeStep()
    ts.setValueUInt(pbd.TimeStepController.NUM_SUB_STEPS, 5)

     
# Create a particle model mesh 
def createMesh(simModel):
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    
    fileNameNode = "../../data/models/armadillo_4k.node"
    fileNameEle = "../../data/models/armadillo_4k.ele"
    (positions, tets) = pbd.TetGenLoader.loadTetgenModel(fileNameNode, fileNameEle)

    # move up and scale
    pos = np.array(positions)
    for x in pos:
        x[0] *= 1.5
        x[1] = x[1]*1.5 + 6
        x[2] *= 1.5
    model.addTetModelCollision(len(pos), int(len(tets)/4), pos, tets)

    # init constraints
    tetModel = model.getTetModels()[0]
    stiffness = 1.0
    if (simModel == 5):
        stiffness = 100000
    poissonRatio = 0.3
    model.addSolidConstraints(tetModel, simModel, stiffness, poissonRatio, stiffness, False, False)
    
    print("Number of triangles: " + str(tetModel.getParticleMesh().numTets()))
    print("Number of vertices: " + str(len(pos)))
        
def render():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    
    rbs = model.getRigidBodies()
    
    for rb in rbs:
        vd = rb.getGeometry().getVertexData()
        mesh = rb.getGeometry().getMesh()
        drawMesh(vd, mesh, 0, [0,0.2,0.7])
        
    pd = model.getParticles()   
    tetModel = model.getTetModels()[0]
    offset = tetModel.getIndexOffset()
    drawMesh(pd, tetModel.getSurfaceMesh(), offset, [0,0.2,0.7])
    
    glPushMatrix()
    glLoadIdentity()
    drawText([-0.95,0.9], "Time: {:.2f}".format(pbd.TimeManager.getCurrent().getTime()))
    glPopMatrix()
    
def timeStep():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    for i in range(8):
        sim.getTimeStep().step(model)
        
    for tetModel in model.getTetModels():
        tetModel.updateMeshNormals(model.getParticles())
     
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