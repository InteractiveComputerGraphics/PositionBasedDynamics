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

def buildModel():
    sim = pbd.Simulation.getCurrent()
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
    
    createMesh(simModel, bendingModel)
    
    ts = sim.getTimeStep()
    ts.setValueUInt(pbd.TimeStepController.NUM_SUB_STEPS, 3)

     
# Create a particle model mesh 
def createMesh(simModel, bendingModel):
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    model.addRegularTriangleModel(nCols, nRows, [0,0,0], rotation_matrix(math.pi*0.5, [1.0, 0.0, 0.0]), [width, height]) 

    # Set mass of points to zero => make it static
    pd = model.getParticles()
    pd.setMass(0, 0.0)
    pd.setMass(nRows-1, 0.0)

    # init constraints
    triModels = model.getTriangleModels()
    for triModel in triModels:
        stiffness = 1.0
        if (simModel == 4):
            stiffness = 100000
        poissonRatio = 0.3
        model.addClothConstraints(triModel, simModel, stiffness, stiffness, stiffness, stiffness, 
            poissonRatio, poissonRatio, False, False)

        bending_stiffness = 0.01
        if (bendingModel == 3):
            bending_stiffness = 50.0
        model.addBendingConstraints(triModel, bendingModel, bending_stiffness)
        
        print("Number of triangles: " + str(triModel.getParticleMesh().numFaces()))
        print("Number of vertices: " + str(nRows*nCols))
        
def render():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
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
	buildModel()
    
def main():
    pbd.Logger.addConsoleSink(pbd.LogLevel.INFO)
    
    initGL(1280, 1024)  
    gluLookAt (5, 10, 20, 5, -5, 0, 0, 1, 0)
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