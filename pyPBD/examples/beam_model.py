import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *
from render_tools import *

import pypbd as pbd
import numpy as np

width = 30
depth = 5
height = 5 

def buildModel():
    sim = pbd.Simulation.getCurrent()
    sim.initDefault()
    model = sim.getModel()
    
    # 1 = distance constraints (PBD)
    # 2 = FEM tet constraints (PBD)
    # 3 = strain tet constraints (PBD)
    # 4 = shape matching
    # 5 = distance constraints (XPBD)
    simModel = 2
    
    createMesh(simModel)

    ts = sim.getTimeStep()
    ts.setValueUInt(pbd.TimeStepController.NUM_SUB_STEPS, 3)

     
# Create a particle model mesh 
def createMesh(simModel):
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    tetModel = model.addRegularTetModel(width, height, depth, translation=[5,0,0], scale=[10, 1.5, 1.5]) 

    pd = model.getParticles()
    
    # Set mass of points to zero => make it static
    for i in range(1):
        for j in range(height):
            for k in range(depth):
                pd.setMass(i*height*depth + j*depth + k, 0.0)
    
    # init constraints
    stiffness = 1.0
    if (simModel == 5):
        stiffness = 100000
    poissonRatio = 0.3
    model.addSolidConstraints(tetModel, simModel, stiffness, poissonRatio, stiffness, False, False)

    tetModel.updateMeshNormals(pd);
    
    print("Number of tets: " + str(tetModel.getParticleMesh().numTets()))
    print("Number of vertices: " + str(width*height*depth))

def render():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
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
	buildModel()
    
def main():
    pbd.Logger.addConsoleSink(pbd.LogLevel.INFO)
    
    initGL(1280, 1024)  
    gluLookAt (5, 4, 15, 5, -1, 0, 0, 1, 0)
    
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