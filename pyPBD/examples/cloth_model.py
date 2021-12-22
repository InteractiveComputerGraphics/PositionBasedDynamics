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
    
    # Generate a regular triangle model with nCols*nRows vertices
    triModel = model.addRegularTriangleModel(nCols, nRows, 
        [0,0,0],                                           # position
        rotation_matrix(math.pi*0.5, [1.0, 0.0, 0.0]),     # rotation matrix
        [width, height],                                   # scaling
        testMesh=False)                                    # test mesh vertices against other signed distance fields 

    # Set mass of points to zero => make it static
    pd = model.getParticles()
    pd.setMass(0, 0.0)
    pd.setMass(nRows-1, 0.0)

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
    
# Render all bodies        
def render():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    
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
	pbd.Simulation.getCurrent().reset()
	pbd.Simulation.getCurrent().getModel().cleanup()
	buildModel()
    
def main():
    # Activate logger to output info on the console
    pbd.Logger.addConsoleSink(pbd.LogLevel.INFO)

    # init OpenGL
    initGL(1280, 1024)  
    gluLookAt (5, 10, 20, 5, -5, 0, 0, 1, 0)
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