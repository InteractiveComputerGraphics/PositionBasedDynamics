import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

from render_tools import *
from math_tools import *

import pypbd as pbd
import numpy as np
import math

numberOfBodies = 10
width = 1.0
height = 0.1
depth = 0.1

def buildModel():
    sim = pbd.Simulation.getCurrent()
    
    # Init simulation and collision detection with standard settings
    sim.initDefault()
    
    createModel()
    
    ts = sim.getTimeStep()
    ts.setValueUInt(pbd.TimeStepController.NUM_SUB_STEPS, 3)
    ts.setValueUInt(pbd.TimeStepController.MAX_ITERATIONS, 1)
     
def createModel():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    rbs = model.getRigidBodies()
    
    # Load geometry from OBJ files 
    fileName = "../../data/models/cube.obj"
    (vd, mesh) = pbd.OBJLoader.loadObjToMesh(fileName, [width, height, depth])
    
    fileName2 = "../../data/models/bunny_10k.obj"
    (vd2, mesh2) = pbd.OBJLoader.loadObjToMesh(fileName2, [2, 2, 2])
    
    # add chain bodies
    density = 1.0
    for i in range(numberOfBodies-1):
        model.addRigidBody(1.0,       # density
        vd, mesh,                     # vertices, mesh
        [i*width, 0.0, 0.0],          # position 
        np.identity(3),               # angle, rotation axis
        [1, 1, 1],                    # scaling
        False,                        # test mesh vertices against other signed distance fields
        None)                         # signed distance field for collision detection

    # Make first body static
    rbs[0].setMass(0.0)
    
    # add bunny
    t = np.array([0.411 + (numberOfBodies - 1.0)*width, -1.776, 0.356])
    rotation = rotation_matrix(math.pi/6.0, [0,0,1])
    model.addRigidBody(density,       # density
        vd2, mesh2,                   # vertices, mesh
        t,                            # position 
        rotation,                     # angle, rotation axis
        [1, 1, 1],                    # scaling
        False,                        # test mesh vertices against other signed distance fields
        None)                         # signed distance field for collision detection

    # generate ball joints 
    for i in range(numberOfBodies-1):
        model.addBallJoint(i, i + 1, [i*width + 0.5*width, 0.0, 0.0])

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
    
    # render constraints
    cs = model.getConstraints()
    for c in cs:
        if (c.getTypeId() == pbd.BallJoint.TYPE_ID):
            drawSphere(c.jointInfo[:,2], 0.15, [0.0, 0.6, 0.2])

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
    initLights()
    gluLookAt (1.5, -2, 20, 1.5, -2, 0, 0, 1, 0)
    
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