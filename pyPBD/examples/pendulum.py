import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *
from render_tools import *

import pypbd as pbd
import numpy as np

numParticles = 30
size = 4.0

def buildModel():
    sim = pbd.Simulation.getCurrent()
    
    # Init simulation and collision detection with standard settings
    sim.initDefault()
    model = sim.getModel()
    
    pd = model.getParticles()
    
    ts = sim.getTimeStep()
    ts.setValueUInt(pbd.TimeStepController.NUM_SUB_STEPS, 10)
    
    # add particles
    dx = size / (numParticles - 1)
    for i in range(numParticles):
        pd.addVertex([dx*i,0,0])
        
    # link them by distance constraints
    for i in range(numParticles-1):
        model.addDistanceConstraint_XPBD(i,i+1, 1000000.0)
    
    # make first particle static
    pd.setMass(0, 0.0)
    
# render pendulum    
def render():
    sim = pbd.Simulation.getCurrent()
    model = sim.getModel()
    
    # render distance constraints
    pd = model.getParticles()
    glLineWidth(2.0)
    glColor3f(1.0, 1.0, 1.0)
    glBegin(GL_LINE_STRIP)
    for i in range(numParticles):
        glVertex3fv(pd.getPosition(i))
    glEnd()
    
    # render particles
    glPointSize(8.0)
    glColor3f(0.1, 0.7, 0.3)
    glBegin(GL_POINTS)
    for i in range(numParticles):
        glVertex3fv(pd.getPosition(i))
    glEnd()

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
    sim = pbd.Simulation.getCurrent()
    sim.reset()
    sim.getModel().cleanup()
    buildModel()        
    
def main():
    # Activate logger to output info on the console
    pbd.Logger.addConsoleSink(pbd.LogLevel.INFO)

    # init OpenGL
    initGL(1280, 1024) 
    gluLookAt (0, -2, -10, 0, -2, 0, 0, 1, 0)
    
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
        pygame.time.wait(5)     # slow down simulation since it is too fast
        
    pygame.quit()
    pbd.Timing.printAverageTimes()
    
if __name__ == "__main__":
    main()