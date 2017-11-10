from SceneGenerator import *
import math 
import random

def scale(vec, s):
    vec[0] *= s
    vec[1] *= s
    vec[2] *= s
    return vec


random.seed(1)

s = 1

scene = generateScene('PileScene', camPosition=[10,20,60], camLookat=[10,0,0])
addParameters(scene, h=0.005, maxIterVel=5, contactTolerance=0.01)

# floor
floorScale=[500, 1, 500]
floorScale = scale(floorScale, s)
floorT = [0,-0.5,0]
floorT = scale(floorT, s)
addRigidBody(scene, '../models/cube.obj', 2, coScale=floorScale, 
             scale=floorScale, translation=floorT, 
             dynamic=0, rest= 0.5)

# piles
restitution = 0.6
num_piles_x = 5
num_piles_z = 5
dx_piles = 5.0
dz_piles = 5.0
startx_piles = -0.5 * (num_piles_x - 1.0)*dx_piles
startz_piles = -0.5 * (num_piles_z - 1.0)*dz_piles
pileScale = [0.25,10,0.25]
pileScale = scale(pileScale, s)

current_z = startz_piles
for i in range(0,num_piles_z):
    current_x = startx_piles
    for j in range(0,num_piles_x):        
        t = [s*current_x, 0.5*pileScale[1], s*current_z]
        addRigidBody(scene, '../models/cylinder.obj', 3, coScale=pileScale, scale=pileScale, 
                     translation=t, dynamic=0, rest=restitution)
        current_x += dx_piles
    current_z += dz_piles

# rigid bodies

num_bodies_x = 2
num_bodies_y = 2
num_bodies_z = 2
dx_bodies = 7.0
dy_bodies = 7.0
dz_bodies = 7.0
startx_bodies = -0.5 * (num_bodies_x - 1.0)*dx_bodies
starty_bodies = 20.0
startz_bodies = -0.5 * (num_bodies_z - 1.0)*dz_bodies

armadilloScale = [2,2,2]
bunnyScale = [4,4,4]
armadilloScale = scale(armadilloScale, s)
bunnyScale = scale(bunnyScale, s)
density = 300
friction = 0.1
restitution = 0.6

current_y = starty_bodies
index = 0
numTypes = 2
for k in range(0,num_bodies_y):
    current_z = startz_bodies
    for i in range(0,num_bodies_z):
        current_x = startx_bodies - 0.25*dx_bodies
        for j in range(0,num_bodies_x):   
            rand_x = 0
            rand_z = 0
            x = [current_x+rand_x, current_y, current_z+rand_z]
            t = x
            if i%2 == 1:
                t[0] += 0.5*dx_bodies
            t = scale(t, s)

            rotaxis = [random.random(), random.random(), random.random()]
            rotangle = 1.0 * random.random()*math.pi

            #rotaxis = [1,0,0]
            #rotangle = 0.0

            index = random.randint(1, 3000)

            if index%numTypes == 1:
                addRigidBody(scene, '../models/bunny_10k.obj', 5, coFile='../sdf/bunny_10k.csdf', coScale=bunnyScale, 
                         translation=t, axis=rotaxis, angle=rotangle, scale=bunnyScale, 
                         dynamic=1, density=density, friction=friction, rest=restitution)
            else:
                addRigidBody(scene, '../models/armadillo.obj', 5, coFile='', coScale=armadilloScale, 
                         translation=t, axis=rotaxis, angle=rotangle, scale=armadilloScale, 
                         dynamic=1, density=density, friction=friction, rest=restitution)
    
            index += 1   
            current_x += dx_bodies    
        current_z += dz_bodies
    current_y += dy_bodies




writeScene(scene, 'PileScene.json')
