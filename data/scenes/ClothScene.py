from SceneGenerator import *
import math

scene = generateScene('ClothScene', camPosition=[0,15,30], camLookat=[0,0,0])
addParameters(scene, h=0.005, contactTolerance=0.05)
friction = 0.1
restitution = 0.2

addRigidBody(scene, '../models/cube.obj', 2, coScale=[100, 1, 100], scale=[100, 1, 100], dynamic=0)

addRigidBody(scene, '../models/torus.obj', 4, coScale=[2, 1, 2], scale=[2, 2, 2], translation=[0,5,0], 
             friction=friction, rest = restitution, dynamic=0)

addTriangleModel(scene, '../models/plane_50x50.obj', translation=[0, 10, 0], 
                 scale=[10,10,10], friction=friction, rest = restitution, staticParticles=[])

writeScene(scene, 'ClothScene.json')
