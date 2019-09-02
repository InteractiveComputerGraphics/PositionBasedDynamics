from SceneGenerator import *
import math 
import random

def scale(vec, s):
    vec[0] *= s
    vec[1] *= s
    vec[2] *= s
    return vec

s = 1

scene = generateScene('CarScene', camPosition=[1,5,20], camLookat=[0,0,0])
addParameters(scene, h=0.005, maxIter=50, maxIterVel=50, contactTolerance=0.01, gravity=[0,-2,0], numberOfStepsPerRenderUpdate=10)

# floor
floorScale=[1000, 1, 1000]
floorScale = scale(floorScale, s)
floorT = [0,-0.5,0]
floorT = scale(floorT, s)
addRigidBody(scene, '../models/cube.obj', 2, coScale=floorScale, 
             scale=floorScale, translation=floorT, 
             dynamic=0, rest= 0.5)

carX = [0,2,0]

# chassis
restitution = 0.6
frict = 0.4
chassisX = add_vector(carX, [0,-0.1,0.1])
chassisScale = [2.5,0.4,1]
chassisScale = scale(chassisScale, s)
chassis = addRigidBody(scene, '../models/cube.obj', 0, coScale=chassisScale, scale=chassisScale, 
                     translation=chassisX, dynamic=1, rest=restitution, friction=0.0, density = 200)

# damper bodies
damperScale = [0.2,0.1,0.2]
damperScale = scale(damperScale, s)
damperDensity = 50000;
damperX1 = add_vector(carX, [1.75, -0.7, 0.5])
dBody1 = addRigidBody(scene, '../models/cube.obj', 0, coScale=damperScale, scale=damperScale, 
                     translation=damperX1, dynamic=1, rest=restitution, density = damperDensity)

damperX2 = add_vector(carX, [1.75, -0.7, -0.5])
dBody2 = addRigidBody(scene, '../models/cube.obj', 0, coScale=damperScale, scale=damperScale, 
                     translation=damperX2, dynamic=1, rest=restitution, density = damperDensity)

damperX3 = add_vector(carX, [-1.75, -0.7, 0.5])
dBody3 = addRigidBody(scene, '../models/cube.obj', 0, coScale=damperScale, scale=damperScale, 
                     translation=damperX3, dynamic=1, rest=restitution, density = damperDensity)

damperX4 = add_vector(carX, [-1.75, -0.7, -0.5])
dBody4 = addRigidBody(scene, '../models/cube.obj', 0, coScale=damperScale, scale=damperScale, 
                     translation=damperX4, dynamic=1, rest=restitution, density = damperDensity)

# steering
steeringBodyX = add_vector(carX, [-1.75, -0.15, 0])
steeringBodyScale = [0.2,0.1,1]
steeringBodyScale = scale(steeringBodyScale, s)
steeringBody = addRigidBody(scene, '../models/cube.obj', 0, coScale=steeringBodyScale, scale=steeringBodyScale, 
                     translation=steeringBodyX, dynamic=1, rest=restitution, density = 10000)

steeringMotorX = add_vector(carX, [-1.75, -0.4, 0])
addTargetAngleMotorHingeJoint(scene, chassis, steeringBody, steeringMotorX, [0, 1, 0], 0.707, [0,0, 2, 0.707, 8, 0.707, 12, -0.707, 18, -0.707, 20, 0], 1)


# wheels 
wheelScale = [0.3,0.3,0.3]
wheelScale = scale(wheelScale, s)
wheelDensity = 600
wheelX1 = add_vector(carX, [1.75, -0.7, 0.9])
wheel1 = addRigidBody(scene, '../models/sphere.obj', 1, coScale=wheelScale, scale=wheelScale, 
                     translation=wheelX1, dynamic=1, rest=restitution, friction=frict, density=wheelDensity)

wheelX2 = add_vector(carX, [1.75, -0.7, -0.9])
wheel2 = addRigidBody(scene, '../models/sphere.obj', 1, coScale=wheelScale, scale=wheelScale, 
                     translation=wheelX2, dynamic=1, rest=restitution, friction=frict, density=wheelDensity)

wheelX3 = add_vector(carX, [-1.75, -0.7, 0.9])
wheel3 = addRigidBody(scene, '../models/sphere.obj', 1, coScale=wheelScale, scale=wheelScale, 
                     translation=wheelX3, dynamic=1, rest=restitution, friction=frict, density=wheelDensity)

wheelX4 = add_vector(carX, [-1.75, -0.7, -0.9])
wheel4 = addRigidBody(scene, '../models/sphere.obj', 1, coScale=wheelScale, scale=wheelScale, 
                     translation=wheelX4, dynamic=1, rest=restitution, friction=frict, density=wheelDensity)

motorX1 = add_vector(carX, [1.75, -0.7, 0.7])
motorX2 = add_vector(carX, [1.75, -0.7, -0.7])
motorX3 = add_vector(carX, [-1.75, -0.7, 0.7])
motorX4 = add_vector(carX, [-1.75, -0.7, -0.7])
addTargetVelocityMotorHingeJoint(scene, dBody1, wheel1, motorX1, [0, 0, 1], 10.0)
addTargetVelocityMotorHingeJoint(scene, dBody2, wheel2, motorX2, [0, 0, 1], 10.0)
addTargetVelocityMotorHingeJoint(scene, dBody3, wheel3, motorX3, [0, 0, 1], 10.0)
addTargetVelocityMotorHingeJoint(scene, dBody4, wheel4, motorX4, [0, 0, 1], 10.0)

addDamperJoint(scene, chassis, dBody1, [0, 1, 0], 500000.0)
addDamperJoint(scene, chassis, dBody2, [0, 1, 0], 500000.0)
addDamperJoint(scene, steeringBody, dBody3, [0, 1, 0], 500000.0)
addDamperJoint(scene, steeringBody, dBody4, [0, 1, 0], 500000.0)

writeScene(scene, 'CarScene.json')
