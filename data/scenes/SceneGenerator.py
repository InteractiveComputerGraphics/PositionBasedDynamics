import json
import math

######################################################
# add parameters
######################################################
def addParameters(scene, h=0.005, maxIter=5, maxIterVel=5, velocityUpdateMethod=0, contactTolerance=0.05, triangleModelSimulationMethod=2, triangleModelBendingMethod=2, 
                  contactStiffnessRigidBody=1.0, contactStiffnessParticleRigidBody=100.0, 
                  cloth_stiffness=1.0, cloth_bendingStiffness=0.005, cloth_xxStiffness=1.0, cloth_yyStiffness=1.0, cloth_xyStiffness=1.0,
                  cloth_xyPoissonRatio=0.3, cloth_yxPoissonRatio=0.3, cloth_normalizeStretch=0, cloth_normalizeShear=0, gravity=[0,-9.81,0], numberOfStepsPerRenderUpdate=4):
    parameters = {  'timeStepSize': h,
                    'gravity': gravity,
                    'maxIter' : maxIter,
				    'maxIterVel' : maxIterVel,
					'numberOfStepsPerRenderUpdate': numberOfStepsPerRenderUpdate,
				    'velocityUpdateMethod' : velocityUpdateMethod,
				    'contactTolerance': contactTolerance,
				    'triangleModelSimulationMethod': triangleModelSimulationMethod,
				    'triangleModelBendingMethod': triangleModelBendingMethod,
				    'contactStiffnessRigidBody' : contactStiffnessRigidBody,
				    'contactStiffnessParticleRigidBody': contactStiffnessParticleRigidBody,
				    'cloth_stiffness': cloth_stiffness,
				    'cloth_bendingStiffness': cloth_bendingStiffness,
				    'cloth_xxStiffness': cloth_xxStiffness,
				    'cloth_yyStiffness': cloth_yyStiffness,
				    'cloth_xyStiffness': cloth_xyStiffness,
				    'cloth_xyPoissonRatio': cloth_xyPoissonRatio,
				    'cloth_yxPoissonRatio': cloth_yxPoissonRatio,
				    'cloth_normalizeStretch': cloth_normalizeStretch,
				    'cloth_normalizeShear': cloth_normalizeShear
                }
    scene['Simulation'] = parameters
    return

######################################################
# add rigid bodies
######################################################
def addRigidBody(scene, geoFile, coType, coFile='', coScale=[1,1,1], translation=[0,0,0], axis=[1,0,0], angle=0.0, scale=[1,1,1], 
                 v=[0,0,0], omega=[0,0,0], dynamic=1, density=500, rest=0.6, friction=0.3, 
                 testMesh = 1):
    global current_id 
    rb = {  'id': current_id,
			'geometryFile': geoFile,
			'isDynamic': dynamic, 
			'density': density, 
			'translation': translation,
			'rotationAxis': axis,
			'rotationAngle': angle,
			'scale': scale,
			'velocity': v,
            'angularVelocity': omega,
			'restitution' : rest,
			'friction' : friction,
			'collisionObjectType': coType,
			'collisionObjectScale': coScale,
			'collisionObjectFileName': coFile, 
            'testMesh': testMesh
          }
    current_id += 1

    scene['RigidBodies'].append(rb)

    return current_id-1

######################################################
# add triangle models
######################################################
def addTriangleModel(scene, geoFile, translation=[0,0,0], axis=[1,0,0], angle=0.0, scale=[1,1,1], 
                     rest=0.6, friction=0.3, staticParticles=[]):
    global current_id 
    tri = { 'id': current_id,
			'geometryFile': geoFile,
			'translation': translation,
			'rotationAxis': axis,
			'rotationAngle': angle,
			'scale': scale,
            'staticParticles': staticParticles,
			'restitution' : rest,
			'friction' : friction
          }
    current_id += 1

    scene['TriangleModels'].append(tri)

    return current_id-1

######################################################
# add tet models
######################################################
def addTetModel(scene, nodeFile, eleFile, visFile='', translation=[0,0,0], axis=[1,0,0], angle=0.0, scale=[1,1,1], 
                     rest=0.6, friction=0.3, staticParticles=[]):
    global current_id 
    tet = { 'id': current_id,
			'nodeFile': nodeFile,
            'eleFile': eleFile,
			'translation': translation,
			'rotationAxis': axis,
			'rotationAngle': angle,
			'scale': scale,
            'staticParticles': staticParticles,
			'restitution' : rest,
			'friction' : friction
          }
    if visFile != '':
        tet['visFile'] = visFile

    current_id += 1

    scene['TetModels'].append(tet)

    return current_id-1

######################################################
# add ball joint
######################################################
def addBallJoint(scene, rbId1, rbId2, position):
    joint = {
		    'bodyID1': rbId1,
			'bodyID2': rbId2,
			'position': position
		}
    scene['BallJoints'].append(joint)
    return

######################################################
# add ball-on-line joint
######################################################
def addBallOnLineJoint(scene, rbId1, rbId2, position, axis):
    joint = {
		    'bodyID1': rbId1,
			'bodyID2': rbId2,
			'position': position,
            'axis': axis
		}
    scene['BallOnLineJoints'].append(joint)
    return

######################################################
# add hinge joint
######################################################
def addHingeJoint(scene, rbId1, rbId2, position, axis):
    joint = {
		    'bodyID1': rbId1,
			'bodyID2': rbId2,
			'position': position,
            'axis': axis
		}
    scene['HingeJoints'].append(joint)
    return

######################################################
# add universal joint
######################################################
def addUniversalJoint(scene, rbId1, rbId2, position, axis1, axis2):
    joint = {
		    'bodyID1': rbId1,
			'bodyID2': rbId2,
			'position': position,
            'axis1': axis1,
            'axis2': axis2
		}
    scene['UniversalJoints'].append(joint)
    return

######################################################
# add slider joint
######################################################
def addSliderJoint(scene, rbId1, rbId2, axis):
    joint = {
		    'bodyID1': rbId1,
			'bodyID2': rbId2,
            'axis': axis
		}
    scene['SliderJoints'].append(joint)
    return
	
######################################################
# add damper joint
######################################################
def addDamperJoint(scene, rbId1, rbId2, axis, stiffness):
	joint = {
			'bodyID1': rbId1,
			'bodyID2': rbId2,
			'axis': axis,
			'stiffness': stiffness
		}
	scene['DamperJoints'].append(joint)
	return	

######################################################
# add RigidBodyParticleBallJoint
######################################################
def addRigidBodyParticleBallJoint(scene, rbId, particleId):
    joint = {
		    'rbID': rbId,
			'particleID': particleId
		}
    scene['RigidBodyParticleBallJoints'].append(joint)
    return


######################################################
# add TargetAngleMotorHingeJoint
######################################################
def addTargetAngleMotorHingeJoint(scene, rbId1, rbId2, position, axis, target, targetSequence=None, repeatSequence=0):
	joint = {
			'bodyID1': rbId1,
			'bodyID2': rbId2,
			'position': position,
			'axis': axis
		}

	if targetSequence != None:
		joint['targetSequence'] = targetSequence
		joint['repeatSequence'] = repeatSequence
	else:
		joint['target'] = target
	scene['TargetAngleMotorHingeJoints'].append(joint)
	return

######################################################
# add TargetVelocityMotorHingeJoint
######################################################
def addTargetVelocityMotorHingeJoint(scene, rbId1, rbId2, position, axis, target):
    joint = {
		    'bodyID1': rbId1,
			'bodyID2': rbId2,
			'position': position,
            'axis': axis,
            'target': target
		}
    scene['TargetVelocityMotorHingeJoints'].append(joint)
    return

######################################################
# add TargetPositionMotorSliderJoint
######################################################
def addTargetPositionMotorSliderJoint(scene, rbId1, rbId2, axis, target):
    joint = {
		    'bodyID1': rbId1,
			'bodyID2': rbId2,
            'axis': axis,
            'target': target
		}
    scene['TargetPositionMotorSliderJoints'].append(joint)
    return

######################################################
# add TargetVelocityMotorSliderJoint
######################################################
def addTargetVelocityMotorSliderJoint(scene, rbId1, rbId2, axis, target):
    joint = {
		    'bodyID1': rbId1,
			'bodyID2': rbId2,
            'axis': axis,
            'target': target
		}
    scene['TargetVelocityMotorSliderJoints'].append(joint)
    return
	
######################################################
# add spring
######################################################
def addRigidBodySpring(scene, rbId1, rbId2, position1, position2, stiffness):
    joint = {
		    'bodyID1': rbId1,
			'bodyID2': rbId2,
			'position1': position1,
			'position2': position2,
			'stiffness': stiffness
		}
    scene['RigidBodySprings'].append(joint)
    return
	
######################################################
# add distance joint
######################################################
def addDistanceJoint(scene, rbId1, rbId2, position1, position2):
    joint = {
		    'bodyID1': rbId1,
			'bodyID2': rbId2,
			'position1': position1,
			'position2': position2
		}
    scene['DistanceJoints'].append(joint)
    return

######################################################
# generate scene
######################################################
def generateScene(name, camPosition=[0, 10, 30], camLookat=[0,0,0]):
    scene = {'Name' : name}
    scene['cameraPosition'] = camPosition
    scene['cameraLookat'] = camLookat
    scene['RigidBodies'] = []
    scene['BallJoints'] = []
    scene['BallOnLineJoints'] = []
    scene['HingeJoints'] = []
    scene['UniversalJoints'] = []
    scene['SliderJoints'] = []
    scene['DamperJoints'] = []
    scene['RigidBodyParticleBallJoints'] = []
    scene['TargetAngleMotorHingeJoints'] = []
    scene['TargetVelocityMotorHingeJoints'] = []
    scene['TargetPositionMotorSliderJoints'] = []
    scene['TargetVelocityMotorSliderJoints'] = []
    scene['RigidBodySprings'] = []
    scene['DistanceJoints'] = []
    scene['TriangleModels'] = []
    scene['TetModels'] = []

    return scene

######################################################
# write scene to file
######################################################
def writeScene(scene, fileName):
    f = open(fileName, 'w')
    json_str = json.dumps(scene, sort_keys=True,indent=4, separators=(',', ': '))
    f.write(json_str)
    #print json_str
    f.close()

######################################################
# compute rotation matrix
######################################################
def rotation_matrix(axis, angle):
    x = axis[0]
    y = axis[1]
    z = axis[2]
    d = math.sqrt(x*x + y*y + z*z)
    if d < 1.0e-6:
        print ("Vector of rotation matrix is zero!")
        return
    x = x/d;
    y = y/d;
    z = z/d;

    x2 = x*x;
    y2 = y*y;
    z2 = z*z;
    s = math.sin(angle);
    c = math.cos(angle);
    c1 = 1.0-c;
    xyc = x*y*c1;
    xzc = x*z*c1;
    yzc = y*z*c1;
    xs=x*s;
    ys=y*s;
    zs=z*s;

    return [[c + x2*c1, xyc-zs, xzc+ys],
			[xyc+zs, c+y2*c1, yzc-xs],
            [xzc-ys, yzc+xs, c+z2*c1]]

######################################################
# compute matrix vector product
######################################################
def matrix_vec_product(A, v):
    res = [0,0,0]
    for i in range(0,3):
        for j in range(0,3):
            res[i] += A[i][j] * v[j];
    return res

######################################################
# compute cross product
######################################################
def cross_product(a, b):
    res = [0,0,0]
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
    return res

######################################################
# scale vector
######################################################
def scale_vector(v, s):
    res = [0,0,0]
    res[0] = s*v[0];
    res[1] = s*v[1];
    res[2] = s*v[2];
    return res
	
######################################################
# add vector
######################################################
def add_vector(v1, v2):
    res = [0,0,0]
    res[0] = v1[0] + v2[0];
    res[1] = v1[1] + v2[1];
    res[2] = v1[2] + v2[2];
    return res	

current_id=1