import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GL import shaders
import numpy as np

sphereQuadric = None
   
def initLights():
    glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.2, 0.2, 0.2, 1])
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  [1,1,1,1])
    glLightfv(GL_LIGHT0, GL_SPECULAR, [1,1,1,1])
    glLightfv(GL_LIGHT0, GL_POSITION, [3,-10,-10,0])
    glEnable(GL_LIGHT0)

    glLightfv(GL_LIGHT1, GL_AMBIENT,  [0.2, 0.2, 0.2, 1])
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  [1,1,1,1])
    glLightfv(GL_LIGHT1, GL_SPECULAR, [1,1,1,1])
    glLightfv(GL_LIGHT1, GL_POSITION, [3,10,-10,0])
    glEnable(GL_LIGHT1)

    glLightfv(GL_LIGHT2, GL_AMBIENT,  [0.2, 0.2, 0.2, 1])
    glLightfv(GL_LIGHT2, GL_DIFFUSE,  [1,1,1,1])
    glLightfv(GL_LIGHT2, GL_SPECULAR, [1,1,1,1])
    glLightfv(GL_LIGHT2, GL_POSITION, [3,-10,10,0])
    glEnable(GL_LIGHT2)
    
    glLightfv(GL_LIGHT3, GL_AMBIENT,  [0.2, 0.2, 0.2, 1])
    glLightfv(GL_LIGHT3, GL_DIFFUSE,  [1,1,1,1])
    glLightfv(GL_LIGHT3, GL_SPECULAR, [1,1,1,1])
    glLightfv(GL_LIGHT3, GL_POSITION, [3,10,10,0])
    glEnable(GL_LIGHT3)

    glEnable(GL_LIGHTING)
    glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE)
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE)
   
def drawMesh(pd, mesh, offset, color):
    global shader
    global UNIFORM_LOCATIONS
    shaders.glUseProgram(shader)

    mv = glGetFloatv(GL_MODELVIEW_MATRIX)
    glUniformMatrix4fv(UNIFORM_LOCATIONS["modelview_matrix"], 1, GL_FALSE, mv)
    pm = glGetFloatv(GL_PROJECTION_MATRIX)
    glUniformMatrix4fv(UNIFORM_LOCATIONS["projection_matrix"], 1, GL_FALSE, pm)
    glUniform3fv(UNIFORM_LOCATIONS["surface_color"], 1, color)
    glUniform1f(UNIFORM_LOCATIONS["shininess"], 5.0)
    glUniform1f(UNIFORM_LOCATIONS["specular_factor"], 0.2)
     
    faces = mesh.getFaces()

    #glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color)
    #glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color)
    #glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [1,1,1,1])
    #glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 15.0)
    #glColor3fv(color)

    v = np.array(pd.getVertices())
    vertices = np.reshape(v[offset:], -1)
    normals = np.reshape(np.negative(np.array(mesh.getVertexNormals())), -1)

    #glEnableClientState(GL_VERTEX_ARRAY)
    #glEnableClientState(GL_NORMAL_ARRAY)
    #glVertexPointer(3, GL_DOUBLE, 0, vertices)
    #glNormalPointer(GL_DOUBLE, 0, normals)
    
    glEnableVertexAttribArray(0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, vertices);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, normals);

    glDrawElements(GL_TRIANGLES, 3 * mesh.numFaces(), GL_UNSIGNED_INT, mesh.getFaces())

    #glDisableClientState(GL_VERTEX_ARRAY)
    #glDisableClientState(GL_NORMAL_ARRAY)
    glDisableVertexAttribArray(0)
    glDisableVertexAttribArray(2)
    shaders.glUseProgram(0)
    
def drawSphere(translation, radius, color, subDivision=16):
    global sphereQuadric
    speccolor = [1.0, 1.0, 1.0, 1.0]
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color)
    glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color)
    glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor)
    glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 5.0);
    glColor3fv(color)

    if (sphereQuadric == None):
        sphereQuadric = gluNewQuadric();
        gluQuadricNormals(sphereQuadric, GLU_SMOOTH);

    glPushMatrix ()
    glTranslated ((translation)[0], (translation)[1], (translation)[2])
    gluSphere(sphereQuadric, radius, subDivision, subDivision)
    glPopMatrix()

 
 
def drawText(position, textString, size = 36):     
    font = pygame.font.Font (None, size)
    textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))     
    textData = pygame.image.tostring(textSurface, "RGBA", True)     
    glRasterPos2d(*position)     
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)    
  
def initShader():
    global shader
    global UNIFORM_LOCATIONS

    VERTEX_SHADER = shaders.compileShader("""#version 330
        uniform mat4 modelview_matrix;
        uniform mat4 projection_matrix;

        layout(location = 0) in vec3 position;
        layout(location = 1) in vec3 normal;

        out VertexData
        {
           vec3 normal;
           vec3 view;
        }
        outData;

        void main()
        {
            vec4 v_position = projection_matrix * modelview_matrix * vec4(position, 1.0);
            outData.normal = mat3(modelview_matrix) * normal;
            outData.view = -v_position.xyz;
            gl_Position = v_position;
        }
        """, GL_VERTEX_SHADER)
        
    FRAGMENT_SHADER = shaders.compileShader("""#version 330
        uniform vec3 surface_color;
        uniform float shininess;
        uniform float specular_factor;

        layout(location = 0, index = 0) out vec4 frag_color;

        in VertexData
        {
            vec3 normal;
            vec3 view;
        }
        inData;

        void main()
        {
            vec3 normal = inData.normal;

            if (!gl_FrontFacing)
            {
                normal = -inData.normal;
            }
            
            const vec3 ambient = vec3(0.6, 0.6, 0.6);
            const vec3 diffuse = vec3(1.0, 1.0, 1.0);
            const vec3 specular = vec3(1.0, 1.0, 1.0);	
            
            vec3 eye_n = normalize(normal);
            vec3 eye_v = normalize(inData.view);
            const vec3 eye_l = vec3(0.0, 0.0, 1.0);	
            float cos_ln = max(dot(eye_l, eye_n), 0.0);    
            vec3 h = normalize(eye_l + eye_v);
            float cos_nh = max(dot(eye_n, h), 0.0);   
            
            vec3 color = surface_color * (ambient + diffuse * cos_ln + specular_factor * specular * pow(cos_nh, shininess));

            frag_color = vec4(color, 1.0);
        }
        """, GL_FRAGMENT_SHADER)
    shader = shaders.compileProgram(VERTEX_SHADER,FRAGMENT_SHADER)
    
    UNIFORM_LOCATIONS = {
            'modelview_matrix': glGetUniformLocation(shader, 'modelview_matrix'),
            'projection_matrix': glGetUniformLocation(shader, 'projection_matrix'),
            'surface_color': glGetUniformLocation(shader, 'surface_color'),
            'shininess': glGetUniformLocation(shader, 'shininess'),
            'specular_factor': glGetUniformLocation(shader, 'specular_factor'),
        }
        
def initGL(resX, resY):   
    pygame.init()
    display = (resX, resY)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    glEnable (GL_DEPTH_TEST)
    glEnable (GL_NORMALIZE)
    glShadeModel (GL_SMOOTH)
    glPolygonMode (GL_FRONT_AND_BACK, GL_FILL)
    
    gluPerspective(45, (display[0]/display[1]), 0.5, 50.0)
    
    initShader()
    
 
