#include "MiniGL.h"

#ifdef WIN32
#include "windows.h"
#else
#include <cstdio>
#endif

#include "GL/glew.h"

#ifdef __APPLE__
#include <OpenGL/GL.h>
#include <OpenGL/GLU.h>
#else
#include "GL/gl.h"
#include "GL/glu.h"
#endif

#include "GL/glut.h"
#include "GL/freeglut_ext.h"

#define _USE_MATH_DEFINES

#include "math.h"
#include <iostream>
#include "Utils/Logger.h"

using namespace PBD;

float MiniGL::fovy = 45;
float MiniGL::znear = 0.5f;
float MiniGL::zfar = 1000;
void (*MiniGL::scenefunc)(void) = NULL;
void (*MiniGL::idlefunc)(void) = NULL;
void (*MiniGL::exitfunc)(void) = NULL;
int MiniGL::idlefunchz = 0;
int MiniGL::width = 0;
int MiniGL::height = 0;
Quaternionr MiniGL::m_rotation;
Real MiniGL::m_zoom = 1.0;
Vector3r MiniGL::m_translation;
Real MiniGL::movespeed = 1.0;
Real MiniGL::turnspeed = 0.01;
int MiniGL::mouse_button = -1;
int MiniGL::modifier_key = 0;
int MiniGL::mouse_pos_x_old = 0;
int MiniGL::mouse_pos_y_old = 0;
void (*MiniGL::keyfunc [MAX_KEY_FUNC])(void) = {NULL, NULL};
unsigned char MiniGL::key [MAX_KEY_FUNC] = {0,0};
int MiniGL::numberOfKeyFunc = 0;
int MiniGL::drawMode = GL_FILL;
TwBar *MiniGL::m_tweakBar = NULL;
Real MiniGL::m_time = 0.0;
Real MiniGL::m_quat[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
unsigned char MiniGL::texData[IMAGE_ROWS][IMAGE_COLS][3];
unsigned int MiniGL::m_texId = 0;
void(*MiniGL::selectionfunc)(const Eigen::Vector2i&, const Eigen::Vector2i&, void*) = NULL;
void *MiniGL::selectionfuncClientData = NULL;
void(*MiniGL::mousefunc)(int, int, void*) = NULL;
int MiniGL::mouseFuncButton;
Eigen::Vector2i MiniGL::m_selectionStart;
GLint MiniGL::m_context_major_version = 0;
GLint MiniGL::m_context_minor_version = 0;
GLint MiniGL::m_context_profile = 0;
bool MiniGL::m_breakPointActive = true;
bool MiniGL::m_breakPointLoop = false;
std::vector<MiniGL::Triangle> MiniGL::m_drawTriangle;
std::vector<MiniGL::Line> MiniGL::m_drawLines;
std::vector<MiniGL::Point> MiniGL::m_drawPoints;



void MiniGL::drawTime( const Real time )
{
	m_time = (Real) time;
	TwRefreshBar(m_tweakBar);
}

void MiniGL::bindTexture()
{
	glBindTexture(GL_TEXTURE_2D, MiniGL::m_texId);
}

void MiniGL::unbindTexture()
{
	glBindTexture(GL_TEXTURE_2D, 0);
}

void MiniGL::getOpenGLVersion(int &major_version, int &minor_version)
{
	sscanf((const char*)glGetString(GL_VERSION), "%d.%d", &major_version, &minor_version);
}

void MiniGL::hsvToRgb(float h, float s, float v, float *rgb)
{
	int i = (int)floor(h * 6);
	float f = h * 6 - i;
	float p = v * (1 - s);
	float q = v * (1 - f * s);
	float t = v * (1 - (1 - f) * s);

	switch (i % 6)
	{
	case 0: rgb[0] = v, rgb[1] = t, rgb[2] = p; break;
	case 1: rgb[0] = q, rgb[1] = v, rgb[2] = p; break;
	case 2: rgb[0] = p, rgb[1] = v, rgb[2] = t; break;
	case 3: rgb[0] = p, rgb[1] = q, rgb[2] = v; break;
	case 4: rgb[0] = t, rgb[1] = p, rgb[2] = v; break;
	case 5: rgb[0] = v, rgb[1] = p, rgb[2] = q; break;
	}
}


void MiniGL::coordinateSystem() 
{
	Eigen::Vector3f a(0,0,0);
	Eigen::Vector3f b(2,0,0);
	Eigen::Vector3f c(0,2,0);
	Eigen::Vector3f d(0,0,2);

	float diffcolor [4] = {1,0,0,1};
	float speccolor [4] = {1,1,1,1};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, diffcolor);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, diffcolor);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glLineWidth (2);

	glBegin (GL_LINES);
		glVertex3fv (&a[0]);
		glVertex3fv (&b[0]);
	glEnd ();

	float diffcolor2[4] = { 0, 1, 0, 1 };
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, diffcolor2);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, diffcolor2);

	glBegin (GL_LINES);
		glVertex3fv (&a[0]);
		glVertex3fv (&c[0]);
	glEnd ();

	float diffcolor3[4] = { 0, 0, 1, 1 };
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, diffcolor3);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, diffcolor3);

	glBegin (GL_LINES);
		glVertex3fv (&a[0]);
		glVertex3fv (&d[0]);
	glEnd ();
	glLineWidth (1);
}

void MiniGL::drawVector(const Vector3r &a, const Vector3r &b, const float w, float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glLineWidth (w);

	glBegin (GL_LINES);
		glVertex3v(&a[0]);
		glVertex3v(&b[0]);
	glEnd ();
	
	glLineWidth (1);
}

/**
* Renders a closed cylinder between two points.
*/
void MiniGL::drawCylinder(const Vector3r &a, const Vector3r &b, const float *color, const float radius, const unsigned int subdivisions)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	Real vx = (b.x() - a.x());
	Real vy = (b.y() - a.y());
	Real vz = (b.z() - a.z());
	//handle the degenerate case with an approximation
	if (vz == 0)
		vz = .00000001;
	Real v = sqrt(vx*vx + vy*vy + vz*vz);
	Real ax = static_cast<Real>(57.2957795)*acos(vz / v);
	if (vz < 0.0)
		ax = -ax;
	Real rx = -vy*vz;
	Real ry = vx*vz;

	GLUquadricObj *quadric = gluNewQuadric();
	gluQuadricNormals(quadric, GLU_SMOOTH);

	glPushMatrix();
	glTranslatef((float)a.x(), (float)a.y(), (float)a.z());
	glRotatef((float)ax, (float)rx, (float)ry, 0.0f);
	//draw the cylinder
	gluCylinder(quadric, radius, radius, v, subdivisions, 1);
	gluQuadricOrientation(quadric, GLU_INSIDE);
	//draw the first cap
	gluDisk(quadric, 0.0, radius, subdivisions, 1);
	glTranslatef(0, 0, (float)v);
	//draw the second cap
	gluQuadricOrientation(quadric, GLU_OUTSIDE);
	gluDisk(quadric, 0.0, radius, subdivisions, 1);
	glPopMatrix();

	gluDeleteQuadric(quadric);
}

void MiniGL::drawSphere(const Vector3r &translation, float radius, float *color, const unsigned int subDivision)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glPushMatrix ();
	glTranslated ((translation)[0], (translation)[1], (translation)[2]);
	glutSolidSphere(radius, subDivision, subDivision);
	glPopMatrix ();
}

void MiniGL::drawTorus(const Vector3r &translation, float innerRadius, float outerRadius, float *color, const unsigned int nsides, const unsigned int rings)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glPushMatrix();
	glTranslated((translation)[0], (translation)[1], (translation)[2]);
	glRotated(90.0, 1.0, 0.0, 0.0);
	glutSolidTorus(innerRadius, outerRadius, nsides, rings);
	glPopMatrix();
}

void MiniGL::drawPoint(const Vector3r &translation, const float pointSize, const float * const color)
{	
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glPointSize(pointSize);

	glBegin (GL_POINTS);
	glVertex3v(&translation[0]);
	glEnd ();

	glPointSize(1);
}


void MiniGL::drawCube(const Vector3r &translation, const Matrix3r &rotation, float width, float height, float depth, float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	Real val[16];
	val[0] = width*(rotation)(0,0); val[1] = width*(rotation)(0,1); val[2] = width*(rotation)(0,2); val[3] = 0;
	val[4] = height*(rotation)(1,0); val[5] = height*(rotation)(1,1); val[6] = height*(rotation)(1,2); val[7] = 0;
	val[8] = depth*(rotation)(2,0); val[9] = depth*(rotation)(2,1); val[10] = depth*(rotation)(2,2); val[11] = 0;
	val[12] = (translation)[0]; val[13] = (translation)[1]; val[14] = (translation)[2]; val[15] = 1;

	glPushMatrix ();
	glMultMatrix (val);
	glutSolidCube(1.0);
	glPopMatrix ();
}


void MiniGL::drawBitmapText(float x, float y, const char *str, int strLength, float *color)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glPushMatrix ();
	glLoadIdentity ();
	glMatrixMode (GL_PROJECTION);
	glPushMatrix ();
	glLoadIdentity ();
	glMatrixMode (GL_MODELVIEW);
	glRasterPos2f (x,y);

	for (int i=0; i < strLength; i++)
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, str[i]);
	glMatrixMode (GL_PROJECTION);
	glPopMatrix ();
	glMatrixMode (GL_MODELVIEW);
	glPopMatrix ();
}

void MiniGL::drawStrokeText(const Real x, const Real y, const Real z, float scale, const char *str, int strLength, float *color)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glPushMatrix ();
	glTranslate (x, y, z);
	glScalef (scale, scale, scale);

	for (int i=0; i < strLength; i++)
		glutStrokeCharacter(GLUT_STROKE_ROMAN, str[i]);
	glPopMatrix ();
}

void MiniGL::drawStrokeText(const Vector3r &pos, float scale, const char *str, int strLength, float *color)
{
	drawStrokeText(pos[0], pos[1], pos[2], scale, str, strLength, color);
}


void MiniGL::drawQuad(const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &d, const Vector3r &norm, float *color)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glBegin (GL_QUADS);
		glNormal3v(&norm[0]);
		glVertex3v(&a[0]);
		glVertex3v(&b[0]);
		glVertex3v(&c[0]);
		glVertex3v(&d[0]);
	glEnd ();
}

void MiniGL::drawTriangle (const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &norm, float *color)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glBegin (GL_TRIANGLES);
		glNormal3v(&norm[0]);
		glVertex3v(&a[0]);
		glVertex3v(&b[0]);
		glVertex3v(&c[0]);
	glEnd ();
}

/** Draw a tetrahedron.
 */
void MiniGL::drawTetrahedron(const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &d, float *color)
{
	Vector3r normal1 = (b-a).cross(c-a);
	Vector3r normal2 = (b-a).cross(d-a);
	Vector3r normal3 = (c-a).cross(d-a);
	Vector3r normal4 = (c-b).cross(d-b);
	drawTriangle(a, b, c, normal1, color);
	drawTriangle(a, b, d, normal2, color);
	drawTriangle(a, c, d, normal3, color);
	drawTriangle(b, c, d, normal4, color);
}

void MiniGL::drawGrid(float *color)
{
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	const int size = 5;

	glBegin(GL_LINES);
	for (int i = -size; i <= size; i++)
	{
		glVertex3f((float) i, 0.0f, (float) -size);
		glVertex3f((float) i, 0.0f, (float) size);
		glVertex3f((float) -size, 0.0f, (float) i);
		glVertex3f((float) size, 0.0f, (float) i);
	}
	glEnd();

	glLineWidth(3.0f);
	glBegin(GL_LINES);
	glVertex3f((float)-size, 0.0f, 0.0f);
	glVertex3f((float)size, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, (float) -size);
	glVertex3f(0.0f, 0.0f, (float) size);
	glEnd();
}

void MiniGL::setViewport(float pfovy, float pznear, float pzfar, const Vector3r &peyepoint, const Vector3r &plookat)
{
	fovy = pfovy;
	znear = pznear;
	zfar = pzfar;

	glLoadIdentity ();
	gluLookAt (peyepoint [0], peyepoint [1], peyepoint [2], plookat[0], plookat[1], plookat[2], 0, 1, 0);

	Matrix4r transformation;
	Real *lookAtMatrix = transformation.data();
	glGetRealv(GL_MODELVIEW_MATRIX, &lookAtMatrix[0]);
	
	Matrix3r rot;
	Vector3r scale;

	rot.row(0) = Vector3r (transformation(0,0), transformation(0,1), transformation(0,2));
	rot.row(1) = Vector3r (transformation(1,0), transformation(1,1), transformation(1,2));
	rot.row(2) = Vector3r (transformation(2,0), transformation(2,1), transformation(2,2));
	scale[0] = rot.col(0).norm();
	scale[1] = rot.col(1).norm();
	scale[2] = rot.col(2).norm();
	m_translation = Vector3r (transformation(0,3), transformation(1,3), transformation(2,3));

	rot.col(0) = 1.0/scale[0] * rot.col(0);
	rot.col(1) = 1.0/scale[1] * rot.col(1);
	rot.col(2) = 1.0/scale[2] * rot.col(2);

	m_zoom = scale[0];
	m_rotation = Quaternionr(rot);

	glLoadIdentity ();
}

void MiniGL::setViewport(float pfovy, float pznear, float pzfar)
{
	fovy = pfovy;
	znear = pznear;
	zfar = pzfar;
}

void MiniGL::setClientSceneFunc (void  (*func)(void))
{
	scenefunc = func;
}

void MiniGL::display ()
{
	glPolygonMode (GL_FRONT_AND_BACK, drawMode); 
	viewport ();

	drawElements();

	if (scenefunc != NULL)
		scenefunc ();

	TwDraw();  // draw the tweak bar(s)
	glutSwapBuffers();
}

void MiniGL::init(int argc, char **argv, const int width, const int height, const int posx, const int posy, const char *name)
{
	fovy = 60;
	znear = 0.5f;
	zfar = 1000;

	scenefunc = NULL;

	glutInit (&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	atexit(destroy);

	glutInitWindowSize (width, height);
	glutInitWindowPosition (posx, posy);

	glutCreateWindow(name);

	// Initialize GLEW
	glewExperimental = GL_TRUE;
	GLenum err = glewInit();

	if (GLEW_OK != err)
	{
		LOG_ERR << "Error: " << glewGetErrorString(err);
		exit(EXIT_FAILURE);
	}
	
	getOpenGLVersion(m_context_major_version, m_context_minor_version);
	glGetIntegerv(GL_CONTEXT_PROFILE_MASK, &m_context_profile);

	LOG_INFO << "OpenGL version " << m_context_major_version << "." << m_context_minor_version;
	LOG_INFO << "Using GLEW " << glewGetString(GLEW_VERSION);
	LOG_INFO << "Vendor: " << glGetString(GL_VENDOR);
	LOG_INFO << "Renderer: " << glGetString(GL_RENDERER);
	LOG_INFO << "Version: " << glGetString(GL_VERSION);


	// Initialize AntTweakBar
	// (note that AntTweakBar could also be initialized after GLUT, no matter)
	if( !TwInit(TW_OPENGL, NULL) )
	{
		// A fatal error occured    
		fprintf(stderr, "AntTweakBar initialization failed: %s\n", TwGetLastError());
		exit(1);
	}
	TwWindowSize(width, height);
	initTweakBar();

	glEnable (GL_DEPTH_TEST);
	glEnable (GL_NORMALIZE);
	glShadeModel (GL_SMOOTH);
	glEnable (GL_BLEND); 
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//glClearColor (0.95f, 0.95f, 1.0f, 1.0f);
	glClearColor(0.4f, 0.4f, 0.4f, 1.0f);

	glutReshapeFunc (reshape);
	glutKeyboardFunc (keyboard);
	glutMouseFunc (mousePress);
	glutMotionFunc (mouseMove);
	glutSpecialFunc (special);
	glutDisplayFunc (display);
	glutIdleFunc (idlefunc);
#ifndef __APPLE__
	glutMouseWheelFunc(mouseWheel);
#endif

	// after GLUT initialization
	// directly redirect GLUT events to AntTweakBar
	glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT); // same as MouseMotion

	// send the ''glutGetModifers'' function pointer to AntTweakBar
	TwGLUTModifiersFunc(glutGetModifiers);

	glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

#ifndef __APPLE__
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif
}

void MiniGL::initTexture()
{
	int value;
	for (int row = 0; row < IMAGE_ROWS; row++)
	{
		for (int col = 0; col < IMAGE_COLS; col++)
		{
			if (((row & 0x8) == 0) ^ ((col & 0x8) == 0))
				value = 192;
			else
				value = 128;
// 			// Each cell is 8x8, value is 0 or 255 (black or white)
// 			value = (((row & 0x8) == 0) ^ ((col & 0x8) == 0)) * 255;
			texData[row][col][0] = (GLubyte)value;
			texData[row][col][1] = (GLubyte)value;
			texData[row][col][2] = (GLubyte)value;
		}
	}

	glGenTextures(1, &m_texId);
	glBindTexture(GL_TEXTURE_2D, m_texId);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, IMAGE_COLS, IMAGE_ROWS, 0, GL_RGB,
		GL_UNSIGNED_BYTE, texData);  // Create texture from image data
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	glEnable(GL_TEXTURE_2D);  // Enable 2D texture 

	// Correct texture distortion in perpective projection
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	glBindTexture(GL_TEXTURE_2D, 0);
}


void MiniGL::initTweakBar()
{
	// Create a tweak bar
	m_tweakBar = TwNewBar("TweakBar");
	TwDefine(" GLOBAL help='MiniGL TweakBar.' "); // Message added to the help bar.
	//TwDefine(" TweakBar size='300 900' valueswidth=120 position='5 5' color='96 200 224' text=dark "); // change default tweak bar size and color
	TwDefine(" TweakBar size='300 900' valueswidth=120 position='5 5'"); // change default tweak bar size and color
}

void MiniGL::initTweakBarParameters()
{
	TwAddVarRO(m_tweakBar, "Time", TW_TYPE_REAL, &m_time, " label='Time' precision=5");

	TwAddVarCB(m_tweakBar, "Rotation", TW_TYPE_QUAT4F, setRotationCB, getRotationCB, &m_quat,
		" label='Rotation' open help='Change the rotation.' ");

	// Add callback to toggle auto-rotate mode (callback functions are defined above).
	TwAddVarCB(m_tweakBar, "Wireframe", TW_TYPE_BOOL32, setWireframeCB, getWireframeCB, NULL,
		" label='Wireframe' key=w help='Toggle wireframe mode.' ");

}

void TW_CALL MiniGL::setWireframeCB(const void *value, void *clientData)
{
	const int val = *(const int *)(value);
	if (val == 0) 
		drawMode = GL_FILL;
	else 
		drawMode = GL_LINE;
}

void TW_CALL MiniGL::getWireframeCB(void *value, void *clientData)
{
	*(int *)(value) = drawMode == GL_LINE;
}

void TW_CALL MiniGL::setRotationCB(const void *value, void *clientData)
{
	const float *val = (const float *)(value);
	m_rotation.x() = (Real) val[0];
	m_rotation.y() = (Real) val[1];
	m_rotation.z() = (Real) val[2];
	m_rotation.w() = -(Real) val[3];

}

void TW_CALL MiniGL::getRotationCB(void *value, void *clientData)
{
	float *val = (float*)(value);
	val[0] = (float)m_rotation.x();
	val[1] = (float)m_rotation.y();
	val[2] = (float)m_rotation.z();
	val[3] = -(float)m_rotation.w();
}

void MiniGL::setMouseMoveFunc(int button, void(*func) (int, int, void*))
{
	mousefunc = func;
	mouseFuncButton = button;
}


void MiniGL::setSelectionFunc(void(*func) (const Eigen::Vector2i&, const Eigen::Vector2i&, void*), void *clientData)
{
	selectionfunc = func;
	selectionfuncClientData = clientData;
}


void MiniGL::cleanupTweakBar()
{

}

void MiniGL::destroy ()
{
	TwTerminate();
}

void MiniGL::reshape (int w, int h)
{
	if ((w > 0) && (h > 0))
	{
		width = w;
		height = h;
		glutReshapeWindow (w,h);

		TwWindowSize(width, height);
		glutPostRedisplay ();
	}
}

void MiniGL::setClientIdleFunc (int hz, void (*func) (void))
{
	if ((hz == 0) || (func == NULL))
	{
		idlefunchz = 0;
		idlefunc = NULL;
		glutIdleFunc (NULL);
	}
	else
	{
		idlefunchz = (int) ((1.0/hz)*1000.0);
		idlefunc = func;
		glutIdleFunc (idle);
	}
}

void MiniGL::setKeyFunc (int nr, unsigned char k, void (*func) (void))
{
	if ((nr >= MAX_KEY_FUNC) || (func == NULL))
	{
		return;
	}
	else
	{
		keyfunc[nr] = func;
		key[nr] = k;
		numberOfKeyFunc++;
	}
}

void MiniGL::idle ()
{
	idlefunc ();
	glutPostRedisplay ();
}

void MiniGL::keyboard (unsigned char k, int x, int y)
{
	if (TwEventKeyboardGLUT(k, x, y))  // send event to AntTweakBar
		return;

	if (k == 27)
	{
		m_breakPointLoop = false;
		m_breakPointActive = false;
#ifndef __APPLE__
		glutLeaveMainLoop();
#else
		exit(0);
#endif
		return;
	}
	else if (k == 97)
		move (0, 0, movespeed);
	else if (k == 121)
		move (0, 0, -movespeed);
	else if (k == 49)
		rotateX(-turnspeed);
	else if (k == 50)
		rotateX(turnspeed);
	else if (k == 51)
		rotateY(-turnspeed);
	else if (k == 52)
		rotateY(turnspeed);
	else 
	{
		for (int i=0; i < numberOfKeyFunc; i++)
		{
			if (k == key[i])
				keyfunc [i] ();
		}
	}
	glutPostRedisplay ();
}

void MiniGL::special (int k, int x, int y)
{
	if (TwEventSpecialGLUT(k, x, y))  // send event to AntTweakBar
		return;

	if (k == GLUT_KEY_UP)
		move (0, -movespeed, 0);
	else if (k == GLUT_KEY_DOWN)
		move (0, movespeed, 0);
	else if (k == GLUT_KEY_LEFT)
		move (movespeed, 0, 0);
	else if (k == GLUT_KEY_RIGHT)
		move (-movespeed, 0, 0);
	else if (k == GLUT_KEY_F5)
		m_breakPointLoop = false;
	glutPostRedisplay ();
}

void MiniGL::setProjectionMatrix (int width, int height) 
{ 
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity(); 
	gluPerspective(fovy, (Real)width / (Real)height, znear, zfar);
}

void MiniGL::viewport ()
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glRenderMode (GL_RENDER);
	glViewport (0, 0, width, height);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	setProjectionMatrix (width, height);
	glMatrixMode (GL_MODELVIEW);

	glTranslatef((float)m_translation[0], (float)m_translation[1], (float)m_translation[2]);
	Matrix3r rot;
	rot = m_rotation.toRotationMatrix();
	Matrix4r transform(Matrix4r::Identity());
	Vector3r scale(m_zoom, m_zoom, m_zoom);
	transform.block<3,3>(0,0) = rot;
	transform.block<3,1>(0,3) = m_translation;
	transform(0,0) *= scale[0];
	transform(1,1) *= scale[1];
	transform(2,2) *= scale[2];
	Real *transformMatrix = transform.data();
	glLoadMatrix(&transformMatrix[0]);
}

void MiniGL::initLights ()
{
	float t = 0.9f;
	float a = 0.2f;
	float amb0 [4] = {a,a,a,1};
	float diff0 [4] = {t,0,0,1};
	float spec0 [4] = {1,1,1,1};
	float pos0 [4] = {-10,10,10,1};
	glLightfv(GL_LIGHT0, GL_AMBIENT,  amb0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  diff0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, spec0);
	glLightfv(GL_LIGHT0, GL_POSITION, pos0);
	glEnable(GL_LIGHT0);

	float amb1 [4] = {a,a,a,1};
	float diff1 [4] = {0,0,t,1};
	float spec1 [4] = {1,1,1,1};
	float pos1 [4] = {10,10,10,1};
	glLightfv(GL_LIGHT1, GL_AMBIENT,  amb1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE,  diff1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, spec1);
	glLightfv(GL_LIGHT1, GL_POSITION, pos1);
	glEnable(GL_LIGHT1);

	float amb2 [4] = {a,a,a,1};
	float diff2 [4] = {0,t,0,1};
	float spec2 [4] = {1,1,1,1};
	float pos2 [4] = {0,10,10,1};
	glLightfv(GL_LIGHT2, GL_AMBIENT,  amb2);
	glLightfv(GL_LIGHT2, GL_DIFFUSE,  diff2);
	glLightfv(GL_LIGHT2, GL_SPECULAR, spec2);
	glLightfv(GL_LIGHT2, GL_POSITION, pos2);
	glEnable(GL_LIGHT2);


	glEnable(GL_LIGHTING);
	glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

}

void MiniGL::move(Real x, Real y, Real z)
{
	m_translation[0] += x;
	m_translation[1] += y;
	m_translation[2] += z;
}

void MiniGL::rotateY (Real y)
{
	AngleAxisr angleAxis(y, Vector3r(0,1,0));
	Quaternionr quat(angleAxis);
	m_rotation = m_rotation*quat;
}

void MiniGL::rotateX(Real x)
{
	AngleAxisr angleAxis(x, Vector3r(1,0,0));
	Quaternionr quat(angleAxis);
	m_rotation = quat*m_rotation;
}

void MiniGL::mousePress (int button, int state, int x, int y)
{
	if (TwEventMouseButtonGLUT(button, state, x, y))  // send event to AntTweakBar
		return;

	if (state == GLUT_DOWN)
		mouse_button = button;
	else 
		mouse_button = -1;
	modifier_key = glutGetModifiers ();

	mouse_pos_x_old = x;
	mouse_pos_y_old = y;

	if (selectionfunc != NULL)
	{
		if (button == GLUT_LEFT_BUTTON)
		{
			if (state == GLUT_DOWN)
				m_selectionStart = Eigen::Vector2i(x, y);
			else
			{
				if (m_selectionStart[0] != -1)
				{
					const Eigen::Vector2i pos(x, y);
					selectionfunc(m_selectionStart, pos, selectionfuncClientData);
				}
				m_selectionStart = Eigen::Vector2i(-1, -1);
			}
		}
	}

	glutPostRedisplay ();
}

void MiniGL::mouseWheel(int button, int dir, int x, int y)
{
	if (dir > 0)
		movespeed *= 2.0;
	else
		movespeed *= 0.5;
}

void MiniGL::mouseMove (int x, int y)
{
	if (TwEventMouseMotionGLUT(x, y))  // send event to AntTweakBar
		return;

	int d_x = mouse_pos_x_old - x;
	int d_y = y - mouse_pos_y_old;

	if (mouse_button == GLUT_LEFT_BUTTON)
	{
		// translate scene in z direction		
		if (modifier_key == GLUT_ACTIVE_CTRL)
		{
			move (0, 0, -(d_x + d_y) / static_cast<Real>(10.0));
		}
		// translate scene in x/y direction
		else if (modifier_key == GLUT_ACTIVE_SHIFT)
		{
			move (-d_x / static_cast<Real>(20.0), -d_y / static_cast<Real>(20.0), 0);
		}
		// rotate scene around x, y axis
		else if (modifier_key == GLUT_ACTIVE_ALT)
		{
			rotateX(d_y/ static_cast<Real>(100.0));
			rotateY(-d_x/ static_cast<Real>(100.0));
		}
	}

	if (mousefunc != NULL)
	{
		if ((mouseFuncButton == -1) || (mouseFuncButton == mouse_button))
			mousefunc(x, y, selectionfuncClientData);
	}

	mouse_pos_x_old = x;
	mouse_pos_y_old = y;

	glutPostRedisplay ();
}


void MiniGL::unproject(const int x, const int y, Vector3r &pos)
{
	GLint viewport[4];
	GLdouble mv[16], pm[16];
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, mv);
	glGetDoublev(GL_PROJECTION_MATRIX, pm);

	GLdouble resx, resy, resz;
	gluUnProject(x, viewport[3] - y, znear, mv, pm, viewport, &resx, &resy, &resz);
	pos[0] = (Real) resx;
	pos[1] = (Real) resy;
	pos[2] = (Real) resz;
}

float MiniGL::getZNear()
{
	return znear;
}

float MiniGL::getZFar()
{
	return zfar;
}

TwBar * MiniGL::getTweakBar()
{
	return m_tweakBar;
}

bool MiniGL::checkOpenGLVersion(const int major_version, const int minor_version)
{
	if ((m_context_major_version > major_version) ||
		((m_context_major_version == major_version) && (m_context_minor_version >= minor_version)))
		return true;
	return false;
}

Shader *MiniGL::createShader(const std::string &vertexShader, const std::string &geometryShader, const std::string &fragmentShader)
{
	if (checkOpenGLVersion(3,3))
	{
		Shader *shader = new Shader();

		if (vertexShader != "")
			shader->compileShaderFile(GL_VERTEX_SHADER, vertexShader);
		if (geometryShader != "")
			shader->compileShaderFile(GL_GEOMETRY_SHADER, geometryShader);
		if (fragmentShader != "")
			shader->compileShaderFile(GL_FRAGMENT_SHADER, fragmentShader);
		shader->createAndLinkProgram();
		return shader;
	}
	return NULL;
}

void MiniGL::drawElements()
{
	for (unsigned int i = 0; i < m_drawLines.size(); i++)
	{
		Line &l = m_drawLines[i];
		drawVector(l.a, l.b, l.lineWidth, l.color);
	}
	for (unsigned int i = 0; i < m_drawTriangle.size(); i++)
	{
		Triangle &t = m_drawTriangle[i];
		Vector3r n = ((t.b - t.a).cross(t.c - t.a));
		n.normalize();
		drawTriangle(t.a, t.b, t.c, n, t.color);
	}
	for (unsigned int i = 0; i < m_drawPoints.size(); i++)
	{
		Point &p = m_drawPoints[i];
		drawSphere(p.a, p.pointSize, p.color);
	}
}

void MiniGL::clearPoints()
{
	m_drawPoints.clear();
}

void MiniGL::clearLines()
{
	m_drawLines.clear();
}

void MiniGL::clearTriangles()
{
	m_drawTriangle.clear();
}

void MiniGL::clearElements()
{
	clearPoints();
	clearLines();
	clearTriangles();
}

void MiniGL::addPoint(const Vector3r &a, const float pointSize, const float *color)
{
	Point p;
	p.a = a;
	p.pointSize = pointSize;
	for (unsigned char i = 0; i < 4; i++)
		p.color[i] = color[i];
	m_drawPoints.push_back(p);
}

void MiniGL::addLine(const Vector3r &a, const Vector3r &b, const float lineWidth, const float *color)
{
	Line l;
	l.a = a;
	l.b = b;
	l.lineWidth = lineWidth;
	for (unsigned char i = 0; i < 4; i++)
		l.color[i] = color[i];
	m_drawLines.push_back(l);
}

void MiniGL::addTriangle(const Vector3r &a, const Vector3r &b, const Vector3r &c, const float *color)
{
	Triangle t;
	t.a = a;
	t.b = b;
	t.c = c;
	for (unsigned char i = 0; i < 4; i++)
		t.color[i] = color[i];
	m_drawTriangle.push_back(t);
}

void MiniGL::setBreakPointActive(const bool active)
{
	m_breakPointActive = active;
}

void MiniGL::breakPoint()
{
	glutPostRedisplay();
	breakPointMainLoop();
}

void MiniGL::breakPointMainLoop()
{
	if (m_breakPointActive)
	{
		m_breakPointLoop = true;
		while (m_breakPointLoop)
		{
			glutMainLoopEvent();
		}
	}
}
