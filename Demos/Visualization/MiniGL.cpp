#include "MiniGL.h"

#ifdef WIN32
#include "windows.h"
#else
#include <cstdio>
#endif

#include "GL/gl.h"
#include "GL/glu.h"
#include "GL/glut.h"
#include "GL/freeglut_ext.h"

#define _USE_MATH_DEFINES

#include "math.h"

using namespace PBD;
using namespace Eigen;

float MiniGL::fovy = 45;
float MiniGL::znear = 0.5f;
float MiniGL::zfar = 1000;
void (*MiniGL::scenefunc)(void) = NULL;
void (*MiniGL::idlefunc)(void) = NULL;
void (*MiniGL::exitfunc)(void) = NULL;
int MiniGL::idlefunchz = 0;
int MiniGL::width = 0;
int MiniGL::height = 0;
Quaternionf MiniGL::m_rotation;
float MiniGL::m_zoom = 1.0f;
Vector3f MiniGL::m_translation;
float MiniGL::movespeed = 1.0f;
float MiniGL::turnspeed = 0.01f;
int MiniGL::mouse_button = -1;
int MiniGL::modifier_key = 0;
int MiniGL::mouse_pos_x_old = 0;
int MiniGL::mouse_pos_y_old = 0;
void (*MiniGL::keyfunc [MAX_KEY_FUNC])(void) = {NULL, NULL};
unsigned char MiniGL::key [MAX_KEY_FUNC] = {0,0};
int MiniGL::numberOfKeyFunc = 0;
int MiniGL::drawMode = GL_FILL;
TwBar *MiniGL::m_tweakBar = NULL;
float MiniGL::m_time = 0.0f;
float MiniGL::m_quat[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
unsigned char MiniGL::texData[IMAGE_ROWS][IMAGE_COLS][3];
unsigned int MiniGL::m_texId = 0;
void(*MiniGL::selectionfunc)(const Eigen::Vector2i&, const Eigen::Vector2i&) = NULL;
void(*MiniGL::mousefunc)(int, int) = NULL;
int MiniGL::mouseFuncButton;
Eigen::Vector2i MiniGL::m_selectionStart;


void MiniGL::drawTime( const float time )
{
	m_time = (float) time;
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

void MiniGL::coordinateSystem() 
{
	Vector3f a(0,0,0);
	Vector3f b(2,0,0);
	Vector3f c(0,2,0);
	Vector3f d(0,0,2);

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

	float diffcolor2 [4] = {0,1,0,1};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, diffcolor2);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, diffcolor2);

	glBegin (GL_LINES);
		glVertex3fv (&a[0]);
		glVertex3fv (&c[0]);
	glEnd ();

	float diffcolor3 [4] = {0,0,1,1};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, diffcolor3);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, diffcolor3);

	glBegin (GL_LINES);
		glVertex3fv (&a[0]);
		glVertex3fv (&d[0]);
	glEnd ();
	glLineWidth (1);
}

void MiniGL::drawVector (const Vector3f &a, const Vector3f &b, const float w, float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glLineWidth (w);

	glBegin (GL_LINES);
		glVertex3fv(&a[0]);
		glVertex3fv(&b[0]);
	glEnd ();
	
	glLineWidth (1);
}

void MiniGL::drawSphere (const Vector3f &translation, float radius, float *color, const unsigned int subDivision)
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

void MiniGL::drawPoint (const Vector3f &translation, const float pointSize, const float * const color)
{	
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(color);

	glPointSize(pointSize);

	glBegin (GL_POINTS);
	glVertex3fv(&translation[0]);
	glEnd ();

	glPointSize(1);
}


void MiniGL::drawCube (const Vector3f &translation, const Matrix3f &rotation, float width, float height, float depth, float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	float val[16];
	val[0] = width*(rotation)(0,0); val[1] = width*(rotation)(0,1); val[2] = width*(rotation)(0,2); val[3] = 0;
	val[4] = height*(rotation)(1,0); val[5] = height*(rotation)(1,1); val[6] = height*(rotation)(1,2); val[7] = 0;
	val[8] = depth*(rotation)(2,0); val[9] = depth*(rotation)(2,1); val[10] = depth*(rotation)(2,2); val[11] = 0;
	val[12] = (translation)[0]; val[13] = (translation)[1]; val[14] = (translation)[2]; val[15] = 1;

	glPushMatrix ();
	glMultMatrixf (val);
	glutSolidCube(1.0);
	glPopMatrix ();
}


void MiniGL::drawBitmapText (float x, float y, const char *str, int strLength, float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
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

void MiniGL::drawStrokeText (const float x, const float y, const float z, float scale, const char *str, int strLength, float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glPushMatrix ();
	glTranslated (x, y, z);
	glScalef (scale, scale, scale);

	for (int i=0; i < strLength; i++)
		glutStrokeCharacter(GLUT_STROKE_ROMAN, str[i]);
	glPopMatrix ();
}

void MiniGL::drawStrokeText (const Vector3f &pos, float scale, const char *str, int strLength, float *color)
{
	drawStrokeText(pos[0], pos[1], pos[2], scale, str, strLength, color);
}


void MiniGL::drawQuad (const Vector3f &a, const Vector3f &b, const Vector3f &c, const Vector3f &d, const Vector3f &norm, float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glBegin (GL_QUADS);
		glNormal3fv(&norm[0]);
		glVertex3fv(&a[0]);
		glVertex3fv(&b[0]);
		glVertex3fv(&c[0]);
		glVertex3fv(&d[0]);
	glEnd ();
}

void MiniGL::drawTriangle (const Vector3f &a, const Vector3f &b, const Vector3f &c, const Vector3f &norm, float *color)
{
	float speccolor [4] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_DIFFUSE, color);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, 100.0);

	glBegin (GL_TRIANGLES);
		glNormal3fv(&norm[0]);
		glVertex3fv(&a[0]);
		glVertex3fv(&b[0]);
		glVertex3fv(&c[0]);
	glEnd ();
}

/** Draw a tetrahedron.
 */
void MiniGL::drawTetrahedron(const Vector3f &a, const Vector3f &b, const Vector3f &c, const Vector3f &d, float *color)
{
	Vector3f normal1 = (b-a).cross(c-a);
	Vector3f normal2 = (b-a).cross(d-a);
	Vector3f normal3 = (c-a).cross(d-a);
	Vector3f normal4 = (c-b).cross(d-b);
	drawTriangle(a, b, c, normal1, color);
	drawTriangle(a, b, d, normal2, color);
	drawTriangle(a, c, d, normal3, color);
	drawTriangle(b, c, d, normal4, color);
}

void MiniGL::setViewport (float pfovy, float pznear, float pzfar, const Vector3f &peyepoint, const Vector3f &plookat)
{
	fovy = pfovy;
	znear = pznear;
	zfar = pzfar;

	glLoadIdentity ();
	gluLookAt (peyepoint [0], peyepoint [1], peyepoint [2], plookat[0], plookat[1], plookat[2], 0, 1, 0);

	Matrix4f transformation;
	float *lookAtMatrix = transformation.data();
	glGetFloatv (GL_MODELVIEW_MATRIX, &lookAtMatrix[0]);
	
	Matrix3f rot;
	Vector3f scale;

	transformation.transposeInPlace();

	rot.row(0) = Vector3f (transformation(0,0), transformation(0,1), transformation(0,2));
	rot.row(1) = Vector3f (transformation(1,0), transformation(1,1), transformation(1,2));
	rot.row(2) = Vector3f (transformation(2,0), transformation(2,1), transformation(2,2));
	scale[0] = rot.col(0).norm();
	scale[1] = rot.col(1).norm();
	scale[2] = rot.col(2).norm();
	m_translation = Vector3f (transformation(3,0), transformation(3,1), transformation(3,2));

	rot.col(0) = 1.0f/scale[0] * rot.col(0);
	rot.col(1) = 1.0f/scale[1] * rot.col(1);
	rot.col(2) = 1.0f/scale[2] * rot.col(2);

	m_zoom = scale[0];
	m_rotation = Quaternionf(rot);

	glLoadIdentity ();
}

void MiniGL::setViewport (float pfovy, float pznear, float pzfar)
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

	if (scenefunc != NULL)
		scenefunc ();

	TwDraw();  // draw the tweak bar(s)
	glutSwapBuffers();
}

void MiniGL::init (int argc, char **argv, int width, int height, int posx, int posy, char *name)
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

	glutCreateWindow (name);

	glEnable (GL_DEPTH_TEST);
	glEnable (GL_NORMALIZE);
	glShadeModel (GL_SMOOTH);
	glEnable (GL_BLEND); 
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glClearColor (0.95f, 0.95f, 1.0f, 1.0f);

	glutReshapeFunc (reshape);
	glutKeyboardFunc (keyboard);
	glutMouseFunc (mousePress);
	glutMotionFunc (mouseMove);
	glutSpecialFunc (special);
	glutDisplayFunc (display);
	glutIdleFunc (idlefunc);
	glutMouseWheelFunc(mouseWheel);

	// after GLUT initialization
	// directly redirect GLUT events to AntTweakBar
	glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT); // same as MouseMotion

	// send the ''glutGetModifers'' function pointer to AntTweakBar
	TwGLUTModifiersFunc(glutGetModifiers);

	glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
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
	TwDefine(" TweakBar size='300 600' valueswidth=120 position='5 5' color='96 200 224' text=dark "); // change default tweak bar size and color

	TwAddVarRO(m_tweakBar, "Time", TW_TYPE_FLOAT, &m_time, " label='Time' precision=5");

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
	m_rotation.x() = (float) val[0];
	m_rotation.y() = (float) val[1];
	m_rotation.z() = (float) val[2];
	m_rotation.w() = -(float) val[3];

}

void TW_CALL MiniGL::getRotationCB(void *value, void *clientData)
{
	float *val = (float*)(value);
	val[0] = (float) m_rotation.x();
	val[1] = (float) m_rotation.y();
	val[2] = (float) m_rotation.z();
	val[3] = -(float) m_rotation.w();
}

void MiniGL::setMouseMoveFunc(int button, void(*func) (int, int))
{
	mousefunc = func;
	mouseFuncButton = button;
}


void MiniGL::setSelectionFunc(void(*func) (const Eigen::Vector2i&, const Eigen::Vector2i&))
{
	selectionfunc = func;
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
		glutLeaveMainLoop();
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
	glutPostRedisplay ();
}

void MiniGL::setProjectionMatrix (int width, int height) 
{ 
	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity(); 
	gluPerspective (fovy, (float)width/(float)height, znear, zfar); 
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

	glTranslatef((float) m_translation[0], (float) m_translation[1], (float) m_translation[2]);
	Matrix3f rot;
	rot = m_rotation.toRotationMatrix();
	Matrix4f transform(Matrix4f::Identity());
	Vector3f scale(m_zoom, m_zoom, m_zoom);
	transform.block<3,3>(0,0) = rot;
	transform.block<3,1>(0,3) = m_translation;
	transform(0,0) *= scale[0];
	transform(1,1) *= scale[1];
	transform(2,2) *= scale[2];
	float *transformMatrix = transform.data();
	glLoadMatrixf(&transformMatrix[0]);
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

void MiniGL::move (float x, float y, float z)
{
	m_translation[0] += x;
	m_translation[1] += y;
	m_translation[2] += z;
}

void MiniGL::rotateY (float y)
{
	AngleAxisf angleAxis(y, Vector3f(0,1,0));
	Quaternionf quat(angleAxis);
	m_rotation = m_rotation*quat;
}

void MiniGL::rotateX (float x)
{
	AngleAxisf angleAxis(x, Vector3f(1,0,0));
	Quaternionf quat(angleAxis);
	m_rotation = m_rotation*quat;
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
					selectionfunc(m_selectionStart, pos);
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
			move (0, 0, -(d_x + d_y) / 10.0f);
		}
		// translate scene in x/y direction
		else if (modifier_key == GLUT_ACTIVE_SHIFT)
		{
			move (-d_x / 20.0f, -d_y / 20.0f, 0);
		}
		// rotate scene around x, y axis
		else if (modifier_key == GLUT_ACTIVE_ALT)
		{
			rotateX(d_y/ 100.0f);
			rotateY(-d_x/ 100.0f);
		}
	}

	if (mousefunc != NULL)
	{
		if ((mouseFuncButton == -1) || (mouseFuncButton == mouse_button))
			mousefunc(x, y);
	}

	mouse_pos_x_old = x;
	mouse_pos_y_old = y;

	glutPostRedisplay ();
}


void MiniGL::unproject(const int x, const int y, Eigen::Vector3f &pos)
{
	GLint viewport[4];
	GLdouble mv[16], pm[16];
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_MODELVIEW_MATRIX, mv);
	glGetDoublev(GL_PROJECTION_MATRIX, pm);

	GLdouble resx, resy, resz;
	gluUnProject(x, viewport[3] - y, znear, mv, pm, viewport, &resx, &resy, &resz);
	pos[0] = (float) resx;
	pos[1] = (float) resy;
	pos[2] = (float) resz;
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
