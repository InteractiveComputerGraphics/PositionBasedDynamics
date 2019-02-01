#ifndef __MINIGL_H__
#define __MINIGL_H__

#include "Common/Common.h"
#include <Eigen/Geometry> 
#include "Shader.h"
#include "extern/AntTweakBar/include/AntTweakBar.h"
#include <vector>

#ifdef USE_DOUBLE
#define glNormal3v glNormal3dv
#define glVertex3v glVertex3dv
#define glVertex3 glVertex3d
#define glMultMatrix glMultMatrixd
#define glGetRealv glGetDoublev
#define glLoadMatrix glLoadMatrixd
#define glTranslate glTranslated
#define GL_REAL GL_DOUBLE
#define TW_TYPE_REAL TW_TYPE_DOUBLE
#define TW_TYPE_DIR3R TW_TYPE_DIR3D
#else
#define glNormal3v glNormal3fv
#define glVertex3v glVertex3fv
#define glVertex3 glVertex3f
#define glMultMatrix glMultMatrixf
#define glGetRealv glGetFloatv
#define glLoadMatrix glLoadMatrixf
#define glTranslate glTranslatef
#define GL_REAL GL_FLOAT
#define TW_TYPE_REAL TW_TYPE_FLOAT
#define TW_TYPE_DIR3R TW_TYPE_DIR3F
#endif


namespace PBD
{
	class MiniGL
	{
	#define MAX_KEY_FUNC 30
	#define IMAGE_ROWS 128
	#define IMAGE_COLS 128

	private:
		struct Line
		{
			Vector3r a;
			Vector3r b;
			float color[4];
			float lineWidth;
		};

		struct Point
		{
			Vector3r a;
			float color[4];
			float pointSize;
		};

		struct Triangle
		{
			Vector3r a;
			Vector3r b;
			Vector3r c;
			float color[4];
		};

		static float fovy;
		static float znear;
		static float zfar;
		static void (*scenefunc)(void);
		static void (*exitfunc)(void);
		static void (*idlefunc)(void);
		static void (*keyfunc [MAX_KEY_FUNC])(void);
		static unsigned char key [MAX_KEY_FUNC];
		static int numberOfKeyFunc;
		static int idlefunchz;
		static int width;
		static int height;
		static Vector3r m_translation;
		static Quaternionr m_rotation;
		static Real m_zoom;
		static Real movespeed;
		static Real turnspeed;
		static int mouse_button;
		static int modifier_key;
		static int mouse_pos_x_old;
		static int mouse_pos_y_old;
		static int drawMode;
		static unsigned char texData[IMAGE_ROWS][IMAGE_COLS][3];		
		static unsigned int m_texId;
		static void(*selectionfunc) (const Eigen::Vector2i&, const Eigen::Vector2i&, void*);
		static void* selectionfuncClientData;
		static void(*mousefunc)(int, int, void*);
		static int mouseFuncButton;		
		static Eigen::Vector2i m_selectionStart;
		static TwBar *m_tweakBar;
		static Real m_time;
		static Real m_quat[4];
		static GLint m_context_major_version;
		static GLint m_context_minor_version;
		static GLint m_context_profile;
		static bool m_breakPointActive;
		static bool m_breakPointLoop;
		static std::vector<Point> m_drawPoints;
		static std::vector<Line> m_drawLines;
		static std::vector<Triangle> m_drawTriangle;

		static void reshape (int w, int h);
		static void idle ();
		static void keyboard (unsigned char k, int x, int y);
		static void special (int k, int x, int y);
		static void mousePress (int button, int state, int x, int y);
		static void mouseMove (int x, int y);
		static void mouseWheel(int button, int dir, int x, int y);

		static void breakPointMainLoop();
		static void drawElements();
		
	public:
		static void getOpenGLVersion(int &major_version, int &minor_version);
		static void coordinateSystem ();
		static void hsvToRgb(float h, float s, float v, float *rgb);
		static void drawVector(const Vector3r &a, const Vector3r &b, const float w, float *color);
		static void drawCylinder(const Vector3r &a, const Vector3r &b, const float *color, const float radius = 0.02, const unsigned int subdivisions = 8);
		static void drawSphere(const Vector3r &translation, float radius, float *color, const unsigned int subDivision = 16);
		static void drawTorus(const Vector3r &translation, float innerRadius, float outerRadius, float *color, const unsigned int nsides = 16, const unsigned int rings = 16);
		static void drawQuad (const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &d, const Vector3r &norm, float *color);
		static void drawTetrahedron(const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &d, float *color);
		static void drawTriangle (const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &norm, float *color);
		static void drawGrid(float *color);
		static void drawBitmapText (float x, float y, const char *str, int strLength, float *color);
		static void drawStrokeText(const Real x, const Real y, const Real z, float scale, const char *str, int strLength, float *color);
		static void drawStrokeText (const Vector3r &pos, float scale, const char *str, int strLength, float *color);
		static void drawCube (const Vector3r &translation, const Matrix3r &rotation, float width, float height, float depth, float *color);		
		static void drawPoint (const Vector3r &translation, const float pointSize, const float * const color);
		static void setViewport (float pfovy, float pznear, float pzfar, const Vector3r &peyepoint, const Vector3r &plookat);
		static void setViewport (float pfovy, float pznear, float pzfar);
		static void setClientSceneFunc (void (*func)(void));
		static void display ();
		static void setClientIdleFunc (int hz, void (*func) (void));
		static void setKeyFunc (int nr, unsigned char k, void (*func) (void));
		static void init(int argc, char **argv, const int width, const int height, const int posx, const int posy, const char *name);
		static void destroy ();
		static void viewport ();
		static void initLights ();
		static Shader *createShader(const std::string &vertexShader, const std::string &geometryShader, const std::string &fragmentShader);
		static bool checkOpenGLVersion(const int major_version, const int minor_version);
		static void initTexture ();
		static void bindTexture();
		static void unbindTexture();
		static void move (Real x, Real y, Real z);
		static void rotateX (Real x);
		static void rotateY (Real y);
		static void setProjectionMatrix (int width, int height);
		static void drawTime(const Real time);
		static void setSelectionFunc(void(*func) (const Eigen::Vector2i&, const Eigen::Vector2i&, void*), void *clientData);
		static void setMouseMoveFunc(int button, void(*func) (int, int, void*));
		static void unproject(const int x, const int y, Vector3r &pos);
		static float getZNear();
		static float getZFar();

		static void setBreakPointActive(const bool active);
		static void breakPoint();
		static void clearPoints();
		static void clearLines();
		static void clearTriangles();
		static void clearElements();
		static void addPoint(const Vector3r &a, const float pointSize, const float *color);
		static void addLine(const Vector3r &a, const Vector3r &b, const float lineWidth, const float *color);
		static void addTriangle(const Vector3r &a, const Vector3r &b, const Vector3r &c, const float *color);

		static void initTweakBar();
		static void initTweakBarParameters();
		static TwBar *getTweakBar();
		static void cleanupTweakBar();
		static void TW_CALL setWireframeCB(const void *value, void *clientData);
		static void TW_CALL getWireframeCB(void *value, void *clientData);		
		static void TW_CALL setRotationCB(const void *value, void *clientData);
		static void TW_CALL getRotationCB(void *value, void *clientData);		
	};
}

#endif
