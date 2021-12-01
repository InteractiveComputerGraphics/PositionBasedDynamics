#ifndef __MiniGL_h__
#define __MiniGL_h__

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

struct GLFWwindow;
typedef class GLUquadric GLUquadricObj;

namespace PBD
{
	class MiniGL
	{
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

		struct KeyFunction
		{
			std::function<void()> fct;
			unsigned char key;
		};

		typedef std::function<void()> SceneFct;
		typedef std::function<void()> IdleFct;
		typedef std::function<void()> DestroyFct;
		typedef std::function<void(int, int)> ReshapeFct;
		typedef std::function<bool(int, int, int, int)> KeyboardFct;
		typedef std::function<bool(int, int)> CharFct;
		typedef std::function<bool(int, int, int)> MousePressFct;
		typedef std::function<bool(int, int)> MouseMoveFct;
		typedef std::function<bool(int, double, double)> MouseWheelFct;

		static float fovy;
		static float znear;
		static float zfar;
		static SceneFct scenefunc;
		static void (*exitfunc)(void);
		static IdleFct idlefunc; 
		static DestroyFct destroyfunc;
		static std::vector<KeyFunction> keyfunc;
		static std::vector<ReshapeFct> m_reshapeFct;
		static std::vector<KeyboardFct> m_keyboardFct;
		static std::vector<CharFct> m_charFct;
		static std::vector<MousePressFct> m_mousePressFct;
		static std::vector<MouseMoveFct> m_mouseMoveFct;
		static std::vector<MouseWheelFct> m_mouseWheelFct;
		static int m_width;
		static int m_height;
		static Vector3r m_translation;
		static Quaternionr m_rotation;
		static Real m_zoom;
		static Real movespeed;
		static Real turnspeed;
		static int mouse_button;
		static double mouse_wheel_pos;
		static int modifier_key;
		static double mouse_pos_x_old;
		static double mouse_pos_y_old;
		static int drawMode;
		static unsigned char texData[IMAGE_ROWS][IMAGE_COLS][3];		
		static unsigned int m_texId;
		static void(*selectionfunc) (const Vector2i&, const Vector2i&, void*);
		static void* selectionfuncClientData;
		static void(*mousefunc)(int, int, void*);
		static int mouseFuncButton;		
		static Vector2i m_selectionStart;
		static TwBar *m_tweakBar;
		static Real m_time;
		static Real m_quat[4];
		static int m_context_major_version;
		static int m_context_minor_version;
		static int m_context_profile;
		static bool m_breakPointActive;
		static bool m_breakPointLoop;
		static std::vector<Point> m_drawPoints;
		static std::vector<Line> m_drawLines;
		static std::vector<Triangle> m_drawTriangle;
		static GLUquadricObj* m_sphereQuadric;
		static GLFWwindow* m_glfw_window;

		static void reshape (GLFWwindow* glfw_window, int w, int h);
		static void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);		
		static void char_callback(GLFWwindow* window, unsigned int codepoint);
		static void mousePress(GLFWwindow* window, int button, int action, int mods);
		static void mouseMove (GLFWwindow* window, double x, double y);
		static void mouseWheel(GLFWwindow* window, double xoffset, double yoffset);

		static void breakPointMainLoop();
		static void drawElements();
		
	public:
		static void getOpenGLVersion(int &major_version, int &minor_version);
		static void coordinateSystem ();
		static void drawVector(const Vector3r &a, const Vector3r &b, const float w, float *color);
		/** Renders a closed cylinder between two points.
		*/
		static void drawCylinder(const Vector3r &a, const Vector3r &b, const float *color, const float radius = 0.02, const unsigned int subdivisions = 8);
		static void drawSphere(const Vector3r &translation, float radius, float *color, const unsigned int subDivision = 16);
		static void drawQuad (const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &d, const Vector3r &norm, float *color);
		/** Draw a tetrahedron.
		*/
		static void drawTetrahedron(const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &d, float *color);
		static void drawTriangle (const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &norm, float *color);
		static void drawGrid_xz(float *color);
		static void drawGrid_xy(float *color);
		static void drawPoint (const Vector3r &translation, const float pointSize, const float * const color);
		static void drawMesh(const std::vector<Vector3r> &vertices, const std::vector<unsigned int> &faces, const std::vector<Vector3r> &vertexNormals, const float * const color);
		static void setViewport (float pfovy, float pznear, float pzfar, const Vector3r &peyepoint, const Vector3r &plookat);
		static void setViewport (float pfovy, float pznear, float pzfar);
		static void setClientSceneFunc (SceneFct func);
		static void setClientIdleFunc (IdleFct func);
		static void setClientDestroyFunc(DestroyFct func);
		static void addKeyFunc(unsigned char k, std::function<void()> const& func);
		static std::vector<KeyFunction> &getKeyFunc() { return keyfunc; }
		static void init(int argc, char **argv, const int width, const int height, const char *name);
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
		static void setSelectionFunc(void(*func) (const Vector2i&, const Vector2i&, void*), void *clientData);
		static void setMouseMoveFunc(int button, void(*func) (int, int, void*));
		static void unproject(const int x, const int y, Vector3r &pos);
		static float getZNear();
		static float getZFar();
		static void hsvToRgb(float h, float s, float v, float *rgb);
		static void rgbToHsv(float r, float g, float b, float *hsv);
		static int getModifierKey() { return modifier_key; }

		static void addReshapeFunc(ReshapeFct func) { m_reshapeFct.push_back(func); }
		static std::vector<ReshapeFct> &getReshapeFunc() { return m_reshapeFct; }
		static void addKeyboardFunc(KeyboardFct func) { m_keyboardFct.push_back(func); }
		static std::vector<KeyboardFct> &getKeyboardFunc() { return m_keyboardFct; }
		static void addCharFunc(CharFct func) { m_charFct.push_back(func); }
		static std::vector<CharFct>& getCharFunc() { return m_charFct; }
		static void addMousePressFunc(MousePressFct func) { m_mousePressFct.push_back(func); }
		static std::vector<MousePressFct> &getMousePressFunc() { return m_mousePressFct; }
		static void addMouseMoveFunc(MouseMoveFct func) { m_mouseMoveFct.push_back(func); }
		static std::vector<MouseMoveFct> &getMouseMoveFunc() { return m_mouseMoveFct; }
		static void addMouseWheelFunc(MouseWheelFct func) { m_mouseWheelFct.push_back(func); }
		static std::vector<MouseWheelFct>& getMouseWheelFunc() { return m_mouseWheelFct; }

		static void setBreakPointActive(const bool active);
		static void breakPoint();

		static int getWidth() { return m_width; }
		static int getHeight() { return m_height; }
		
		static int getDrawMode() { return drawMode; }
		static void setDrawMode(int val) { drawMode = val; }
		static Quaternionr getRotation() { return m_rotation; }
		static void setRotation(Quaternionr val) { m_rotation = val; }

		static void error_callback(int error, const char* description);
		static void mainLoop();
		static void leaveMainLoop();
		static void swapBuffers();

		static GLFWwindow* getWindow() { return m_glfw_window; }
		
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
