#ifndef __MINIGL_H__
#define __MINIGL_H__

#include "Demos/Utils/Config.h"
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "extern/AntTweakBar/AntTweakBar.h"


namespace PBD
{
	class MiniGL
	{
	#define MAX_KEY_FUNC 30
	#define IMAGE_ROWS 128
	#define IMAGE_COLS 128

	private:
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
		static Eigen::Vector3f m_translation;
		static Eigen::Quaternionf m_rotation;
		static float m_zoom;
		static float movespeed;
		static float turnspeed;
		static int mouse_button;
		static int modifier_key;
		static int mouse_pos_x_old;
		static int mouse_pos_y_old;
		static int drawMode;
		static unsigned char texData[IMAGE_ROWS][IMAGE_COLS][3];		
		static unsigned int m_texId;
		static void(*selectionfunc) (const Eigen::Vector2i&, const Eigen::Vector2i&);
		static void(*mousefunc)(int, int);
		static int mouseFuncButton;		
		static Eigen::Vector2i m_selectionStart;
		static TwBar *m_tweakBar;
		static float m_time;
		static float m_quat[4];

		static void reshape (int w, int h);
		static void idle ();
		static void keyboard (unsigned char k, int x, int y);
		static void special (int k, int x, int y);
		static void mousePress (int button, int state, int x, int y);
		static void mouseMove (int x, int y);
		static void mouseWheel(int button, int dir, int x, int y);
		
	public:
		static void coordinateSystem ();
		static void drawVector (const Eigen::Vector3f &a, const Eigen::Vector3f &b, const float w, float *color);
		static void drawSphere (const Eigen::Vector3f &translation, float radius, float *color, const unsigned int subDivision =  16);
		static void drawQuad (const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c, const Eigen::Vector3f &d, const Eigen::Vector3f &norm, float *color);
		static void drawTetrahedron(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c, const Eigen::Vector3f &d, float *color);
		static void drawTriangle (const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c, const Eigen::Vector3f &norm, float *color);
		static void drawBitmapText (float x, float y, const char *str, int strLength, float *color);
		static void drawStrokeText (const float x, const float y, const float z, float scale, const char *str, int strLength, float *color);
		static void drawStrokeText (const Eigen::Vector3f &pos, float scale, const char *str, int strLength, float *color);
		static void drawCube (const Eigen::Vector3f &translation, const Eigen::Matrix3f &rotation, float width, float height, float depth, float *color);		
		static void drawPoint (const Eigen::Vector3f &translation, const float pointSize, const float * const color);
		static void setViewport (float pfovy, float pznear, float pzfar, const Eigen::Vector3f &peyepoint, const Eigen::Vector3f &plookat);
		static void setViewport (float pfovy, float pznear, float pzfar);
		static void setClientSceneFunc (void (*func)(void));
		static void display ();
		static void setClientIdleFunc (int hz, void (*func) (void));
		static void setKeyFunc (int nr, unsigned char k, void (*func) (void));
		static void init (int argc, char **argv, int width, int height, int posx, int posy, char *name);
		static void destroy ();
		static void viewport ();
		static void initLights ();
		static void initTexture ();
		static void bindTexture();
		static void unbindTexture();
		static void move (float x, float y, float z);
		static void rotateX (float x);
		static void rotateY (float y);
		static void setProjectionMatrix (int width, int height);
		static void drawTime(const float time);
		static void setSelectionFunc(void(*func) (const Eigen::Vector2i&, const Eigen::Vector2i&));
		static void setMouseMoveFunc(int button, void(*func) (int, int));
		static void unproject(const int x, const int y, Eigen::Vector3f &pos);
		static float getZNear();
		static float getZFar();

		static void initTweakBar();
		static TwBar *getTweakBar();
		static void cleanupTweakBar();
		static void TW_CALL setWireframeCB(const void *value, void *clientData);
		static void TW_CALL getWireframeCB(void *value, void *clientData);		
		static void TW_CALL setRotationCB(const void *value, void *clientData);
		static void TW_CALL getRotationCB(void *value, void *clientData);		
	};
}

#endif
