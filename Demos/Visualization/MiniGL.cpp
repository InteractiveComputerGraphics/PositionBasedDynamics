#include "MiniGL.h"

#ifdef WIN32
#include "windows.h"
#else
#include <cstdio>
#endif

#include <glad/gl.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#ifdef __APPLE__
#include <OpenGL/GL.h>
#else
#include "GL/gl.h"
#endif

#define _USE_MATH_DEFINES

#include "math.h"
#include <iostream>
#include "Utils/Logger.h"

using namespace PBD;

float MiniGL::fovy = 45;
float MiniGL::znear = 0.5f;
float MiniGL::zfar = 1000;
MiniGL::SceneFct MiniGL::scenefunc = nullptr;
MiniGL::IdleFct MiniGL::idlefunc = nullptr;
MiniGL::DestroyFct MiniGL::destroyfunc = nullptr;
void (*MiniGL::exitfunc)(void) = NULL;
int MiniGL::m_width = 0;
int MiniGL::m_height = 0;
int MiniGL::m_windowWidth = 0;
int MiniGL::m_windowHeight = 0;
Real MiniGL::m_devicePixelRatio = 1.0;
Quaternionr MiniGL::m_rotation;
Real MiniGL::m_zoom = 1.0;
Vector3r MiniGL::m_translation;
Real MiniGL::movespeed = 1.0;
Real MiniGL::turnspeed = 0.01;
int MiniGL::mouse_button = -1;
double MiniGL::mouse_wheel_pos = 0;
int MiniGL::modifier_key = 0;
double MiniGL::mouse_pos_x_old = 0;
double MiniGL::mouse_pos_y_old = 0;
std::vector<MiniGL::KeyFunction> MiniGL::keyfunc;
int MiniGL::drawMode = GL_FILL;
unsigned char MiniGL::texData[IMAGE_ROWS][IMAGE_COLS][3];
unsigned int MiniGL::m_texId = 0;
void(*MiniGL::selectionfunc)(const Vector2i&, const Vector2i&, void*) = NULL;
void *MiniGL::selectionfuncClientData = NULL;
void(*MiniGL::mousefunc)(int, int, void*) = NULL;
int MiniGL::mouseFuncButton;
Vector2i MiniGL::m_selectionStart;
GLint MiniGL::m_context_major_version = 0;
GLint MiniGL::m_context_minor_version = 0;
GLint MiniGL::m_context_profile = 0;
bool MiniGL::m_breakPointActive = true;
bool MiniGL::m_breakPointLoop = false;
std::vector<MiniGL::ReshapeFct> MiniGL::m_reshapeFct;
std::vector<MiniGL::KeyboardFct> MiniGL::m_keyboardFct;
std::vector<MiniGL::CharFct> MiniGL::m_charFct;
std::vector<MiniGL::MousePressFct> MiniGL::m_mousePressFct;
std::vector<MiniGL::MouseMoveFct> MiniGL::m_mouseMoveFct;
std::vector<MiniGL::MouseWheelFct> MiniGL::m_mouseWheelFct;
GLFWwindow* MiniGL::m_glfw_window = nullptr;
std::vector<MiniGL::Triangle> MiniGL::m_drawTriangle;
std::vector<MiniGL::Line> MiniGL::m_drawLines;
std::vector<MiniGL::Point> MiniGL::m_drawPoints;
bool MiniGL::m_vsync = false;
double MiniGL::m_lastTime;
Shader MiniGL::m_shader;
Shader MiniGL::m_shader_screen;
Matrix4r MiniGL::m_modelview_matrix;
Matrix4r MiniGL::m_projection_matrix;
Vector3f MiniGL::m_ambientIntensity;
unsigned int MiniGL::m_numLights = 0;
VectorXf MiniGL::m_diffuseIntensity;
VectorXf MiniGL::m_specularIntensity;
VectorXf MiniGL::m_lightPosition;
GLuint MiniGL::m_vao = 0;
GLuint MiniGL::m_vbo_vertices = 0;
GLuint MiniGL::m_vbo_normals = 0;
GLuint MiniGL::m_vbo_texcoords = 0;
GLuint MiniGL::m_vbo_faces = 0;


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


void MiniGL::coordinateSystem() 
{
	Vector3r a(0,0,0);
	Vector3r b(2,0,0);
	Vector3r c(0,2,0);
	Vector3r d(0,0,2);
	float lineWidth = 3.0;
	Vector3f color;

	color << 1,0,0;
	drawVector(a, b, lineWidth, &color(0));
	color << 0,1,0;
	drawVector(a, c, lineWidth, &color(0));
	color << 0,0,1;
	drawVector(a, d, lineWidth, &color(0));
}

void MiniGL::drawVector(const Vector3r &a, const Vector3r &b, const float w, float *color)
{
	float thickness = 0.005f * w;

	// Draw a thick line as a cylinder with constant color.
	drawCylinder(a, b, color, thickness, 32, false);
}

void MiniGL::drawCylinder(const Vector3r &a, const Vector3r &b, const float *color, const float radius, const unsigned int subdivisions, const bool lighting)
{
	Vector3f diffcolor(color);
	Vector3f speccolor(1.0, 1.0, 1.0);

	// To simplify computations, the cylinder of height v is generated
	// along the z axis from z=0 to z=v and then transformed to the axis ab.
	Vector3r ab = b - a;
	Real v = ab.norm();
	Eigen::Transform<Real, 3, Eigen::Affine> transform =
		Eigen::Translation<Real, 3>(a) *
		Quaternionr::FromTwoVectors(Vector3r(0.0, 0.0, v), ab);
	Vector3r xMid = transform * Vector3r(0.0, 0.0, 0.5 * v);

	// Both the lateral surface and the base disks are subdivided into n slices (cf. gluCylinder & gluDisk).
	// For this purpose, the base circle is parametrized as a function of the angle theta.
	// The lateral slices are again subdivided into two triangles each for rendering.
	// Smooth normals are obtained by going outward from the midpoint.
	unsigned int n = subdivisions;
	VectorXr vertices((n+1) * 2 * 3);
	VectorXr normals((n+1) * 2 * 3);
	std::vector<unsigned int> faces;
	unsigned int iMidBottom = 2 * n;
	unsigned int iMidTop = 2 * n + 1;
	for (unsigned int i = 0; i < n; i++)
	{
		Real theta = (Real)i / (Real)n * 2.0 * M_PI;
		Vector3r x(radius * cos(theta), radius * sin(theta), 0.0);
		Vector3r xBottom = transform * x;
		x(2) = v;
		Vector3r xTop = transform * x;
		vertices.segment<3>(2 * 3 * i) = xBottom;
		vertices.segment<3>(2 * 3 * i + 3) = xTop;
		normals.segment<3>(2 * 3 * i) = (xBottom - xMid).normalized();
		normals.segment<3>(2 * 3 * i + 3) = (xTop - xMid).normalized();

		// [TESSELLATION]
		//      iMidTop
		//     /        \
		//  iTop ---- iNextTop
		//    |   \      |
		//    |     \    |
		// iBottom - iNextBottom
		//     \        /
		//     iMidBottom
		unsigned int iBottom = 2 * i;
		unsigned int iTop = 2 * i + 1;
		unsigned int iNextBottom = (2 * (i+1)) % (2 * n);
		unsigned int iNextTop = (2 * (i+1) + 1) % (2 * n);
		faces.insert(faces.end(), {iTop, iNextTop, iMidTop});
		faces.insert(faces.end(), {iBottom, iNextBottom, iTop});
		faces.insert(faces.end(), {iTop, iNextBottom, iNextTop});
		faces.insert(faces.end(), {iBottom, iMidBottom, iNextBottom});
	}
	Vector3r xMidBottom = transform * Vector3r(0.0, 0.0, 0.0);
	Vector3r xMidTop = transform * Vector3r(0.0, 0.0, v);
	vertices.segment<3>(3 * iMidBottom) = xMidBottom;
	vertices.segment<3>(3 * iMidTop) = xMidTop;
	normals.segment<3>(3 * iMidBottom) = (xMidBottom - xMid).normalized();
	normals.segment<3>(3 * iMidTop) = (xMidTop - xMid).normalized();

	enableShader(diffcolor, diffcolor, speccolor, 100.0);
	if (!lighting) glUniform1i(m_shader.getUniform("lighting"), GL_FALSE);
	supplyVertices(0, (n+1) * 2, &vertices(0));
	supplyNormals(1, (n+1) * 2, &normals(0));
	supplyFaces(faces.size(), &faces[0]);
	glDrawElements(GL_TRIANGLES, faces.size(), GL_UNSIGNED_INT, (void*)0);
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	disableShader();
}

void MiniGL::drawSphere(const Vector3r &translation, float radius, float *color, const unsigned int subDivision)
{
	Vector3f diffcolor(color);
	Vector3f speccolor(1.0, 1.0, 1.0);

	// The surface of the sphere is subdivided into n slices and stacks (cf. gluSphere).
	// For this purpose, it is parametrized as a function of the longitude theta and the colatitude phi.
	// The slices and stacks are again subdivided into two triangles each for rendering.
	// Smooth normals are obtained by going outward from the midpoint.
	unsigned int n = subDivision;
	VectorXr vertices((n+1) * n * 3);
	VectorXr normals((n+1) * n * 3);
	std::vector<unsigned int> faces;
	for (unsigned int i = 0; i <= n; i++)
	{
		Real phi = (Real)i / (Real)n * M_PI;
		Real xy = radius * sin(phi);
		Vector3r x(xy, xy, radius * cos(phi));
		for (unsigned int j = 0; j < n; j++)
		{
			Real theta = (Real)j / (Real)n * 2.0 * M_PI;
			x(0) = xy * cos(theta);
			x(1) = xy * sin(theta);
			vertices.segment<3>(n * 3 * i + 3 * j) = x + translation;
			normals.segment<3>(n * 3 * i + 3 * j) = x.normalized();

			if (i < n)
			{
				// [TESSELATION]
				//  iTop ---- iNextTop
				//    |   \      |
				//    |     \    |
				// iBottom - iNextBottom
				unsigned int iBottom = n * i + j;
				unsigned int iTop = n * (i+1) + j;
				unsigned int iNextBottom = n * i + ((j+1) % n);
				unsigned int iNextTop = n * (i+1) + ((j+1) % n);
				faces.insert(faces.end(), {iBottom, iNextBottom, iTop});
				faces.insert(faces.end(), {iTop, iNextBottom, iNextTop});
			}
		}
	}

	enableShader(diffcolor, diffcolor, speccolor, 100.0);
	supplyVertices(0, (n+1) * n, &vertices(0));
	supplyNormals(1, (n+1) * n, &normals(0));
	supplyFaces(faces.size(), &faces[0]);
	glDrawElements(GL_TRIANGLES, faces.size(), GL_UNSIGNED_INT, (void*)0);
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	disableShader();
}

void MiniGL::drawPoint(const Vector3r &translation, const float pointSize, const float * const color)
{	
	Vector3f diffcolor(color);
	Vector3f speccolor(1.0, 1.0, 1.0);

	enableShader(diffcolor, diffcolor, speccolor, 100.0, pointSize);
	supplyVertices(0, 1, &translation(0));
	glDrawArrays(GL_POINTS, 0, 1);
	glDisableVertexAttribArray(0);
	disableShader();
}

void MiniGL::drawMesh(const std::vector<Vector3r> &vertices, const std::vector<unsigned int> &faces,
		const std::vector<Vector3r> &vertexNormals, const float * const color)
{
	// draw mesh 
	supplyVertices(0, vertices.size(), &vertices[0][0]);
	if (vertexNormals.size() > 0)
	{
		supplyNormals(2, vertexNormals.size(), &vertexNormals[0][0]);
	}
	supplyFaces(faces.size(), faces.data());

	glDrawElements(GL_TRIANGLES, (GLsizei) faces.size(), GL_UNSIGNED_INT, (void*)0);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(2);
}

void MiniGL::drawQuad(const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &d, const Vector3r &norm, float *color)
{
	// The quad is subdivided into two triangles for rendering.
	drawTriangle(a, b, c, norm, color);
	drawTriangle(a, c, d, norm, color);
}

void MiniGL::drawTriangle (const Vector3r &a, const Vector3r &b, const Vector3r &c, const Vector3r &norm, float *color)
{
	VectorXr triangle(3 * 3);
	triangle << a, b, c;

	Vector3f diffcolor(color);
	Vector3f speccolor(1.0, 1.0, 1.0);

	glVertexAttrib3rv(1, &norm(0));

	enableShader(diffcolor, diffcolor, speccolor, 100.0);
	supplyVertices(0, 3, &triangle(0));
	glDrawArrays(GL_TRIANGLES, 0, 3);
	glDisableVertexAttribArray(0);
	disableShader();

	glVertexAttrib3r(1, 0.0, 0.0, 1.0);
}

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

void MiniGL::drawGrid_xz(float *color)
{
	Vector3f diffcolor(color);
	Vector3f speccolor(1.0, 1.0, 1.0);

	const int size = 5;

	VectorXr grid((2*size + 1) * 4 * 3);
	for (int i = -size; i <= size; i++)
	{
		grid.segment<12>((i+size) * 12) << (float) i, 0.0f, (float) -size,
										   (float) i, 0.0f, (float) size,
										   (float) -size, 0.0f, (float) i,
										   (float) size, 0.0f, (float) i;
	}
	enableShader(diffcolor, diffcolor, speccolor, 100.0);
	glUniform1i(m_shader.getUniform("lighting"), GL_FALSE);
	supplyVertices(0, (2*size + 1) * 4, &grid(0));
	glDrawArrays(GL_LINES, 0, (2*size + 1) * 4);
	glDisableVertexAttribArray(0);
	disableShader();

	float lineWidth = 3.0;

	drawVector(Vector3r(-size, 0.0, 0.0), Vector3r(0.0, 0.0, 0.0), lineWidth, color);
	drawVector(Vector3r(2.0, 0.0, 0.0), Vector3r(size, 0.0, 0.0), lineWidth, color);
	drawVector(Vector3r(0.0, 0.0, -size), Vector3r(0.0, 0.0, 0.0), lineWidth, color);
	drawVector(Vector3r(0.0, 0.0, 2.0), Vector3r(0.0, 0.0, size), lineWidth, color);
}

void MiniGL::drawGrid_xy(float *color)
{
	Vector3f diffcolor(color);
	Vector3f speccolor(1.0, 1.0, 1.0);

	const int size = 5;

	VectorXr grid((2*size + 1) * 4 * 3);
	for (int i = -size; i <= size; i++)
	{
		grid.segment<12>((i+size) * 12) << (float)i, (float)-size, 0.0f,
										   (float)i, (float)size, 0.0f,
										   (float)-size, (float)i, 0.0f,
										   (float)size, (float)i, 0.0f;
	}
	enableShader(diffcolor, diffcolor, speccolor, 100.0);
	glUniform1i(m_shader.getUniform("lighting"), GL_FALSE);
	supplyVertices(0, (2*size + 1) * 4, &grid(0));
	glDrawArrays(GL_LINES, 0, (2*size + 1) * 4);
	glDisableVertexAttribArray(0);
	disableShader();

	float lineWidth = 3.0;

	drawVector(Vector3r(-size, 0.0, 0.0), Vector3r(0.0, 0.0, 0.0), lineWidth, color);
	drawVector(Vector3r(2.0, 0.0, 0.0), Vector3r(size, 0.0, 0.0), lineWidth, color);
	drawVector(Vector3r(0.0, -size, 0.0), Vector3r(0.0, 0.0, 0.0), lineWidth, color);
	drawVector(Vector3r(0.0, 2.0, 0.0), Vector3r(0.0, size, 0.0), lineWidth, color);
}

void MiniGL::setViewport(float pfovy, float pznear, float pzfar, const Vector3r &peyepoint, const Vector3r &plookat)
{
	fovy = pfovy;
	znear = pznear;
	zfar = pzfar;

	// Compute the lookAt modelview matrix (cf. gluLookAt).
	Vector3r f = (plookat - peyepoint).normalized();
	Vector3r up(0.0, 1.0, 0.0);
	Vector3r s = f.cross(up);
	Vector3r u = s.normalized().cross(f);
	m_modelview_matrix.setIdentity();
	m_modelview_matrix.block<1,3>(0,0) = s;
	m_modelview_matrix.block<1,3>(1,0) = u;
	m_modelview_matrix.block<1,3>(2,0) = -f;
	m_modelview_matrix(0,3) = -s.dot(peyepoint);
	m_modelview_matrix(1,3) = -u.dot(peyepoint);
	m_modelview_matrix(2,3) = f.dot(peyepoint);
	
	const Matrix4r& transformation = m_modelview_matrix;
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
}

void MiniGL::setViewport(float pfovy, float pznear, float pzfar)
{
	fovy = pfovy;
	znear = pznear;
	zfar = pzfar;
}

void MiniGL::setClientSceneFunc (SceneFct func)
{
	scenefunc = func;
}

void MiniGL::init(int argc, char **argv, const int width, const int height, const char *name, const bool vsync, const bool maximized)
{
	fovy = 60;
	znear = 0.5f;
	zfar = 1000;

	m_width = width;
	m_height = height;
	m_vsync = vsync;

	scenefunc = nullptr;

	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		exit(EXIT_FAILURE);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_SAMPLES, 4);

	if (maximized)
		glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

	if (m_vsync)
		glfwWindowHint(GLFW_DOUBLEBUFFER, GL_TRUE);
	else
		glfwWindowHint(GLFW_DOUBLEBUFFER, GL_FALSE);

	m_glfw_window = glfwCreateWindow(width, height, name, NULL, NULL);
	if (!m_glfw_window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(m_glfw_window);
	gladLoadGL(glfwGetProcAddress);
	glfwSwapInterval(0);

	glfwSetFramebufferSizeCallback(m_glfw_window, reshape);

	getOpenGLVersion(m_context_major_version, m_context_minor_version);
	glGetIntegerv(GL_CONTEXT_PROFILE_MASK, &m_context_profile);

	LOG_INFO << "OpenGL version " << m_context_major_version << "." << m_context_minor_version;
	LOG_INFO << "Vendor: " << glGetString(GL_VENDOR);
	LOG_INFO << "Renderer: " << glGetString(GL_RENDERER);
	LOG_INFO << "Version: " << glGetString(GL_VERSION);

	glEnable(GL_MULTISAMPLE);
	glEnable (GL_DEPTH_TEST);
	glEnable (GL_BLEND); 
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glClearColor(0.4f, 0.4f, 0.4f, 1.0f);

	glfwSetKeyCallback(m_glfw_window, keyboard);
	glfwSetCharCallback(m_glfw_window, char_callback);
	glfwSetMouseButtonCallback(m_glfw_window, mousePress);
	glfwSetCursorPosCallback(m_glfw_window, mouseMove);
	glfwSetScrollCallback(m_glfw_window, mouseWheel);

	glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);

	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);
	glGenBuffers(1, &m_vbo_vertices);
	glGenBuffers(1, &m_vbo_normals);
	glGenBuffers(1, &m_vbo_texcoords);
	glGenBuffers(1, &m_vbo_faces);
	// Set the default normal (cf. glNormal).
	glVertexAttrib3r(1, 0.0, 0.0, 1.0);

	m_lastTime = glfwGetTime();
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
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, IMAGE_COLS, IMAGE_ROWS, 0, GL_RGB,
		GL_UNSIGNED_BYTE, texData);  // Create texture from image data
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	glBindTexture(GL_TEXTURE_2D, 0);
}

void MiniGL::setMouseMoveFunc(int button, void(*func) (int, int, void*))
{
	mousefunc = func;
	mouseFuncButton = button;
}


void MiniGL::setSelectionFunc(void(*func) (const Vector2i&, const Vector2i&, void*), void *clientData)
{
	selectionfunc = func;
	selectionfuncClientData = clientData;
}

void MiniGL::destroy ()
{
	glDeleteBuffers(1, &m_vbo_vertices);
	glDeleteBuffers(1, &m_vbo_normals);
	glDeleteBuffers(1, &m_vbo_texcoords);
	glDeleteBuffers(1, &m_vbo_faces);
	glDeleteVertexArrays(1, &m_vao);
}

void MiniGL::reshape (GLFWwindow* glfw_window, int w, int h)
{
	if ((w > 0) && (h > 0))
	{
		m_width = w;
		m_height = h;

		for (auto i = 0; i < m_reshapeFct.size(); i++)
			m_reshapeFct[i](m_width, m_height);
		glViewport(0, 0, m_width, m_height);
	}
}

void MiniGL::setClientIdleFunc (IdleFct func)
{
	idlefunc = func;
}

void MiniGL::setClientDestroyFunc(DestroyFct func)
{
	destroyfunc = func;
}

void MiniGL::addKeyFunc (unsigned char k, std::function<void()> const& func)
{
	if (func == nullptr)
		return;
	else
		keyfunc.push_back({ func, k });
}

void MiniGL::keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	// Check if registered listener wants the event
	for (auto i=0; i < m_keyboardFct.size(); i++)
	{
		if (m_keyboardFct[i](key, scancode, action, mods)) 
			return;
	}

	if (key == GLFW_KEY_ESCAPE)
	{
		m_breakPointLoop = false;
		m_breakPointActive = false;
#ifndef __APPLE__
		leaveMainLoop();
#else
		exit(0);
#endif
		return;
	}
	else if (key == GLFW_KEY_A)
		move (0, 0, movespeed);
	else if (key == GLFW_KEY_Y)
		move (0, 0, -movespeed);
	else if (key == GLFW_KEY_UP)
		move(0, -movespeed, 0);
	else if (key == GLFW_KEY_DOWN)
		move(0, movespeed, 0);
	else if (key == GLFW_KEY_LEFT)
		move(movespeed, 0, 0);
	else if (key == GLFW_KEY_RIGHT)
		move(-movespeed, 0, 0);
	else if (key == GLFW_KEY_F5)
		m_breakPointLoop = false;	
}

void MiniGL::char_callback(GLFWwindow* window, unsigned int codepoint)
{
	// Check if registered listener wants the event
	for (auto i = 0; i < m_charFct.size(); i++)
	{
		if (m_charFct[i](codepoint, GLFW_PRESS))
			return;
	}

	for (int i = 0; i < keyfunc.size(); i++)
	{
		if (codepoint == keyfunc[i].key)
			keyfunc[i].fct();
		else if (codepoint == GLFW_KEY_1)
			rotateX(-turnspeed);
		else if (codepoint == GLFW_KEY_2)
			rotateX(turnspeed);
		else if (codepoint == GLFW_KEY_3)
			rotateY(-turnspeed);
		else if (codepoint == GLFW_KEY_4)
			rotateY(turnspeed);
	}
}

void MiniGL::setProjectionMatrix (int width, int height) 
{
	// Compute the perspective projection matrix (cf. gluPerspective).
	Real aspect = (Real)width / (Real)height;
	Real fovy_rad = fovy * M_PI / 180.0;
	Real f = cos(0.5 * fovy_rad) / sin(0.5 * fovy_rad);
	m_projection_matrix.setZero();
	m_projection_matrix(0,0) = f / aspect;
	m_projection_matrix(1,1) = f;
	m_projection_matrix(2,2) = (zfar + znear) / (znear - zfar);
	m_projection_matrix(2,3) = 2.0 * zfar * znear / (znear - zfar);
	m_projection_matrix(3,2) = -1.0;
}

void MiniGL::viewport ()
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glfwGetFramebufferSize(m_glfw_window, &m_width, &m_height);
	getWindowSize(m_windowWidth, m_windowHeight);
	m_devicePixelRatio = static_cast<Real>(m_width) / static_cast<Real>(m_windowWidth);
	glViewport (0, 0, m_width, m_height);
	setProjectionMatrix (m_width, m_height);

	Matrix3r rot;
	rot = m_rotation.toRotationMatrix();
	Matrix4r& transform = m_modelview_matrix;
	transform.setIdentity();
	Vector3r scale(m_zoom, m_zoom, m_zoom);
	transform.block<3,3>(0,0) = rot;
	transform.block<3,1>(0,3) = m_translation;
	transform(0,0) *= scale[0];
	transform(1,1) *= scale[1];
	transform(2,2) *= scale[2];
}

void MiniGL::initLights ()
{
	m_ambientIntensity << 0.2f, 0.2f, 0.2f;

	m_numLights = 3;
	m_diffuseIntensity.resize(m_numLights * 3);
	m_specularIntensity.resize(m_numLights * 3);
	m_lightPosition.resize(m_numLights * 3);

	m_diffuseIntensity.segment<3>(0) << 0.9f, 0.0f, 0.0f;
	m_specularIntensity.segment<3>(0) << 1.0f, 1.0f, 1.0f;
	m_lightPosition.segment<3>(0) << -10.0f, 10.0f, 10.0f;

	m_diffuseIntensity.segment<3>(1 * 3) << 0.0f, 0.0f, 0.9f;
	m_specularIntensity.segment<3>(1 * 3) << 1.0f, 1.0f, 1.0f;
	m_lightPosition.segment<3>(1 * 3) << 10.0f, 10.0f, 10.0f;

	m_diffuseIntensity.segment<3>(2 * 3) << 0.0f, 0.9f, 0.0f;
	m_specularIntensity.segment<3>(2 * 3) << 1.0f, 1.0f, 1.0f;
	m_lightPosition.segment<3>(2 * 3) << 0.0f, 10.0f, 10.0f;
}

void MiniGL::initShaders(const std::string& shaderPath)
{
	Shader& shader = m_shader;
	shader.compileShaderFile(GL_VERTEX_SHADER, shaderPath + "/mini.vert");
	shader.compileShaderFile(GL_FRAGMENT_SHADER, shaderPath + "/mini.frag");
	shader.createAndLinkProgram();
	shader.begin();
	shader.addUniform("modelview_matrix");
	shader.addUniform("projection_matrix");
	shader.addUniform("pointSize");
	shader.addUniform("lighting");
	shader.addUniform("ambientIntensity");
	shader.addUniform("diffuseIntensity");
	shader.addUniform("specularIntensity");
	shader.addUniform("lightPosition");
	shader.addUniform("ambientReflectance");
	shader.addUniform("diffuseReflectance");
	shader.addUniform("specularReflectance");
	shader.addUniform("shininess");
	shader.end();

	Shader& screenShader = m_shader_screen;
	screenShader.compileShaderFile(GL_VERTEX_SHADER, shaderPath + "/mini_screen.vert");
	screenShader.compileShaderFile(GL_FRAGMENT_SHADER, shaderPath + "/mini_screen.frag");
	screenShader.createAndLinkProgram();
	screenShader.begin();
	screenShader.addUniform("width");
	screenShader.addUniform("height");
	screenShader.addUniform("color");
	screenShader.end();
}

void MiniGL::destroyShaders()
{
	m_shader.destroy();
	m_shader_screen.destroy();
}

void MiniGL::enableShader(const Vector3f& ambientReflectance, const Vector3f& diffuseReflectance, const Vector3f& specularReflectance, const float shininess, const float pointSize)
{
	Shader& shader = m_shader;
	shader.begin();
	const Matrix4f modelview_matrix(m_modelview_matrix.cast<float>());
	glUniformMatrix4fv(shader.getUniform("modelview_matrix"), 1, GL_FALSE, &modelview_matrix(0,0));
	const Matrix4f projection_matrix(m_projection_matrix.cast<float>());
	glUniformMatrix4fv(shader.getUniform("projection_matrix"), 1, GL_FALSE, &projection_matrix(0,0));
	glUniform1f(shader.getUniform("pointSize"), pointSize);
	glUniform1i(shader.getUniform("lighting"), GL_TRUE);
	glUniform3fv(shader.getUniform("ambientIntensity"), 1, &m_ambientIntensity(0));
	glUniform3fv(shader.getUniform("diffuseIntensity"), m_numLights, &m_diffuseIntensity(0));
	glUniform3fv(shader.getUniform("specularIntensity"), m_numLights, &m_specularIntensity(0));
	glUniform3fv(shader.getUniform("lightPosition"), m_numLights, &m_lightPosition(0));
	glUniform3fv(shader.getUniform("ambientReflectance"), 1, &ambientReflectance(0));
	glUniform3fv(shader.getUniform("diffuseReflectance"), 1, &diffuseReflectance(0));
	glUniform3fv(shader.getUniform("specularReflectance"), 1, &specularReflectance(0));
	glUniform1f(shader.getUniform("shininess"), shininess);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_PROGRAM_POINT_SIZE);
}

void MiniGL::disableShader()
{
	Shader& shader = m_shader;
	shader.end();
}

void MiniGL::enableScreenShader(const Vector3f& color)
{
	Shader& shader = m_shader_screen;
	shader.begin();
	glUniform1f(shader.getUniform("width"), static_cast<float>(m_windowWidth));
	glUniform1f(shader.getUniform("height"), static_cast<float>(m_windowHeight));
	glUniform3fv(shader.getUniform("color"), 1, &color(0));
}

void MiniGL::disableScreenShader()
{
	Shader& shader = m_shader_screen;
	shader.end();
}

void MiniGL::supplyVectors(GLuint index, GLuint vbo, unsigned int dim, unsigned int n, const float* data)
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, n * dim * sizeof(float), data, GL_STREAM_DRAW);
	glVertexAttribPointer(index, dim, GL_FLOAT, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(index);
}

void MiniGL::supplyVectors(GLuint index, GLuint vbo, unsigned int dim, unsigned int n, const double* data)
{
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, n * dim * sizeof(double), data, GL_STREAM_DRAW);
	glVertexAttribPointer(index, dim, GL_DOUBLE, GL_FALSE, 0, (void*)0);
	glEnableVertexAttribArray(index);
}

void MiniGL::supplyIndices(GLuint vbo, unsigned int n, const unsigned int* data)
{
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, n * sizeof(unsigned int), data, GL_STREAM_DRAW);
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

void MiniGL::mousePress(GLFWwindow* window, int button, int action, int mods)
{
	//getting cursor position
	glfwGetCursorPos(m_glfw_window, &mouse_pos_x_old, &mouse_pos_y_old);

	// Check if registered listener wants the event
	for (auto i = 0; i < m_mousePressFct.size(); i++)
	{
		if (m_mousePressFct[i](button, action, mods))
			return;
	}

	modifier_key = mods;

	if (action == GLFW_PRESS)
		mouse_button = button;
	else 
		mouse_button = -1;

	if ((selectionfunc != NULL) && (modifier_key == 0))
	{
		if (button == GLFW_MOUSE_BUTTON_1)
		{
			if (action == GLFW_PRESS)
				m_selectionStart = Vector2i(mouse_pos_x_old, mouse_pos_y_old);
			else
			{
				if (m_selectionStart[0] != -1)
				{
					const Vector2i pos(mouse_pos_x_old, mouse_pos_y_old);
					selectionfunc(m_selectionStart, pos, selectionfuncClientData);
				}
				m_selectionStart = Vector2i(-1, -1);
			}
		}
	}
}

void MiniGL::mouseWheel(GLFWwindow* window, double xoffset, double yoffset)
{
	mouse_wheel_pos += yoffset;

	// Check if registered listener wants the event
	for (auto i = 0; i < m_mouseWheelFct.size(); i++)
	{
		if (m_mouseWheelFct[i](static_cast<int>(mouse_wheel_pos), xoffset, yoffset))
			return;
	}

	if (yoffset > 0)
		movespeed *= 2.0;
	else
		movespeed *= 0.5;
}

void MiniGL::mouseMove (GLFWwindow* window, double x, double y)
{
	// Check if registered listener wants the event
	for (auto i = 0; i < m_mouseMoveFct.size(); i++)
	{
		if (m_mouseMoveFct[i](static_cast<int>(x), static_cast<int>(y)))
			return;
	}

	double d_x = mouse_pos_x_old - x;
	double d_y = y - mouse_pos_y_old;

	if (mouse_button == GLFW_MOUSE_BUTTON_1)
	{
		// translate scene in z direction		
		if (modifier_key == GLFW_MOD_CONTROL)
		{
			move (0, 0, -static_cast<Real>(d_x + d_y) / static_cast<Real>(10.0));
		}
		// translate scene in x/y direction
		else if (modifier_key == GLFW_MOD_SHIFT)
		{
			move (-static_cast<Real>(d_x) / static_cast<Real>(20.0), -static_cast<Real>(d_y) / static_cast<Real>(20.0), 0);
		}
		// rotate scene around x, y axis
		else if (modifier_key == GLFW_MOD_ALT)
		{
			rotateX(static_cast<Real>(d_y)/ static_cast<Real>(100.0));
			rotateY(-static_cast<Real>(d_x)/ static_cast<Real>(100.0));
		}
	}

	if (mousefunc != NULL)
	{
		if ((mouseFuncButton == -1) || (mouseFuncButton == mouse_button))
			mousefunc(static_cast<int>(x), static_cast<int>(y), selectionfuncClientData);
	}

	mouse_pos_x_old = x;
	mouse_pos_y_old = y;
}


void MiniGL::unproject(const int x, const int y, Vector3r &pos)
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	unproject(
		Vector3r(
			static_cast<Real>(x) * m_devicePixelRatio,
			static_cast<Real>(viewport[3]) - static_cast<Real>(y) * m_devicePixelRatio,
			static_cast<Real>(znear)
		),
		pos
	);
}

void MiniGL::unproject(const Vector3r& win, Vector3r& pos)
{
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	// Map the specified window coordinates to object coordinates (cf. gluUnProject).
	Vector4r ndc;
	ndc(0) = static_cast<Real>(2.0) * (win(0) - static_cast<Real>(viewport[0])) / static_cast<Real>(viewport[2]) - static_cast<Real>(1.0);
	ndc(1) = static_cast<Real>(2.0) * (win(1) - static_cast<Real>(viewport[1])) / static_cast<Real>(viewport[3]) - static_cast<Real>(1.0);
	ndc(2) = static_cast<Real>(2.0) * win(2) - static_cast<Real>(1.0);
	ndc(3) = static_cast<Real>(1.0);
	Vector4r obj = (m_projection_matrix * m_modelview_matrix).inverse() * ndc;
	if (obj(3) == static_cast<Real>(0.0)) obj.setZero();
	else obj /= obj(3);

	pos = obj.segment<3>(0);
}

float MiniGL::getZNear()
{
	return znear;
}

float MiniGL::getZFar()
{
	return zfar;
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

bool MiniGL::checkOpenGLVersion(const int major_version, const int minor_version)
{
	if ((m_context_major_version > major_version) ||
		((m_context_major_version == major_version) && (m_context_minor_version >= minor_version)))
		return true;
	return false;
}

Shader *MiniGL::createShader(const std::string &vertexShader, const std::string &geometryShader, const std::string &fragmentShader)
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

void MiniGL::setBreakPointActive(const bool active)
{
	m_breakPointActive = active;
}

void MiniGL::breakPoint()
{
	breakPointMainLoop();
}

void MiniGL::error_callback(int error, const char* description)
{
	LOG_ERR << description;
}

void MiniGL::mainLoop()
{
	while (!glfwWindowShouldClose(m_glfw_window))
	{
		if (idlefunc != nullptr)
			idlefunc();

		double currentTime = glfwGetTime();
		if (currentTime - m_lastTime >= 1.0 / 60.0)  // render at maximum at 60 fps
		{
			glfwPollEvents();

			glPolygonMode(GL_FRONT_AND_BACK, drawMode);
			viewport();

			drawElements();

			if (scenefunc != nullptr)
				scenefunc();

			if (m_vsync)
				glfwSwapBuffers(m_glfw_window);
			else
				glFlush();
			m_lastTime = currentTime;
		}
	}

	if (destroyfunc != nullptr)
		destroyfunc();

	destroy();

	glfwDestroyWindow(m_glfw_window);

	glfwTerminate();
}

void MiniGL::leaveMainLoop()
{
	glfwSetWindowShouldClose(m_glfw_window, 1);
}

void MiniGL::swapBuffers()
{
	if (m_vsync)
		glfwSwapBuffers(m_glfw_window);
	else
		glFlush();
}

void MiniGL::getWindowPos(int& x, int& y)
{
	glfwGetWindowPos(m_glfw_window, &x, &y);
}

void MiniGL::getWindowSize(int& w, int& h)
{
	glfwGetWindowSize(m_glfw_window, &w, &h);
}

void MiniGL::setWindowPos(int x, int y)
{
	glfwSetWindowPos(m_glfw_window, x, y);
}

void MiniGL::setWindowSize(int w, int h)
{
	glfwSetWindowSize(m_glfw_window, w, h);
}

bool MiniGL::getWindowMaximized()
{
	return glfwGetWindowAttrib(m_glfw_window, GLFW_MAXIMIZED);
}

void MiniGL::setWindowMaximized(const bool b)
{
	if (b)
		glfwRestoreWindow(m_glfw_window);
	else
		glfwRestoreWindow(m_glfw_window);
}

void MiniGL::breakPointMainLoop()
{
	if (m_breakPointActive)
	{
		m_breakPointLoop = true;
		while (m_breakPointLoop)
		{
			glPolygonMode(GL_FRONT_AND_BACK, drawMode);
			viewport();

			if (scenefunc != nullptr)
				scenefunc();

			if (m_vsync)
				glfwSwapBuffers(m_glfw_window);
			else
				glFlush();
			glfwPollEvents();
		}
	}
}
