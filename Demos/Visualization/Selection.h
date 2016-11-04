#ifndef __SELECTION_H__
#define __SELECTION_H__

#include "MiniGL.h"

#ifdef WIN32
#define NOMINMAX
#include "windows.h"
#endif

#ifdef __APPLE__
#include <OpenGL/GL.h>
#include <OpenGL/GLU.h>
#else
#include "GL/gl.h"
#include "GL/glu.h"
#endif

#include <vector>


namespace PBD
{
	class Selection
	{
	public:
		Selection(){}
		~Selection(){}

		struct SelectionPlane
		{
			Vector3r normal;
			Real d;
		};

		template<typename PositionIteratorType>
		static void selectRect(const Eigen::Vector2i &start, const Eigen::Vector2i &end, const PositionIteratorType &posBegin, const PositionIteratorType &posEnd, std::vector<unsigned int> &hits)
		{
			int ip1y = end[1];
			int ip2y = start[1];
			int ileft = end[0] > start[0] ?	start[0] : end[0];
			int iright = end[0] < start[0] ? start[0] : end[0];
			int itop = ip1y > ip2y ? ip1y : ip2y;
			int ibottom = ip1y < ip2y ? ip1y : ip2y;

			float left = (float)ileft;
			float right = (float)iright;
			float top = (float)itop;
			float bottom = (float)ibottom;

			if (left != right && top != bottom)
			{
				GLint viewport[4];
				GLdouble mv[16],pm[16];
				glGetIntegerv(GL_VIEWPORT, viewport);
				glGetDoublev(GL_MODELVIEW_MATRIX, mv);
				glGetDoublev(GL_PROJECTION_MATRIX, pm);

				GLdouble resx,resy,resz;
				float zNear = MiniGL::getZNear();
				float zFar = MiniGL::getZFar();
				gluUnProject(left, viewport[3] - top, zNear , mv, pm, viewport, &resx, &resy, &resz);
				const Vector3r vector0((Real)resx, (Real)resy, (Real)resz);
				gluUnProject(left, viewport[3] - top, zFar , mv, pm, viewport, &resx, &resy, &resz);
				const Vector3r vector1((Real)resx, (Real)resy, (Real)resz);
				gluUnProject(left, viewport[3] - bottom, zNear , mv, pm, viewport, &resx, &resy, &resz);
				const Vector3r vector2((Real)resx, (Real)resy, (Real)resz);
				gluUnProject(right, viewport[3] - top, zNear , mv, pm, viewport, &resx, &resy, &resz);
				const Vector3r vector3((Real)resx, (Real)resy, (Real)resz);
				gluUnProject(right, viewport[3] - bottom, zNear , mv, pm, viewport, &resx, &resy, &resz);
				const Vector3r vector4((Real)resx, (Real)resy, (Real)resz);
				gluUnProject(right, viewport[3] - bottom, zFar , mv, pm, viewport, &resx, &resy, &resz);
				const Vector3r vector5((Real)resx, (Real)resy, (Real)resz);

				SelectionPlane plane[4];
				plane[0].normal = (vector3-vector0).cross(vector1-vector0);
				plane[0].d = -plane[0].normal.dot(vector0);
				plane[1].normal = (vector5-vector4).cross(vector3-vector4);
				plane[1].d = -plane[1].normal.dot(vector4);
				plane[2].normal = (vector2-vector4).cross(vector5-vector4);
				plane[2].d = -plane[2].normal.dot(vector4);
				plane[3].normal = (vector1-vector0).cross(vector2-vector0);
				plane[3].d = -plane[3].normal.dot(vector0);

				processSelectionHits(plane, posBegin, posEnd, hits);
			}
		}

		template<typename PositionIteratorType>
		static void processSelectionHits(SelectionPlane *planes, const PositionIteratorType &posBegin, const PositionIteratorType &posEnd, std::vector<unsigned int> &hits)
		{
			/////determine selected vertices//////////			
			PositionIteratorType posIter;
			unsigned int index = 0;
			posIter = posBegin;
			while (true) 
			{
				bool inQuad = true;
				const Vector3r &p = *posIter;
				for (int j = 0; j < 4; ++j)
				{
					const Real d = planes[j].normal.dot(p) + planes[j].d;
					inQuad &= (d < 0);
					if (!inQuad)
						break;
				}
				if (inQuad)
				{
					hits.push_back(index);			
				}
				index++;

				if (posIter == posEnd)
					break;
				posIter++;
			}
		}
	};
}

#endif
