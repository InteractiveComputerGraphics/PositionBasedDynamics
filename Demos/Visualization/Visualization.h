#ifndef __VISUALIZATION_H__
#define __VISUALIZATION_H__

#include "Common/Common.h"
#include "MiniGL.h"
#include "Utils/IndexedFaceMesh.h"

namespace PBD
{
	class Visualization
	{	
	public:
		template<class PositionData>
		static void drawMesh(const PositionData &pd, const Utilities::IndexedFaceMesh &mesh, const unsigned int offset, const float * const color);
		template<class PositionData>
		static void drawTexturedMesh(const PositionData &pd, const Utilities::IndexedFaceMesh &mesh, const unsigned int offset, const float * const color);
	};

	template<class PositionData>
	void Visualization::drawMesh(const PositionData &pd, const Utilities::IndexedFaceMesh &mesh, const unsigned int offset, const float * const color)
	{
		// draw mesh 
		const unsigned int *faces = mesh.getFaces().data();
		const unsigned int nFaces = mesh.numFaces();
		const Vector3r *vertexNormals = mesh.getVertexNormals().data();

		MiniGL::supplyVertices(0, mesh.numVertices(), &pd.getPosition(offset)[0]);
		MiniGL::supplyNormals(2, mesh.numVertices(), &vertexNormals[0][0]);
		MiniGL::supplyFaces(3 * nFaces, faces);

		glDrawElements(GL_TRIANGLES, (GLsizei)3 * nFaces, GL_UNSIGNED_INT, (void*)0);

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(2);
	}

	template<class PositionData>
	void Visualization::drawTexturedMesh(const PositionData &pd, const Utilities::IndexedFaceMesh &mesh, const unsigned int offset, const float * const color)
	{
		// draw mesh 
		const unsigned int *faces = mesh.getFaces().data();
		const unsigned int nFaces = mesh.numFaces();
		const Vector3r *vertexNormals = mesh.getVertexNormals().data();
		const Vector2r *uvs = mesh.getUVs().data();

		MiniGL::bindTexture();

		MiniGL::supplyVertices(0, mesh.numVertices(), &pd.getPosition(offset)[0]);
		MiniGL::supplyTexcoords(1, mesh.numUVs(), &uvs[0][0]);
		MiniGL::supplyNormals(2, mesh.numVertices(), &vertexNormals[0][0]);
		MiniGL::supplyFaces(3 * nFaces, faces);

		glDrawElements(GL_TRIANGLES, (GLsizei)3 * nFaces, GL_UNSIGNED_INT, (void*)0);

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);

		MiniGL::unbindTexture();
	}
}

#endif
