#ifndef __OBJLOADER_H__
#define __OBJLOADER_H__

#include "Demos/Utils/IndexedFaceMesh.h"
#include "Demos/Simulation/ParticleData.h"
#include <string>

namespace PBD
{
	struct MeshFaceIndices
	{
		int posIndices[3];
		int texIndices[3];
		int normalIndices[3];
	};

	class OBJLoader
	{
	public:
		/** This function loads an OBJ file in an IndexedFaceMesh data structure. 
		  * Only triangulated meshes are supported at the moment. 
		  */
		static void loadObj(const std::string &filename, VertexData &vertexData, IndexedFaceMesh &mesh, const Vector3r &scale = Vector3r(1.0, 1.0, 1.0));
		static void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " ");
	};
}
 
#endif