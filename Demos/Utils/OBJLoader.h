#ifndef __OBJLOADER_H__
#define __OBJLOADER_H__

#include "Demos/Utils/IndexedFaceMesh.h"
#include "Demos/Simulation/ParticleData.h"
#include <string>

namespace PBD
{
	class OBJLoader
	{
	public:
		/** This function loads an OBJ file in an IndexedFaceMesh data structure. 
		  * Only triangulated meshes are supported at the moment. 
		  */
		static void loadObj(const std::string &filename, VertexData &vertexData, IndexedFaceMesh &mesh, const Eigen::Vector3f &scale = Eigen::Vector3f(1.0f, 1.0f, 1.0f));
	};
}
 
#endif