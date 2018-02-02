#ifndef __TETGENLOADER_H__
#define __TETGENLOADER_H__

#include <string>
#include <vector>
#include "Common/Common.h"


namespace Utilities
{
	class TetGenLoader
	{
	public:
		static void loadTetFile(const std::string &filename, std::vector<Vector3r>& vertices, std::vector<unsigned int>& tets);
		static void loadTetgenModel(const std::string &nodeFilename, const std::string &eleFilename, std::vector<Vector3r>& vertices, std::vector<unsigned int>& tets);
		static void loadMSHModel(const std::string &mshFilename, std::vector<Vector3r>& vertices, std::vector<unsigned int>& tets);
	};
}
 
#endif