#ifndef __PLYLoader_h__
#define __PLYLoader_h__

#include <string>
#include "Logger.h"
#include "extern/happly/happly.h"
#include <array>

namespace Utilities
{
	/** \brief Read for PLY files. 
	*/
	class PLYLoader
	{
	public:
		/** This function loads an PLY file.
		  * Only triangulated meshes are supported.
		  */

		static void loadPly(const std::string &filename, std::vector<std::array<float, 3>> &x, std::vector<std::array<int, 3>> &faces, const std::array<float, 3>&scale)
		{
			LOG_INFO << "Loading " << filename;
			
			happly::PLYData plyIn(filename.c_str());
			std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
			std::vector<std::vector<int>> fInd = plyIn.getFaceIndices<int>();

			x.resize(vPos.size());
			for (unsigned int i = 0; i < vPos.size(); i++)
			{
				x[i] = {
					scale[0] * static_cast<float>(vPos[i][0]),
					scale[1] * static_cast<float>(vPos[i][1]),
					scale[2] * static_cast<float>(vPos[i][2])
				};
			}

			faces.resize(fInd.size());
			for (unsigned int i = 0; i < fInd.size(); i++)
				faces[i] = { static_cast<int>(fInd[i][0]), static_cast<int>(fInd[i][1]), static_cast<int>(fInd[i][2]) };
		}

	};
}
 
#endif