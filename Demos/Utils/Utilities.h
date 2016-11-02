#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include <string>

namespace PBD
{
	class Utilities
	{
	public:
		/** Extracts the path of a file.
		*/
		static std::string getFilePath(const std::string &path);
		static bool isRelativePath(const std::string &path);
	};
}
 
#endif