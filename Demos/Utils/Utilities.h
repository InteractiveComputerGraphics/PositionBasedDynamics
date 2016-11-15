#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include <string>
#include <vector>

namespace PBD
{
	class Utilities
	{
	public:
		/** Extracts the path of a file.
		*/
		static std::string getFilePath(const std::string &path);
		static bool isRelativePath(const std::string &path);
		static std::string getFileName(const std::string &path);
		static std::string normalizePath(const std::string &path);
		static void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " ");
	};
}
 
#endif