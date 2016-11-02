#include "Utilities.h"
#include <algorithm>

using namespace PBD;
using namespace std;

string Utilities::getFilePath(const std::string &path)
{
	std::string result = path;
	size_t i = result.rfind('.', result.length());
	if (i != std::string::npos)
	{
		result = result.substr(0, i);
	}
	size_t p1 = result.rfind('\\', result.length());
	size_t p2 = result.rfind('/', result.length());
	if ((p1 != std::string::npos) && (p2 != std::string::npos))
		result = result.substr(0, std::max(p1,p2));
	else if (p1 != std::string::npos)
		result = result.substr(0, p1);
	else if (p2 != std::string::npos)
		result = result.substr(0, p2);
	return result;
}

bool Utilities::isRelativePath(const std::string &path)
{
	// Windows
	size_t i = path.find(':', path.length());
	if (i != std::string::npos)
		return false;
	else if (path[0] == '/')
		return false;
	return true;
}