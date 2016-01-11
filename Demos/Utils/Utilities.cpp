#include "Utilities.h"

using namespace PBD;
using namespace std;
 
string Utilities::getFilePath(const string &path)
{
	string result = path;
	size_t i = path.rfind('\\', path.length());
	if (i != string::npos)
	{
		result = path.substr(0, i + 1);
	}
	i = result.rfind('/', result.length());
	if (i != string::npos)
	{
		result = (result.substr(0, i + 1));
	}

	return result;
}
