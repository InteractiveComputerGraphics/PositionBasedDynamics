#include "Utilities.h"
#include <algorithm>
#include <vector>

using namespace PBD;
using namespace std;

std::string Utilities::getFilePath(const std::string &path)
{
	std::string npath =	normalizePath(path);

	std::string result = npath;
	size_t i = result.rfind('.', result.length());
	if (i != std::string::npos)
	{
		result = result.substr(0, i);
	}
	size_t p1 = result.rfind('\\', result.length());
	size_t p2 = result.rfind('/', result.length());
	if ((p1 != std::string::npos) && (p2 != std::string::npos))
		result = result.substr(0, std::max(p1, p2));
	else if (p1 != std::string::npos)
		result = result.substr(0, p1);
	else if (p2 != std::string::npos)
		result = result.substr(0, p2);
	return result;
}

std::string Utilities::getFileName(const std::string &path)
{
	std::string npath = normalizePath(path);

	std::string result = npath;
	size_t i = result.rfind('.', result.length());
	if (i != std::string::npos)
	{
		result = result.substr(0, i);
	}
	size_t p1 = result.rfind('\\', result.length());
	size_t p2 = result.rfind('/', result.length());
	if ((p1 != std::string::npos) && (p2 != std::string::npos))
		result = result.substr(std::max(p1, p2)+1, result.length());
	else if (p1 != std::string::npos)
		result = result.substr(p1+1, result.length());
	else if (p2 != std::string::npos)
		result = result.substr(p2+1, result.length());
	return result;
}

bool Utilities::isRelativePath(const std::string &path)
{
	std::string npath = normalizePath(path);

	// Windows
	size_t i = npath.find(':', npath.length());
	if (i != std::string::npos)
		return false;
	else if (npath[0] == '/')
		return false;
	return true;
}

std::string Utilities::normalizePath(const std::string &path)
{
	std::string result = path;
	std::replace(result.begin(), result.end(), '\\', '/'); 
	std::vector<std::string> tokens;
	tokenize(result, tokens, "/");
	unsigned int index = 0;
	while (index < tokens.size())
	{
		if ((tokens[index] == "..") && (index > 0))
		{
			tokens.erase(tokens.begin() + index-1, tokens.begin() + index + 1);
			index--;
		}
		index++;
	}
	result = "";
	if (path[0] == '/')
		result = "/";
	result = result + tokens[0];
	for (unsigned int i = 1; i < tokens.size(); i++)
		result = result + "/" + tokens[i];

	return result;
}

void Utilities::tokenize(const string& str, vector<string>& tokens, const string& delimiters)
{
	string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	string::size_type pos = str.find_first_of(delimiters, lastPos);

	while (string::npos != pos || string::npos != lastPos)
	{
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		lastPos = str.find_first_not_of(delimiters, pos);
		pos = str.find_first_of(delimiters, lastPos);
	}
}
