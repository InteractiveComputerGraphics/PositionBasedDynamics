#ifndef __StringTools_h__
#define __StringTools_h__

#include <vector>
#include <iostream>

namespace Utilities
{
	/** \brief Tools to handle std::string objects 
	*/
	class StringTools
	{
	public:

		static void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters = " ")
		{
			std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
			std::string::size_type pos = str.find_first_of(delimiters, lastPos);

			while (std::string::npos != pos || std::string::npos != lastPos)
			{
				tokens.push_back(str.substr(lastPos, pos - lastPos));
				lastPos = str.find_first_not_of(delimiters, pos);
				pos = str.find_first_of(delimiters, lastPos);
			}
		}

	};
}

#endif
