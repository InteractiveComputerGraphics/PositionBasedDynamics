#ifndef __LogWindow_h__
#define __LogWindow_h__

#include "Common/Common.h"
#include "Utils/Logger.h"
#include <vector>
#include "imgui.h"

struct ImFont;

namespace PBD 
{	
	class LogWindow 
	{
	protected:
		std::shared_ptr<Utilities::BufferSink> m_bufferSink;
		bool m_scrollToBottom;
		size_t m_lastSize;
		int m_selectedFilter;

	public:
		LogWindow();
		~LogWindow();

		void drawWindow(ImFont *textFont);
		int getSelectedFilter() const { return m_selectedFilter; }
		void setSelectedFilter(const int i) { m_selectedFilter = i; }
	};
}

#endif