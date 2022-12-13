#include "LogWindow.h"
#include "imgui_internal.h"


using namespace std;
using namespace PBD;
using namespace Utilities;


LogWindow::LogWindow()
{	
	m_lastSize = 0;
	m_selectedFilter = 1;
}

LogWindow::~LogWindow(void)
{
}

void LogWindow::drawWindow(ImFont* textFont)
{
	if (m_bufferSink != 0)
	{
		std::vector<std::pair<Utilities::LogLevel, std::string>>& buffer = m_bufferSink->getBuffer();

		float alpha = 0.8f;
		if (ImGui::IsWindowDocked())
			alpha = 1.0f;
		ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, alpha));

		ImGui::Begin("Log");

		if (ImGui::Button("Clear")) 
			m_bufferSink->clearBuffer();
		ImGui::SameLine();
		bool copy = ImGui::Button("Copy");
		ImGui::SameLine();
		
		const char* items[] = { "Debug", "Info", "Warning", "Error" };
		const char* currentItem = items[m_selectedFilter];  
		ImGui::PushItemWidth(200.0f);
		if (ImGui::BeginCombo("Filter", currentItem))
		{
			for (int n = 0; n < IM_ARRAYSIZE(items); n++)
			{
				const bool is_selected = (m_selectedFilter == n);
				if (ImGui::Selectable(items[n], is_selected))
				{
					if (n != m_selectedFilter)
						m_selectedFilter = n;
				}
				if (is_selected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}
		ImGui::PopItemWidth();


		ImGui::Separator();
		ImGui::BeginChild("Scrolling");
		ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, 1));
		if (copy) 
			ImGui::LogToClipboard();

		ImGui::PushFont(textFont);
		for (size_t i = 0; i < buffer.size(); i++)
		{
			if ((m_selectedFilter == 0) && (buffer[i].first == LogLevel::DEBUG))
				ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), buffer[i].second.c_str());
			else if ((m_selectedFilter <= 1) && (buffer[i].first == LogLevel::INFO))
				ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 1.0f), buffer[i].second.c_str());
			else if ((m_selectedFilter <= 2) && (buffer[i].first == LogLevel::WARN))
				ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), buffer[i].second.c_str());
			else if ((m_selectedFilter <= 3) && (buffer[i].first == LogLevel::ERR))
				ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), buffer[i].second.c_str());
		}
		ImGui::PopFont();
		// if there are new lines, scroll to bottom
		if (buffer.size() > m_lastSize)
		{
			m_lastSize = buffer.size();
			ImGui::SetScrollHereY(1.0f);
		}
		ImGui::PopStyleVar();
		ImGui::EndChild();
		ImGui::PopStyleColor(1);
		ImGui::End();
	}
	else
	{
		auto& sinks = Utilities::logger.getSinks();
		for (auto it = sinks.begin(); it != sinks.end(); it++)
		{
			m_bufferSink = dynamic_pointer_cast<BufferSink>(*it);
			if (m_bufferSink != 0)
				break;
		}
	}
}
