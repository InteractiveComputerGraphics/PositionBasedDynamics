#ifndef __imguiParameters_h__
#define __imguiParameters_h__

#include "Common/Common.h"
#include <vector>
#include "ParameterObject.h"
#include "imgui.h"

#ifdef USE_DOUBLE
#define InputReal ImGui::InputDouble
#define ImGuiDataType_Real ImGuiDataType_Double
#else
#define InputReal ImGui::InputFloat
#define ImGuiDataType_Real ImGuiDataType_Float
#endif

namespace PBD
{
	class imguiParameters
	{
	public: 
		struct imguiParameter
		{
			std::string name;
			std::string label;
			std::string description;
			bool readOnly;

			imguiParameter() { readOnly = false; }
			virtual ~imguiParameter() {};
		};

		template<typename T>
		struct imguiNumericParameter : public imguiParameter
		{
			T minValue;
			T maxValue;
			std::function<T()> getFct;
			std::function<void(const T)> setFct;
			imguiNumericParameter() : imguiParameter() { minValue = std::numeric_limits<T>::min(); maxValue = std::numeric_limits<T>::max(); getFct = nullptr; setFct = nullptr; }
			virtual ~imguiNumericParameter() {};
		};

		struct imguiBoolParameter : public imguiParameter
		{
			std::function<bool()> getFct;
			std::function<void(const bool)> setFct;
			imguiBoolParameter() : imguiParameter() { getFct = nullptr; setFct = nullptr; }
			virtual ~imguiBoolParameter() {};
		};

		struct imguiStringParameter : public imguiParameter
		{
			std::function<std::string()> getFct;
			std::function<void(const std::string&)> setFct;
			imguiStringParameter() : imguiParameter() { getFct = nullptr; setFct = nullptr; }
			virtual ~imguiStringParameter() {};
		};

		struct imguiVec3rParameter : public imguiParameter
		{
			std::function<Vector3r()> getFct;
			std::function<void(Vector3r&)> setFct;
			imguiVec3rParameter() : imguiParameter() { getFct = nullptr; setFct = nullptr; }
			virtual ~imguiVec3rParameter() {};
		};

		struct imguiVec3fParameter : public imguiParameter
		{
			std::function<Eigen::Vector3f ()> getFct;
			std::function<void(Eigen::Vector3f&)> setFct;
			imguiVec3fParameter() : imguiParameter() { getFct = nullptr; setFct = nullptr; }
			virtual ~imguiVec3fParameter() {};
		};

		struct imguiEnumParameter : public imguiParameter
		{
			std::vector<std::string> items;
			std::function<int()> getFct;
			std::function<void(const int)> setFct;
			imguiEnumParameter() : imguiParameter() { getFct = nullptr; setFct = nullptr; }
			virtual ~imguiEnumParameter() { items.clear();  };
		};

		struct imguiFunctionParameter : public imguiParameter
		{
			std::function<void()> function;
			imguiFunctionParameter() : imguiParameter() { function = nullptr; }
			virtual ~imguiFunctionParameter() {};
		};

		static std::string m_format;
		static Real m_step;
		static Real m_faststep;
		static int m_istep;
		static int m_ifaststep;
		static std::vector<std::function<void()>> m_setFcts;

		typedef std::pair<GenParam::ParameterObject*, unsigned int> ParameterIndex;

		static void createParameterGUI();
		static void createParameterObjectGUI(GenParam::ParameterObject* paramObj);

		static void createRealParameter(imguiParameters::imguiParameter* param, ImGuiInputTextFlags flags, const std::string& helpText);
		static void createBoolParameter(imguiParameters::imguiParameter* param, ImGuiInputTextFlags flags, const std::string& helpText);
		static void createStringParameter(imguiParameters::imguiParameter* param, ImGuiInputTextFlags flags, const std::string& helpText);
		static void createVec3rParameter(imguiParameters::imguiParameter* param, ImGuiInputTextFlags flags, const std::string& helpText);
		static void createVec3fParameter(imguiParameters::imguiParameter* param, ImGuiInputTextFlags flags, const std::string& helpText);
		static bool createEnumParameter(imguiParameters::imguiParameter* param, ImGuiInputTextFlags flags, const std::string& helpText);
		template<typename T>
		static void createNumericParameter(imguiParameters::imguiParameter* param, ImGuiInputTextFlags flags, const std::string& helpText);
		static void createFunctionParameter(imguiParameters::imguiParameter* param, ImGuiInputTextFlags flags, const std::string& helpText);

		static void addParam(const std::string& group, const std::string& subgroup, imguiParameter* param);
		static void addGroup(const std::string& group, const std::string& subgroup);

		static void cleanup();

	protected:
		// vector<groupName, vector<subgroupName, vector<params>>>
		static std::vector<std::pair<std::string, std::vector<std::pair<std::string, std::vector<imguiParameter*>>>>> m_imguiParams;

		static void createSubgroupParameters(const std::vector<std::pair<std::string, std::vector<imguiParameter*>>>& params);
	};


	template<typename T>
	inline void imguiParameters::createNumericParameter(imguiParameters::imguiParameter* param, ImGuiInputTextFlags flags, const std::string& helpText)
	{
		imguiParameters::imguiNumericParameter<T>* rparam = dynamic_cast<imguiParameters::imguiNumericParameter<T>*>(param);
		if ((rparam != nullptr) && (rparam->getFct != nullptr))
		{
			int value = static_cast<int>(rparam->getFct());
			ImGui::InputInt(rparam->label.c_str(), &value, m_istep, m_ifaststep, flags);

			if (ImGui::IsItemDeactivatedAfterEdit())
			{
				if (rparam->minValue != std::numeric_limits<T>::min())
					value = std::max(static_cast<int>(rparam->minValue), value);
				if (rparam->maxValue != std::numeric_limits<T>::max())
					value = std::min(static_cast<int>(rparam->maxValue), value);
				if (rparam->setFct)
					rparam->setFct(value);
			}
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip(helpText.c_str());
		}
	}
}
 
#endif