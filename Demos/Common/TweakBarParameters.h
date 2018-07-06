#ifndef __TweakBarParameters_h__
#define __TweakBarParameters_h__

#include "Common/Common.h"
#include "extern/AntTweakBar/include/AntTweakBar.h"
#include <vector>
#include "ParameterObject.h"

namespace PBD
{
	class TweakBarParameters
	{
	public: 
		typedef std::pair<GenParam::ParameterObject*, unsigned int> ParameterIndex;

		static void createParameterGUI();
		static void createParameterObjectGUI(GenParam::ParameterObject *paramObj);

		static void TW_CALL setParameterValue(const void *value, void *clientData);
		static void TW_CALL getParameterValue(void *value, void *clientData);

		static void TW_CALL setTimeStepSizeCB(const void *value, void *clientData);
		static void TW_CALL getTimeStepSizeCB(void *value, void *clientData);

		static void cleanup();

	protected:
		static std::vector<std::unique_ptr<ParameterIndex>> m_params;
		static std::vector<std::string> m_objectNames;
	};
}
 
#endif