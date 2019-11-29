#include "TweakBarParameters.h"
#include "Simulation/TimeManager.h"
#include "Demos/Visualization/MiniGL.h"
#include "Utils/Logger.h"

using namespace PBD;
using namespace std;
using namespace GenParam;

vector<std::unique_ptr<TweakBarParameters::ParameterIndex>> TweakBarParameters::m_params;
vector<string> TweakBarParameters::m_objectNames;

 
void TweakBarParameters::createParameterGUI()
{
	// Add callback for the time step size
	TwAddVarCB(MiniGL::getTweakBar(), "TimeStepSize", TW_TYPE_DOUBLE, setTimeStepSizeCB, getTimeStepSizeCB, NULL,
		" label='Time step size' min=0.000001 max=0.1 step=0.0001 precision=5 help='Set time step size' group=Simulation ");

}

void TweakBarParameters::createParameterObjectGUI(ParameterObject *paramObj)
{
	if (paramObj == nullptr)
		return;

	// Find unique name
	unsigned int index = 1;
	std::string objName = "GenParam" + std::to_string(index);
	for (unsigned int i = 0; i < m_objectNames.size(); i++)
	{
		if (objName == m_objectNames[i])
		{
			index++;
			objName = "GenParam" + std::to_string(index);
		}
	}
	m_objectNames.push_back(objName);

	const unsigned int numParams = paramObj->numParameters();

	for (unsigned int i= 0; i < numParams; i++)
	{
		ParameterBase *paramBase = paramObj->getParameter(i);
		m_params.push_back(std::unique_ptr<ParameterIndex>(new ParameterIndex(paramObj, i)));

		char str[1000];

		// Defining new unique var name
		char varName[500];
		sprintf(varName, "%s::%s", objName.c_str(), paramBase->getName().c_str());

		std::string tmp = "";
		if (paramBase->getHotKey() != "")
			tmp = "key='" + paramBase->getHotKey() + "' ";

		if (paramBase->getReadOnly())
			tmp = tmp + "readonly=true";
		else
			tmp = tmp + "readonly=false";

		if (paramBase->getType() == RealParameterType)
		{
			sprintf(str, " label='%s' help='%s' precision=5 step=0.01 group='%s' %s ", paramBase->getLabel().c_str(), paramBase->getDescription().c_str(), paramBase->getGroup().c_str(), tmp.c_str());
			TwAddVarCB(MiniGL::getTweakBar(), varName, TW_TYPE_REAL, setParameterValue, getParameterValue, m_params[m_params.size() - 1].get(), str);
		}
 		else if (paramBase->getType() == ParameterBase::UINT32)
 		{
 			sprintf(str, " label='%s' help='%s' step=1 group='%s' %s ", paramBase->getLabel().c_str(), paramBase->getDescription().c_str(), paramBase->getGroup().c_str(), tmp.c_str());
 			TwAddVarCB(MiniGL::getTweakBar(), varName, TW_TYPE_UINT32, setParameterValue, getParameterValue, m_params[m_params.size() - 1].get(), str);
 		}
		else if (paramBase->getType() == ParameterBase::UINT16)
		{
			sprintf(str, " label='%s' help='%s' step=1 group='%s' %s ", paramBase->getLabel().c_str(), paramBase->getDescription().c_str(), paramBase->getGroup().c_str(), tmp.c_str());
			TwAddVarCB(MiniGL::getTweakBar(), varName, TW_TYPE_UINT16, setParameterValue, getParameterValue, m_params[m_params.size() - 1].get(), str);
		}
		else if (paramBase->getType() == ParameterBase::UINT8)
		{
			sprintf(str, " label='%s' help='%s' step=1 group='%s' %s ", paramBase->getLabel().c_str(), paramBase->getDescription().c_str(), paramBase->getGroup().c_str(), tmp.c_str());
			TwAddVarCB(MiniGL::getTweakBar(), varName, TW_TYPE_UINT8, setParameterValue, getParameterValue, m_params[m_params.size() - 1].get(), str);
		}
		else if (paramBase->getType() == ParameterBase::INT32)
		{
			sprintf(str, " label='%s' help='%s' step=1 group='%s' %s ", paramBase->getLabel().c_str(), paramBase->getDescription().c_str(), paramBase->getGroup().c_str(), tmp.c_str());
			TwAddVarCB(MiniGL::getTweakBar(), varName, TW_TYPE_INT32, setParameterValue, getParameterValue, m_params[m_params.size() - 1].get(), str);
		}
		else if (paramBase->getType() == ParameterBase::INT16)
		{
			sprintf(str, " label='%s' help='%s' step=1 group='%s' %s ", paramBase->getLabel().c_str(), paramBase->getDescription().c_str(), paramBase->getGroup().c_str(), tmp.c_str());
			TwAddVarCB(MiniGL::getTweakBar(), varName, TW_TYPE_INT16, setParameterValue, getParameterValue, m_params[m_params.size() - 1].get(), str);
		}
		else if (paramBase->getType() == ParameterBase::INT8)
		{
			sprintf(str, " label='%s' help='%s' step=1 group='%s' %s ", paramBase->getLabel().c_str(), paramBase->getDescription().c_str(), paramBase->getGroup().c_str(), tmp.c_str());
			TwAddVarCB(MiniGL::getTweakBar(), varName, TW_TYPE_INT8, setParameterValue, getParameterValue, m_params[m_params.size() - 1].get(), str);
		}
		else if (paramBase->getType() == ParameterBase::ENUM)
		{
			// Defining new enum type
			char enumTypeName[500];
			sprintf(enumTypeName, "Enum_%s::%s", objName.c_str(), paramBase->getName().c_str());

			std::string enumStr = "enum='";
			EnumParameter *enumParam = static_cast<EnumParameter*>(paramBase);
			auto enumValues = enumParam->getEnumValues();

			std::ostringstream oss;
			oss << enumValues[0].id << " {" << enumValues[0].name.c_str() << "}";
			for (unsigned int j = 1; j < enumValues.size(); j++)
			{
				oss << ", " << enumValues[j].id << " {" << enumValues[j].name.c_str() << "}";
			}
			oss << "'";
			enumStr = enumStr + oss.str();

			TwType enumType = TwDefineEnum(enumTypeName, NULL, 0);
			sprintf(str, " label='%s' help='%s' group='%s' %s %s ", paramBase->getLabel().c_str(), paramBase->getDescription().c_str(), paramBase->getGroup().c_str(), tmp.c_str(), enumStr.c_str());
			TwAddVarCB(MiniGL::getTweakBar(), varName, enumType, setParameterValue, getParameterValue, m_params[m_params.size() - 1].get(), str);
		}
		else if (paramBase->getType() == ParameterBase::BOOL)
		{
			sprintf(str, " label='%s' help='%s' group='%s' %s ", paramBase->getLabel().c_str(), paramBase->getDescription().c_str(), paramBase->getGroup().c_str(), tmp.c_str());
			TwAddVarCB(MiniGL::getTweakBar(), varName, TW_TYPE_BOOL32, setParameterValue, getParameterValue, m_params[m_params.size() - 1].get(), str);
		}
		else if (paramBase->getType() == RealVectorParameterType)
		{
			if (static_cast<VectorParameter<Real>*>(paramBase)->getDim() == 3)
			{
				sprintf(str, " label='%s' help='%s' group='%s' %s ", paramBase->getLabel().c_str(), paramBase->getDescription().c_str(), paramBase->getGroup().c_str(), tmp.c_str());
				TwAddVarCB(MiniGL::getTweakBar(), varName, TW_TYPE_DIR3R, setParameterValue, getParameterValue, m_params[m_params.size() - 1].get(), str);
			}
		}

// 		else if ((desc->type == IBDS::IBDS_STRING_PARAMETER) || (desc->type == IBDS::IBDS_FILEPATH_PARAMETER))
// 		{
// 			//const bool &defaultValue = paramObj->getParameterDefaultValue<bool>(j);
// 			char str[400];
// 			char tmp.c_str()[20];
// 			if (desc->readOnly)
// 				sprintf(tmp.c_str(), "%s", "readonly=true");
// 			else
// 				sprintf(tmp.c_str(), "%s", "readonly=false");
// 
// 			// Defining new unique var name
// 			char varName[500];
// 			sprintf(varName, "%s::%s", objName.c_str(), desc->fullName.c_str());
// 
// 			sprintf(str, " label='%s' help='%s' group='%s' %s ", desc->label.c_str(), desc->description.c_str(), objName.c_str(), tmp.c_str());
// 			TwAddVarCB(MiniGL::getTwBar(), varName, TW_TYPE_CDSTRING, setParameterValue, getParameterValue, &m_params[m_params.size() - 1], str);
// 		}
// 		
	}
}

void TweakBarParameters::cleanup()
{
	m_params.clear();
	m_objectNames.clear();
}

void TW_CALL TweakBarParameters::setTimeStepSizeCB(const void *value, void *clientData)
{
	const double val = *(const double *)(value);
	TimeManager::getCurrent()->setTimeStepSize((Real)val);
}

void TW_CALL TweakBarParameters::getTimeStepSizeCB(void *value, void *clientData)
{
	*(double *)(value) = (double)TimeManager::getCurrent()->getTimeStepSize();
}


void TW_CALL TweakBarParameters::setParameterValue(const void *value, void *clientData)
{
	ParameterIndex *pi = (ParameterIndex*)clientData;
	ParameterObject *paramObj = pi->first;
	ParameterBase *paramBase = paramObj->getParameter(pi->second);

	if (paramBase->getReadOnly())
		return;

	if (paramBase->getType() == RealParameterType)
	{
		const Real val = *(const Real *)(value);
		static_cast<NumericParameter<Real>*>(paramBase)->setValue(val);
	}
 	else if (paramBase->getType() == ParameterBase::UINT32)
 	{
 		const unsigned int val = *(const unsigned int *)(value);
		static_cast<NumericParameter<unsigned int>*>(paramBase)->setValue(val);
 	}
	else if (paramBase->getType() == ParameterBase::UINT16)
	{
		const unsigned short val = *(const unsigned short *)(value);
		static_cast<NumericParameter<unsigned short>*>(paramBase)->setValue(val);
	}
	else if (paramBase->getType() == ParameterBase::UINT8)
	{
		const unsigned char val = *(const unsigned char *)(value);
		static_cast<NumericParameter<unsigned char>*>(paramBase)->setValue(val);
	}
	else if (paramBase->getType() == ParameterBase::INT32)
	{
		const int val = *(const int *)(value);
		static_cast<NumericParameter<int>*>(paramBase)->setValue(val);
	}
	else if (paramBase->getType() == ParameterBase::INT16)
	{
		const short val = *(const short *)(value);
		static_cast<NumericParameter<short>*>(paramBase)->setValue(val);
	}
	else if (paramBase->getType() == ParameterBase::INT8)
	{
		const char val = *(const char *)(value);
		static_cast<NumericParameter<char>*>(paramBase)->setValue(val);
	}
	else if (paramBase->getType() == ParameterBase::ENUM)
	{
		const short val = *(const short *)(value);
		static_cast<EnumParameter*>(paramBase)->setValue(val);
	}
	else if (paramBase->getType() == ParameterBase::BOOL)
	{
		const bool val = *(const bool *)(value);
		static_cast<BoolParameter*>(paramBase)->setValue(val);
	}
	else if(paramBase->getType() == RealVectorParameterType)
	{
		if (static_cast<VectorParameter<Real>*>(paramBase)->getDim() == 3)
		{
			Real * const val = (Real* const)(value);
			static_cast<VectorParameter<Real>*>(paramBase)->setValue(val);
		}
	}
// 	else if ((desc->type == IBDS::IBDS_STRING_PARAMETER) || (desc->type == IBDS::IBDS_FILEPATH_PARAMETER))
// 	{
// 		const std::string val = std::string(*(const char **)(value));
// 		paramObj->setParameter<std::string>(desc->id, val);
// 	}
}

void TW_CALL TweakBarParameters::getParameterValue(void *value, void *clientData)
{
	ParameterIndex *pi = (ParameterIndex*)clientData;
	ParameterObject *paramObj = pi->first;
	ParameterBase *paramBase = paramObj->getParameter(pi->second);

	if (paramBase->getType() == RealParameterType)
	{
		const Real val = static_cast<NumericParameter<Real>*>(paramBase)->getValue();
		*(Real*)(value) = val;
	}
	else if (paramBase->getType() == ParameterBase::UINT32)
	{
		const unsigned int val = static_cast<NumericParameter<unsigned int>*>(paramBase)->getValue();
		*(unsigned int*)(value) = val;
	}
	else if (paramBase->getType() == ParameterBase::UINT16)
	{
		const unsigned short val = static_cast<NumericParameter<unsigned short>*>(paramBase)->getValue();
		*(unsigned short*)(value) = val;
	}
	else if (paramBase->getType() == ParameterBase::UINT8)
	{
		const unsigned char val = static_cast<NumericParameter<unsigned char>*>(paramBase)->getValue();
		*(unsigned char*)(value) = val;
	}
	else if (paramBase->getType() == ParameterBase::INT32)
	{
		const int val = static_cast<NumericParameter<int>*>(paramBase)->getValue();
		*(int*)(value) = val;
	}
	else if (paramBase->getType() == ParameterBase::INT16)
	{
		const short val = static_cast<NumericParameter<short>*>(paramBase)->getValue();
		*(short*)(value) = val;
	}
	else if (paramBase->getType() == ParameterBase::INT8)
	{
		const char val = static_cast<NumericParameter<char>*>(paramBase)->getValue();
		*(char*)(value) = val;
	}
	else if (paramBase->getType() == ParameterBase::ENUM)
	{
		const int val = static_cast<EnumParameter*>(paramBase)->getValue();
		*(short*)(value) = static_cast<short>(val);
	}
	else if (paramBase->getType() == ParameterBase::BOOL)
	{
		const bool val = static_cast<BoolParameter*>(paramBase)->getValue();
		*(bool*)(value) = val;
	}
	else if (paramBase->getType() == RealVectorParameterType)
	{
		if (static_cast<VectorParameter<Real>*>(paramBase)->getDim() == 3)
		{
			Real * const val = static_cast<VectorParameter<Real>*>(paramBase)->getValue();
			((Real*)value)[0] = val[0];
			((Real*)value)[1] = val[1];
			((Real*)value)[2] = val[2];
		}
	}
// 	else if ((desc->type == IBDS::IBDS_STRING_PARAMETER) || (desc->type == IBDS::IBDS_FILEPATH_PARAMETER))
// 	{
// 		const std::string &val = paramObj->getParameter<std::string>(desc->id);
// 		*(const char **)(value) = val.c_str();
// 	}
}

