#ifndef _TIMEMANAGER_H
#define _TIMEMANAGER_H

#include "Demos/Utils/Config.h"

namespace PBD
{
	class TimeManager
	{
	private:
		float time;
		static TimeManager *current;
		float h;

	public:
		TimeManager ();
		~TimeManager ();

		// Singleton
		static TimeManager* getCurrent ();
		static void setCurrent (TimeManager* tm);
		static bool hasCurrent();

		float getTime();
		void setTime(float t);
		float getTimeStepSize();
		void setTimeStepSize(float tss);
	};
}

#endif
