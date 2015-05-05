#include "TimeManager.h"

using namespace PBD;

TimeManager* TimeManager::current = 0;

TimeManager::TimeManager () 
{
	time = 0;
	h = 0.005f;
}

TimeManager::~TimeManager () 
{
	current = 0;
}

TimeManager* TimeManager::getCurrent ()
{
	if (current == 0)
	{
		current = new TimeManager ();
	}
	return current;
}

void TimeManager::setCurrent (TimeManager* tm)
{
	current = tm;
}

bool TimeManager::hasCurrent()
{
	return (current != 0);
}

float TimeManager::getTime()
{
	return time;
}

void TimeManager::setTime(float t)
{
	time = t;
}

float TimeManager::getTimeStepSize()
{
	return h;
}

void TimeManager::setTimeStepSize(float tss)
{
	h = tss;
}
