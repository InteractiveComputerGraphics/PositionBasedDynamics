#include "Timing.h"

using namespace PBD;

std::unordered_map<int, AverageTime> Timing::m_averageTimes;
std::stack<TimingHelper> Timing::m_timingStack;
bool Timing::m_dontPrintTimes = false;
unsigned int Timing::m_startCounter = 0;
unsigned int Timing::m_stopCounter = 0;
