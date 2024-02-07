#ifndef __TIMING_H__
#define __TIMING_H__

#if defined(WIN32) || defined(_WIN32) || defined(WIN64)
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE __attribute__((always_inline))
#endif

#include <iostream>
#include <stack>
#include <unordered_map>
//#include "Common/Common.h"

#include <chrono>
#include "IDFactory.h"
#include "cuNSearch_export.h"

namespace cuNSearch
{
	struct TimingHelper
	{
		std::chrono::time_point<std::chrono::high_resolution_clock> start;
		std::string name;
	};

	struct AverageTime
	{
		double totalTime;
		unsigned int counter;
		std::string name;
	};

	class Timing
	{
	public:
		static cuNSearch_EXPORT bool m_dontPrintTimes;
		static cuNSearch_EXPORT unsigned int m_startCounter;
		static cuNSearch_EXPORT unsigned int m_stopCounter;
		static cuNSearch_EXPORT std::stack<TimingHelper> m_timingStack;
		static cuNSearch_EXPORT std::unordered_map<int, AverageTime> m_averageTimes;

		static void reset()
		{
			while (!m_timingStack.empty())
				m_timingStack.pop();
			m_averageTimes.clear();
			m_startCounter = 0;
			m_stopCounter = 0;
		}

		FORCE_INLINE static void startTiming(const std::string& name = std::string(""))
		{
			TimingHelper h;
			h.start = std::chrono::high_resolution_clock::now();
			h.name = name;
			Timing::m_timingStack.push(h);
			Timing::m_startCounter++;
		}

		FORCE_INLINE static double stopTiming(bool print = true)
		{
			if (!Timing::m_timingStack.empty())
			{
				Timing::m_stopCounter++;
				std::chrono::time_point<std::chrono::high_resolution_clock> stop = std::chrono::high_resolution_clock::now();
				TimingHelper h = Timing::m_timingStack.top();
				Timing::m_timingStack.pop();
				std::chrono::duration<double> elapsed_seconds = stop - h.start;
				double t = elapsed_seconds.count() * 1000.0;

				if (print)
					std::cout << "time " << h.name.c_str() << ": " << t << " ms\n" << std::flush;
				return t;
		}
			return 0;
	}

		FORCE_INLINE static double stopTiming(bool print, int &id)
		{
			if (id == -1)
				id = IDFactory::getId();
			if (!Timing::m_timingStack.empty())
			{
				Timing::m_stopCounter++;
				std::chrono::time_point<std::chrono::high_resolution_clock> stop = std::chrono::high_resolution_clock::now();
				TimingHelper h = Timing::m_timingStack.top();
				Timing::m_timingStack.pop();

				std::chrono::duration<double> elapsed_seconds = stop - h.start;
				double t = elapsed_seconds.count() * 1000.0;

				if (print && !Timing::m_dontPrintTimes)
					std::cout << "time " << h.name.c_str() << ": " << t << " ms\n" << std::flush;

				if (id >= 0)
				{
					std::unordered_map<int, AverageTime>::iterator iter;
					iter = Timing::m_averageTimes.find(id);
					if (iter != Timing::m_averageTimes.end())
					{
						Timing::m_averageTimes[id].totalTime += t;
						Timing::m_averageTimes[id].counter++;
					}
					else
					{
						AverageTime at;
						at.counter = 1;
						at.totalTime = t;
						at.name = h.name;
						Timing::m_averageTimes[id] = at;
					}
				}
				return t;
			}
			return 0;
		}

		FORCE_INLINE static void printAverageTimes()
		{
			std::unordered_map<int, AverageTime>::iterator iter;
			for (iter = Timing::m_averageTimes.begin(); iter != Timing::m_averageTimes.end(); iter++)
			{
				AverageTime &at = iter->second;
				const double avgTime = at.totalTime / at.counter;
				std::cout << "Average time " << at.name.c_str() << ": " << avgTime << " ms\n" << std::flush;
			}
			if (Timing::m_startCounter != Timing::m_stopCounter)
				std::cout << "Problem: " << Timing::m_startCounter << " calls of startTiming and " << Timing::m_stopCounter << " calls of stopTiming.\n " << std::flush;
			std::cout << "---------------------------------------------------------------------------\n\n";
		}

		FORCE_INLINE static void printTimeSums()
		{
			std::unordered_map<int, AverageTime>::iterator iter;
			for (iter = Timing::m_averageTimes.begin(); iter != Timing::m_averageTimes.end(); iter++)
			{
				AverageTime &at = iter->second;
				const double timeSum = at.totalTime;
				std::cout << "Time sum " << at.name.c_str() << ": " << timeSum << " ms\n" << std::flush;
			}
			if (Timing::m_startCounter != Timing::m_stopCounter)
				std::cout << "Problem: " << Timing::m_startCounter << " calls of startTiming and " << Timing::m_stopCounter << " calls of stopTiming.\n " << std::flush;
			std::cout << "---------------------------------------------------------------------------\n\n";
		}
	};

}

#endif