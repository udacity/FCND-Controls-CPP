// Precision timing functions
// Win32 / Linux compatable
// 2002-2018 sergei lupashin
// License: BSD-3-clause

#pragma once

#include "Common.h"
#ifdef _WIN32
#ifdef WS2
#include <winsock2.h>
#endif
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef max
#undef max
#endif

#include <limits>
using namespace std;

typedef uint32_t TICKS; 
#define SECS_TO_TICKS(seconds) ((TICKS)((seconds)*10000.0))
#define TICKS_TO_SECS(ticks) ((double)((double)(ticks)/10000.0))
// to/from microseconds
#define TICKS_TO_US(ticks) ((__int64)((__int64)(ticks)*100))
#define US_TO_TICKS(us) ((TICKS)((us)/100))
#define US_TO_SECS(us) ((double)((double)(us)/1000000.0))

// frequency of the hi-res counter
extern int64_t __highResCounterFreq;

// We use a global instance of this variable to handle the housekeeping
// of initializing and de-initializing Windows high-res timing and sleep
class CHighResTimingScope{
public:
  CHighResTimingScope();
  ~CHighResTimingScope();
};
extern CHighResTimingScope __highResTimingScope;

#ifndef _WIN32
inline int64_t GetTimeMicroseconds()
{
	struct timezone tz;
	struct timeval t;
	gettimeofday(&t, &tz);
	return ((int64_t)t.tv_sec*1e6 + t.tv_usec);
}
#endif

class Timer
{
public: 
	// the timer times stuff from its creation or most recent reset

	Timer(bool autostart=true)
	{
    _isBaseClass=true;
		if(autostart)
		{
			Reset();
		}
		else
		{
			_begin_t=0;
		}
	}

	virtual void Reset()
	{
#ifdef _WIN32
		QueryPerformanceCounter((LARGE_INTEGER*)&_begin_t);    
#else
		_begin_t = GetTimeMicroseconds();
#endif
	}

  virtual double Seconds() const
  {
    return ElapsedSeconds();
  }

  virtual operator double() const{
    return ElapsedSeconds();
  }

	virtual double ElapsedSeconds() const
	{
		if(!Valid()) return numeric_limits<double>::max();

		int64_t current;
#ifdef _WIN32
		QueryPerformanceCounter((LARGE_INTEGER*)&current);
#else
		current = GetTimeMicroseconds();
#endif
		return ((double)(current-_begin_t)) / (double)__highResCounterFreq;
	}

	virtual void AddSeconds(const double& s) 
	{
		if(!Valid()) return;
		_begin_t = _begin_t + (int64_t)(s*__highResCounterFreq);
	}

  virtual TICKS Ticks() const
  {
    return ElapsedTicks();
  }

	virtual TICKS ElapsedTicks() const
	{
		if(!Valid()) return numeric_limits<TICKS>::max();

		int64_t current;
#ifdef _WIN32
		QueryPerformanceCounter((LARGE_INTEGER*)&current);
#else
		current = GetTimeMicroseconds();
#endif
		if(current<_begin_t) return 0;
		return (TICKS) ( ((double)(current-_begin_t)) * 10000.0 / (double)__highResCounterFreq);
	}

	virtual uint64_t ElapsedMicroseconds() const
	{
		if(!Valid()) return numeric_limits<uint64_t>::max();

		int64_t current;
#ifdef _WIN32
		QueryPerformanceCounter((LARGE_INTEGER*)&current);
#else
		current = GetTimeMicroseconds();
#endif
		if(current<_begin_t) return 0;
		return (int64_t) ( ((double)(current-_begin_t)) * 1000000.0 / (double)__highResCounterFreq);
	}

	// returns a timer that appears as if started a *long* time ago
	static const Timer InvalidTimer()
	{
		Timer ret;
		ret._begin_t = 0;
		return ret;
	}

	virtual bool Valid() const
	{
		return _begin_t != 0;
	}

  bool IsBaseClass() const {return _isBaseClass;}

protected:
  int64_t _begin_t;
  bool _isBaseClass;
};

class RunEveryNSeconds
{
public:
  RunEveryNSeconds(const double N)
  {
    _n=N;
  }
  ~RunEveryNSeconds()
  {
    // seems to be buggy!!
    double dt = timer.Seconds();
    if(_n-dt > 0)
    {
      Sleep((int)((_n-dt)*1000.f));
    }
  }
protected:
  double _n;
  Timer timer;
};
