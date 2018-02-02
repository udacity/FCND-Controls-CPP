#include "../Common.h"
#include "Timer.h"
#include <stdio.h>

#ifdef _WIN32
#include <mmsystem.h>
#pragma comment(lib,"winmm.lib")
#endif

CHighResTimingScope __highResTimingScope;
int64_t __highResCounterFreq=0;

CHighResTimingScope::CHighResTimingScope(){
#ifdef _WIN32
  timeBeginPeriod(1); // inits hi-res sleep
  QueryPerformanceFrequency((LARGE_INTEGER*)&__highResCounterFreq);
#else
  __highResCounterFreq = 1e6;
#endif
  if(__highResCounterFreq==0){
    printf("ERROR: no performance counter found\n");
    __highResCounterFreq = 1;
  }
};

CHighResTimingScope::~CHighResTimingScope()
{
#ifdef _WIN32
  timeEndPeriod(1); // de-inits hi-res sleep
#endif
};




