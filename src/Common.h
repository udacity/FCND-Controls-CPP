#pragma once

// Common.h

// The purpose of this file is to improve the overall sanity of the other code
// by making compilers behave

#include <string>
#include <time.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#ifndef __PX4_NUTTX
#include <sys/timeb.h>
#endif

#define SLR_ERROR0(A) SLR::PrintError(__FUNCTION__,__LINE__,A)
#define SLR_ERROR1(A,B) SLR::PrintError(__FUNCTION__,__LINE__,A,B)
#define SLR_ERROR2(A,B,C) SLR::PrintError(__FUNCTION__,__LINE__,A,B,C)
#define SLR_ERROR3(A,B,C,D) SLR::PrintError(__FUNCTION__,__LINE__,A,B,C,D)
#define SLR_ERROR4(A,B,C,D,E) SLR::PrintError(__FUNCTION__,__LINE__,A,B,C,D,E)
#define SLR_ERROR5(A,B,C,D,E,F) SLR::PrintError(__FUNCTION__,__LINE__,A,B,C,D,E,F)
#define SLR_ERROR6(A,B,C,D,E,F,G) SLR::PrintError(__FUNCTION__,__LINE__,A,B,C,D,E,F,G)

#define SLR_ERROR0S(S,A) SLR::PrintError(S,__LINE__,A)
#define SLR_ERROR1S(S,A,B) SLR::PrintError(S,__LINE__,A,B)
#define SLR_ERROR2S(S,A,B,C) SLR::PrintError(S,__LINE__,A,B,C)
#define SLR_ERROR3S(S,A,B,C,D) SLR::PrintError(S,__LINE__,A,B,C,D)
#define SLR_ERROR4S(S,A,B,C,D,E) SLR::PrintError(S,__LINE__,A,B,C,D,E)
#define SLR_ERROR5S(S,A,B,C,D,E,F) SLR::PrintError(S,__LINE__,A,B,C,D,E,F)
#define SLR_ERROR6S(S,A,B,C,D,E,F,G) SLR::PrintError(S,__LINE__,A,B,C,D,E,F,G)

#define SLR_WARNING0(A) SLR::PrintWarning(__FUNCTION__,__LINE__,A)
#define SLR_WARNING1(A,B) SLR::PrintWarning(__FUNCTION__,__LINE__,A,B)
#define SLR_WARNING2(A,B,C) SLR::PrintWarning(__FUNCTION__,__LINE__,A,B,C)
#define SLR_WARNING3(A,B,C,D) SLR::PrintWarning(__FUNCTION__,__LINE__,A,B,C,D)
#define SLR_WARNING4(A,B,C,D,E) SLR::PrintWarning(__FUNCTION__,__LINE__,A,B,C,D,E)
#define SLR_WARNING5(A,B,C,D,E,F) SLR::PrintWarning(__FUNCTION__,__LINE__,A,B,C,D,E,F)

#ifndef _WIN32
// not technically 100% correct, but lets us move on with our lives
#define sprintf_s snprintf
#define vsprintf_s vsnprintf
#else
#ifndef _SCL_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif
#pragma warning(disable: 4996) //strcpy unsafe
#endif

namespace SLR {
  inline void TimestampString(char* buf, int cnt, const char* format="%y.%m.%d.%H.%M.%S")
  {
#ifndef __PX4_NUTTX
    // timestamp
    struct tm   newTime;
    time_t      szClock;
    time(&szClock);
#ifdef _WIN32
    _timeb tstruct;
    _ftime(&tstruct);
    localtime_s(&newTime, &szClock);
#else
    // linux and apple
    timeb tstruct;
    ftime(&tstruct);
    localtime_r(&szClock,&newTime);
#endif
    strftime(buf, cnt, format, &newTime);
    sprintf_s(buf, cnt, "%s.%03d", buf, tstruct.millitm);
#else
    if(cnt>0)
    {
      buf[0]=0;
    }

#endif
  }

  inline void PrintError(const char* funcName, const int lineNum, const char* format, ...)
  {
    char tsBuf[100]; tsBuf[99]=0;
    char buf[512]; buf[511] = 0;
    char buf2[2048]; buf2[2047] = 0;
    char buf3[2560]; buf3[2559] = 0;

    TimestampString(tsBuf,99,"%d-%H.%M.%S");

    // error location
    sprintf_s(buf, 511, "%s ERR %s(%d) : ", tsBuf, funcName, lineNum);

    va_list args;
    va_start(args, format);
    vsprintf_s(buf2, 2047, format, args);
    va_end(args);

    // push everything out to stderr
    sprintf_s(buf3, 2047, "%s%s\n", buf, buf2);
    fprintf(stderr,"%s",buf3);
    fflush(stderr);
  }

  inline void PrintWarning(const char* funcName, const int lineNum, const char* format, ...)
  {
    char tsBuf[100]; tsBuf[99]=0;
    char buf[512]; buf[511] = 0;
    char buf2[2048]; buf2[2047] = 0;
    char buf3[2560]; buf3[2559] = 0;

    TimestampString(tsBuf,99,"%d-%H.%M.%S");

    // error location
    sprintf_s(buf, 511, "%s WRN %s(%d) : ", tsBuf, funcName, lineNum);

    va_list args;
    va_start(args, format);
    vsprintf_s(buf2, 2047, format, args);
    va_end(args);

    // push everything out to stderr
    sprintf_s(buf3, 2047, "%s%s\n", buf, buf2);
    fprintf(stderr,"%s",buf3);
    fflush(stderr);
  }


} //namespace SLR

#ifdef _WIN32


namespace SLR {
  inline void PrintError(const char* funcName, const int lineNum, const char* format, ...);
  inline void PrintWarning(const char* funcName, const int lineNum, const char* format, ...);
}



// boost::numeric::ublas shortcuts
#define BNU boost::numeric::ublas
#define BNUV BNU::bounded_vector
#define BNUM BNU::c_matrix

#if _MSC_VER>=1600 // visual studio 2010
#include <memory>
#include <functional>
using std::shared_ptr;
using std::weak_ptr;
using std::placeholders::_1;
#define TR1 std
#else
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include "Common/Math/Attitude.h"
#include "Common/Math/Transform3D.h"
using boost::shared_ptr;
using boost::weak_ptr;
using boost::static_pointer_cast;
#define TR1 boost
#endif

#include <winsock2.h> 
#include <Windows.h>


#else
// not _WIN32
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
inline void Sleep(int msec)
{
  usleep(msec*1000);
}

#include <memory>
#include <functional>
using std::shared_ptr;
using std::weak_ptr;
using std::placeholders::_1;

#ifndef __PX4_NUTTX
#include <cmath>
inline bool _isnan(const double& v){return std::isnan(v);}
inline bool _isnan(const float& v){return std::isnan(v);}
#endif

#endif // #ifdef _WIN32

#include "Math/Constants.h"
#include "Math/V3F.h"
#include "Math/V3D.h"
