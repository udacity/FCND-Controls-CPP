#pragma once

#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

#ifndef CONSTRAIN
#define CONSTRAIN(a,low,high) MAX((low),MIN((a),(high)))
#endif