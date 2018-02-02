#pragma once

#include "../Math/V3D.h"

// h from 0-259
// s from 0 to 1
// v from 0 to 1
V3F HSVtoRGB( float h, float s, float v );

// False color (color gradient) functions
V3F FalseColorBGR(float v, float intensityMult=1.0);
V3F FalseColorRGB(float v, float intensityMult = 1.0);
V3F FalseColor_RedGreen(float v, float intensityMult=1.0);

#ifdef _WIN32
// Sets the stdout text color.
// Example of use: SetConsoleColor(FOREGROUND_BLUE|BACKGROUND_INTENSITY)
namespace SLR{
	const uint16_t CONSOLE_FG_RED = FOREGROUND_RED | FOREGROUND_INTENSITY;
	const uint16_t CONSOLE_FG_GREEN = FOREGROUND_GREEN | FOREGROUND_INTENSITY;
  const uint16_t CONSOLE_FG_YELLOW = FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_INTENSITY;
  const uint16_t CONSOLE_FG_DARK_YELLOW = FOREGROUND_GREEN | FOREGROUND_RED;
  const uint16_t CONSOLE_FG_DARK_GREEN = FOREGROUND_GREEN;
	const uint16_t CONSOLE_FG_BLUE = FOREGROUND_BLUE | FOREGROUND_INTENSITY;
  const uint16_t CONSOLE_FG_DARK_BLUE = FOREGROUND_BLUE;
  const uint16_t CONSOLE_FG_DARK_TEAL = FOREGROUND_BLUE | FOREGROUND_GREEN;
	const uint16_t CONSOLE_FG_TEAL = FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_INTENSITY;
  const uint16_t CONSOLE_FG_GRAY = FOREGROUND_RED | FOREGROUND_BLUE | FOREGROUND_GREEN;
  const uint16_t CONSOLE_FG_WHITE = FOREGROUND_RED | FOREGROUND_BLUE | FOREGROUND_GREEN | FOREGROUND_INTENSITY;
}
#endif
void SetConsoleColor(unsigned char attr);
void ResetConsoleColor();
