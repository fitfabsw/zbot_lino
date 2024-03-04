#include <Arduino.h>
#include "debugmessage.h"

void FITDBGbegin()
{
#ifdef FITDEBUGMESSAGE
#if defined(ARDUINO_TEENSY41)
  Serial5.begin(115200);
#endif
#endif
}

void dbg_printf(const char *fmt, ...)
{
#ifdef FITDEBUGMESSAGE
#if defined(ARDUINO_TEENSY41)
  va_list args;
  char buffer[128];
  va_start(args,fmt);
  vsprintf(buffer,fmt,args);
  va_end(args);
  Serial5.printf("%s",buffer);
#endif
#endif
}
