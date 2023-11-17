#include <Arduino.h>
#include "debugmessage.h"

void FITDBGbegin()
{
#ifdef FITDEBUGMESSAGE
  Serial8.begin(115200);
#endif
}

void dbg_printf(const char *fmt, ...)
{
#ifdef FITDEBUGMESSAGE
  va_list args;
  char buffer[128];
  va_start(args,fmt);
  vsprintf(buffer,fmt,args);
  va_end(args);
  Serial8.printf("%s",buffer);
#endif
}
