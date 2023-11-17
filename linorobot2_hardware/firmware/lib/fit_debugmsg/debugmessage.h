#ifndef FIT_DEBUGMESSAGE_INCLUDE_DEBUGMESSAGE_H_
#define FIT_DEBUGMESSAGE_INCLUDE_DEBUGMESSAGE_H_

#include <stdio.h>
#include <stdarg.h>

#define FITDEBUGMESSAGE 1

void FITDBGbegin();
void dbg_printf(const char *fmt, ...);


#endif /* FIT_DEBUGMESSAGE_INCLUDE_DEBUGMESSAGE_H_ */