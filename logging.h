/*
 * @file logging.h
 * @brief File to log useful variables during debugging
 * To utilize the logging features, use "#define LOGGING" in your main file (e.g., <name>.ino)
 * @author Vasanth Sarathy
 */

// Check if LOGGING is defined
#ifdef LOGGING

// logging is enabled
#include <stdarg.h>


void log(char* format, ...)
{
	char line[1024];
	va_list args;
	va_start(args, format);
	vsnprintf(line, sizeof(line), format, args);
	va_end(args);
	Serial.print(line);
}

#else

// logging is disabled
#define log(...)

#endif
