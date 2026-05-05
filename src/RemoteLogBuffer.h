#pragma once

#include <Arduino.h>

void appLogAppend(const char* text);
void appLogLine(const char* text);
void appLogPrintf(const char* fmt, ...);
String appLogGetLastLine();
String appLogGetTail(size_t maxBytes = 512);
