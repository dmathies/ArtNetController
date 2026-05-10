#pragma once

#include <Arduino.h>

void appLogAppend(const char* text);
void appLogLine(const char* text);
void appLogPrintf(const char* fmt, ...);
String appLogGetLastLine();
String appLogGetTail(size_t maxBytes = 512);
size_t appLogCopyLastLine(char* out, size_t outSize, size_t maxBytes = 0);
size_t appLogCopyTail(char* out, size_t outSize, size_t maxBytes = 512);
