#include "RemoteLogBuffer.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace {
constexpr size_t LOG_BUFFER_SIZE = 8192;

portMUX_TYPE g_logMux = portMUX_INITIALIZER_UNLOCKED;
char g_logBuffer[LOG_BUFFER_SIZE];
size_t g_logHead = 0;
size_t g_logSize = 0;
String g_lastLine;

void appendLocked(const char* text, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    g_logBuffer[g_logHead] = text[i];
    g_logHead = (g_logHead + 1) % LOG_BUFFER_SIZE;
    if (g_logSize < LOG_BUFFER_SIZE) {
      g_logSize++;
    }
  }
}
}  // namespace

void appLogAppend(const char* text) {
  if (!text || text[0] == '\0') return;
  size_t len = strlen(text);
  if (len == 0) return;

  portENTER_CRITICAL(&g_logMux);
  appendLocked(text, len);
  portEXIT_CRITICAL(&g_logMux);
}

void appLogLine(const char* text) {
  const char* value = text ? text : "";
  portENTER_CRITICAL(&g_logMux);
  g_lastLine = value;
  appendLocked(value, strlen(value));
  appendLocked("\n", 1);
  portEXIT_CRITICAL(&g_logMux);
}

void appLogPrintf(const char* fmt, ...) {
  if (!fmt) return;

  char stackBuf[256];
  va_list args;
  va_start(args, fmt);
  int written = vsnprintf(stackBuf, sizeof(stackBuf), fmt, args);
  va_end(args);
  if (written <= 0) return;

  if ((size_t)written < sizeof(stackBuf)) {
    appLogAppend(stackBuf);
    return;
  }

  size_t needed = (size_t)written + 1;
  char* heapBuf = (char*)malloc(needed);
  if (!heapBuf) {
    appLogAppend(stackBuf);
    return;
  }

  va_start(args, fmt);
  vsnprintf(heapBuf, needed, fmt, args);
  va_end(args);
  appLogAppend(heapBuf);
  free(heapBuf);
}

String appLogGetLastLine() {
  portENTER_CRITICAL(&g_logMux);
  String value = g_lastLine;
  portEXIT_CRITICAL(&g_logMux);
  return value;
}

String appLogGetTail(size_t maxBytes) {
  if (maxBytes == 0) return String();

  portENTER_CRITICAL(&g_logMux);
  size_t count = g_logSize;
  if (count > maxBytes) count = maxBytes;
  size_t start = (g_logHead + LOG_BUFFER_SIZE - count) % LOG_BUFFER_SIZE;
  String out;
  out.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    out += g_logBuffer[(start + i) % LOG_BUFFER_SIZE];
  }
  portEXIT_CRITICAL(&g_logMux);
  return out;
}
