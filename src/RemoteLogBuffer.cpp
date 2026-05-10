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

size_t appLogCopyLastLine(char* out, size_t outSize, size_t maxBytes) {
  if (!out || outSize == 0) return 0;
  out[0] = '\0';

  portENTER_CRITICAL(&g_logMux);
  const char* src = g_lastLine.c_str();
  size_t srcLen = src ? strlen(src) : 0;
  if (maxBytes > 0 && srcLen > maxBytes) srcLen = maxBytes;
  size_t copyLen = (srcLen < (outSize - 1)) ? srcLen : (outSize - 1);
  if (copyLen > 0 && src) {
    memcpy(out, src + (srcLen - copyLen), copyLen);
  }
  out[copyLen] = '\0';
  portEXIT_CRITICAL(&g_logMux);
  return copyLen;
}

size_t appLogCopyTail(char* out, size_t outSize, size_t maxBytes) {
  if (!out || outSize == 0) return 0;
  if (maxBytes == 0) {
    out[0] = '\0';
    return 0;
  }

  portENTER_CRITICAL(&g_logMux);
  size_t count = g_logSize;
  if (count > maxBytes) count = maxBytes;
  if (count > (outSize - 1)) count = outSize - 1;
  size_t start = (g_logHead + LOG_BUFFER_SIZE - count) % LOG_BUFFER_SIZE;
  for (size_t i = 0; i < count; ++i) {
    out[i] = g_logBuffer[(start + i) % LOG_BUFFER_SIZE];
  }
  out[count] = '\0';
  portEXIT_CRITICAL(&g_logMux);
  return count;
}
