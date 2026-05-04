#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <esp_http_server.h>
#include <LittleFS.h>
#include <Update.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <ArduinoJson.h>
#include <lwip/sockets.h>
#include <lwip/tcp.h>
#include <cctype>
#include <cstring>

#include "main_common.h"


static constexpr int MAX_WS_CLIENTS = 6;
static constexpr uint32_t SLOW_HTTP_MS = 100;
static constexpr uint32_t SLOW_WS_BROADCAST_MS = 50;
static constexpr uint32_t SLOW_FS_SEND_MS = 500;
static constexpr uint32_t SLOW_FS_TOTAL_MS = 1000;
static constexpr size_t HTTP_FILE_CHUNK_SIZE = 1024;
static constexpr size_t HTTP_FILE_DIRECT_SEND_LIMIT = 32768;
static constexpr size_t STATUS_JSON_BUFFER_SIZE = 2048;
static constexpr uint32_t HEALTH_LOG_INTERVAL_MS = 5000;
static constexpr uint32_t STATUS_CACHE_REFRESH_MS = 100;

#ifndef WEB_DEBUG_LOG
#define WEB_DEBUG_LOG 0
#endif

#ifndef WEB_SOCKET_ENABLE
#define WEB_SOCKET_ENABLE 1
#endif

#ifndef HTTP_TIMING_LOG_ENABLE
#define HTTP_TIMING_LOG_ENABLE 1
#endif

#ifndef HEALTH_LOG_ENABLE
#define HEALTH_LOG_ENABLE 0
#endif

#ifndef FS_BENCHMARK_LOG_ENABLE
#define FS_BENCHMARK_LOG_ENABLE 0
#endif

#ifndef CONFIG_DEBUG_LOG_ENABLE
#define CONFIG_DEBUG_LOG_ENABLE 0
#endif

Configuration g_config;
WifiManagerClass g_wifiManager(g_config);

static AppRuntimeHooks g_hooks = {
  "/wifi-manager/index.html",
  "CableCar",
  nullptr,
  nullptr,
  nullptr,
};

static httpd_handle_t g_webHttpd = nullptr;
static TaskHandle_t g_webTaskHandle = nullptr;
static WebServer g_webServer(80);
static AsyncWebServer g_asyncWebServer(80);
static AsyncWebSocket g_statusWs("/ws");
static bool g_webServerStarted = false;
static portMUX_TYPE g_statusMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t g_lastArtMs = 0;
static uint32_t g_webTaskLastActiveMs = 0;
static uint32_t g_webTaskWindowStartUs = 0;
static uint32_t g_webTaskBusyUsAccum = 0;
static uint16_t g_webTaskUtilPermille = 0;

static portMUX_TYPE g_wsClientMux = portMUX_INITIALIZER_UNLOCKED;
static int g_wsClientFds[MAX_WS_CLIENTS] = {-1, -1, -1, -1, -1, -1};
static bool g_wsClientPending[MAX_WS_CLIENTS] = {false, false, false, false, false, false};
static uint32_t g_lastWsPushMs = 0;
static uint32_t g_lastWsPingMs = 0;
static uint32_t g_lastHealthLogMs = 0;
static portMUX_TYPE g_fileSendMux = portMUX_INITIALIZER_UNLOCKED;
static int g_activeFileSends = 0;
static portMUX_TYPE g_httpSockMux = portMUX_INITIALIZER_UNLOCKED;
static int g_httpSockFds[MAX_WS_CLIENTS + 8] = {-1};
static uint32_t g_httpSockOpenMs[MAX_WS_CLIENTS + 8] = {0};
static char g_statusJsonCache[STATUS_JSON_BUFFER_SIZE];
static size_t g_statusJsonCacheLen = 0;
static uint32_t g_statusJsonCacheBuiltMs = 0;
static bool g_asyncUpdateOk = false;

struct PendingWsFrame {
  int fd;
  size_t len;
  uint8_t* payload;
};

struct PendingWsPing {
  int fd;
};

static void noteWebTaskWorkUs(uint32_t busyUs) {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  portENTER_CRITICAL(&g_statusMux);
  g_webTaskLastActiveMs = nowMs;
  if (g_webTaskWindowStartUs == 0) {
    g_webTaskWindowStartUs = nowUs;
  }
  g_webTaskBusyUsAccum += busyUs;

  uint32_t elapsedUs = nowUs - g_webTaskWindowStartUs;
  if (elapsedUs >= 1000000UL) {
    uint32_t permille = (g_webTaskBusyUsAccum * 1000UL) / elapsedUs;
    g_webTaskUtilPermille = (uint16_t)(permille > 1000UL ? 1000UL : permille);
    g_webTaskBusyUsAccum = 0;
    g_webTaskWindowStartUs = nowUs;
  }
  portEXIT_CRITICAL(&g_statusMux);
}

static void logFileReadPerf(const char* path) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f || f.isDirectory()) {
    Serial.printf("[FS] open failed: %s\n", path);
    return;
  }

  const size_t size = f.size();
  uint8_t buf[1024];
  size_t total = 0;
  const uint32_t start = millis();

  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    if (n == 0) break;
    total += n;
  }

  const uint32_t elapsed = millis() - start;
  f.close();
  Serial.printf("[FS] read %s %lu/%lu B in %lu ms\n",
                path,
                (unsigned long)total,
                (unsigned long)size,
                (unsigned long)elapsed);
}

Configuration& appConfig() { return g_config; }
WifiManagerClass& appWifiManager() { return g_wifiManager; }

ArtDmxPacket appParseArtDmx(const uint8_t* p, int len) {
  ArtDmxPacket r;
  if (len < 18) return r;
  if (memcmp(p, "Art-Net\0", 8) != 0) return r;
  if (!(p[8] == 0x00 && p[9] == 0x50)) return r;

  uint8_t subUni = p[14];
  uint8_t net = p[15];
  uint16_t dlen = ((uint16_t)p[16] << 8) | p[17];
  if (dlen > 512 || 18 + dlen > len) return r;

  uint8_t subnet = (subUni >> 4) & 0x0F;
  uint8_t uni = subUni & 0x0F;
  uint16_t flat = ((uint16_t)net << 8) | ((uint16_t)subnet << 4) | uni;

  r.ok = true;
  r.universe_flat = flat;
  r.length = dlen;
  r.data = p + 18;
  return r;
}

void appMarkArtnetActivity() {
  portENTER_CRITICAL(&g_statusMux);
  g_lastArtMs = millis();
  portEXIT_CRITICAL(&g_statusMux);
}

uint32_t appGetLastArtnetMs() {
  uint32_t last;
  portENTER_CRITICAL(&g_statusMux);
  last = g_lastArtMs;
  portEXIT_CRITICAL(&g_statusMux);
  return last;
}

const char* appResetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN: return "unknown";
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_EXT: return "external";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "other";
  }
}

float appReadBoardTemperatureC() {
#if defined(ARDUINO_ARCH_ESP32)
  float celsius = temperatureRead();
  if (isnan(celsius) || isinf(celsius)) return NAN;
  if (celsius < -100.0f || celsius > 150.0f) return NAN;
  return celsius;
#else
  return NAN;
#endif
}

static const char* httpMethodToString(int method) {
  switch (method) {
    case HTTP_GET: return "GET";
    case HTTP_POST: return "POST";
    case HTTP_PUT: return "PUT";
    case HTTP_PATCH: return "PATCH";
    case HTTP_DELETE: return "DELETE";
    case HTTP_HEAD: return "HEAD";
    case HTTP_OPTIONS: return "OPTIONS";
    default: return "OTHER";
  }
}

static bool shouldLogRequest(httpd_req_t* req, uint32_t durationMs, int statusCode) {
  if (statusCode >= 400 || durationMs >= SLOW_HTTP_MS) return true;
  if (strcmp(req->uri, "/") == 0) return true;
  if (strcmp(req->uri, "/index.html") == 0) return true;
  if (strcmp(req->uri, "/settings.html") == 0) return true;
  if (strcmp(req->uri, "/ota.html") == 0) return true;
  if (strcmp(req->uri, "/credentials") == 0) return true;
  if (strcmp(req->uri, "/update") == 0) return true;
  if (strcmp(req->uri, "/updatefs") == 0) return true;
  return false;
}

static void logHttpRequest(httpd_req_t* req, uint32_t startMs, int statusCode, const char* detail = nullptr) {
  uint32_t durationMs = millis() - startMs;
  noteWebTaskWorkUs(durationMs * 1000UL);
#if !WEB_DEBUG_LOG
  (void)req;
  (void)statusCode;
  (void)detail;
  return;
#else
  if (!shouldLogRequest(req, durationMs, statusCode)) return;

  if (detail && detail[0] != '\0') {
    Serial.printf("[HTTP] %s %s -> %d in %lu ms (%s)\n",
                  httpMethodToString(req->method),
                  req->uri,
                  statusCode,
                  (unsigned long)durationMs,
                  detail);
  } else {
    Serial.printf("[HTTP] %s %s -> %d in %lu ms\n",
                  httpMethodToString(req->method),
                  req->uri,
                  statusCode,
                  (unsigned long)durationMs);
  }
#endif
}

static void setCorsHeaders(httpd_req_t* req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,PUT,POST,OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

static void setConnectionCloseHeader(httpd_req_t* req) {
  httpd_resp_set_hdr(req, "Connection", "close");
}

static void noteHttpSocketOpen(int sockfd) {
  portENTER_CRITICAL(&g_httpSockMux);
  for (size_t i = 0; i < (sizeof(g_httpSockFds) / sizeof(g_httpSockFds[0])); i++) {
    if (g_httpSockFds[i] == sockfd || g_httpSockFds[i] < 0) {
      g_httpSockFds[i] = sockfd;
      g_httpSockOpenMs[i] = millis();
      break;
    }
  }
  portEXIT_CRITICAL(&g_httpSockMux);
}

static void noteHttpSocketClose(int sockfd) {
  portENTER_CRITICAL(&g_httpSockMux);
  for (size_t i = 0; i < (sizeof(g_httpSockFds) / sizeof(g_httpSockFds[0])); i++) {
    if (g_httpSockFds[i] == sockfd) {
      g_httpSockFds[i] = -1;
      g_httpSockOpenMs[i] = 0;
      break;
    }
  }
  portEXIT_CRITICAL(&g_httpSockMux);
}

static uint32_t getHttpSocketAgeMs(int sockfd) {
  uint32_t ageMs = 0xFFFFFFFFu;
  uint32_t nowMs = millis();
  portENTER_CRITICAL(&g_httpSockMux);
  for (size_t i = 0; i < (sizeof(g_httpSockFds) / sizeof(g_httpSockFds[0])); i++) {
    if (g_httpSockFds[i] == sockfd) {
      ageMs = (g_httpSockOpenMs[i] == 0) ? 0xFFFFFFFFu : (nowMs - g_httpSockOpenMs[i]);
      break;
    }
  }
  portEXIT_CRITICAL(&g_httpSockMux);
  return ageMs;
}

static esp_err_t onHttpSocketOpen(httpd_handle_t hd, int sockfd) {
  (void)hd;
  int flag = 1;
  if (setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) != 0) {
    return ESP_FAIL;
  }
  noteHttpSocketOpen(sockfd);
  return ESP_OK;
}

static void onHttpSocketClose(httpd_handle_t hd, int sockfd) {
  (void)hd;
  noteHttpSocketClose(sockfd);
}

static esp_err_t optionsHandler(httpd_req_t* req) {
  uint32_t startMs = millis();
  setCorsHeaders(req);
  setConnectionCloseHeader(req);
  httpd_resp_set_status(req, "204 No Content");
  esp_err_t ret = httpd_resp_send(req, nullptr, 0);
  logHttpRequest(req, startMs, ret == ESP_OK ? 204 : 500);
  return ret;
}

static void logHttpTimingBreakdown(httpd_req_t* req,
                                   const char* tag,
                                   uint32_t totalMs,
                                   uint32_t prepMs,
                                   uint32_t sendMs,
                                   int statusCode,
                                   int sendResult) {
#if !HTTP_TIMING_LOG_ENABLE
  (void)req;
  (void)tag;
  (void)totalMs;
  (void)prepMs;
  (void)sendMs;
  (void)statusCode;
  (void)sendResult;
#else
  int fd = req ? httpd_req_to_sockfd(req) : -1;
  uint32_t sockAgeMs = (fd >= 0) ? getHttpSocketAgeMs(fd) : 0xFFFFFFFFu;
  Serial.printf("[HTIM] %s %s %s fd=%d age=%ld status=%d total=%lu prep=%lu send=%lu ret=%d\n",
                tag,
                req ? httpMethodToString(req->method) : "?",
                req ? req->uri : "?",
                fd,
                (long)(sockAgeMs == 0xFFFFFFFFu ? -1 : (int32_t)sockAgeMs),
                statusCode,
                (unsigned long)totalMs,
                (unsigned long)prepMs,
                (unsigned long)sendMs,
                sendResult);
#endif
}

static bool requestAcceptsGzip(httpd_req_t* req) {
  if (!req) return false;
  size_t len = httpd_req_get_hdr_value_len(req, "Accept-Encoding");
  if (len == 0 || len > 255) return false;

  char value[256];
  if (httpd_req_get_hdr_value_str(req, "Accept-Encoding", value, sizeof(value)) != ESP_OK) {
    return false;
  }

  return strstr(value, "gzip") != nullptr;
}

static esp_err_t sendFsFile(httpd_req_t* req, const char* path, const char* contentType) {
  uint32_t startMs = millis();

  portENTER_CRITICAL(&g_fileSendMux);
  g_activeFileSends++;
  portEXIT_CRITICAL(&g_fileSendMux);

  String resolvedPath = path;
  bool servingGzip = false;
  if (requestAcceptsGzip(req)) {
    String gzPath = resolvedPath + ".gz";
    File gzFile = LittleFS.open(gzPath, FILE_READ);
    if (gzFile && !gzFile.isDirectory()) {
      resolvedPath = gzPath;
      servingGzip = true;
      gzFile.close();
    }
  }

  File file = LittleFS.open(resolvedPath.c_str(), FILE_READ);
  if (!file || file.isDirectory()) {
    httpd_resp_set_status(req, "404 Not Found");
    esp_err_t ret = httpd_resp_send(req, "Not Found", HTTPD_RESP_USE_STRLEN);
    logHttpRequest(req, startMs, 404, path);
    portENTER_CRITICAL(&g_fileSendMux);
    g_activeFileSends--;
    portEXIT_CRITICAL(&g_fileSendMux);
    return ret;
  }

  size_t size = file.size();
  size_t allocSize = (size <= HTTP_FILE_DIRECT_SEND_LIMIT) ? (size + 1) : HTTP_FILE_CHUNK_SIZE;
  char* buf = (char*)malloc(allocSize);
  if (!buf) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    esp_err_t ret = httpd_resp_send(req, "OOM", HTTPD_RESP_USE_STRLEN);
    logHttpRequest(req, startMs, 500, "malloc failed");
    portENTER_CRITICAL(&g_fileSendMux);
    g_activeFileSends--;
    portEXIT_CRITICAL(&g_fileSendMux);
    return ret;
  }

  httpd_resp_set_type(req, contentType);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  setConnectionCloseHeader(req);
  if (servingGzip) {
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Vary", "Accept-Encoding");
  }
  esp_err_t ret = ESP_OK;
  size_t totalSent = 0;
  uint32_t readMs = 0;
  uint32_t sendMs = 0;

  if (size <= HTTP_FILE_DIRECT_SEND_LIMIT) {
    uint32_t t0 = millis();
    size_t bytesRead = file.read((uint8_t*)buf, size);
    readMs = millis() - t0;
    if (bytesRead != size) {
      ret = ESP_FAIL;
      totalSent = bytesRead;
    } else {
      t0 = millis();
      ret = httpd_resp_send(req, buf, size);
      sendMs = millis() - t0;
      totalSent = size;
    }
  } else {
    while (file.available()) {
      uint32_t t0 = millis();
      size_t bytesRead = file.read((uint8_t*)buf, HTTP_FILE_CHUNK_SIZE);
      readMs += millis() - t0;
      if (bytesRead == 0) {
        break;
      }

      t0 = millis();
      ret = httpd_resp_send_chunk(req, buf, bytesRead);
      sendMs += millis() - t0;
      if (ret != ESP_OK) {
        break;
      }
      totalSent += bytesRead;
    }

    if (ret == ESP_OK) {
      ret = httpd_resp_send_chunk(req, nullptr, 0);
    }
  }

  const uint32_t totalMs = millis() - startMs;
  if (WEB_DEBUG_LOG || sendMs >= SLOW_FS_SEND_MS || totalMs >= SLOW_FS_TOTAL_MS) {
    Serial.printf("[FS] %s%s read=%lu ms send=%lu ms total=%lu ms size=%lu\n",
                  path,
                  servingGzip ? " (gzip)" : "",
                  (unsigned long)readMs,
                  (unsigned long)sendMs,
                  (unsigned long)totalMs,
                  (unsigned long)size);
  }

  char detail[128];
  snprintf(detail,
           sizeof(detail),
           "%s%s, %lu/%lu B, heap=%lu, err=%d",
           path,
           servingGzip ? " (gzip)" : "",
           (unsigned long)totalSent,
           (unsigned long)size,
           (unsigned long)ESP.getFreeHeap(),
           (int)ret);
  file.close();
  free(buf);
  logHttpTimingBreakdown(req,
                         "fs",
                         millis() - startMs,
                         readMs,
                         sendMs,
                         ret == ESP_OK ? 200 : 500,
                         (int)ret);
  logHttpRequest(req, startMs, ret == ESP_OK ? 200 : 500, detail);

  portENTER_CRITICAL(&g_fileSendMux);
  g_activeFileSends--;
  portEXIT_CRITICAL(&g_fileSendMux);

  return ret;
}

static esp_err_t indexHandler(httpd_req_t* req) {
  return sendFsFile(req, g_hooks.indexPagePath, "text/html");
}
static esp_err_t otaPageHandler(httpd_req_t* req) {
  return sendFsFile(req, "/wifi-manager/ota.html", "text/html");
}
static esp_err_t settingHandler(httpd_req_t* req) {
  return sendFsFile(req, "/wifi-manager/settings.html", "text/html");
}

static esp_err_t statusHandler(httpd_req_t* req) {
  uint32_t startMs = millis();
  if (!g_hooks.buildStatusJson) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    esp_err_t ret = httpd_resp_send(req, "Status callback missing", HTTPD_RESP_USE_STRLEN);
    logHttpRequest(req, startMs, 500, "status callback missing");
    return ret;
  }

  bool details = false;
  size_t qlen = httpd_req_get_url_query_len(req);
  if (qlen > 0 && qlen < 64) {
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
      char value[8];
      if (httpd_query_key_value(query, "details", value, sizeof(value)) == ESP_OK) {
        details = (strcmp(value, "1") == 0 || strcmp(value, "true") == 0);
      }
    }
  }

  char out[STATUS_JSON_BUFFER_SIZE];
  size_t outLen = 0;
  uint32_t prepStartMs = millis();
  if (!details) {
    portENTER_CRITICAL(&g_statusMux);
    outLen = g_statusJsonCacheLen;
    if (outLen > sizeof(out)) outLen = sizeof(out);
    if (outLen > 0) {
      memcpy(out, g_statusJsonCache, outLen);
    }
    portEXIT_CRITICAL(&g_statusMux);
  }

  if (outLen == 0) {
    outLen = g_hooks.buildStatusJson(out, sizeof(out), details);
    if (!details && outLen > 0 && outLen <= sizeof(g_statusJsonCache)) {
      portENTER_CRITICAL(&g_statusMux);
      memcpy(g_statusJsonCache, out, outLen);
      g_statusJsonCacheLen = outLen;
      g_statusJsonCacheBuiltMs = millis();
      portEXIT_CRITICAL(&g_statusMux);
    }
  }
  uint32_t prepMs = millis() - prepStartMs;
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  setConnectionCloseHeader(req);
  uint32_t sendStartMs = millis();
  esp_err_t ret = httpd_resp_send(req, out, outLen);
  uint32_t sendMs = millis() - sendStartMs;
  char detail[48];
  snprintf(detail, sizeof(detail), "details=%d, %lu B", details ? 1 : 0, (unsigned long)outLen);
  logHttpTimingBreakdown(req,
                         "status",
                         millis() - startMs,
                         prepMs,
                         sendMs,
                         ret == ESP_OK ? 200 : 500,
                         (int)ret);
  logHttpRequest(req, startMs, ret == ESP_OK ? 200 : 500, detail);
  return ret;
}

static esp_err_t notFoundHandler(httpd_req_t* req, httpd_err_code_t error) {
  (void)error;
  uint32_t startMs = millis();
  setConnectionCloseHeader(req);
  uint32_t sendStartMs = millis();
  esp_err_t ret = httpd_resp_send_404(req);
  uint32_t sendMs = millis() - sendStartMs;
  logHttpTimingBreakdown(req,
                         "404",
                         millis() - startMs,
                         0,
                         sendMs,
                         404,
                         (int)ret);
  logHttpRequest(req, startMs, 404, "not found");
  return ret;
}

static void wsAddClient(int fd) {
  if (fd < 0) return;
  portENTER_CRITICAL(&g_wsClientMux);
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (g_wsClientFds[i] == fd) {
      portEXIT_CRITICAL(&g_wsClientMux);
      return;
    }
  }
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (g_wsClientFds[i] < 0) {
      g_wsClientFds[i] = fd;
      g_wsClientPending[i] = false;
      break;
    }
  }
  portEXIT_CRITICAL(&g_wsClientMux);
}

static void wsRemoveClient(int fd) {
  if (fd < 0) return;
  portENTER_CRITICAL(&g_wsClientMux);
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (g_wsClientFds[i] == fd) {
      g_wsClientFds[i] = -1;
      g_wsClientPending[i] = false;
      break;
    }
  }
  portEXIT_CRITICAL(&g_wsClientMux);
}

static int wsCopyClients(int* out, int maxCount) {
  int count = 0;
  portENTER_CRITICAL(&g_wsClientMux);
  for (int i = 0; i < MAX_WS_CLIENTS && count < maxCount; i++) {
    if (g_wsClientFds[i] >= 0) out[count++] = g_wsClientFds[i];
  }
  portEXIT_CRITICAL(&g_wsClientMux);
  return count;
}

static int wsClientCount() {
  int count = 0;
  portENTER_CRITICAL(&g_wsClientMux);
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (g_wsClientFds[i] >= 0) count++;
  }
  portEXIT_CRITICAL(&g_wsClientMux);
  return count;
}

static bool wsTryMarkPending(int fd) {
  bool marked = false;
  portENTER_CRITICAL(&g_wsClientMux);
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (g_wsClientFds[i] == fd) {
      if (!g_wsClientPending[i]) {
        g_wsClientPending[i] = true;
        marked = true;
      }
      break;
    }
  }
  portEXIT_CRITICAL(&g_wsClientMux);
  return marked;
}

static void wsClearPending(int fd) {
  if (fd < 0) return;
  portENTER_CRITICAL(&g_wsClientMux);
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (g_wsClientFds[i] == fd) {
      g_wsClientPending[i] = false;
      break;
    }
  }
  portEXIT_CRITICAL(&g_wsClientMux);
}

static void freePendingWsFrame(PendingWsFrame* pending) {
  if (!pending) return;
  free(pending->payload);
  free(pending);
}

static void wsSendPingWork(void* arg) {
  PendingWsPing* pending = (PendingWsPing*)arg;
  if (!pending) return;

  if (httpd_ws_get_fd_info(g_webHttpd, pending->fd) == HTTPD_WS_CLIENT_WEBSOCKET) {
    httpd_ws_frame_t ping = {};
    ping.type = HTTPD_WS_TYPE_PING;
    if (httpd_ws_send_frame_async(g_webHttpd, pending->fd, &ping) != ESP_OK) {
      wsRemoveClient(pending->fd);
    }
  } else {
    wsRemoveClient(pending->fd);
  }

  free(pending);
}

static esp_err_t wsQueuePingToClient(int fd) {
  if (httpd_ws_get_fd_info(g_webHttpd, fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
    wsRemoveClient(fd);
    return ESP_ERR_INVALID_STATE;
  }

  PendingWsPing* pending = (PendingWsPing*)calloc(1, sizeof(PendingWsPing));
  if (!pending) return ESP_ERR_NO_MEM;

  pending->fd = fd;
  esp_err_t ret = httpd_queue_work(g_webHttpd, wsSendPingWork, pending);
  if (ret != ESP_OK) {
    free(pending);
  }
  return ret;
}

static esp_err_t buildStatusPayload(uint8_t** payloadOut, size_t* lenOut) {
  if (!payloadOut || !lenOut || !g_hooks.buildStatusJson) return ESP_ERR_INVALID_ARG;
  char* out = (char*)malloc(STATUS_JSON_BUFFER_SIZE);
  if (!out) return ESP_ERR_NO_MEM;

  size_t outLen = 0;
  portENTER_CRITICAL(&g_statusMux);
  outLen = g_statusJsonCacheLen;
  if (outLen > STATUS_JSON_BUFFER_SIZE) outLen = STATUS_JSON_BUFFER_SIZE;
  if (outLen > 0) {
    memcpy(out, g_statusJsonCache, outLen);
  }
  portEXIT_CRITICAL(&g_statusMux);

  if (outLen == 0) {
    outLen = g_hooks.buildStatusJson(out, STATUS_JSON_BUFFER_SIZE, false);
    if (outLen > 0 && outLen <= sizeof(g_statusJsonCache)) {
      portENTER_CRITICAL(&g_statusMux);
      memcpy(g_statusJsonCache, out, outLen);
      g_statusJsonCacheLen = outLen;
      g_statusJsonCacheBuiltMs = millis();
      portEXIT_CRITICAL(&g_statusMux);
    }
  }

  *payloadOut = (uint8_t*)out;
  *lenOut = outLen;
  return ESP_OK;
}

static void refreshStatusCacheIfDue(uint32_t nowMs) {
  if (!g_hooks.buildStatusJson) return;

  uint32_t lastBuiltMs = 0;
  portENTER_CRITICAL(&g_statusMux);
  lastBuiltMs = g_statusJsonCacheBuiltMs;
  portEXIT_CRITICAL(&g_statusMux);

  if (lastBuiltMs != 0 && (nowMs - lastBuiltMs) < STATUS_CACHE_REFRESH_MS) {
    return;
  }

  char out[STATUS_JSON_BUFFER_SIZE];
  size_t outLen = g_hooks.buildStatusJson(out, sizeof(out), false);
  if (outLen == 0 || outLen > sizeof(g_statusJsonCache)) {
    return;
  }

  portENTER_CRITICAL(&g_statusMux);
  memcpy(g_statusJsonCache, out, outLen);
  g_statusJsonCacheLen = outLen;
  g_statusJsonCacheBuiltMs = nowMs;
  portEXIT_CRITICAL(&g_statusMux);
}

static void wsSendStatusWork(void* arg) {
  PendingWsFrame* pending = (PendingWsFrame*)arg;
  if (!pending) return;

  if (httpd_ws_get_fd_info(g_webHttpd, pending->fd) == HTTPD_WS_CLIENT_WEBSOCKET) {
    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = pending->payload;
    frame.len = pending->len;
    if (httpd_ws_send_frame_async(g_webHttpd, pending->fd, &frame) != ESP_OK) {
      wsRemoveClient(pending->fd);
    }
  } else {
    wsRemoveClient(pending->fd);
  }

  wsClearPending(pending->fd);
  freePendingWsFrame(pending);
}

static esp_err_t wsSendStatusToClient(int fd) {
  if (!g_hooks.buildStatusJson) return ESP_FAIL;
  if (httpd_ws_get_fd_info(g_webHttpd, fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
    wsRemoveClient(fd);
    return ESP_ERR_INVALID_STATE;
  }
  if (!wsTryMarkPending(fd)) {
    return ESP_OK;
  }

  PendingWsFrame* pending = (PendingWsFrame*)calloc(1, sizeof(PendingWsFrame));
  if (!pending) {
    wsClearPending(fd);
    return ESP_ERR_NO_MEM;
  }

  esp_err_t ret = buildStatusPayload(&pending->payload, &pending->len);
  if (ret != ESP_OK) {
    wsClearPending(fd);
    freePendingWsFrame(pending);
    return ret;
  }

  pending->fd = fd;
  ret = httpd_queue_work(g_webHttpd, wsSendStatusWork, pending);
  if (ret != ESP_OK) {
    wsClearPending(fd);
    freePendingWsFrame(pending);
  }
  return ret;
}

static void wsBroadcastStatus() {
  int activeSends;
  portENTER_CRITICAL(&g_fileSendMux);
  activeSends = g_activeFileSends;
  portEXIT_CRITICAL(&g_fileSendMux);

  if (activeSends > 0) {
    return;
  }

  int clients[MAX_WS_CLIENTS];
  int count = wsCopyClients(clients, MAX_WS_CLIENTS);
  if (count <= 0) {
    return;
  }

  uint32_t startMs = millis();
  int failed = 0;
  for (int i = 0; i < count; i++) {
    if (wsSendStatusToClient(clients[i]) != ESP_OK) {
      wsRemoveClient(clients[i]);
      failed++;
    }
  }

  uint32_t durationMs = millis() - startMs;
  noteWebTaskWorkUs(durationMs * 1000UL);
#if WEB_DEBUG_LOG
  if (failed > 0 || durationMs >= SLOW_WS_BROADCAST_MS) {
    Serial.printf("[WS] broadcast clients=%d failed=%d in %lu ms heap=%lu\n",
                  count,
                  failed,
                  (unsigned long)durationMs,
                  (unsigned long)ESP.getFreeHeap());
  }
#else
  (void)durationMs;
#endif
}

static esp_err_t wsHandler(httpd_req_t* req) {
  uint32_t startMs = millis();
  int fd = httpd_req_to_sockfd(req);
  if (req->method == HTTP_GET) {
    wsAddClient(fd);
#if WEB_DEBUG_LOG
    Serial.printf("[WS] client connected fd=%d uri=%s\n", fd, req->uri);
#endif
    return ESP_OK;
  }

  httpd_ws_frame_t frame = {};
  esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
  if (ret != ESP_OK) {
    wsRemoveClient(fd);
#if WEB_DEBUG_LOG
    Serial.printf("[WS] recv header failed fd=%d err=%d\n", fd, ret);
#endif
    return ret;
  }

  if (frame.type == HTTPD_WS_TYPE_CLOSE) {
    wsRemoveClient(fd);
#if WEB_DEBUG_LOG
    Serial.printf("[WS] client closed fd=%d\n", fd);
#endif
    return ESP_OK;
  }

  uint8_t* payload = nullptr;
  if (frame.len > 0) {
    payload = (uint8_t*)malloc(frame.len + 1);
    if (!payload) return ESP_ERR_NO_MEM;
    frame.payload = payload;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret == ESP_OK) payload[frame.len] = '\0';
  }

  wsAddClient(fd);
  if (ret == ESP_OK && frame.type == HTTPD_WS_TYPE_TEXT) {
    if (!payload || strcmp((const char*)payload, "status") == 0 || strcmp((const char*)payload, "ping") == 0) {
      ret = wsSendStatusToClient(fd);
    }
  }

  if (payload) free(payload);
  uint32_t durationMs = millis() - startMs;
#if WEB_DEBUG_LOG
  if (ret != ESP_OK || durationMs >= SLOW_HTTP_MS) {
    Serial.printf("[WS] fd=%d type=%d len=%u err=%d in %lu ms\n",
                  fd,
                  (int)frame.type,
                  (unsigned int)frame.len,
                  ret,
                  (unsigned long)durationMs);
  }
#else
  (void)durationMs;
#endif
  return ret;
}

static esp_err_t networksHandler(httpd_req_t* req) {
  uint32_t startMs = millis();
  bool details = false;
  bool refresh = false;
  bool refreshProvided = false;
  bool scanProvided = false;
  bool legacyDetailsOnly = false;
  size_t qlen = httpd_req_get_url_query_len(req);
  if (qlen > 0 && qlen < 64) {
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
      char value[8];
      if (httpd_query_key_value(query, "details", value, sizeof(value)) == ESP_OK) details = true;
      if (httpd_query_key_value(query, "refresh", value, sizeof(value)) == ESP_OK) {
        refreshProvided = true;
        refresh = (strcmp(value, "1") == 0 || strcmp(value, "true") == 0);
      }
      if (httpd_query_key_value(query, "scan", value, sizeof(value)) == ESP_OK) {
        scanProvided = true;
        refresh = (strcmp(value, "1") == 0 || strcmp(value, "true") == 0);
      }
      legacyDetailsOnly = details && !refreshProvided && !scanProvided;
    }
  }
  // Backward compatibility: older settings pages request details only on refresh click.
  if (legacyDetailsOnly) refresh = true;
  String payload = g_wifiManager.getNetworksPayload(details, refresh);
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t ret = httpd_resp_send(req, payload.c_str(), payload.length());
  char detail[48];
  snprintf(detail, sizeof(detail), "details=%d refresh=%d, %u B", details ? 1 : 0, refresh ? 1 : 0, (unsigned int)payload.length());
  logHttpRequest(req, startMs, ret == ESP_OK ? 200 : 500, detail);
  return ret;
}

static esp_err_t credentialsGetHandler(httpd_req_t* req) {
  uint32_t startMs = millis();
  DynamicJsonDocument doc(1024);
  int channel = g_config.getDMXAddress();
  int universe = g_config.getDMXUniverse();
  doc["ssid"] = g_config.getSSID();
  doc["hostname"] = g_config.getHostname();
  doc["password"] = g_config.getPass();
  doc["dhcp"] = g_config.getDhcpEnabled();
  doc["ip"] = g_config.getStaticIP();
  doc["gateway"] = g_config.getGateway();
  doc["subnet"] = g_config.getSubnet();
  doc["dns1"] = g_config.getDNS1();
  doc["dns2"] = g_config.getDNS2();
  doc["start_value"] = g_config.getStartValue();
  doc["channel"] = channel;
  doc["universe"] = universe;
  doc["dmx_address"] = channel;
  doc["dmx_universe"] = universe;

  String out;
  out.reserve(512);
  size_t outLen = serializeJson(doc, out);
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t ret = httpd_resp_send(req, out.c_str(), outLen);
  logHttpRequest(req, startMs, ret == ESP_OK ? 200 : 500);
  return ret;
}

static esp_err_t credentialsPutHandler(httpd_req_t* req) {
  uint32_t startMs = millis();
  int len = req->content_len;
  if (len <= 0 || len > 1024) {
    setCorsHeaders(req);
    httpd_resp_set_status(req, "400 Bad Request");
    esp_err_t ret = httpd_resp_send(req, "", 0);
    logHttpRequest(req, startMs, 400, "invalid content length");
    return ret;
  }

  char* body = (char*)malloc(len + 1);
  if (!body) {
    setCorsHeaders(req);
    httpd_resp_set_status(req, "500 Internal Server Error");
    esp_err_t ret = httpd_resp_send(req, "", 0);
    logHttpRequest(req, startMs, 500, "request body alloc failed");
    return ret;
  }

  int remaining = len;
  int offset = 0;
  char buf[256];
  while (remaining > 0) {
    int toRead = remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining;
    int read = httpd_req_recv(req, buf, toRead);
    if (read <= 0) {
      free(body);
      setCorsHeaders(req);
      httpd_resp_set_status(req, "500 Internal Server Error");
      esp_err_t ret = httpd_resp_send(req, "", 0);
      logHttpRequest(req, startMs, 500, "request body read failed");
      return ret;
    }

    memcpy(body + offset, buf, read);
    offset += read;
    remaining -= read;
  }
  body[offset] = '\0';

  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    free(body);
    setCorsHeaders(req);
    httpd_resp_set_status(req, "400 Bad Request");
    esp_err_t ret = httpd_resp_send(req, "", 0);
    logHttpRequest(req, startMs, 400, "invalid json");
    return ret;
  }
  auto invalidTextField = [](const char* value, bool allowEmpty, size_t maxLen) {
    if (!value) return !allowEmpty;
    size_t len = strlen(value);
    if (len == 0) return !allowEmpty;
    if (len > maxLen) return true;
    for (size_t i = 0; i < len; i++) {
      unsigned char ch = (unsigned char)value[i];
      if (ch < 32 || ch == 127) return true;
    }
    return false;
  };

  String ssidText = doc.containsKey("ssid") ? doc["ssid"].as<String>() : String();
  String hostnameText = doc.containsKey("hostname") ? doc["hostname"].as<String>() : String();
  String passwordText = doc.containsKey("password") ? doc["password"].as<String>() : String();
  const char* ssidValue = doc.containsKey("ssid") ? ssidText.c_str() : nullptr;
  const char* hostnameValue = doc.containsKey("hostname") ? hostnameText.c_str() : nullptr;
  const char* passwordValue = doc.containsKey("password") ? passwordText.c_str() : nullptr;
  bool dhcpValue = doc.containsKey("dhcp") ? doc["dhcp"].as<bool>() : g_config.getDhcpEnabled();
  float startValue = doc.containsKey("start_value") ? doc["start_value"].as<float>() : g_config.getStartValue();
#if CONFIG_DEBUG_LOG_ENABLE
  Serial.printf("[CFG PUT] ssid=\"%s\" hostname=\"%s\" pass_len=%u dhcp=%d start=%.6g\n",
                ssidValue ? ssidValue : "(null)",
                hostnameValue ? hostnameValue : "(null)",
                (unsigned int)(passwordValue ? strlen(passwordValue) : 0),
                dhcpValue ? 1 : 0,
                (double)startValue);
#endif
  if (ssidValue && invalidTextField(ssidValue, false, 32)) {
    setCorsHeaders(req);
    httpd_resp_set_status(req, "400 Bad Request");
    esp_err_t ret = httpd_resp_send(req, "Invalid SSID", HTTPD_RESP_USE_STRLEN);
    free(body);
    logHttpRequest(req, startMs, 400, "invalid ssid");
    return ret;
  }
  if (hostnameValue && invalidTextField(hostnameValue, true, 63)) {
    setCorsHeaders(req);
    httpd_resp_set_status(req, "400 Bad Request");
    esp_err_t ret = httpd_resp_send(req, "Invalid hostname", HTTPD_RESP_USE_STRLEN);
    free(body);
    logHttpRequest(req, startMs, 400, "invalid hostname");
    return ret;
  }

  bool writeOk = true;
  if (doc.containsKey("ssid")) writeOk = g_config.writeSSID(ssidText) && writeOk;
  if (doc.containsKey("password")) writeOk = g_config.writePass(passwordText) && writeOk;
  if (doc.containsKey("hostname")) writeOk = g_config.writeHostname(hostnameText) && writeOk;
  if (doc.containsKey("dhcp")) writeOk = g_config.writeDhcpEnabled(dhcpValue) && writeOk;
  if (doc.containsKey("ip")) writeOk = g_config.writeStaticIP(doc["ip"].as<String>()) && writeOk;
  if (doc.containsKey("gateway")) writeOk = g_config.writeGateway(doc["gateway"].as<String>()) && writeOk;
  if (doc.containsKey("subnet")) writeOk = g_config.writeSubnet(doc["subnet"].as<String>()) && writeOk;
  if (doc.containsKey("dns1")) writeOk = g_config.writeDNS1(doc["dns1"].as<String>()) && writeOk;
  if (doc.containsKey("dns2")) writeOk = g_config.writeDNS2(doc["dns2"].as<String>()) && writeOk;
  if (doc.containsKey("start_value")) writeOk = g_config.writeStartValue(startValue) && writeOk;

  if (doc.containsKey("channel")) {
    int channel = doc["channel"].as<int>();
    if (channel >= 1 && channel <= 512) writeOk = g_config.writeDMXAddress(channel) && writeOk;
  } else if (doc.containsKey("dmx_address")) {
    int channel = doc["dmx_address"].as<int>();
    if (channel >= 1 && channel <= 512) writeOk = g_config.writeDMXAddress(channel) && writeOk;
  }

  if (doc.containsKey("universe")) {
    int universe = doc["universe"].as<int>();
    if (universe >= 0 && universe <= 32767) writeOk = g_config.writeDMXUniverse(universe) && writeOk;
  } else if (doc.containsKey("dmx_universe")) {
    int universe = doc["dmx_universe"].as<int>();
    if (universe >= 0 && universe <= 32767) writeOk = g_config.writeDMXUniverse(universe) && writeOk;
  }

  if (!writeOk) {
    free(body);
    setCorsHeaders(req);
    httpd_resp_set_status(req, "500 Internal Server Error");
    esp_err_t ret = httpd_resp_send(req, "Failed to persist settings", HTTPD_RESP_USE_STRLEN);
    logHttpRequest(req, startMs, 500, "config write failed");
    return ret;
  }

#if CONFIG_DEBUG_LOG_ENABLE
  Serial.printf("[CFG VERIFY] ssid=\"%s\" hostname=\"%s\" pass_len=%u dhcp=%d start=%.6g\n",
                g_config.getSSID().c_str(),
                g_config.getHostname().c_str(),
                (unsigned int)g_config.getPass().length(),
                g_config.getDhcpEnabled() ? 1 : 0,
                (double)g_config.getStartValue());
#endif

  free(body);
  g_wifiManager.scheduleRestart(1000);
  setCorsHeaders(req);
  httpd_resp_set_status(req, "204 No Content");
  esp_err_t ret = httpd_resp_send(req, nullptr, 0);
  char detail[32];
  snprintf(detail, sizeof(detail), "body=%d B", len);
  logHttpRequest(req, startMs, ret == ESP_OK ? 204 : 500, detail);
  return ret;
}

static esp_err_t updateInfoHandler(httpd_req_t* req) {
  uint32_t startMs = millis();
  StaticJsonDocument<128> doc;
  doc["version"] = "1.0.0";
  doc["device"] = g_hooks.deviceName;
  doc["ota_ready"] = true;

  char out[128];
  size_t outLen = serializeJson(doc, out, sizeof(out));
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t ret = httpd_resp_send(req, out, outLen);
  logHttpRequest(req, startMs, ret == ESP_OK ? 200 : 500);
  return ret;
}

static void buildUpdateErrorJson(char* out, size_t outSize, const char* message) {
  StaticJsonDocument<256> doc;
  doc["success"] = false;
  doc["message"] = message ? message : "Update failed";
  doc["error"] = Update.errorString();
  serializeJson(doc, out, outSize);
}

static esp_err_t performUpdate(httpd_req_t* req, int updateCmd) {
  uint32_t startMs = millis();
  g_wifiManager.setOtaInProgress(true);

  bool beginOk = false;
  if (updateCmd == U_SPIFFS) {
    const esp_partition_t* fsPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "littlefs");
    beginOk = (fsPart != nullptr) && Update.begin(fsPart->size, U_SPIFFS);
  } else {
    beginOk = Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH);
  }

  if (!beginOk) {
    Update.printError(Serial);
    g_wifiManager.setOtaInProgress(false);
    setCorsHeaders(req);
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    char out[256];
    buildUpdateErrorJson(out, sizeof(out), updateCmd == U_SPIFFS ? "Filesystem update begin failed" : "Firmware update begin failed");
    esp_err_t ret = httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
    logHttpRequest(req, startMs, 400, Update.errorString());
    return ret;
  }

  int remaining = req->content_len;
  uint8_t buf2[1024];
  while (remaining > 0) {
    int toRead = remaining > (int)sizeof(buf2) ? (int)sizeof(buf2) : remaining;
    int read = httpd_req_recv(req, (char*)buf2, toRead);
    if (read <= 0) {
      Update.abort();
      g_wifiManager.setOtaInProgress(false);
      setCorsHeaders(req);
      httpd_resp_set_status(req, "500 Internal Server Error");
      esp_err_t ret = httpd_resp_send(req, "{\"success\":false,\"message\":\"Upload read failed\"}", HTTPD_RESP_USE_STRLEN);
      logHttpRequest(req, startMs, 500, "update upload read failed");
      return ret;
    }
    if (Update.write(buf2, read) != (size_t)read) {
      Update.printError(Serial);
      Update.abort();
      g_wifiManager.setOtaInProgress(false);
      setCorsHeaders(req);
      httpd_resp_set_status(req, "500 Internal Server Error");
      esp_err_t ret = httpd_resp_send(req, "{\"success\":false,\"message\":\"Update write failed\"}", HTTPD_RESP_USE_STRLEN);
      logHttpRequest(req, startMs, 500, "update write failed");
      return ret;
    }
    remaining -= read;
  }

  bool ok = Update.end(true);
  g_wifiManager.setOtaInProgress(false);
  if (!ok) {
    Update.printError(Serial);
    setCorsHeaders(req);
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    char out[256];
    buildUpdateErrorJson(out, sizeof(out), updateCmd == U_SPIFFS ? "Filesystem update finalize failed" : "Firmware update finalize failed");
    esp_err_t ret = httpd_resp_send(req, out, HTTPD_RESP_USE_STRLEN);
    logHttpRequest(req, startMs, 400, Update.errorString());
    return ret;
  }

  g_wifiManager.scheduleRestart(1000);
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  esp_err_t ret = httpd_resp_send(req, "{\"success\":true,\"message\":\"Update successful. Restarting...\"}", HTTPD_RESP_USE_STRLEN);
  logHttpRequest(req, startMs, ret == ESP_OK ? 200 : 500, updateCmd == U_SPIFFS ? "fs update ok" : "flash update ok");
  return ret;
}

static esp_err_t firmwareUpdateHandler(httpd_req_t* req) { return performUpdate(req, U_FLASH); }
static esp_err_t fsUpdateHandler(httpd_req_t* req) { return performUpdate(req, U_SPIFFS); }

static void asyncAddCommonHeaders(AsyncWebServerResponse* response, bool json = false) {
  if (!response) return;
  response->addHeader("Access-Control-Allow-Origin", "*");
  response->addHeader("Access-Control-Allow-Methods", "GET,PUT,POST,OPTIONS");
  response->addHeader("Access-Control-Allow-Headers", "Content-Type");
  if (json) {
    response->addHeader("Cache-Control", "no-store");
    response->setContentType("application/json");
  }
}

static bool asyncRequestAcceptsGzip(AsyncWebServerRequest* request) {
  if (!request || !request->hasHeader("Accept-Encoding")) return false;
  const AsyncWebHeader* hdr = request->getHeader("Accept-Encoding");
  return hdr && hdr->value().indexOf("gzip") >= 0;
}

static void asyncSendOptions(AsyncWebServerRequest* request) {
  uint32_t startMs = millis();
  AsyncWebServerResponse* response = request->beginResponse(204);
  asyncAddCommonHeaders(response);
  request->send(response);
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void asyncSendFsFile(AsyncWebServerRequest* request, const char* path, const char* contentType) {
  uint32_t startMs = millis();
  portENTER_CRITICAL(&g_fileSendMux);
  g_activeFileSends++;
  portEXIT_CRITICAL(&g_fileSendMux);

  String resolvedPath = path;
  bool servingGzip = false;
  if (asyncRequestAcceptsGzip(request)) {
    String gzPath = resolvedPath + ".gz";
    File gzFile = LittleFS.open(gzPath, FILE_READ);
    if (gzFile && !gzFile.isDirectory()) {
      resolvedPath = gzPath;
      servingGzip = true;
      gzFile.close();
    }
  }

  if (!LittleFS.exists(resolvedPath)) {
    AsyncWebServerResponse* response = request->beginResponse(404, "text/plain", "Not Found");
    asyncAddCommonHeaders(response);
    request->send(response);
    portENTER_CRITICAL(&g_fileSendMux);
    g_activeFileSends--;
    portEXIT_CRITICAL(&g_fileSendMux);
    return;
  }

  AsyncWebServerResponse* response = request->beginResponse(LittleFS, resolvedPath, contentType);
  asyncAddCommonHeaders(response);
  if (servingGzip) {
    response->addHeader("Content-Encoding", "gzip");
    response->addHeader("Vary", "Accept-Encoding");
  }
  request->send(response);
  uint32_t totalMs = millis() - startMs;
  noteWebTaskWorkUs(totalMs * 1000UL);
  if (HTTP_TIMING_LOG_ENABLE) {
    Serial.printf("[HTIM] fs-async GET %s status=200 total=%lu prep=0 send=%lu ret=0\n",
                  path,
                  (unsigned long)totalMs,
                  (unsigned long)totalMs);
  }
  portENTER_CRITICAL(&g_fileSendMux);
  g_activeFileSends--;
  portEXIT_CRITICAL(&g_fileSendMux);
}

static void asyncStatusHandler(AsyncWebServerRequest* request) {
  uint32_t startMs = millis();
  bool details = false;
  if (request->hasParam("details")) {
    String value = request->getParam("details")->value();
    details = (value == "1" || value == "true");
  }

  char out[STATUS_JSON_BUFFER_SIZE];
  size_t outLen = 0;
  if (!details) {
    portENTER_CRITICAL(&g_statusMux);
    outLen = g_statusJsonCacheLen;
    if (outLen > sizeof(out)) outLen = sizeof(out);
    if (outLen > 0) memcpy(out, g_statusJsonCache, outLen);
    portEXIT_CRITICAL(&g_statusMux);
  }
  if (outLen == 0 && g_hooks.buildStatusJson) {
    outLen = g_hooks.buildStatusJson(out, sizeof(out), details);
    if (!details && outLen > 0 && outLen <= sizeof(g_statusJsonCache)) {
      portENTER_CRITICAL(&g_statusMux);
      memcpy(g_statusJsonCache, out, outLen);
      g_statusJsonCacheLen = outLen;
      g_statusJsonCacheBuiltMs = millis();
      portEXIT_CRITICAL(&g_statusMux);
    }
  }

  AsyncWebServerResponse* response = request->beginResponse(200, "application/json", String(out).substring(0, outLen));
  asyncAddCommonHeaders(response, true);
  request->send(response);
  uint32_t totalMs = millis() - startMs;
  noteWebTaskWorkUs(totalMs * 1000UL);
  if (HTTP_TIMING_LOG_ENABLE) {
    Serial.printf("[HTIM] status-async GET /status status=200 total=%lu prep=0 send=%lu ret=0\n",
                  (unsigned long)totalMs,
                  (unsigned long)totalMs);
  }
}

static void asyncNetworksHandler(AsyncWebServerRequest* request) {
  uint32_t startMs = millis();
  bool details = false;
  bool refresh = false;
  bool refreshProvided = false;
  bool scanProvided = false;
  if (request->hasParam("details")) {
    String value = request->getParam("details")->value();
    details = (value == "1" || value == "true");
  }
  if (request->hasParam("refresh")) {
    refreshProvided = true;
    String value = request->getParam("refresh")->value();
    refresh = (value == "1" || value == "true");
  }
  if (request->hasParam("scan")) {
    scanProvided = true;
    String value = request->getParam("scan")->value();
    refresh = (value == "1" || value == "true");
  }
  if (details && !refreshProvided && !scanProvided) refresh = true;
  String payload = g_wifiManager.getNetworksPayload(details, refresh);
  AsyncWebServerResponse* response = request->beginResponse(200, "application/json", payload);
  asyncAddCommonHeaders(response, true);
  request->send(response);
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void asyncCredentialsGetHandler(AsyncWebServerRequest* request) {
  uint32_t startMs = millis();
  DynamicJsonDocument doc(1024);
  int channel = g_config.getDMXAddress();
  int universe = g_config.getDMXUniverse();
  doc["ssid"] = g_config.getSSID();
  doc["hostname"] = g_config.getHostname();
  doc["password"] = g_config.getPass();
  doc["dhcp"] = g_config.getDhcpEnabled();
  doc["ip"] = g_config.getStaticIP();
  doc["gateway"] = g_config.getGateway();
  doc["subnet"] = g_config.getSubnet();
  doc["dns1"] = g_config.getDNS1();
  doc["dns2"] = g_config.getDNS2();
  doc["start_value"] = g_config.getStartValue();
  doc["channel"] = channel;
  doc["universe"] = universe;
  doc["dmx_address"] = channel;
  doc["dmx_universe"] = universe;
  String out;
  serializeJson(doc, out);
  AsyncWebServerResponse* response = request->beginResponse(200, "application/json", out);
  asyncAddCommonHeaders(response, true);
  request->send(response);
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void asyncCredentialsPutBody(AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
  if (!index) {
    char* body = (char*)malloc(total + 1);
    if (!body) return;
    request->_tempObject = body;
  }
  char* body = (char*)request->_tempObject;
  if (!body) return;
  memcpy(body + index, data, len);
  if (index + len == total) {
    body[total] = '\0';
  }
}

static void asyncCredentialsPutHandler(AsyncWebServerRequest* request) {
  uint32_t startMs = millis();
  char* body = (char*)request->_tempObject;
  if (!body) {
    AsyncWebServerResponse* response = request->beginResponse(400, "text/plain", "");
    asyncAddCommonHeaders(response);
    request->send(response);
    noteWebTaskWorkUs((millis() - startMs) * 1000UL);
    return;
  }

  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, static_cast<const char*>(body));
  free(body);
  request->_tempObject = nullptr;
  if (err) {
    AsyncWebServerResponse* response = request->beginResponse(400, "text/plain", "");
    asyncAddCommonHeaders(response);
    request->send(response);
    noteWebTaskWorkUs((millis() - startMs) * 1000UL);
    return;
  }

  auto invalidTextField = [](const char* value, bool allowEmpty, size_t maxLen) {
    if (!value) return !allowEmpty;
    size_t llen = strlen(value);
    if (llen == 0) return !allowEmpty;
    if (llen > maxLen) return true;
    for (size_t i = 0; i < llen; i++) {
      unsigned char ch = (unsigned char)value[i];
      if (ch < 32 || ch == 127) return true;
    }
    return false;
  };

  String ssidText = doc.containsKey("ssid") ? doc["ssid"].as<String>() : String();
  String hostnameText = doc.containsKey("hostname") ? doc["hostname"].as<String>() : String();
  String passwordText = doc.containsKey("password") ? doc["password"].as<String>() : String();
  const char* ssidValue = doc.containsKey("ssid") ? ssidText.c_str() : nullptr;
  const char* hostnameValue = doc.containsKey("hostname") ? hostnameText.c_str() : nullptr;
  const char* passwordValue = doc.containsKey("password") ? passwordText.c_str() : nullptr;
  bool dhcpValue = doc.containsKey("dhcp") ? doc["dhcp"].as<bool>() : g_config.getDhcpEnabled();
  float startValue = doc.containsKey("start_value") ? doc["start_value"].as<float>() : g_config.getStartValue();
  if (ssidValue && invalidTextField(ssidValue, false, 32)) {
    AsyncWebServerResponse* response = request->beginResponse(400, "text/plain", "Invalid SSID");
    asyncAddCommonHeaders(response);
    request->send(response);
    noteWebTaskWorkUs((millis() - startMs) * 1000UL);
    return;
  }
  if (hostnameValue && invalidTextField(hostnameValue, true, 63)) {
    AsyncWebServerResponse* response = request->beginResponse(400, "text/plain", "Invalid hostname");
    asyncAddCommonHeaders(response);
    request->send(response);
    noteWebTaskWorkUs((millis() - startMs) * 1000UL);
    return;
  }

  bool writeOk = true;
  if (doc.containsKey("ssid")) {
    bool ok = g_config.writeSSID(ssidText);
    writeOk = ok && writeOk;
  }
  if (doc.containsKey("password")) {
    bool ok = g_config.writePass(passwordText);
    writeOk = ok && writeOk;
  }
  if (doc.containsKey("hostname")) {
    bool ok = g_config.writeHostname(hostnameText);
    writeOk = ok && writeOk;
  }
  if (doc.containsKey("dhcp")) {
    bool ok = g_config.writeDhcpEnabled(dhcpValue);
    writeOk = ok && writeOk;
  }
  if (doc.containsKey("ip")) {
    String value = doc["ip"].as<String>();
    bool ok = g_config.writeStaticIP(value);
    writeOk = ok && writeOk;
  }
  if (doc.containsKey("gateway")) {
    String value = doc["gateway"].as<String>();
    bool ok = g_config.writeGateway(value);
    writeOk = ok && writeOk;
  }
  if (doc.containsKey("subnet")) {
    String value = doc["subnet"].as<String>();
    bool ok = g_config.writeSubnet(value);
    writeOk = ok && writeOk;
  }
  if (doc.containsKey("dns1")) {
    String value = doc["dns1"].as<String>();
    bool ok = g_config.writeDNS1(value);
    writeOk = ok && writeOk;
  }
  if (doc.containsKey("dns2")) {
    String value = doc["dns2"].as<String>();
    bool ok = g_config.writeDNS2(value);
    writeOk = ok && writeOk;
  }
  if (doc.containsKey("start_value")) {
    bool ok = g_config.writeStartValue(startValue);
    writeOk = ok && writeOk;
  }
  if (doc.containsKey("channel")) {
    int channel = doc["channel"].as<int>();
    if (channel >= 1 && channel <= 512) {
      bool ok = g_config.writeDMXAddress(channel);
      writeOk = ok && writeOk;
    }
  } else if (doc.containsKey("dmx_address")) {
    int channel = doc["dmx_address"].as<int>();
    if (channel >= 1 && channel <= 512) {
      bool ok = g_config.writeDMXAddress(channel);
      writeOk = ok && writeOk;
    }
  }
  if (doc.containsKey("universe")) {
    int universe = doc["universe"].as<int>();
    if (universe >= 0 && universe <= 32767) {
      bool ok = g_config.writeDMXUniverse(universe);
      writeOk = ok && writeOk;
    }
  } else if (doc.containsKey("dmx_universe")) {
    int universe = doc["dmx_universe"].as<int>();
    if (universe >= 0 && universe <= 32767) {
      bool ok = g_config.writeDMXUniverse(universe);
      writeOk = ok && writeOk;
    }
  }

  if (!writeOk) {
    AsyncWebServerResponse* response = request->beginResponse(500, "text/plain", "Failed to persist settings");
    asyncAddCommonHeaders(response);
    request->send(response);
    noteWebTaskWorkUs((millis() - startMs) * 1000UL);
    return;
  }
  g_wifiManager.scheduleRestart(1000);
  AsyncWebServerResponse* response = request->beginResponse(204);
  asyncAddCommonHeaders(response);
  request->send(response);
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void asyncUpdateInfoHandler(AsyncWebServerRequest* request) {
  uint32_t startMs = millis();
  StaticJsonDocument<128> doc;
  doc["version"] = "1.0.0";
  doc["device"] = g_hooks.deviceName;
  doc["ota_ready"] = true;
  String out;
  serializeJson(doc, out);
  AsyncWebServerResponse* response = request->beginResponse(200, "application/json", out);
  asyncAddCommonHeaders(response, true);
  request->send(response);
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void asyncHandleUpdateBody(AsyncWebServerRequest* request, int updateCmd, uint8_t* data, size_t len, size_t index, size_t total) {
  if (!index) {
    g_asyncUpdateOk = false;
    g_wifiManager.setOtaInProgress(true);
    bool beginOk = false;
    if (updateCmd == U_SPIFFS) {
      const esp_partition_t* fsPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "littlefs");
      beginOk = (fsPart != nullptr) && Update.begin(fsPart->size, U_SPIFFS);
    } else {
      beginOk = Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH);
    }
    if (!beginOk) {
      Update.printError(Serial);
      return;
    }
  }
  if (Update.write(data, len) != len) {
    Update.printError(Serial);
    return;
  }
  if (index + len == total) {
    g_asyncUpdateOk = Update.end(true);
    if (!g_asyncUpdateOk) Update.printError(Serial);
  }
  (void)request;
}

static void asyncHandleUpdateRequest(AsyncWebServerRequest* request, int updateCmd) {
  uint32_t startMs = millis();
  g_wifiManager.setOtaInProgress(false);
  AsyncWebServerResponse* response;
  if (g_asyncUpdateOk) {
    response = request->beginResponse(200, "application/json", "{\"success\":true,\"message\":\"Update successful. Restarting...\"}");
    g_wifiManager.scheduleRestart(1000);
  } else {
    char out[256];
    buildUpdateErrorJson(out, sizeof(out), updateCmd == U_SPIFFS ? "Filesystem update failed" : "Firmware update failed");
    response = request->beginResponse(400, "application/json", out);
  }
  asyncAddCommonHeaders(response, true);
  request->send(response);
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
  (void)updateCmd;
}

static void asyncNotFoundHandler(AsyncWebServerRequest* request) {
  uint32_t startMs = millis();
  AsyncWebServerResponse* response = request->beginResponse(404, "text/plain", "Not Found");
  asyncAddCommonHeaders(response);
  request->send(response);
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static String buildAsyncStatusPayload() {
  char out[STATUS_JSON_BUFFER_SIZE];
  size_t outLen = 0;
  portENTER_CRITICAL(&g_statusMux);
  outLen = g_statusJsonCacheLen;
  if (outLen > sizeof(out)) outLen = sizeof(out);
  if (outLen > 0) memcpy(out, g_statusJsonCache, outLen);
  portEXIT_CRITICAL(&g_statusMux);

  if (outLen == 0 && g_hooks.buildStatusJson) {
    outLen = g_hooks.buildStatusJson(out, sizeof(out), false);
    if (outLen > 0 && outLen <= sizeof(g_statusJsonCache)) {
      portENTER_CRITICAL(&g_statusMux);
      memcpy(g_statusJsonCache, out, outLen);
      g_statusJsonCacheLen = outLen;
      g_statusJsonCacheBuiltMs = millis();
      portEXIT_CRITICAL(&g_statusMux);
    }
  }

  return String(out).substring(0, outLen);
}

static void asyncBroadcastStatusIfDue(uint32_t nowMs, uint32_t wsStatusPushMs) {
  if (!WEB_SOCKET_ENABLE || wsStatusPushMs == 0) return;
  if (g_statusWs.count() == 0) return;
  if ((nowMs - g_lastWsPushMs) < wsStatusPushMs) return;

  String payload = buildAsyncStatusPayload();
  if (payload.length() == 0) return;

  uint32_t startMs = millis();
  g_statusWs.textAll(payload);
  g_statusWs.cleanupClients();
  g_lastWsPushMs = nowMs;
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void onAsyncWsEvent(AsyncWebSocket* server,
                           AsyncWebSocketClient* client,
                           AwsEventType type,
                           void* arg,
                           uint8_t* data,
                           size_t len) {
  (void)server;
  (void)arg;
  uint32_t startMs = millis();
  switch (type) {
    case WS_EVT_CONNECT: {
      String payload = buildAsyncStatusPayload();
      if (payload.length() > 0 && client) {
        client->text(payload);
      }
      break;
    }
    case WS_EVT_DATA: {
      if (client && len > 0) {
        String msg;
        msg.reserve(len);
        for (size_t i = 0; i < len; i++) msg += (char)data[i];
        if (msg == "status" || msg == "ping") {
          String payload = buildAsyncStatusPayload();
          if (payload.length() > 0) {
            client->text(payload);
          }
        }
      }
      break;
    }
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
    case WS_EVT_DISCONNECT:
    default:
      break;
  }
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void webSetCommonHeaders(bool json = false) {
  g_webServer.sendHeader("Access-Control-Allow-Origin", "*");
  g_webServer.sendHeader("Access-Control-Allow-Methods", "GET,PUT,POST,OPTIONS");
  g_webServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  g_webServer.sendHeader("Cache-Control", "no-store");
  g_webServer.sendHeader("Connection", "close");
  g_webServer.client().setNoDelay(true);
  if (json) {
    g_webServer.sendHeader("Content-Type", "application/json");
  }
}

static bool webRequestAcceptsGzip() {
  if (!g_webServer.hasHeader("Accept-Encoding")) return false;
  return g_webServer.header("Accept-Encoding").indexOf("gzip") >= 0;
}

static void webSendOptions() {
  uint32_t startMs = millis();
  webSetCommonHeaders();
  g_webServer.send(204, "text/plain", "");
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void webSendFsFile(const char* path, const char* contentType) {
  uint32_t startMs = millis();
  portENTER_CRITICAL(&g_fileSendMux);
  g_activeFileSends++;
  portEXIT_CRITICAL(&g_fileSendMux);

  String resolvedPath = path;
  bool servingGzip = false;
  if (webRequestAcceptsGzip()) {
    String gzPath = resolvedPath + ".gz";
    File gzFile = LittleFS.open(gzPath, FILE_READ);
    if (gzFile && !gzFile.isDirectory()) {
      resolvedPath = gzPath;
      servingGzip = true;
      gzFile.close();
    }
  }

  File file = LittleFS.open(resolvedPath.c_str(), FILE_READ);
  if (!file || file.isDirectory()) {
    webSetCommonHeaders();
    g_webServer.send(404, "text/plain", "Not Found");
    portENTER_CRITICAL(&g_fileSendMux);
    g_activeFileSends--;
    portEXIT_CRITICAL(&g_fileSendMux);
    return;
  }

  const size_t size = file.size();
  webSetCommonHeaders();
  if (servingGzip) {
    g_webServer.sendHeader("Content-Encoding", "gzip");
    g_webServer.sendHeader("Vary", "Accept-Encoding");
  }

  uint32_t readMs = 0;
  uint32_t sendMs = 0;
  size_t sent = 0;
  uint32_t t0 = millis();
  sent = g_webServer.streamFile(file, contentType);
  sendMs = millis() - t0;

  const uint32_t totalMs = millis() - startMs;
  if (HTTP_TIMING_LOG_ENABLE) {
    Serial.printf("[HTIM] fs-web GET %s status=%d total=%lu prep=%lu send=%lu ret=%d\n",
                  path,
                  sent == size ? 200 : 500,
                  (unsigned long)totalMs,
                  (unsigned long)readMs,
                  (unsigned long)sendMs,
                  sent == size ? 0 : -1);
  }
  if (WEB_DEBUG_LOG || sendMs >= SLOW_FS_SEND_MS || totalMs >= SLOW_FS_TOTAL_MS) {
    Serial.printf("[FS] %s%s read=%lu ms send=%lu ms total=%lu ms size=%lu\n",
                  path,
                  servingGzip ? " (gzip)" : "",
                  (unsigned long)readMs,
                  (unsigned long)sendMs,
                  (unsigned long)totalMs,
                  (unsigned long)size);
  }
  noteWebTaskWorkUs(totalMs * 1000UL);

  file.close();
  portENTER_CRITICAL(&g_fileSendMux);
  g_activeFileSends--;
  portEXIT_CRITICAL(&g_fileSendMux);
}

static void webStatusHandler() {
  uint32_t startMs = millis();
  bool details = false;
  if (g_webServer.hasArg("details")) {
    String value = g_webServer.arg("details");
    details = (value == "1" || value == "true");
  }

  char out[STATUS_JSON_BUFFER_SIZE];
  size_t outLen = 0;
  uint32_t prepStartMs = millis();
  if (!details) {
    portENTER_CRITICAL(&g_statusMux);
    outLen = g_statusJsonCacheLen;
    if (outLen > sizeof(out)) outLen = sizeof(out);
    if (outLen > 0) memcpy(out, g_statusJsonCache, outLen);
    portEXIT_CRITICAL(&g_statusMux);
  }
  if (outLen == 0 && g_hooks.buildStatusJson) {
    outLen = g_hooks.buildStatusJson(out, sizeof(out), details);
    if (!details && outLen > 0 && outLen <= sizeof(g_statusJsonCache)) {
      portENTER_CRITICAL(&g_statusMux);
      memcpy(g_statusJsonCache, out, outLen);
      g_statusJsonCacheLen = outLen;
      g_statusJsonCacheBuiltMs = millis();
      portEXIT_CRITICAL(&g_statusMux);
    }
  }
  uint32_t prepMs = millis() - prepStartMs;
  webSetCommonHeaders(true);
  uint32_t sendStartMs = millis();
  g_webServer.send(200, "application/json", String(out).substring(0, outLen));
  uint32_t sendMs = millis() - sendStartMs;
  uint32_t totalMs = millis() - startMs;
  if (HTTP_TIMING_LOG_ENABLE) {
    Serial.printf("[HTIM] status-web GET /status status=200 total=%lu prep=%lu send=%lu ret=0\n",
                  (unsigned long)totalMs,
                  (unsigned long)prepMs,
                  (unsigned long)sendMs);
  }
  noteWebTaskWorkUs(totalMs * 1000UL);
}

static void webNetworksHandler() {
  uint32_t startMs = millis();
  bool details = false;
  bool refresh = false;
  bool refreshProvided = false;
  bool scanProvided = false;
  if (g_webServer.hasArg("details")) {
    String value = g_webServer.arg("details");
    details = (value == "1" || value == "true");
  }
  if (g_webServer.hasArg("refresh")) {
    refreshProvided = true;
    String value = g_webServer.arg("refresh");
    refresh = (value == "1" || value == "true");
  }
  if (g_webServer.hasArg("scan")) {
    scanProvided = true;
    String value = g_webServer.arg("scan");
    refresh = (value == "1" || value == "true");
  }
  if (details && !refreshProvided && !scanProvided) refresh = true;
  String payload = g_wifiManager.getNetworksPayload(details, refresh);
  webSetCommonHeaders(true);
  g_webServer.send(200, "application/json", payload);
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void webCredentialsGetHandler() {
  uint32_t startMs = millis();
  DynamicJsonDocument doc(1024);
  int channel = g_config.getDMXAddress();
  int universe = g_config.getDMXUniverse();
  doc["ssid"] = g_config.getSSID();
  doc["hostname"] = g_config.getHostname();
  doc["password"] = g_config.getPass();
  doc["dhcp"] = g_config.getDhcpEnabled();
  doc["ip"] = g_config.getStaticIP();
  doc["gateway"] = g_config.getGateway();
  doc["subnet"] = g_config.getSubnet();
  doc["dns1"] = g_config.getDNS1();
  doc["dns2"] = g_config.getDNS2();
  doc["start_value"] = g_config.getStartValue();
  doc["channel"] = channel;
  doc["universe"] = universe;
  doc["dmx_address"] = channel;
  doc["dmx_universe"] = universe;
  String out;
  serializeJson(doc, out);
  webSetCommonHeaders(true);
  g_webServer.send(200, "application/json", out);
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void webCredentialsPutHandler() {
  uint32_t startMs = millis();
  String body = g_webServer.arg("plain");
  if (body.length() <= 0 || body.length() > 1024) {
    webSetCommonHeaders();
    g_webServer.send(400, "text/plain", "");
    noteWebTaskWorkUs((millis() - startMs) * 1000UL);
    return;
  }

  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    webSetCommonHeaders();
    g_webServer.send(400, "text/plain", "");
    noteWebTaskWorkUs((millis() - startMs) * 1000UL);
    return;
  }
  auto invalidTextField = [](const char* value, bool allowEmpty, size_t maxLen) {
    if (!value) return !allowEmpty;
    size_t len = strlen(value);
    if (len == 0) return !allowEmpty;
    if (len > maxLen) return true;
    for (size_t i = 0; i < len; i++) {
      unsigned char ch = (unsigned char)value[i];
      if (ch < 32 || ch == 127) return true;
    }
    return false;
  };

  String ssidText = doc.containsKey("ssid") ? doc["ssid"].as<String>() : String();
  String hostnameText = doc.containsKey("hostname") ? doc["hostname"].as<String>() : String();
  String passwordText = doc.containsKey("password") ? doc["password"].as<String>() : String();
  const char* ssidValue = doc.containsKey("ssid") ? ssidText.c_str() : nullptr;
  const char* hostnameValue = doc.containsKey("hostname") ? hostnameText.c_str() : nullptr;
  const char* passwordValue = doc.containsKey("password") ? passwordText.c_str() : nullptr;
  bool dhcpValue = doc.containsKey("dhcp") ? doc["dhcp"].as<bool>() : g_config.getDhcpEnabled();
  float startValue = doc.containsKey("start_value") ? doc["start_value"].as<float>() : g_config.getStartValue();

  if (ssidValue && invalidTextField(ssidValue, false, 32)) {
    webSetCommonHeaders();
    g_webServer.send(400, "text/plain", "Invalid SSID");
    noteWebTaskWorkUs((millis() - startMs) * 1000UL);
    return;
  }
  if (hostnameValue && invalidTextField(hostnameValue, true, 63)) {
    webSetCommonHeaders();
    g_webServer.send(400, "text/plain", "Invalid hostname");
    noteWebTaskWorkUs((millis() - startMs) * 1000UL);
    return;
  }

  bool writeOk = true;
  if (doc.containsKey("ssid")) writeOk = g_config.writeSSID(ssidText) && writeOk;
  if (doc.containsKey("password")) writeOk = g_config.writePass(passwordText) && writeOk;
  if (doc.containsKey("hostname")) writeOk = g_config.writeHostname(hostnameText) && writeOk;
  if (doc.containsKey("dhcp")) writeOk = g_config.writeDhcpEnabled(dhcpValue) && writeOk;
  if (doc.containsKey("ip")) writeOk = g_config.writeStaticIP(doc["ip"].as<String>()) && writeOk;
  if (doc.containsKey("gateway")) writeOk = g_config.writeGateway(doc["gateway"].as<String>()) && writeOk;
  if (doc.containsKey("subnet")) writeOk = g_config.writeSubnet(doc["subnet"].as<String>()) && writeOk;
  if (doc.containsKey("dns1")) writeOk = g_config.writeDNS1(doc["dns1"].as<String>()) && writeOk;
  if (doc.containsKey("dns2")) writeOk = g_config.writeDNS2(doc["dns2"].as<String>()) && writeOk;
  if (doc.containsKey("start_value")) writeOk = g_config.writeStartValue(startValue) && writeOk;
  if (doc.containsKey("channel")) {
    int channel = doc["channel"].as<int>();
    if (channel >= 1 && channel <= 512) writeOk = g_config.writeDMXAddress(channel) && writeOk;
  } else if (doc.containsKey("dmx_address")) {
    int channel = doc["dmx_address"].as<int>();
    if (channel >= 1 && channel <= 512) writeOk = g_config.writeDMXAddress(channel) && writeOk;
  }
  if (doc.containsKey("universe")) {
    int universe = doc["universe"].as<int>();
    if (universe >= 0 && universe <= 32767) writeOk = g_config.writeDMXUniverse(universe) && writeOk;
  } else if (doc.containsKey("dmx_universe")) {
    int universe = doc["dmx_universe"].as<int>();
    if (universe >= 0 && universe <= 32767) writeOk = g_config.writeDMXUniverse(universe) && writeOk;
  }

  if (!writeOk) {
    webSetCommonHeaders();
    g_webServer.send(500, "text/plain", "Failed to persist settings");
    noteWebTaskWorkUs((millis() - startMs) * 1000UL);
    return;
  }

  g_wifiManager.scheduleRestart(1000);
  webSetCommonHeaders();
  g_webServer.send(204, "text/plain", "");
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void webUpdateInfoHandler() {
  uint32_t startMs = millis();
  StaticJsonDocument<128> doc;
  doc["version"] = "1.0.0";
  doc["device"] = g_hooks.deviceName;
  doc["ota_ready"] = true;
  String out;
  serializeJson(doc, out);
  webSetCommonHeaders(true);
  g_webServer.send(200, "application/json", out);
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void webHandleUpdatePost(int updateCmd) {
  uint32_t startMs = millis();
  bool ok = !Update.hasError();
  g_wifiManager.setOtaInProgress(false);
  webSetCommonHeaders(true);
  if (ok) {
    g_webServer.send(200, "application/json", "{\"success\":true,\"message\":\"Update successful. Restarting...\"}");
    g_wifiManager.scheduleRestart(1000);
  } else {
    g_webServer.send(400, "application/json", "{\"success\":false,\"message\":\"Update failed\"}");
  }
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
  (void)updateCmd;
}

static void webHandleUpdateUpload(int updateCmd) {
  HTTPUpload& upload = g_webServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    g_wifiManager.setOtaInProgress(true);
    bool beginOk = false;
    if (updateCmd == U_SPIFFS) {
      const esp_partition_t* fsPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "littlefs");
      beginOk = (fsPart != nullptr) && Update.begin(fsPart->size, U_SPIFFS);
    } else {
      beginOk = Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH);
    }
    if (!beginOk) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    g_wifiManager.setOtaInProgress(false);
  }
}

static void webNotFoundHandler() {
  uint32_t startMs = millis();
  webSetCommonHeaders();
  g_webServer.send(404, "text/plain", "Not Found");
  noteWebTaskWorkUs((millis() - startMs) * 1000UL);
}

static void registerUri(const char* path, httpd_method_t method, esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t uri = {};
  uri.uri = path;
  uri.method = method;
  uri.handler = handler;
  uri.user_ctx = nullptr;
  httpd_register_uri_handler(g_webHttpd, &uri);
}

static void registerWsUri(const char* path, esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t uri = {};
  uri.uri = path;
  uri.method = HTTP_GET;
  uri.handler = handler;
  uri.user_ctx = nullptr;
  uri.is_websocket = true;
  httpd_register_uri_handler(g_webHttpd, &uri);
}

static void startWebServer() {
  if (g_webServerStarted) return;
  Serial.printf("[HTTPCFG] backend=async-webserver timing=%d ws=%d prio=async-tcp core=%d sockets=na backlog=na\n",
                HTTP_TIMING_LOG_ENABLE,
                WEB_SOCKET_ENABLE,
                CONFIG_ASYNC_TCP_RUNNING_CORE);

  g_asyncWebServer.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    asyncSendFsFile(request, g_hooks.indexPagePath, "text/html");
  });
  g_asyncWebServer.on("/index.html", HTTP_GET, [](AsyncWebServerRequest* request) {
    asyncSendFsFile(request, g_hooks.indexPagePath, "text/html");
  });
  g_asyncWebServer.on("/ota.html", HTTP_GET, [](AsyncWebServerRequest* request) {
    asyncSendFsFile(request, "/wifi-manager/ota.html", "text/html");
  });
  g_asyncWebServer.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest* request) {
    asyncSendFsFile(request, "/wifi-manager/settings.html", "text/html");
  });
  g_asyncWebServer.on("/status", HTTP_GET, asyncStatusHandler);
  g_asyncWebServer.on("/networks", HTTP_GET, asyncNetworksHandler);
  g_asyncWebServer.on("/credentials", HTTP_GET, asyncCredentialsGetHandler);
  g_asyncWebServer.on(
      "/credentials",
      HTTP_PUT,
      asyncCredentialsPutHandler,
      nullptr,
      asyncCredentialsPutBody);
  g_asyncWebServer.on("/update", HTTP_GET, asyncUpdateInfoHandler);
  g_asyncWebServer.on(
      "/update",
      HTTP_POST,
      [](AsyncWebServerRequest* request) { asyncHandleUpdateRequest(request, U_FLASH); },
      nullptr,
      [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        asyncHandleUpdateBody(request, U_FLASH, data, len, index, total);
      });
  g_asyncWebServer.on(
      "/updatefs",
      HTTP_POST,
      [](AsyncWebServerRequest* request) { asyncHandleUpdateRequest(request, U_SPIFFS); },
      nullptr,
      [](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
        asyncHandleUpdateBody(request, U_SPIFFS, data, len, index, total);
      });
  g_asyncWebServer.on("/status", HTTP_OPTIONS, asyncSendOptions);
  g_asyncWebServer.on("/networks", HTTP_OPTIONS, asyncSendOptions);
  g_asyncWebServer.on("/credentials", HTTP_OPTIONS, asyncSendOptions);
  g_asyncWebServer.on("/update", HTTP_OPTIONS, asyncSendOptions);
  g_asyncWebServer.on("/updatefs", HTTP_OPTIONS, asyncSendOptions);
  g_asyncWebServer.onNotFound(asyncNotFoundHandler);
  if (WEB_SOCKET_ENABLE) {
    g_statusWs.onEvent(onAsyncWsEvent);
    g_asyncWebServer.addHandler(&g_statusWs);
  }
  g_asyncWebServer.begin();
  g_webServerStarted = true;
  g_webTaskHandle = nullptr;
  Serial.println("ESPAsyncWebServer started on :80");
}

static void logStoredConfig() {
#if CONFIG_DEBUG_LOG_ENABLE
  String ssid = g_config.getSSID();
  String hostname = g_config.getHostname();
  String ip = g_config.getStaticIP();
  String gateway = g_config.getGateway();
  String subnet = g_config.getSubnet();
  String dns1 = g_config.getDNS1();
  String dns2 = g_config.getDNS2();
  bool dhcp = g_config.getDhcpEnabled();
  int channel = g_config.getDMXAddress();
  int universe = g_config.getDMXUniverse();
  float startValue = g_config.getStartValue();
  size_t passLen = g_config.getPass().length();

  Serial.printf("[CFG] ssid=\"%s\" hostname=\"%s\" pass_len=%u dhcp=%d ip=\"%s\" gw=\"%s\" subnet=\"%s\" dns1=\"%s\" dns2=\"%s\" channel=%d universe=%d start=%.6g\n",
                ssid.c_str(),
                hostname.c_str(),
                (unsigned int)passLen,
                dhcp ? 1 : 0,
                ip.c_str(),
                gateway.c_str(),
                subnet.c_str(),
                dns1.c_str(),
                dns2.c_str(),
                channel,
                universe,
                (double)startValue);
#endif
}

TaskHandle_t appGetWebServerTaskHandle() {
  return g_webTaskHandle;
}

AppTaskRuntimeStats appGetWebTaskRuntimeStats() {
  AppTaskRuntimeStats stats;
  portENTER_CRITICAL(&g_statusMux);
  stats.lastActiveMs = g_webTaskLastActiveMs;
  stats.utilPermille = g_webTaskUtilPermille;
  portEXIT_CRITICAL(&g_statusMux);
  return stats;
}


void appInitializeBaseRuntime() {
  Serial.begin(115200);

#if defined(ARDUINO_USB_CDC_ON_BOOT) && (ARDUINO_USB_CDC_ON_BOOT == 1)
  uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 1500) {
    delay(10);
  }
#endif

  Serial.println("Startup...");

  if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) {
    Serial.println("LittleFS mount failed");
  }
}

void appInitRuntime(const AppRuntimeHooks& hooks) {
  g_hooks = hooks;
}

void appStartCommonServices() {
  startWebServer();
  WiFi.setSleep(false);

#if FS_BENCHMARK_LOG_ENABLE
  logFileReadPerf("/wifi-manager/index.html");
  logFileReadPerf("/wifi-manager/index_led.html");
  logFileReadPerf("/wifi-manager/settings.html");
  logFileReadPerf("/wifi-manager/ota.html");
#endif
}

void appConnectWifi() {
  logStoredConfig();
  bool connected = g_wifiManager.connectToWifi();
  if (!connected) {
    Serial.println("No WiFi... Starting AP");
    g_wifiManager.startManagementAP();
  }

  IPAddress ip = g_wifiManager.getIP();
  Serial.printf("IP Address: %s\n", ip.toString().c_str());
}

void appCommonLoop(uint32_t wsStatusPushMs) {
  if (g_hooks.pollInputs) {
    g_hooks.pollInputs();
  }

  g_wifiManager.check();

  uint32_t now = millis();
  refreshStatusCacheIfDue(now);
  asyncBroadcastStatusIfDue(now, wsStatusPushMs);
#if HEALTH_LOG_ENABLE
  if (now - g_lastHealthLogMs >= HEALTH_LOG_INTERVAL_MS) {
    wl_status_t wifiStatus = WiFi.status();
    IPAddress ip = g_wifiManager.getIP();
    char artSummary[256] = "";
    if (g_hooks.buildHealthSummary) {
      g_hooks.buildHealthSummary(artSummary, sizeof(artSummary));
    }

    if (artSummary[0] != '\0') {
      Serial.printf("[HEALTH] up=%lu ms wifi=%d ip=%s rssi=%d heap=%lu rec_attempts=%lu rec_ok=%lu %s\n",
                    (unsigned long)now,
                    (int)wifiStatus,
                    ip.toString().c_str(),
                    (int)g_wifiManager.getRSSI(),
                    (unsigned long)ESP.getFreeHeap(),
                    (unsigned long)g_wifiManager.getReconnectAttempts(),
                    (unsigned long)g_wifiManager.getReconnectSuccesses(),
                    artSummary);
    } else {
      Serial.printf("[HEALTH] up=%lu ms wifi=%d ip=%s rssi=%d heap=%lu rec_attempts=%lu rec_ok=%lu\n",
                    (unsigned long)now,
                    (int)wifiStatus,
                    ip.toString().c_str(),
                    (int)g_wifiManager.getRSSI(),
                    (unsigned long)ESP.getFreeHeap(),
                    (unsigned long)g_wifiManager.getReconnectAttempts(),
                    (unsigned long)g_wifiManager.getReconnectSuccesses());
    }
    g_lastHealthLogMs = now;
  }
#else
  (void)g_lastHealthLogMs;
#endif

  (void)wsStatusPushMs;
}
