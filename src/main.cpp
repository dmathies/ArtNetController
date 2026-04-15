#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_http_server.h>
#include <LittleFS.h>
#include <Update.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <ArduinoJson.h>
#include <cstring>

#include "main_common.h"

static constexpr uint16_t ARTNET_PORT = 6454;
static constexpr int MAX_WS_CLIENTS = 6;
static constexpr uint32_t SLOW_HTTP_MS = 100;
static constexpr uint32_t SLOW_WS_BROADCAST_MS = 50;
static constexpr uint32_t SLOW_FS_SEND_MS = 500;
static constexpr uint32_t SLOW_FS_TOTAL_MS = 1000;
static constexpr size_t HTTP_FILE_CHUNK_SIZE = 1024;
static constexpr size_t HTTP_FILE_DIRECT_SEND_LIMIT = 32768;
static constexpr uint32_t HEALTH_LOG_INTERVAL_MS = 5000;

#ifndef WEB_DEBUG_LOG
#define WEB_DEBUG_LOG 0
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
static WiFiUDP g_artudp;

static portMUX_TYPE g_statusMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t g_lastArtMs = 0;

static portMUX_TYPE g_wsClientMux = portMUX_INITIALIZER_UNLOCKED;
static int g_wsClientFds[MAX_WS_CLIENTS] = {-1, -1, -1, -1, -1, -1};
static uint32_t g_lastWsPushMs = 0;
static uint32_t g_lastHealthLogMs = 0;
static portMUX_TYPE g_fileSendMux = portMUX_INITIALIZER_UNLOCKED;
static int g_activeFileSends = 0;

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
WiFiUDP& appArtnetUdp() { return g_artudp; }

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
#if !WEB_DEBUG_LOG
  (void)req;
  (void)startMs;
  (void)statusCode;
  (void)detail;
  return;
#else
  uint32_t durationMs = millis() - startMs;
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

static esp_err_t optionsHandler(httpd_req_t* req) {
  uint32_t startMs = millis();
  setCorsHeaders(req);
  httpd_resp_set_status(req, "204 No Content");
  esp_err_t ret = httpd_resp_send(req, nullptr, 0);
  logHttpRequest(req, startMs, ret == ESP_OK ? 204 : 500);
  return ret;
}

static esp_err_t sendFsFile(httpd_req_t* req, const char* path, const char* contentType) {
  uint32_t startMs = millis();

  portENTER_CRITICAL(&g_fileSendMux);
  g_activeFileSends++;
  portEXIT_CRITICAL(&g_fileSendMux);

  File file = LittleFS.open(path, FILE_READ);
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
    Serial.printf("[FS] %s read=%lu ms send=%lu ms total=%lu ms size=%lu\n",
                  path,
                  (unsigned long)readMs,
                  (unsigned long)sendMs,
                  (unsigned long)totalMs,
                  (unsigned long)size);
  }

  char detail[128];
  snprintf(detail,
           sizeof(detail),
           "%s, %lu/%lu B, heap=%lu, err=%d",
           path,
           (unsigned long)totalSent,
           (unsigned long)size,
           (unsigned long)ESP.getFreeHeap(),
           (int)ret);
  file.close();
  free(buf);
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

  char* out = (char*)malloc(2048);
  if (!out) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    esp_err_t ret = httpd_resp_send(req, "OOM", HTTPD_RESP_USE_STRLEN);
    logHttpRequest(req, startMs, 500, "status buffer alloc failed");
    return ret;
  }

  size_t outLen = g_hooks.buildStatusJson(out, 2048, details);
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t ret = httpd_resp_send(req, out, outLen);
  free(out);
  char detail[48];
  snprintf(detail, sizeof(detail), "details=%d, %lu B", details ? 1 : 0, (unsigned long)outLen);
  logHttpRequest(req, startMs, ret == ESP_OK ? 200 : 500, detail);
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

static esp_err_t wsSendStatusToClient(int fd) {
  if (!g_hooks.buildStatusJson) return ESP_FAIL;
  char out[512];
  size_t outLen = g_hooks.buildStatusJson(out, sizeof(out), false);
  httpd_ws_frame_t frame = {};
  frame.type = HTTPD_WS_TYPE_TEXT;
  frame.payload = (uint8_t*)out;
  frame.len = outLen;
  return httpd_ws_send_frame_async(g_webHttpd, fd, &frame);
}

static void wsBroadcastStatus() {
  int activeSends;
  portENTER_CRITICAL(&g_fileSendMux);
  activeSends = g_activeFileSends;
  portEXIT_CRITICAL(&g_fileSendMux);

  if (activeSends > 0) {
    return;
  }

  uint32_t startMs = millis();
  int clients[MAX_WS_CLIENTS];
  int count = wsCopyClients(clients, MAX_WS_CLIENTS);
  int failed = 0;
  for (int i = 0; i < count; i++) {
    if (wsSendStatusToClient(clients[i]) != ESP_OK) {
      wsRemoveClient(clients[i]);
      failed++;
    }
  }

  uint32_t durationMs = millis() - startMs;
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
  bool legacyDetailsOnly = false;
  size_t qlen = httpd_req_get_url_query_len(req);
  if (qlen > 0 && qlen < 64) {
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
      char value[8];
      if (httpd_query_key_value(query, "details", value, sizeof(value)) == ESP_OK) details = true;
      if (httpd_query_key_value(query, "refresh", value, sizeof(value)) == ESP_OK) {
        refresh = (strcmp(value, "1") == 0 || strcmp(value, "true") == 0);
      }
      if (httpd_query_key_value(query, "scan", value, sizeof(value)) == ESP_OK) {
        refresh = (strcmp(value, "1") == 0 || strcmp(value, "true") == 0);
      }
      legacyDetailsOnly = details && !refresh;
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
  StaticJsonDocument<256> doc;
  int channel = g_config.getDMXAddress();
  int universe = g_config.getDMXUniverse();
  doc["ssid"] = g_config.getSSID();
  doc["hostname"] = g_config.getHostname();
  doc["password"] = g_config.getPass();
  doc["channel"] = channel;
  doc["universe"] = universe;
  doc["dmx_address"] = channel;
  doc["dmx_universe"] = universe;

  char out[256];
  size_t outLen = serializeJson(doc, out, sizeof(out));
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t ret = httpd_resp_send(req, out, outLen);
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

  String body;
  body.reserve(len + 1);
  int remaining = len;
  char buf[256];
  while (remaining > 0) {
    int toRead = remaining > (int)sizeof(buf) ? (int)sizeof(buf) : remaining;
    int read = httpd_req_recv(req, buf, toRead);
    if (read <= 0) {
      setCorsHeaders(req);
      httpd_resp_set_status(req, "500 Internal Server Error");
      esp_err_t ret = httpd_resp_send(req, "", 0);
      logHttpRequest(req, startMs, 500, "request body read failed");
      return ret;
    }

    char tmp[257];
    memcpy(tmp, buf, read);
    tmp[read] = '\0';
    body += tmp;
    remaining -= read;
  }

  StaticJsonDocument<384> doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    setCorsHeaders(req);
    httpd_resp_set_status(req, "400 Bad Request");
    esp_err_t ret = httpd_resp_send(req, "", 0);
    logHttpRequest(req, startMs, 400, "invalid json");
    return ret;
  }

  if (doc.containsKey("ssid")) g_config.writeSSID(doc["ssid"].as<const char*>());
  if (doc.containsKey("password")) g_config.writePass(doc["password"].as<const char*>());
  if (doc.containsKey("hostname")) g_config.writeHostname(doc["hostname"].as<const char*>());

  if (doc.containsKey("channel")) {
    int channel = doc["channel"].as<int>();
    if (channel >= 1 && channel <= 512) g_config.writeDMXAddress(channel);
  } else if (doc.containsKey("dmx_address")) {
    int channel = doc["dmx_address"].as<int>();
    if (channel >= 1 && channel <= 512) g_config.writeDMXAddress(channel);
  }

  if (doc.containsKey("universe")) {
    int universe = doc["universe"].as<int>();
    if (universe >= 0 && universe <= 32767) g_config.writeDMXUniverse(universe);
  } else if (doc.containsKey("dmx_universe")) {
    int universe = doc["dmx_universe"].as<int>();
    if (universe >= 0 && universe <= 32767) g_config.writeDMXUniverse(universe);
  }

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
    esp_err_t ret = httpd_resp_send(req, "{\"success\":false,\"message\":\"Update begin failed\"}", HTTPD_RESP_USE_STRLEN);
    logHttpRequest(req, startMs, 400, updateCmd == U_SPIFFS ? "update begin failed: fs" : "update begin failed: flash");
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
    esp_err_t ret = httpd_resp_send(req, "{\"success\":false,\"message\":\"Update finalize failed\"}", HTTPD_RESP_USE_STRLEN);
    logHttpRequest(req, startMs, 400, "update finalize failed");
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
  if (g_webHttpd != nullptr) return;

  httpd_config_t configHttp = HTTPD_DEFAULT_CONFIG();
  configHttp.server_port = 80;
  configHttp.ctrl_port = 32768;
  configHttp.max_uri_handlers = 24;

  if (httpd_start(&g_webHttpd, &configHttp) == ESP_OK) {
    registerUri("/", HTTP_GET, indexHandler);
    registerUri("/index.html", HTTP_GET, indexHandler);
    registerUri("/ota.html", HTTP_GET, otaPageHandler);
    registerUri("/settings.html", HTTP_GET, settingHandler);
    registerUri("/status", HTTP_GET, statusHandler);
    registerUri("/networks", HTTP_GET, networksHandler);
    registerUri("/credentials", HTTP_GET, credentialsGetHandler);
    registerUri("/credentials", HTTP_PUT, credentialsPutHandler);
    registerUri("/update", HTTP_GET, updateInfoHandler);
    registerUri("/update", HTTP_POST, firmwareUpdateHandler);
    registerUri("/updatefs", HTTP_POST, fsUpdateHandler);
    registerWsUri("/ws", wsHandler);

    registerUri("/status", HTTP_OPTIONS, optionsHandler);
    registerUri("/networks", HTTP_OPTIONS, optionsHandler);
    registerUri("/credentials", HTTP_OPTIONS, optionsHandler);
    registerUri("/update", HTTP_OPTIONS, optionsHandler);
    registerUri("/updatefs", HTTP_OPTIONS, optionsHandler);
    Serial.println("Native HTTP server started on :80");
  } else {
    Serial.println("Failed to start native HTTP server");
  }
}

void appInitRuntime(const AppRuntimeHooks& hooks) {
  g_hooks = hooks;
}

void appStartCommonServices() {
  startWebServer();
  WiFi.setSleep(false);
  g_artudp.begin(ARTNET_PORT);

  logFileReadPerf("/wifi-manager/index.html");
  logFileReadPerf("/wifi-manager/index_led.html");
  logFileReadPerf("/wifi-manager/settings.html");
  logFileReadPerf("/wifi-manager/ota.html");
}

void appConnectWifi() {
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

  if (now - g_lastWsPushMs >= wsStatusPushMs) {
    wsBroadcastStatus();
    g_lastWsPushMs = now;
  }
}
