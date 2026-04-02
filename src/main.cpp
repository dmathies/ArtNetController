#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_http_server.h>
#include <SPIFFS.h>
#include <Update.h>
#include <esp_partition.h>
#include <ArduinoJson.h>
#include <cstring>
#include "WifiManager.h"
#include "esp_dmx.h"
#include "bldc_uart.h"

Configuration config;
WifiManagerClass WifiManager(config);

static constexpr int CH1 = 1;
static constexpr int CH2 = 2;
static constexpr int CH3 = 3;
static constexpr int CH4 = 21;

const int LEDC_RES_BITS = 12;
const int LEDC_MAX_DUTY = (1 << LEDC_RES_BITS) - 1;
const int LEDC_FREQ_HZ = 1000;
static constexpr uint32_t CONTROL_TASK_DELAY_MS = 2;
static constexpr uint32_t WS_STATUS_PUSH_MS = 250;
static constexpr int MAX_WS_CLIENTS = 6;

float currentValues[4];
static portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t controlTaskHandle = nullptr;
static portMUX_TYPE wsClientMux = portMUX_INITIALIZER_UNLOCKED;
static int wsClientFds[MAX_WS_CLIENTS] = { -1, -1, -1, -1, -1, -1 };
static uint32_t lastWsPushMs = 0;

static constexpr dmx_port_t DMX_PORT = DMX_NUM_1;
static uint8_t dmx_buf[DMX_PACKET_SIZE];

struct AppConfig {
  uint16_t dmx_start_address = config.getDMXAddress();
  uint16_t artnet_universe = config.getDMXUniverse();
} CFG;

WiFiUDP artudp;
static constexpr uint16_t ARTNET_PORT = 6454;

struct ArtDmxPacket {
  bool ok = false;
  uint16_t universe_flat = 0;
  uint16_t length = 0;
  const uint8_t* data = nullptr;
};

static ArtDmxPacket parseArtDmx(const uint8_t* p, int len) {
  ArtDmxPacket r;
  if (len < 18) return r;
  if (memcmp(p, "Art-Net\0", 8) != 0) return r;
  if (!(p[8] == 0x00 && p[9] == 0x50)) return r;
  uint8_t subUni = p[14];
  uint8_t net = p[15];
  uint16_t dlen = ((uint16_t)p[16] << 8) | p[17];
  if (dlen > 512 || 18 + dlen > len) return r;
  uint8_t subnet = (subUni >> 4) & 0x0F;
  uint8_t uni = (subUni) & 0x0F;
  uint16_t flat = ((uint16_t)net << 8) | ((uint16_t)subnet << 4) | uni;
  r.ok = true;
  r.universe_flat = flat;
  r.length = dlen;
  r.data = p + 18;
  return r;
}

enum class Source { None, DMX, Artnet };
static uint32_t lastDmxMs = 0, lastArtMs = 0;
static constexpr uint32_t SOURCE_HOLD_MS = 1000;

struct StatusSnapshot {
  float ledValues[4];
  uint32_t lastDmxMs;
  uint32_t lastArtMs;
  uint32_t artUdpPacketsTotal;
  uint32_t artUdpBytesTotal;
  uint32_t artNetPacketsTotal;
  uint32_t artPollTotal;
  uint32_t artPollReplyTotal;
  uint32_t artSyncTotal;
  uint32_t artOtherOpcodeTotal;
  uint32_t artDmxRxTotal;
  uint32_t artDmxUniverseMatchTotal;
  uint32_t artDmxUniverseMismatchTotal;
  uint32_t artDmxInvalidTotal;
  uint32_t artDmxTruncatedTotal;
  uint32_t artDmxLastArrivalMs;
  uint32_t artDmxLastSecondRate;
  uint32_t artDmxMaxGapMs;
  uint32_t artLastSourceIp;
  uint16_t artLastSourcePort;
  uint32_t artMatchSourceIp;
  uint16_t artMatchSourcePort;
  uint32_t artSourceChangeTotal;
  uint32_t artPacketsLastLoop;
  uint32_t artPacketsMaxLoop;
  uint32_t controlLoopLastUs;
  uint32_t controlLoopMaxUs;
  uint32_t controlLoopOver10ms;
};

static httpd_handle_t webHttpd = nullptr;

static uint32_t artDmxRxTotal = 0;
static uint32_t artDmxUniverseMatchTotal = 0;
static uint32_t artDmxUniverseMismatchTotal = 0;
static uint32_t artDmxInvalidTotal = 0;
static uint32_t artDmxTruncatedTotal = 0;
static uint32_t artDmxLastArrivalMs = 0;
static uint32_t artDmxLastSecondStartMs = 0;
static uint32_t artDmxThisSecond = 0;
static uint32_t artDmxLastSecondRate = 0;
static uint32_t artDmxMaxGapMs = 0;
static uint32_t artUdpPacketsTotal = 0;
static uint32_t artUdpBytesTotal = 0;
static uint32_t artNetPacketsTotal = 0;
static uint32_t artPollTotal = 0;
static uint32_t artPollReplyTotal = 0;
static uint32_t artSyncTotal = 0;
static uint32_t artOtherOpcodeTotal = 0;
static uint32_t artLastSourceIp = 0;
static uint16_t artLastSourcePort = 0;
static uint32_t artMatchSourceIp = 0;
static uint16_t artMatchSourcePort = 0;
static uint32_t artSourceChangeTotal = 0;

static uint32_t artPacketsLastLoop = 0;
static uint32_t artPacketsMaxLoop = 0;
static uint32_t controlLoopLastUs = 0;
static uint32_t controlLoopMaxUs = 0;
static uint32_t controlLoopOver10ms = 0;

static void recordArtUdpPacketMeta(size_t packetLen, uint32_t sourceIp, uint16_t sourcePort) {
  portENTER_CRITICAL(&statusMux);
  artUdpPacketsTotal++;
  artUdpBytesTotal += packetLen;
  if (artLastSourceIp != 0 && (artLastSourceIp != sourceIp || artLastSourcePort != sourcePort)) {
    artSourceChangeTotal++;
  }
  artLastSourceIp = sourceIp;
  artLastSourcePort = sourcePort;
  portEXIT_CRITICAL(&statusMux);
}

static void recordArtNetOpcode(uint16_t opcode) {
  portENTER_CRITICAL(&statusMux);
  artNetPacketsTotal++;
  if (opcode == 0x2000) artPollTotal++;
  else if (opcode == 0x2100) artPollReplyTotal++;
  else if (opcode == 0x5200) artSyncTotal++;
  else if (opcode != 0x5000) artOtherOpcodeTotal++;
  portEXIT_CRITICAL(&statusMux);
}

static void recordControlLoopTiming(uint32_t loopDeltaUs) {
  portENTER_CRITICAL(&statusMux);
  controlLoopLastUs = loopDeltaUs;
  if (loopDeltaUs > controlLoopMaxUs) controlLoopMaxUs = loopDeltaUs;
  if (loopDeltaUs > 10000) controlLoopOver10ms++;
  portEXIT_CRITICAL(&statusMux);
}

static void recordArtLoopDrain(uint32_t packetsDrained) {
  portENTER_CRITICAL(&statusMux);
  artPacketsLastLoop = packetsDrained;
  if (packetsDrained > artPacketsMaxLoop) artPacketsMaxLoop = packetsDrained;
  portEXIT_CRITICAL(&statusMux);
}

static void recordArtDmxFrame(bool matchedUniverse, bool truncated, uint32_t sourceIp, uint16_t sourcePort) {
  uint32_t now = millis();
  portENTER_CRITICAL(&statusMux);
  artDmxRxTotal++;
  if (matchedUniverse) artDmxUniverseMatchTotal++;
  else artDmxUniverseMismatchTotal++;
  if (truncated) artDmxTruncatedTotal++;
  if (matchedUniverse) {
    artMatchSourceIp = sourceIp;
    artMatchSourcePort = sourcePort;
  }

  if (artDmxLastArrivalMs != 0) {
    uint32_t gap = now - artDmxLastArrivalMs;
    if (gap > artDmxMaxGapMs) artDmxMaxGapMs = gap;
  }
  artDmxLastArrivalMs = now;

  if (artDmxLastSecondStartMs == 0) artDmxLastSecondStartMs = now;
  if (now - artDmxLastSecondStartMs >= 1000) {
    artDmxLastSecondRate = artDmxThisSecond;
    artDmxThisSecond = 0;
    artDmxLastSecondStartMs = now;
  }
  artDmxThisSecond++;
  portEXIT_CRITICAL(&statusMux);
}

static void recordArtInvalidFrame(bool truncated) {
  portENTER_CRITICAL(&statusMux);
  artDmxInvalidTotal++;
  if (truncated) artDmxTruncatedTotal++;
  portEXIT_CRITICAL(&statusMux);
}

static StatusSnapshot readStatusSnapshot() {
  StatusSnapshot snapshot;
  portENTER_CRITICAL(&statusMux);
  for (int led = 0; led < 4; led++) snapshot.ledValues[led] = currentValues[led];
  snapshot.lastDmxMs = lastDmxMs;
  snapshot.lastArtMs = lastArtMs;
  snapshot.artUdpPacketsTotal = artUdpPacketsTotal;
  snapshot.artUdpBytesTotal = artUdpBytesTotal;
  snapshot.artNetPacketsTotal = artNetPacketsTotal;
  snapshot.artPollTotal = artPollTotal;
  snapshot.artPollReplyTotal = artPollReplyTotal;
  snapshot.artSyncTotal = artSyncTotal;
  snapshot.artOtherOpcodeTotal = artOtherOpcodeTotal;
  snapshot.artDmxRxTotal = artDmxRxTotal;
  snapshot.artDmxUniverseMatchTotal = artDmxUniverseMatchTotal;
  snapshot.artDmxUniverseMismatchTotal = artDmxUniverseMismatchTotal;
  snapshot.artDmxInvalidTotal = artDmxInvalidTotal;
  snapshot.artDmxTruncatedTotal = artDmxTruncatedTotal;
  snapshot.artDmxLastArrivalMs = artDmxLastArrivalMs;
  snapshot.artDmxLastSecondRate = artDmxLastSecondRate;
  snapshot.artDmxMaxGapMs = artDmxMaxGapMs;
  snapshot.artLastSourceIp = artLastSourceIp;
  snapshot.artLastSourcePort = artLastSourcePort;
  snapshot.artMatchSourceIp = artMatchSourceIp;
  snapshot.artMatchSourcePort = artMatchSourcePort;
  snapshot.artSourceChangeTotal = artSourceChangeTotal;
  snapshot.artPacketsLastLoop = artPacketsLastLoop;
  snapshot.artPacketsMaxLoop = artPacketsMaxLoop;
  snapshot.controlLoopLastUs = controlLoopLastUs;
  snapshot.controlLoopMaxUs = controlLoopMaxUs;
  snapshot.controlLoopOver10ms = controlLoopOver10ms;
  portEXIT_CRITICAL(&statusMux);
  return snapshot;
}

static void setSourceTimestamp(Source source) {
  uint32_t now = millis();
  portENTER_CRITICAL(&statusMux);
  if (source == Source::DMX) lastDmxMs = now;
  else if (source == Source::Artnet) lastArtMs = now;
  portEXIT_CRITICAL(&statusMux);
}

static void formatIp(uint32_t rawIp, char* out, size_t outSize) {
  snprintf(out, outSize, "%u.%u.%u.%u",
           (unsigned int)(rawIp & 0xFF),
           (unsigned int)((rawIp >> 8) & 0xFF),
           (unsigned int)((rawIp >> 16) & 0xFF),
           (unsigned int)((rawIp >> 24) & 0xFF));
}

static size_t buildStatusJson(char* out, size_t outSize, bool details) {
  DynamicJsonDocument j(1536);
  uint32_t now = millis();
  char key[16];
  StatusSnapshot snapshot = readStatusSnapshot();
  const char* src = ((now - snapshot.lastDmxMs) <= SOURCE_HOLD_MS) ? "DMX" : ((now - snapshot.lastArtMs) <= SOURCE_HOLD_MS) ? "Art-Net" : "none";
  j["source"] = src;
  j["dmx_age_ms"] = now - snapshot.lastDmxMs;
  j["art_age_ms"] = now - snapshot.lastArtMs;
  if (details) {
    j["art_udp_packets_total"] = snapshot.artUdpPacketsTotal;
    j["art_udp_bytes_total"] = snapshot.artUdpBytesTotal;
    j["artnet_packets_total"] = snapshot.artNetPacketsTotal;
    j["artpoll_total"] = snapshot.artPollTotal;
    j["artpoll_reply_total"] = snapshot.artPollReplyTotal;
    j["artsync_total"] = snapshot.artSyncTotal;
    j["art_other_opcode_total"] = snapshot.artOtherOpcodeTotal;
    j["art_rx_fps"] = snapshot.artDmxLastSecondRate;
    j["art_rx_total"] = snapshot.artDmxRxTotal;
    j["art_rx_universe_total"] = snapshot.artDmxUniverseMatchTotal;
    j["art_rx_universe_mismatch_total"] = snapshot.artDmxUniverseMismatchTotal;
    j["art_rx_invalid_total"] = snapshot.artDmxInvalidTotal;
    j["art_rx_truncated_total"] = snapshot.artDmxTruncatedTotal;
    j["art_rx_max_gap_ms"] = snapshot.artDmxMaxGapMs;
    j["art_source_change_total"] = snapshot.artSourceChangeTotal;
    j["art_rx_packets_last_loop"] = snapshot.artPacketsLastLoop;
    j["art_rx_packets_max_loop"] = snapshot.artPacketsMaxLoop;
    j["control_loop_last_us"] = snapshot.controlLoopLastUs;
    j["control_loop_max_us"] = snapshot.controlLoopMaxUs;
    j["control_loop_over_10ms"] = snapshot.controlLoopOver10ms;
    if (snapshot.artLastSourceIp != 0) {
      char sourceIp[16];
      formatIp(snapshot.artLastSourceIp, sourceIp, sizeof(sourceIp));
      j["art_last_src_ip"] = sourceIp;
      j["art_last_src_port"] = snapshot.artLastSourcePort;
    }
    if (snapshot.artMatchSourceIp != 0) {
      char matchIp[16];
      formatIp(snapshot.artMatchSourceIp, matchIp, sizeof(matchIp));
      j["art_match_src_ip"] = matchIp;
      j["art_match_src_port"] = snapshot.artMatchSourcePort;
    }
  }
  if (snapshot.artDmxLastArrivalMs == 0) {
    j["art_rx_last_ms_ago"] = -1;
  } else {
    j["art_rx_last_ms_ago"] = now - snapshot.artDmxLastArrivalMs;
  }
  for (int led = 0; led < 4; led++) {
    snprintf(key, sizeof(key), "led_value%d", led);
    j[key] = snapshot.ledValues[led];
  }
  return serializeJson(j, out, outSize);
}

static inline Source activeSource() {
  StatusSnapshot snapshot = readStatusSnapshot();
  uint32_t now = millis();
  bool dmx = (now - snapshot.lastDmxMs) <= SOURCE_HOLD_MS;
  bool art = (now - snapshot.lastArtMs) <= SOURCE_HOLD_MS;
  if (dmx) return Source::DMX;
  if (art) return Source::Artnet;
  return Source::None;
}

static inline void processDmx(uint16_t len) {
  setSourceTimestamp(Source::DMX);
  uint16_t addr = CFG.dmx_start_address;
  if (addr < 1 || addr > len) return;
  analogWrite(CH1, dmx_buf[addr]);
  analogWrite(CH2, dmx_buf[addr] + 1);
  analogWrite(CH3, dmx_buf[addr] + 2);
  analogWrite(CH4, dmx_buf[addr] + 3);
}

void setLedFloat(int ledcChannel, float brightness01) {
  if (brightness01 < 0.0f) brightness01 = 0.0f;
  if (brightness01 > 1.0f) brightness01 = 1.0f;
  int duty = (int)(brightness01 * LEDC_MAX_DUTY + 0.5f);
  ledcWrite(ledcChannel, duty);
  portENTER_CRITICAL(&statusMux);
  currentValues[ledcChannel] = brightness01;
  portEXIT_CRITICAL(&statusMux);
}

static inline void writeValue(int ledcChannel, const uint8_t* data, int addr) {
  uint16_t value = data[addr] * 256 + data[addr + 1];
  setLedFloat(ledcChannel, (float)value / 65535.0f);
}

static inline void processArtnet(const ArtDmxPacket& a) {
  setSourceTimestamp(Source::Artnet);
  if (a.universe_flat != CFG.artnet_universe) return;
  uint16_t addr = CFG.dmx_start_address;
  if (addr < 1 || addr > a.length) return;
  addr -= 1;
  writeValue(0, a.data, addr);
  addr += 2;
  writeValue(1, a.data, addr);
  addr += 2;
  writeValue(2, a.data, addr);
  addr += 2;
  writeValue(3, a.data, addr);
}

static void controlTask(void* parameter) {
  uint32_t prevLoopStartUs = micros();
  while (true) {
    uint32_t loopStartUs = micros();
    recordControlLoopTiming(loopStartUs - prevLoopStartUs);
    prevLoopStartUs = loopStartUs;

    dmx_packet_t packet;
    int size = dmx_receive(DMX_PORT, &packet, 0);
    if (size > 0) {
      dmx_read(DMX_PORT, dmx_buf, size);
      processDmx((uint16_t)size);
    }
    static uint8_t artbuf[600];
    uint32_t drainedThisLoop = 0;
    int psize = artudp.parsePacket();
    while (psize > 0) {
      int readLen = psize;
      bool truncated = false;
      if (readLen > (int)sizeof(artbuf)) {
        readLen = sizeof(artbuf);
        truncated = true;
      }
      IPAddress remoteIp = artudp.remoteIP();
      uint16_t remotePort = artudp.remotePort();
      int n = artudp.read(artbuf, readLen);
      if (n > 0) {
        uint32_t srcIpRaw = (uint32_t)remoteIp;
        recordArtUdpPacketMeta((size_t)n, srcIpRaw, remotePort);

        bool isArtNet = (n >= 10 && memcmp(artbuf, "Art-Net\0", 8) == 0);
        if (isArtNet) {
          uint16_t opcode = ((uint16_t)artbuf[9] << 8) | artbuf[8];
          recordArtNetOpcode(opcode);
          if (opcode == 0x5000) {
            ArtDmxPacket a = parseArtDmx(artbuf, n);
            if (a.ok) {
              bool matchedUniverse = (a.universe_flat == CFG.artnet_universe);
              recordArtDmxFrame(matchedUniverse, truncated, srcIpRaw, remotePort);
              processArtnet(a);
            } else {
              recordArtInvalidFrame(truncated);
            }
          }
        } else {
          recordArtInvalidFrame(truncated);
        }
        drainedThisLoop++;
      }
      psize = artudp.parsePacket();
    }
    recordArtLoopDrain(drainedThisLoop);
    vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
  }
}

static void wsAddClient(int fd) {
  if (fd < 0) return;
  portENTER_CRITICAL(&wsClientMux);
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (wsClientFds[i] == fd) {
      portEXIT_CRITICAL(&wsClientMux);
      return;
    }
  }
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (wsClientFds[i] < 0) {
      wsClientFds[i] = fd;
      break;
    }
  }
  portEXIT_CRITICAL(&wsClientMux);
}

static void wsRemoveClient(int fd) {
  if (fd < 0) return;
  portENTER_CRITICAL(&wsClientMux);
  for (int i = 0; i < MAX_WS_CLIENTS; i++) {
    if (wsClientFds[i] == fd) {
      wsClientFds[i] = -1;
      break;
    }
  }
  portEXIT_CRITICAL(&wsClientMux);
}

static int wsCopyClients(int* out, int maxCount) {
  int count = 0;
  portENTER_CRITICAL(&wsClientMux);
  for (int i = 0; i < MAX_WS_CLIENTS && count < maxCount; i++) {
    if (wsClientFds[i] >= 0) out[count++] = wsClientFds[i];
  }
  portEXIT_CRITICAL(&wsClientMux);
  return count;
}

static esp_err_t wsSendStatusToClient(int fd) {
  char out[512];
  size_t outLen = buildStatusJson(out, sizeof(out), false);
  httpd_ws_frame_t frame = {};
  frame.type = HTTPD_WS_TYPE_TEXT;
  frame.payload = (uint8_t*)out;
  frame.len = outLen;
  return httpd_ws_send_frame_async(webHttpd, fd, &frame);
}

static void wsBroadcastStatus() {
  int clients[MAX_WS_CLIENTS];
  int count = wsCopyClients(clients, MAX_WS_CLIENTS);
  for (int i = 0; i < count; i++) {
    if (wsSendStatusToClient(clients[i]) != ESP_OK) {
      wsRemoveClient(clients[i]);
    }
  }
}

static void setCorsHeaders(httpd_req_t* req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,PUT,POST,OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

static esp_err_t optionsHandler(httpd_req_t* req) {
  setCorsHeaders(req);
  httpd_resp_set_status(req, "204 No Content");
  return httpd_resp_send(req, nullptr, 0);
}

static esp_err_t sendSpiffsFile(httpd_req_t* req, const char* path, const char* contentType) {
  File file = SPIFFS.open(path, FILE_READ);
  if (!file || file.isDirectory()) {
    httpd_resp_set_status(req, "404 Not Found");
    return httpd_resp_send(req, "Not Found", HTTPD_RESP_USE_STRLEN);
  }
  size_t size = file.size();
  char* buf = (char*)malloc(size + 1);
  if (!buf) {
    httpd_resp_set_status(req, "500 Internal Server Error");
    return httpd_resp_send(req, "OOM", HTTPD_RESP_USE_STRLEN);
  }
  file.readBytes(buf, size);
  buf[size] = '\0';
  httpd_resp_set_type(req, contentType);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t ret = httpd_resp_send(req, buf, size);
  free(buf);
  return ret;
}

static esp_err_t indexHandler(httpd_req_t* req) { return sendSpiffsFile(req, "/wifi-manager/index.html", "text/html"); }
static esp_err_t otaPageHandler(httpd_req_t* req) { return sendSpiffsFile(req, "/wifi-manager/ota.html", "text/html"); }

static esp_err_t statusHandler(httpd_req_t* req) {
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
    return httpd_resp_send(req, "OOM", HTTPD_RESP_USE_STRLEN);
  }
  size_t outLen = buildStatusJson(out, 2048, details);
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t ret = httpd_resp_send(req, out, outLen);
  free(out);
  return ret;
}

static esp_err_t wsHandler(httpd_req_t* req) {
  int fd = httpd_req_to_sockfd(req);
  if (req->method == HTTP_GET) {
    wsAddClient(fd);
    return ESP_OK;
  }

  httpd_ws_frame_t frame = {};
  esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
  if (ret != ESP_OK) {
    wsRemoveClient(fd);
    return ret;
  }

  if (frame.type == HTTPD_WS_TYPE_CLOSE) {
    wsRemoveClient(fd);
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
  return ret;
}

static esp_err_t networksHandler(httpd_req_t* req) {
  bool details = false;
  size_t qlen = httpd_req_get_url_query_len(req);
  if (qlen > 0 && qlen < 64) {
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
      char value[8];
      if (httpd_query_key_value(query, "details", value, sizeof(value)) == ESP_OK) details = true;
    }
  }
  String payload = WifiManager.getNetworksPayload(details);
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, payload.c_str(), payload.length());
}

static esp_err_t credentialsGetHandler(httpd_req_t* req) {
  StaticJsonDocument<256> doc;
  doc["ssid"] = config.getSSID();
  doc["hostname"] = config.getHostname();
  doc["password"] = config.getPass();
  doc["dmx_address"] = config.getDMXAddress();
  doc["dmx_universe"] = config.getDMXUniverse();
  char out[256];
  size_t outLen = serializeJson(doc, out, sizeof(out));
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, out, outLen);
}

static esp_err_t credentialsPutHandler(httpd_req_t* req) {
  int len = req->content_len;
  if (len <= 0 || len > 1024) {
    setCorsHeaders(req);
    httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_send(req, "", 0);
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
      return httpd_resp_send(req, "", 0);
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
    return httpd_resp_send(req, "", 0);
  }
  if (doc.containsKey("ssid")) config.writeSSID(doc["ssid"].as<const char*>());
  if (doc.containsKey("password")) config.writePass(doc["password"].as<const char*>());
  if (doc.containsKey("hostname")) config.writeHostname(doc["hostname"].as<const char*>());
  if (doc.containsKey("dmx_address")) config.writeDMXAddress(doc["dmx_address"].as<int>());
  if (doc.containsKey("dmx_universe")) config.writeDMXUniverse(doc["dmx_universe"].as<int>());
  WifiManager.scheduleRestart(1000);
  setCorsHeaders(req);
  httpd_resp_set_status(req, "204 No Content");
  return httpd_resp_send(req, nullptr, 0);
}

static esp_err_t updateInfoHandler(httpd_req_t* req) {
  StaticJsonDocument<128> doc;
  doc["version"] = "1.0.0";
  doc["device"] = "CableCar";
  doc["ota_ready"] = true;
  char out[128];
  size_t outLen = serializeJson(doc, out, sizeof(out));
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, out, outLen);
}

static esp_err_t performUpdate(httpd_req_t* req, int updateCmd) {
  WifiManager.setOtaInProgress(true);

  bool beginOk = false;
  if (updateCmd == U_SPIFFS) {
    const esp_partition_t* spiffsPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
    beginOk = (spiffsPart != nullptr) && Update.begin(spiffsPart->size, U_SPIFFS);
  } else {
    beginOk = Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH);
  }

  if (!beginOk) {
    Update.printError(Serial);
    WifiManager.setOtaInProgress(false);
    setCorsHeaders(req);
    httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_send(req, "{\"success\":false,\"message\":\"Update begin failed\"}", HTTPD_RESP_USE_STRLEN);
  }

  int remaining = req->content_len;
  uint8_t buf2[1024];
  while (remaining > 0) {
    int toRead = remaining > (int)sizeof(buf2) ? (int)sizeof(buf2) : remaining;
    int read = httpd_req_recv(req, (char*)buf2, toRead);
    if (read <= 0) {
      Update.abort();
      WifiManager.setOtaInProgress(false);
      setCorsHeaders(req);
      httpd_resp_set_status(req, "500 Internal Server Error");
      return httpd_resp_send(req, "{\"success\":false,\"message\":\"Upload read failed\"}", HTTPD_RESP_USE_STRLEN);
    }
    if (Update.write(buf2, read) != (size_t)read) {
      Update.printError(Serial);
      Update.abort();
      WifiManager.setOtaInProgress(false);
      setCorsHeaders(req);
      httpd_resp_set_status(req, "500 Internal Server Error");
      return httpd_resp_send(req, "{\"success\":false,\"message\":\"Update write failed\"}", HTTPD_RESP_USE_STRLEN);
    }
    remaining -= read;
  }

  bool ok = Update.end(true);
  WifiManager.setOtaInProgress(false);
  if (!ok) {
    Update.printError(Serial);
    setCorsHeaders(req);
    httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_send(req, "{\"success\":false,\"message\":\"Update finalize failed\"}", HTTPD_RESP_USE_STRLEN);
  }

  WifiManager.scheduleRestart(1000);
  setCorsHeaders(req);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, "{\"success\":true,\"message\":\"Update successful. Restarting...\"}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t firmwareUpdateHandler(httpd_req_t* req) { return performUpdate(req, U_FLASH); }
static esp_err_t fsUpdateHandler(httpd_req_t* req) { return performUpdate(req, U_SPIFFS); }

static void registerUri(const char* path, httpd_method_t method, esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t uri = {};
  uri.uri = path;
  uri.method = method;
  uri.handler = handler;
  uri.user_ctx = nullptr;
  httpd_register_uri_handler(webHttpd, &uri);
}

static void registerWsUri(const char* path, esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t uri = {};
  uri.uri = path;
  uri.method = HTTP_GET;
  uri.handler = handler;
  uri.user_ctx = nullptr;
  uri.is_websocket = true;
  httpd_register_uri_handler(webHttpd, &uri);
}

static void startWebServer() {
  if (webHttpd != nullptr) return;
  httpd_config_t configHttp = HTTPD_DEFAULT_CONFIG();
  configHttp.server_port = 80;
  configHttp.ctrl_port = 32768;
  configHttp.max_uri_handlers = 24;

  if (httpd_start(&webHttpd, &configHttp) == ESP_OK) {
    registerUri("/", HTTP_GET, indexHandler);
    registerUri("/index.html", HTTP_GET, indexHandler);
    registerUri("/ota.html", HTTP_GET, otaPageHandler);
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

void setup() {
  Serial.begin(115200);
  Serial.println("Startup...");

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
  }

  ledcSetup(0, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH1, 0);
  ledcSetup(1, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH2, 1);
  ledcSetup(2, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH3, 2);
  ledcSetup(3, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH4, 3);

  bool connected = WifiManager.connectToWifi();
  if (!connected) {
    Serial.println("No WiFi... Starting AP");
    WifiManager.startManagementAP();
  }

  IPAddress ip = WifiManager.getIP();
  Serial.printf("IP Address: %s\n", ip.toString().c_str());

  startWebServer();

  WiFi.setSleep(false);
  artudp.begin(ARTNET_PORT);

  dmx_config_t dmx_config = DMX_CONFIG_DEFAULT;
  const int personality_count = 1;
  dmx_personality_t personalities[] = {
    {1, "RX Only"}
  };
  dmx_driver_install(DMX_PORT, &dmx_config, personalities, personality_count);

  xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    4096,
    nullptr,
    1,
    &controlTaskHandle,
    1);
}

void loop() {
  WifiManager.check();
  uint32_t now = millis();
  if (now - lastWsPushMs >= WS_STATUS_PUSH_MS) {
    wsBroadcastStatus();
    lastWsPushMs = now;
  }
  delay(1);
}
