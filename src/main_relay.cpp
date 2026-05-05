#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <AsyncUDP.h>
#include <ArduinoJson.h>

#include "main_common.h"
#include "runtime_metrics.h"

#ifndef RELAY_OUTPUT_PIN
#define RELAY_OUTPUT_PIN 9
#endif

#ifndef RELAY_ACTIVE_HIGH
#define RELAY_ACTIVE_HIGH 1
#endif

static constexpr int RELAY_PIN = RELAY_OUTPUT_PIN;
static constexpr uint8_t RELAY_ON_THRESHOLD = 128;
static constexpr uint32_t CONTROL_TASK_DELAY_MS = 2;
static constexpr uint32_t CONTROL_TASK_IDLE_WAIT_MS = 20;
static constexpr uint32_t RELAY_SWITCH_MIN_INTERVAL_MS = 500;
static constexpr uint32_t SOURCE_HOLD_MS = 1000;
static constexpr uint32_t SERIAL_NET_LOG_INTERVAL_MS = 1000;
static constexpr uint32_t WS_STATUS_PUSH_MS = 1000;
static constexpr UBaseType_t CONTROL_TASK_PRIORITY = 2;
static constexpr BaseType_t CONTROL_TASK_CORE = 1;

#ifndef ARTNET_TIMING_LOG_ENABLE
#define ARTNET_TIMING_LOG_ENABLE 0
#endif

static constexpr uint32_t ARTNET_TIMING_LOG_WINDOW_MS = 3000;
static constexpr uint32_t ARTNET_INTERVAL_WARN_US = 40000;
static constexpr uint32_t ARTNET_INTERVAL_FREEZE_US = 100000;

struct AppConfigCache {
  uint16_t dmxStartAddress = 1;
  uint16_t artnetUniverse = 0;
  float startValue = 0.0f;
};

struct StatusSnapshot {
  uint8_t dmxValue = 0;
  bool relayOn = false;
  uint32_t lastArtMs = 0;
  uint32_t artDmxRxTotal = 0;
  uint32_t artDmxUniverseMatchTotal = 0;
  uint32_t artDmxUniverseMismatchTotal = 0;
  uint32_t artDmxInvalidTotal = 0;
  uint32_t artDmxLastArrivalMs = 0;
  uint32_t controlTaskLastLoopMs = 0;
  uint16_t controlTaskUtilPermille = 0;
};

static TaskHandle_t controlTaskHandle = nullptr;
static portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
static AsyncUDP g_artudp;
static bool g_artudpListening = false;
static uint32_t g_lastSerialNetLogMs = 0;
static bool g_havePendingRelayValue = false;
static uint8_t g_pendingRelayValue = 0;

static uint8_t g_dmxValue = 0;
static bool g_relayOn = false;
static uint32_t g_lastRelaySwitchMs = 0;
static uint32_t artDmxRxTotal = 0;
static uint32_t artDmxUniverseMatchTotal = 0;
static uint32_t artDmxUniverseMismatchTotal = 0;
static uint32_t artDmxInvalidTotal = 0;
static uint32_t artDmxLastArrivalMs = 0;
static uint32_t artUdpRxTotal = 0;
static uint32_t artUdpRxBytesTotal = 0;
static uint32_t artUdpLastPacketMs = 0;
static uint32_t artUdpRebindTotal = 0;
static uint16_t artLastUniverseFlat = 0;
static uint16_t artUdpLastPacketSize = 0;
static uint16_t artUdpLastRemotePort = 0;
static IPAddress artUdpLastRemoteIp(0, 0, 0, 0);
static AppConfigCache g_cfg;
static TaskMetrics g_controlTaskMetrics;
static ArtnetTimingWindow g_artnetTiming;

static inline bool relayRawToState(uint8_t raw) {
  return raw >= RELAY_ON_THRESHOLD;
}

static void writeRelayOutput(bool on) {
#if RELAY_ACTIVE_HIGH
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
#else
  digitalWrite(RELAY_PIN, on ? LOW : HIGH);
#endif

  portENTER_CRITICAL(&statusMux);
  g_relayOn = on;
  portEXIT_CRITICAL(&statusMux);
}

static uint8_t clampStartRaw(float value) {
  if (value < 0.0f) value = 0.0f;
  if (value > 255.0f) value = 255.0f;
  return (uint8_t)(value + 0.5f);
}

static void applyRelayValue(uint8_t raw) {
  bool targetState = relayRawToState(raw);
  bool currentState;
  uint32_t now = millis();

  portENTER_CRITICAL(&statusMux);
  g_dmxValue = raw;
  currentState = g_relayOn;
  portEXIT_CRITICAL(&statusMux);

  if (targetState == currentState) {
    return;
  }

  if ((now - g_lastRelaySwitchMs) < RELAY_SWITCH_MIN_INTERVAL_MS) {
    return;
  }

  writeRelayOutput(targetState);
  portENTER_CRITICAL(&statusMux);
  g_lastRelaySwitchMs = now;
  portEXIT_CRITICAL(&statusMux);
}

static void applyStartValue(float value) {
  applyRelayValue(clampStartRaw(value));
}

static void logArtnetTimingWindowIfDue(uint32_t nowMs) {
#if ARTNET_TIMING_LOG_ENABLE
  ArtnetTimingWindow& t = g_artnetTiming;
  if (t.windowStartMs == 0) {
    t.windowStartMs = nowMs;
    return;
  }
  if ((nowMs - t.windowStartMs) < ARTNET_TIMING_LOG_WINDOW_MS) {
    return;
  }

  uint32_t windowMs = nowMs - t.windowStartMs;
  float pktHz = (windowMs > 0) ? ((float)t.packetCount * 1000.0f / (float)windowMs) : 0.0f;
  float avgIatMs = (t.packetCount > 1) ? ((float)t.intervalSumUs / (float)(t.packetCount - 1) / 1000.0f) : -1.0f;
  float minIatMs = (t.intervalMinUs == 0xFFFFFFFFu) ? -1.0f : ((float)t.intervalMinUs / 1000.0f);
  float maxIatMs = (t.intervalMaxUs == 0) ? -1.0f : ((float)t.intervalMaxUs / 1000.0f);
  float avgLoopMs = (t.loopSamples > 0) ? ((float)t.loopPeriodSumUs / (float)t.loopSamples / 1000.0f) : -1.0f;
  float minLoopMs = (t.loopPeriodMinUs == 0xFFFFFFFFu) ? -1.0f : ((float)t.loopPeriodMinUs / 1000.0f);
  float maxLoopMs = (t.loopPeriodMaxUs == 0) ? -1.0f : ((float)t.loopPeriodMaxUs / 1000.0f);
  float ctrlUtilPct;
  portENTER_CRITICAL(&statusMux);
  ctrlUtilPct = (float)g_controlTaskMetrics.utilPermille / 10.0f;
  portEXIT_CRITICAL(&statusMux);
  uint32_t idleGapMs = (t.lastPacketUs == 0) ? 0xFFFFFFFFu : ((micros() - t.lastPacketUs) / 1000U);

  Serial.printf("[ARTTIM] win=%lums hz=%.1f iat_ms=%.2f/%.2f/%.2f warn40=%lu freeze100=%lu idle=%lums loop_ms=%.2f/%.2f/%.2f late10=%lu util=%.1f\n",
                (unsigned long)windowMs,
                pktHz,
                minIatMs,
                avgIatMs,
                maxIatMs,
                (unsigned long)t.intervalWarnCount,
                (unsigned long)t.intervalFreezeCount,
                (unsigned long)idleGapMs,
                minLoopMs,
                avgLoopMs,
                maxLoopMs,
                (unsigned long)t.loopLateCount,
                ctrlUtilPct);

  Serial.printf("[ARTSRC] seq_on=%lu seq_disc=%lu seq_rep=%lu seq_back=%lu src_sw=%lu\n",
                (unsigned long)t.seqEnabledPackets,
                (unsigned long)t.seqDiscontCount,
                (unsigned long)t.seqRepeatCount,
                (unsigned long)t.seqBackwardCount,
                (unsigned long)t.sourceSwitchCount);

  resetArtnetTimingWindow(t, nowMs);
#else
  (void)nowMs;
#endif
}

static void startArtnetListener() {
  if (g_artudpListening) {
    return;
  }

  g_artudp.onPacket([](AsyncUDPPacket& packet) {
    const uint8_t* data = packet.data();
    const size_t len = packet.length();
    if (!data || len == 0) {
      return;
    }

    uint32_t packetNowUs = micros();
    IPAddress remoteIp = packet.remoteIP();
    uint16_t remotePort = packet.remotePort();
    uint32_t nowMs = millis();
    uint16_t packetSize = (uint16_t)(len > 0xFFFFu ? 0xFFFFu : len);

    noteArtnetPacketTiming(g_artnetTiming, packetNowUs, ARTNET_INTERVAL_WARN_US, ARTNET_INTERVAL_FREEZE_US);
    noteArtnetSource(g_artnetTiming, remoteIp, remotePort);
    noteArtnetSequence(g_artnetTiming, data, (int)len);

    ArtDmxPacket a = appParseArtDmx(data, (int)len);
    portENTER_CRITICAL(&statusMux);
    artUdpRxTotal++;
    artUdpRxBytesTotal += (uint32_t)len;
    artUdpLastPacketMs = nowMs;
    artUdpLastPacketSize = packetSize;
    artUdpLastRemoteIp = remoteIp;
    artUdpLastRemotePort = remotePort;

    if (!a.ok) {
      artDmxInvalidTotal++;
      portEXIT_CRITICAL(&statusMux);
      return;
    }

    artLastUniverseFlat = a.universe_flat;
    artDmxRxTotal++;

    uint16_t universe = g_cfg.artnetUniverse;
    if (a.universe_flat != universe) {
      artDmxUniverseMismatchTotal++;
      portEXIT_CRITICAL(&statusMux);
      return;
    }

    uint16_t addr = g_cfg.dmxStartAddress;
    if (addr < 1 || addr > a.length) {
      portEXIT_CRITICAL(&statusMux);
      return;
    }

    g_pendingRelayValue = a.data[addr - 1];
    g_havePendingRelayValue = true;
    artDmxUniverseMatchTotal++;
    artDmxLastArrivalMs = nowMs;
    portEXIT_CRITICAL(&statusMux);

    if (controlTaskHandle) {
      xTaskNotifyGive(controlTaskHandle);
    }
  });

  g_artudpListening = g_artudp.listen(APP_ARTNET_PORT);
  if (g_artudpListening) {
    Serial.printf("[ARTNET] AsyncUDP listening on :%u\n", APP_ARTNET_PORT);
  } else {
    Serial.printf("[ARTNET] AsyncUDP listen failed err=%d\n", (int)g_artudp.lastErr());
  }
}

static void pollArtnet() {
  if (!g_artudpListening) {
    startArtnetListener();
  }

  bool havePendingValue = false;
  uint8_t value = 0;
  portENTER_CRITICAL(&statusMux);
  if (g_havePendingRelayValue) {
    value = g_pendingRelayValue;
    g_havePendingRelayValue = false;
    havePendingValue = true;
  }
  portEXIT_CRITICAL(&statusMux);

  if (havePendingValue) {
    applyRelayValue(value);
    appMarkArtnetActivity();
  }
}

static void controlTask(void* parameter) {
  (void)parameter;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CONTROL_TASK_IDLE_WAIT_MS));
    uint32_t loopNowUs = micros();
    noteControlLoopTiming(g_artnetTiming, loopNowUs);
    uint32_t busyStartUs = micros();
    pollArtnet();
    recordTaskMetrics(g_controlTaskMetrics, statusMux, micros() - busyStartUs);
    logArtnetTimingWindowIfDue(millis());
    vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
  }
}

static StatusSnapshot readStatusSnapshot() {
  StatusSnapshot snapshot;
  portENTER_CRITICAL(&statusMux);
  snapshot.dmxValue = g_dmxValue;
  snapshot.relayOn = g_relayOn;
  snapshot.lastArtMs = appGetLastArtnetMs();
  snapshot.artDmxRxTotal = artDmxRxTotal;
  snapshot.artDmxUniverseMatchTotal = artDmxUniverseMatchTotal;
  snapshot.artDmxUniverseMismatchTotal = artDmxUniverseMismatchTotal;
  snapshot.artDmxInvalidTotal = artDmxInvalidTotal;
  snapshot.artDmxLastArrivalMs = artDmxLastArrivalMs;
  snapshot.controlTaskLastLoopMs = g_controlTaskMetrics.lastLoopMs;
  snapshot.controlTaskUtilPermille = g_controlTaskMetrics.utilPermille;
  portEXIT_CRITICAL(&statusMux);
  return snapshot;
}

static size_t buildStatusJson(char* out, size_t outSize, bool details) {
  StaticJsonDocument<1024> j;
  StatusSnapshot snapshot = readStatusSnapshot();
  uint32_t now = millis();
  uint32_t artAge = (snapshot.lastArtMs == 0) ? 0xFFFFFFFFu : (now - snapshot.lastArtMs);
  TaskHandle_t webTaskHandle = appGetWebServerTaskHandle();
  AppTaskRuntimeStats webTaskStats = appGetWebTaskRuntimeStats();
  WifiManagerClass& wifiManager = appWifiManager();
  IPAddress ip = wifiManager.getIP();
  char ipBuf[16];
  char macBuf[18];
  snprintf(ipBuf, sizeof(ipBuf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  wifiManager.getMacAddress(macBuf, sizeof(macBuf));

  j["source"] = (artAge <= SOURCE_HOLD_MS) ? "Art-Net" : "none";
  j["art_age_ms"] = (artAge == 0xFFFFFFFFu) ? -1 : (int32_t)artAge;
  j["relay_value"] = snapshot.dmxValue;
  j["relay_on"] = snapshot.relayOn;
  j["relay_state"] = snapshot.relayOn ? "ON" : "OFF";
  j["relay_threshold"] = RELAY_ON_THRESHOLD;
  j["wifi_rssi"] = wifiManager.getRSSI();
  j["hostname"] = wifiManager.getHostnameCStr();
  j["wifi_ip"] = ipBuf;
  j["wifi_mac"] = macBuf;
  j["free_heap"] = ESP.getFreeHeap();
  j["min_free_heap"] = ESP.getMinFreeHeap();
  j["board_temp_c"] = appReadBoardTemperatureC();
  j["reset_reason"] = appResetReasonToString(esp_reset_reason());
  j["task_control_state"] = controlTaskHandle ? taskStateToString(eTaskGetState(controlTaskHandle)) : "not_created";
  j["task_control_stack_hwm"] = controlTaskHandle ? (int32_t)uxTaskGetStackHighWaterMark(controlTaskHandle) : -1;
  j["task_control_last_ms_ago"] = (snapshot.controlTaskLastLoopMs == 0) ? -1 : (int32_t)(now - snapshot.controlTaskLastLoopMs);
  j["task_control_util_pct"] = (float)snapshot.controlTaskUtilPermille / 10.0f;
  j["task_web_state"] = webTaskHandle ? taskStateToString(eTaskGetState(webTaskHandle)) : "unknown";
  j["task_web_stack_hwm"] = webTaskHandle ? (int32_t)uxTaskGetStackHighWaterMark(webTaskHandle) : -1;
  j["task_web_last_ms_ago"] = (webTaskStats.lastActiveMs == 0) ? -1 : (int32_t)(now - webTaskStats.lastActiveMs);
  j["task_web_util_pct"] = (float)webTaskStats.utilPermille / 10.0f;

  if (details) {
    j["art_rx_total"] = snapshot.artDmxRxTotal;
    j["art_rx_universe_total"] = snapshot.artDmxUniverseMatchTotal;
    j["art_rx_universe_mismatch_total"] = snapshot.artDmxUniverseMismatchTotal;
    j["art_rx_invalid_total"] = snapshot.artDmxInvalidTotal;
    j["art_rx_last_ms_ago"] = (snapshot.artDmxLastArrivalMs == 0) ? -1 : (int32_t)(now - snapshot.artDmxLastArrivalMs);
  }

  return serializeJson(j, out, outSize);
}

static size_t buildHealthSummary(char* out, size_t outSize) {
  StatusSnapshot snapshot = readStatusSnapshot();
  uint32_t now = millis();
  int32_t lastMsAgo = (snapshot.artDmxLastArrivalMs == 0) ? -1 : (int32_t)(now - snapshot.artDmxLastArrivalMs);
  uint32_t artAge = (snapshot.lastArtMs == 0) ? 0xFFFFFFFFu : (now - snapshot.lastArtMs);
  int32_t udpLastMsAgo;
  uint32_t udpRx;
  uint32_t udpBytes;
  uint32_t udpRebinds;
  uint16_t lastUni;
  uint16_t lastSize;
  uint16_t lastPort;
  IPAddress lastIp;

  portENTER_CRITICAL(&statusMux);
  udpLastMsAgo = (artUdpLastPacketMs == 0) ? -1 : (int32_t)(now - artUdpLastPacketMs);
  udpRx = artUdpRxTotal;
  udpBytes = artUdpRxBytesTotal;
  udpRebinds = artUdpRebindTotal;
  lastUni = artLastUniverseFlat;
  lastSize = artUdpLastPacketSize;
  lastPort = artUdpLastRemotePort;
  lastIp = artUdpLastRemoteIp;
  portEXIT_CRITICAL(&statusMux);

  return snprintf(out,
                  outSize,
                  "art_src=%s art_rx=%lu match=%lu mismatch=%lu invalid=%lu last=%ldms udp_rx=%lu udp_b=%lu udp_last=%ldms from=%s:%u pkt=%u uni=%u rebind=%lu relay=%s dmx=%u",
                  (artAge <= SOURCE_HOLD_MS) ? "live" : "idle",
                  (unsigned long)snapshot.artDmxRxTotal,
                  (unsigned long)snapshot.artDmxUniverseMatchTotal,
                  (unsigned long)snapshot.artDmxUniverseMismatchTotal,
                  (unsigned long)snapshot.artDmxInvalidTotal,
                  (long)lastMsAgo,
                  (unsigned long)udpRx,
                  (unsigned long)udpBytes,
                  (long)udpLastMsAgo,
                  lastIp.toString().c_str(),
                  (unsigned int)lastPort,
                  (unsigned int)lastSize,
                  (unsigned int)lastUni,
                  (unsigned long)udpRebinds,
                  snapshot.relayOn ? "on" : "off",
                  (unsigned int)snapshot.dmxValue);
}

static void logNetworkInfoIfDue() {
  uint32_t now = millis();
  if ((now - g_lastSerialNetLogMs) < SERIAL_NET_LOG_INTERVAL_MS) {
    return;
  }
  g_lastSerialNetLogMs = now;

  WifiManagerClass& wifiManager = appWifiManager();
  IPAddress ip = wifiManager.getIP();
  char macBuf[18];
  wifiManager.getMacAddress(macBuf, sizeof(macBuf));

  Serial.printf("[NET] ip=%u.%u.%u.%u mac=%s\n",
                ip[0],
                ip[1],
                ip[2],
                ip[3],
                macBuf);
}

void setup() {
  appInitializeBaseRuntime();

  pinMode(RELAY_PIN, OUTPUT);

  g_cfg.dmxStartAddress = appConfig().getDMXAddress();
  g_cfg.artnetUniverse = appConfig().getDMXUniverse();
  g_cfg.startValue = appConfig().getStartValue();
  g_lastRelaySwitchMs = millis() - RELAY_SWITCH_MIN_INTERVAL_MS;
  applyStartValue(g_cfg.startValue);

  appInitRuntime({
    "/wifi-manager/index_relay.html",
    "CableCar Relay",
    buildStatusJson,
    buildHealthSummary,
    nullptr,
  });

  appStartCommonServices();
  appConnectWifi();

  xTaskCreatePinnedToCore(controlTask,
                          "ControlTask",
                          4096,
                          nullptr,
                          CONTROL_TASK_PRIORITY,
                          &controlTaskHandle,
                          CONTROL_TASK_CORE);
}

void loop() {
  logNetworkInfoIfDue();
  appCommonLoop(WS_STATUS_PUSH_MS);
  delay(1);
}
