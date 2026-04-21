#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <AsyncUDP.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#include "main_common.h"

static constexpr int CH1 = 1;
static constexpr int CH2 = 2;
static constexpr int CH3 = 3;
static constexpr int CH4 = 21;

static constexpr int LEDC_RES_BITS = 12;
static constexpr int LEDC_MAX_DUTY = (1 << LEDC_RES_BITS) - 1;
static constexpr int LEDC_FREQ_HZ = 1000;

static constexpr uint32_t CONTROL_TASK_DELAY_MS = 2;
static constexpr uint32_t SOURCE_HOLD_MS = 1000;
static constexpr uint32_t WS_STATUS_PUSH_MS = 1000;

#ifndef ARTNET_TIMING_LOG_ENABLE
#define ARTNET_TIMING_LOG_ENABLE 0
#endif

static constexpr uint32_t ARTNET_TIMING_LOG_WINDOW_MS = 3000;
static constexpr uint32_t ARTNET_INTERVAL_WARN_US = 40000;
static constexpr uint32_t ARTNET_INTERVAL_FREEZE_US = 100000;

static TaskHandle_t controlTaskHandle = nullptr;
static portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
static AsyncUDP g_artudp;
static bool g_artudpListening = false;
static bool g_havePendingLedValues = false;
static uint16_t g_pendingLedValues[4] = {0, 0, 0, 0};

static float currentValues[4] = {0, 0, 0, 0};

struct AppConfigCache {
  uint16_t dmxStartAddress = 1;
  uint16_t artnetUniverse = 0;
  float startValue = 0.0f;
};

struct TaskMetrics {
  uint32_t lastLoopMs = 0;
  uint32_t windowStartUs = 0;
  uint32_t busyUsAccum = 0;
  uint16_t utilPermille = 0;
};

struct ArtnetTimingWindow {
  uint32_t windowStartMs = 0;
  uint32_t lastPacketUs = 0;
  uint32_t lastControlLoopUs = 0;

  uint32_t packetCount = 0;
  uint64_t intervalSumUs = 0;
  uint32_t intervalMinUs = 0xFFFFFFFFu;
  uint32_t intervalMaxUs = 0;
  uint32_t intervalWarnCount = 0;
  uint32_t intervalFreezeCount = 0;

  uint32_t seqEnabledPackets = 0;
  uint32_t seqDiscontCount = 0;
  uint32_t seqRepeatCount = 0;
  uint32_t seqBackwardCount = 0;
  uint8_t lastSeq = 0;
  bool haveLastSeq = false;

  uint32_t sourceSwitchCount = 0;
  uint64_t lastSourceKey = 0;

  uint32_t loopSamples = 0;
  uint64_t loopPeriodSumUs = 0;
  uint32_t loopPeriodMinUs = 0xFFFFFFFFu;
  uint32_t loopPeriodMaxUs = 0;
  uint32_t loopLateCount = 0;
};

struct StatusSnapshot {
  float ledValues[4];
  uint32_t lastArtMs;
  uint32_t artDmxRxTotal;
  uint32_t artDmxUniverseMatchTotal;
  uint32_t artDmxUniverseMismatchTotal;
  uint32_t artDmxInvalidTotal;
  uint32_t artDmxLastArrivalMs;
  uint32_t controlTaskLastLoopMs;
  uint16_t controlTaskUtilPermille;
};

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

static void noteArtnetPacketTiming(uint32_t nowUs) {
  ArtnetTimingWindow& t = g_artnetTiming;
  if (t.lastPacketUs != 0) {
    uint32_t dtUs = nowUs - t.lastPacketUs;
    t.intervalSumUs += dtUs;
    if (dtUs < t.intervalMinUs) t.intervalMinUs = dtUs;
    if (dtUs > t.intervalMaxUs) t.intervalMaxUs = dtUs;
    if (dtUs >= ARTNET_INTERVAL_WARN_US) t.intervalWarnCount++;
    if (dtUs >= ARTNET_INTERVAL_FREEZE_US) t.intervalFreezeCount++;
  }
  t.lastPacketUs = nowUs;
  t.packetCount++;
}

static void noteArtnetSource(IPAddress remoteIp, uint16_t remotePort) {
  ArtnetTimingWindow& t = g_artnetTiming;
  uint32_t ipRaw = ((uint32_t)remoteIp[0] << 24) |
                   ((uint32_t)remoteIp[1] << 16) |
                   ((uint32_t)remoteIp[2] << 8) |
                   (uint32_t)remoteIp[3];
  uint64_t key = (((uint64_t)ipRaw) << 16) | (uint64_t)remotePort;
  if (t.lastSourceKey != 0 && t.lastSourceKey != key) {
    t.sourceSwitchCount++;
  }
  t.lastSourceKey = key;
}

static void noteArtnetSequence(const uint8_t* p, int len) {
  if (len < 18) return;
  if (memcmp(p, "Art-Net\0", 8) != 0) return;
  if (!(p[8] == 0x00 && p[9] == 0x50)) return;

  uint8_t seq = p[12];
  if (seq == 0) {
    return;
  }

  ArtnetTimingWindow& t = g_artnetTiming;
  t.seqEnabledPackets++;

  if (t.haveLastSeq) {
    uint8_t expected = (uint8_t)(t.lastSeq + 1);
    if (seq == t.lastSeq) {
      t.seqRepeatCount++;
    } else if (seq != expected) {
      t.seqDiscontCount++;
      if ((uint8_t)(seq - t.lastSeq) > 128U) {
        t.seqBackwardCount++;
      }
    }
  }

  t.lastSeq = seq;
  t.haveLastSeq = true;
}

static void noteControlLoopTiming(uint32_t nowUs) {
  ArtnetTimingWindow& t = g_artnetTiming;
  if (t.lastControlLoopUs != 0) {
    uint32_t dtUs = nowUs - t.lastControlLoopUs;
    t.loopPeriodSumUs += dtUs;
    if (dtUs < t.loopPeriodMinUs) t.loopPeriodMinUs = dtUs;
    if (dtUs > t.loopPeriodMaxUs) t.loopPeriodMaxUs = dtUs;
    if (dtUs >= 10000U) t.loopLateCount++;
    t.loopSamples++;
  }
  t.lastControlLoopUs = nowUs;
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

  t.windowStartMs = nowMs;
  t.packetCount = 0;
  t.intervalSumUs = 0;
  t.intervalMinUs = 0xFFFFFFFFu;
  t.intervalMaxUs = 0;
  t.intervalWarnCount = 0;
  t.intervalFreezeCount = 0;
  t.seqEnabledPackets = 0;
  t.seqDiscontCount = 0;
  t.seqRepeatCount = 0;
  t.seqBackwardCount = 0;
  t.sourceSwitchCount = 0;
  t.loopSamples = 0;
  t.loopPeriodSumUs = 0;
  t.loopPeriodMinUs = 0xFFFFFFFFu;
  t.loopPeriodMaxUs = 0;
  t.loopLateCount = 0;
#else
  (void)nowMs;
#endif
}

static void recordTaskMetrics(TaskMetrics& metrics, uint32_t busyUs) {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  portENTER_CRITICAL(&statusMux);
  if (metrics.windowStartUs == 0) {
    metrics.windowStartUs = nowUs;
  }

  metrics.lastLoopMs = nowMs;
  metrics.busyUsAccum += busyUs;

  uint32_t elapsedUs = nowUs - metrics.windowStartUs;
  if (elapsedUs >= 1000000UL) {
    uint32_t permille = (metrics.busyUsAccum * 1000UL) / elapsedUs;
    metrics.utilPermille = (uint16_t)(permille > 1000UL ? 1000UL : permille);
    metrics.busyUsAccum = 0;
    metrics.windowStartUs = nowUs;
  }
  portEXIT_CRITICAL(&statusMux);
}

static const char* taskStateToString(eTaskState state) {
  switch (state) {
    case eRunning: return "running";
    case eReady: return "ready";
    case eBlocked: return "blocked";
    case eSuspended: return "suspended";
    case eDeleted: return "deleted";
    default: return "unknown";
  }
}

static void setLedFloat(int ledcChannel, float brightness01) {
  if (brightness01 < 0.0f) brightness01 = 0.0f;
  if (brightness01 > 1.0f) brightness01 = 1.0f;

  int duty = (int)(brightness01 * LEDC_MAX_DUTY + 0.5f);
  ledcWrite(ledcChannel, duty);

  portENTER_CRITICAL(&statusMux);
  currentValues[ledcChannel] = brightness01;
  portEXIT_CRITICAL(&statusMux);
}

static void applyStartValue(float value) {
  for (int i = 0; i < 4; i++) {
    setLedFloat(i, value);
  }
}

static inline void writeValue(int ledcChannel, const uint8_t* data, uint16_t addr) {
  uint16_t value = ((uint16_t)data[addr] << 8) | data[addr + 1];
  setLedFloat(ledcChannel, (float)value / 65535.0f);
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

    noteArtnetPacketTiming(packetNowUs);
    noteArtnetSource(remoteIp, remotePort);
    noteArtnetSequence(data, (int)len);

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
    if (addr < 1 || addr + 7 > a.length) {
      portEXIT_CRITICAL(&statusMux);
      return;
    }

    addr -= 1;
    uint16_t value0 = ((uint16_t)a.data[addr] << 8) | a.data[addr + 1];
    uint16_t value1 = ((uint16_t)a.data[addr + 2] << 8) | a.data[addr + 3];
    uint16_t value2 = ((uint16_t)a.data[addr + 4] << 8) | a.data[addr + 5];
    uint16_t value3 = ((uint16_t)a.data[addr + 6] << 8) | a.data[addr + 7];

    g_pendingLedValues[0] = value0;
    g_pendingLedValues[1] = value1;
    g_pendingLedValues[2] = value2;
    g_pendingLedValues[3] = value3;
    g_havePendingLedValues = true;
    artDmxUniverseMatchTotal++;
    artDmxLastArrivalMs = nowMs;
    portEXIT_CRITICAL(&statusMux);
  });

  g_artudpListening = g_artudp.listen(6454);
  if (g_artudpListening) {
    Serial.println("[ARTNET] AsyncUDP listening on :6454");
  } else {
    Serial.printf("[ARTNET] AsyncUDP listen failed err=%d\n", (int)g_artudp.lastErr());
  }
}

static void pollArtnet() {
  if (!g_artudpListening) {
    startArtnetListener();
  }

  bool havePendingValues = false;
  uint16_t values[4] = {0, 0, 0, 0};
  portENTER_CRITICAL(&statusMux);
  if (g_havePendingLedValues) {
    values[0] = g_pendingLedValues[0];
    values[1] = g_pendingLedValues[1];
    values[2] = g_pendingLedValues[2];
    values[3] = g_pendingLedValues[3];
    g_havePendingLedValues = false;
    havePendingValues = true;
  }
  portEXIT_CRITICAL(&statusMux);

  if (havePendingValues) {
    setLedFloat(0, (float)values[0] / 65535.0f);
    setLedFloat(1, (float)values[1] / 65535.0f);
    setLedFloat(2, (float)values[2] / 65535.0f);
    setLedFloat(3, (float)values[3] / 65535.0f);
    appMarkArtnetActivity();
  }
}

static void controlTask(void* parameter) {
  (void)parameter;
  for (;;) {
    uint32_t loopNowUs = micros();
    noteControlLoopTiming(loopNowUs);
    uint32_t busyStartUs = micros();
    pollArtnet();
    recordTaskMetrics(g_controlTaskMetrics, micros() - busyStartUs);
    logArtnetTimingWindowIfDue(millis());
    vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
  }
}

static StatusSnapshot readStatusSnapshot() {
  StatusSnapshot snapshot;
  portENTER_CRITICAL(&statusMux);
  for (int i = 0; i < 4; i++) snapshot.ledValues[i] = currentValues[i];
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
  j["wifi_rssi"] = wifiManager.getRSSI();
  j["hostname"] = wifiManager.getHostnameCStr();
  j["wifi_ip"] = ipBuf;
  j["wifi_mac"] = macBuf;
  j["free_heap"] = ESP.getFreeHeap();
  j["min_free_heap"] = ESP.getMinFreeHeap();
  j["board_temp_c"] = appReadBoardTemperatureC();
  j["reset_reason"] = appResetReasonToString(esp_reset_reason());
  j["task_control_state"] = taskStateToString(eTaskGetState(controlTaskHandle));
  j["task_control_stack_hwm"] = (int32_t)uxTaskGetStackHighWaterMark(controlTaskHandle);
  j["task_control_last_ms_ago"] = (snapshot.controlTaskLastLoopMs == 0) ? -1 : (int32_t)(now - snapshot.controlTaskLastLoopMs);
  j["task_control_util_pct"] = (float)snapshot.controlTaskUtilPermille / 10.0f;
  j["task_web_state"] = webTaskHandle ? taskStateToString(eTaskGetState(webTaskHandle)) : "unknown";
  j["task_web_stack_hwm"] = webTaskHandle ? (int32_t)uxTaskGetStackHighWaterMark(webTaskHandle) : -1;
  j["task_web_last_ms_ago"] = (webTaskStats.lastActiveMs == 0) ? -1 : (int32_t)(now - webTaskStats.lastActiveMs);
  j["task_web_util_pct"] = (float)webTaskStats.utilPermille / 10.0f;

  j["led_value0"] = snapshot.ledValues[0];
  j["led_value1"] = snapshot.ledValues[1];
  j["led_value2"] = snapshot.ledValues[2];
  j["led_value3"] = snapshot.ledValues[3];

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
                  "art_src=%s art_rx=%lu match=%lu mismatch=%lu invalid=%lu last=%ldms udp_rx=%lu udp_b=%lu udp_last=%ldms from=%s:%u pkt=%u uni=%u rebind=%lu led0=%.3f",
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
                  snapshot.ledValues[0]);
}

void setup() {
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

  ledcSetup(0, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH1, 0);
  ledcSetup(1, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH2, 1);
  ledcSetup(2, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH3, 2);
  ledcSetup(3, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH4, 3);

  g_cfg.dmxStartAddress = appConfig().getDMXAddress();
  g_cfg.artnetUniverse = appConfig().getDMXUniverse();
  g_cfg.startValue = appConfig().getStartValue();
  applyStartValue(g_cfg.startValue);

  appInitRuntime({
    "/wifi-manager/index_led.html",
    "CableCar LED",
    buildStatusJson,
    buildHealthSummary,
    nullptr,
  });

  appConnectWifi();
  appStartCommonServices();

  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, nullptr, 1, &controlTaskHandle, 0);

}

void loop() {
  appCommonLoop(WS_STATUS_PUSH_MS);
  delay(1);
}
