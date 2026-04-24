#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <AsyncUDP.h>
#include <ArduinoJson.h>

#include "main_common.h"
#include "runtime_metrics.h"
#include "bldc_uart.h"

#ifndef BLDC_UART_TX_PIN
#define BLDC_UART_TX_PIN 27
#endif

#ifndef BLDC_UART_RX_PIN
#define BLDC_UART_RX_PIN 14
#endif

#ifndef BLDC_UART_OE_PIN
#define BLDC_UART_OE_PIN 19
#endif

static constexpr int BLDC_TX = BLDC_UART_TX_PIN;
static constexpr int BLDC_RX = BLDC_UART_RX_PIN;
static constexpr int UART_OE = BLDC_UART_OE_PIN;

static constexpr uint32_t CONTROL_TASK_DELAY_MS = 2;
static constexpr uint32_t MOTOR_TASK_DELAY_MS = 10;
static constexpr uint32_t SOURCE_HOLD_MS = 1000;
static constexpr uint32_t WS_STATUS_PUSH_MS = 1000;
static constexpr uint32_t ARTNET_HEALTH_LOG_INTERVAL_MS = 5000;
static constexpr uint32_t STATUS_DIAGNOSTICS_REFRESH_MS = 2000;

#ifndef ARTNET_TIMING_LOG_ENABLE
#define ARTNET_TIMING_LOG_ENABLE 0
#endif

#ifndef ARTNET_HEALTH_LOG_ENABLE
#define ARTNET_HEALTH_LOG_ENABLE 0
#endif

#ifndef BLDC_RX_LOG_ENABLE
#define BLDC_RX_LOG_ENABLE 0
#endif

static constexpr uint32_t ARTNET_TIMING_LOG_WINDOW_MS = 3000;
static constexpr uint32_t ARTNET_INTERVAL_WARN_US = 40000;
static constexpr uint32_t ARTNET_INTERVAL_FREEZE_US = 100000;

HardwareSerial BLDCSerial(2);
BLDC::Driver bldc(BLDCSerial, BLDC_RX, BLDC_TX, 9600);

struct MotorCmd {
  uint8_t value;
};

struct AppConfigCache {
  uint16_t dmxStartAddress = 1;
  uint16_t artnetUniverse = 0;
  float startValue = 0.0f;
};

struct StatusSnapshot {
  uint8_t motorRaw = 0;
  uint8_t motorStep = 0;
  uint32_t lastArtMs = 0;
  uint32_t artDmxRxTotal = 0;
  uint32_t artDmxUniverseMatchTotal = 0;
  uint32_t artDmxUniverseMismatchTotal = 0;
  uint32_t artDmxInvalidTotal = 0;
  uint32_t artDmxLastArrivalMs = 0;
  uint32_t controlTaskLastLoopMs = 0;
  uint32_t motorTaskLastLoopMs = 0;
  uint16_t controlTaskUtilPermille = 0;
  uint16_t motorTaskUtilPermille = 0;
};

struct CachedDiagnostics {
  float boardTempC = NAN;
  int32_t controlTaskStackHwm = -1;
  int32_t motorTaskStackHwm = -1;
  int32_t webTaskStackHwm = -1;
  const char* controlTaskState = "unknown";
  const char* motorTaskState = "unknown";
  const char* webTaskState = "unknown";
};

static TaskHandle_t motorTaskHandle = nullptr;
static TaskHandle_t controlTaskHandle = nullptr;
static QueueHandle_t motorQueue = nullptr;
static uint32_t g_lastArtnetHealthLogMs = 0;
static AsyncUDP g_artudp;
static bool g_artudpListening = false;
static bool g_havePendingMotorRaw = false;
static uint8_t g_pendingMotorRaw = 0;

static portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t g_motorRaw = 0;
static uint8_t g_motorStep = 0;
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
static TaskMetrics g_motorTaskMetrics;
static ArtnetTimingWindow g_artnetTiming;
static CachedDiagnostics g_cachedDiagnostics;
static uint32_t g_statusDiagnosticsBuiltMs = 0;

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
  float motorUtilPct;
  portENTER_CRITICAL(&statusMux);
  ctrlUtilPct = (float)g_controlTaskMetrics.utilPermille / 10.0f;
  motorUtilPct = (float)g_motorTaskMetrics.utilPermille / 10.0f;
  portEXIT_CRITICAL(&statusMux);
  uint32_t idleGapMs = (t.lastPacketUs == 0) ? 0xFFFFFFFFu : ((micros() - t.lastPacketUs) / 1000U);

  Serial.printf("[ARTTIM] win=%lums hz=%.1f iat_ms=%.2f/%.2f/%.2f warn40=%lu freeze100=%lu idle=%lums loop_ms=%.2f/%.2f/%.2f late10=%lu util=%.1f/%.1f\n",
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
                ctrlUtilPct,
                motorUtilPct);

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

    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();
    const uint16_t packetSize = (uint16_t)(len > 0xFFFFu ? 0xFFFFu : len);
    const IPAddress remoteIp = packet.remoteIP();
    const uint16_t remotePort = packet.remotePort();
    const uint16_t configuredUniverse = g_cfg.artnetUniverse;
    const uint16_t configuredChannel = g_cfg.dmxStartAddress;

    bool dmxOk = false;
    bool universeMatch = false;
    uint16_t universeFlat = 0;
    uint8_t motorRaw = 0;

    if (len >= 18 &&
        memcmp(data, "Art-Net\0", 8) == 0 &&
        data[8] == 0x00 &&
        data[9] == 0x50) {
      uint8_t subUni = data[14];
      uint8_t net = data[15];
      uint16_t dlen = ((uint16_t)data[16] << 8) | data[17];
      if (dlen <= 512 && (18 + dlen) <= len) {
        uint8_t subnet = (subUni >> 4) & 0x0F;
        uint8_t uni = subUni & 0x0F;
        universeFlat = ((uint16_t)net << 8) | ((uint16_t)subnet << 4) | uni;
        dmxOk = true;
        universeMatch = (universeFlat == configuredUniverse);
        if (universeMatch && configuredChannel >= 1 && configuredChannel <= dlen) {
          motorRaw = data[17 + configuredChannel];
        }
      }
    }

    portENTER_CRITICAL(&statusMux);
    artUdpRxTotal++;
    artUdpRxBytesTotal += (uint32_t)len;
    artUdpLastPacketMs = nowMs;
    artUdpLastPacketSize = packetSize;
    artUdpLastRemoteIp = remoteIp;
    artUdpLastRemotePort = remotePort;
    if (!dmxOk) {
      artDmxInvalidTotal++;
    } else {
      artDmxRxTotal++;
      artLastUniverseFlat = universeFlat;
      artDmxLastArrivalMs = nowMs;
#if ARTNET_TIMING_LOG_ENABLE
      noteArtnetPacketTiming(g_artnetTiming, nowUs, ARTNET_INTERVAL_WARN_US, ARTNET_INTERVAL_FREEZE_US);
      noteArtnetSource(g_artnetTiming, remoteIp, remotePort);
      noteArtnetSequence(g_artnetTiming, data, (int)len);
#endif
      if (universeMatch) {
        artDmxUniverseMatchTotal++;
        if (configuredChannel >= 1) {
          g_pendingMotorRaw = motorRaw;
          g_havePendingMotorRaw = true;
        }
      } else {
        artDmxUniverseMismatchTotal++;
      }
    }
    portEXIT_CRITICAL(&statusMux);
  });

  g_artudpListening = g_artudp.listen(APP_ARTNET_PORT);
  if (g_artudpListening) {
    Serial.printf("[ARTNET] AsyncUDP listening on :%u\n", APP_ARTNET_PORT);
  } else {
    Serial.printf("[ARTNET] AsyncUDP listen failed err=%d\n", (int)g_artudp.lastErr());
  }
}

static void logArtnetHealthIfDue(uint32_t nowMs) {
#if !ARTNET_HEALTH_LOG_ENABLE
  (void)nowMs;
  return;
#else
  if ((nowMs - g_lastArtnetHealthLogMs) < ARTNET_HEALTH_LOG_INTERVAL_MS) {
    return;
  }

  uint32_t udpLastMsAgo;
  uint32_t udpRx;
  uint32_t udpBytes;
  uint32_t udpRebinds;
  uint16_t lastUni;
  uint16_t lastSize;
  uint16_t lastPort;
  IPAddress lastIp;
  uint32_t dmxRx;
  uint32_t dmxMatch;
  uint32_t dmxMismatch;
  uint32_t dmxInvalid;
  uint16_t ctrlUtilPermille;
  uint16_t motorUtilPermille;
  uint32_t ctrlLastMs;
  uint32_t motorLastMs;
  uint8_t motorRaw;

  portENTER_CRITICAL(&statusMux);
  udpLastMsAgo = (artUdpLastPacketMs == 0) ? 0xFFFFFFFFu : (nowMs - artUdpLastPacketMs);
  udpRx = artUdpRxTotal;
  udpBytes = artUdpRxBytesTotal;
  udpRebinds = artUdpRebindTotal;
  lastUni = artLastUniverseFlat;
  lastSize = artUdpLastPacketSize;
  lastPort = artUdpLastRemotePort;
  lastIp = artUdpLastRemoteIp;
  dmxRx = artDmxRxTotal;
  dmxMatch = artDmxUniverseMatchTotal;
  dmxMismatch = artDmxUniverseMismatchTotal;
  dmxInvalid = artDmxInvalidTotal;
  ctrlUtilPermille = g_controlTaskMetrics.utilPermille;
  motorUtilPermille = g_motorTaskMetrics.utilPermille;
  ctrlLastMs = g_controlTaskMetrics.lastLoopMs;
  motorLastMs = g_motorTaskMetrics.lastLoopMs;
  motorRaw = g_motorRaw;
  portEXIT_CRITICAL(&statusMux);

  uint32_t artLastMs = appGetLastArtnetMs();
  uint32_t artLastMsAgo = (artLastMs == 0) ? 0xFFFFFFFFu : (nowMs - artLastMs);

  Serial.printf("[ARTHEALTH] up=%lu udp_last=%ldms art_last=%ldms udp_rx=%lu dmx=%lu match=%lu mismatch=%lu invalid=%lu bytes=%lu from=%s:%u pkt=%u uni=%u rebind=%lu motor=%u ctrl=%.1f%%/%ldms motor=%.1f%%/%ldms heap=%lu\n",
                (unsigned long)nowMs,
                (long)(udpLastMsAgo == 0xFFFFFFFFu ? -1 : (int32_t)udpLastMsAgo),
                (long)(artLastMsAgo == 0xFFFFFFFFu ? -1 : (int32_t)artLastMsAgo),
                (unsigned long)udpRx,
                (unsigned long)dmxRx,
                (unsigned long)dmxMatch,
                (unsigned long)dmxMismatch,
                (unsigned long)dmxInvalid,
                (unsigned long)udpBytes,
                lastIp.toString().c_str(),
                (unsigned int)lastPort,
                (unsigned int)lastSize,
                (unsigned int)lastUni,
                (unsigned long)udpRebinds,
                (unsigned int)motorRaw,
                (float)ctrlUtilPermille / 10.0f,
                (long)(ctrlLastMs == 0 ? -1 : (int32_t)(nowMs - ctrlLastMs)),
                (float)motorUtilPermille / 10.0f,
                (long)(motorLastMs == 0 ? -1 : (int32_t)(nowMs - motorLastMs)),
                (unsigned long)ESP.getFreeHeap());

  g_lastArtnetHealthLogMs = nowMs;
#endif
}

static inline uint8_t rawToStep(uint8_t raw) {
  return (uint8_t)((((uint16_t)raw) * 60U + 127U) / 255U);
}

static uint8_t clampStartRaw(float value) {
  if (value < 0.0f) value = 0.0f;
  if (value > 255.0f) value = 255.0f;
  return (uint8_t)(value + 0.5f);
}

static void applyStartValue(float value) {
  uint8_t raw = clampStartRaw(value);
  uint8_t step = rawToStep(raw);
  bldc.sendSpeed(step);

  portENTER_CRITICAL(&statusMux);
  g_motorRaw = raw;
  g_motorStep = step;
  portEXIT_CRITICAL(&statusMux);
}

static void enqueueMotor(uint8_t raw) {
  if (!motorQueue) return;
  MotorCmd cmd{raw};
  xQueueOverwrite(motorQueue, &cmd);
}

static void motorTask(void* parameter) {
  uint8_t currentRaw = 0;

#if BLDC_RX_LOG_ENABLE
  bldc.onFrame = [](const uint8_t* f) {
    Serial.printf("[BLDC RX] %02X %02X %02X %02X %02X %02X %02X\n",
                  f[0], f[1], f[2], f[3], f[4], f[5], f[6]);
  };
#endif

  for (;;) {
    uint32_t workUs = 0;
    MotorCmd cmd;
    if (xQueueReceive(motorQueue, &cmd, pdMS_TO_TICKS(MOTOR_TASK_DELAY_MS)) == pdPASS) {
      uint32_t t0 = micros();
      currentRaw = cmd.value;
      uint8_t step = rawToStep(currentRaw);
      bldc.sendSpeed(step);

      portENTER_CRITICAL(&statusMux);
      g_motorRaw = currentRaw;
      g_motorStep = step;
      portEXIT_CRITICAL(&statusMux);
      workUs += micros() - t0;
    }

    uint32_t t1 = micros();
    bldc.poll();
    workUs += micros() - t1;

    // Queue wait time is idle time and must not be counted as task busy load.
    recordTaskMetrics(g_motorTaskMetrics, statusMux, workUs);
  }
}

static void pollArtnet() {
  if (!g_artudpListening) {
    startArtnetListener();
  }

  bool havePendingRaw = false;
  uint8_t pendingRaw = 0;
  portENTER_CRITICAL(&statusMux);
  if (g_havePendingMotorRaw) {
    pendingRaw = g_pendingMotorRaw;
    g_havePendingMotorRaw = false;
    havePendingRaw = true;
  }
  portEXIT_CRITICAL(&statusMux);

  if (havePendingRaw) {
    enqueueMotor(pendingRaw);
    appMarkArtnetActivity();
  }
}

static CachedDiagnostics readCachedDiagnostics() {
  CachedDiagnostics diagnostics;
  portENTER_CRITICAL(&statusMux);
  diagnostics = g_cachedDiagnostics;
  portEXIT_CRITICAL(&statusMux);
  return diagnostics;
}

static void refreshStatusDiagnosticsIfDue(uint32_t nowMs) {
  uint32_t lastBuiltMs;
  portENTER_CRITICAL(&statusMux);
  lastBuiltMs = g_statusDiagnosticsBuiltMs;
  portEXIT_CRITICAL(&statusMux);

  if (lastBuiltMs != 0 && (nowMs - lastBuiltMs) < STATUS_DIAGNOSTICS_REFRESH_MS) {
    return;
  }

  CachedDiagnostics diagnostics;
  TaskHandle_t webTaskHandle = appGetWebServerTaskHandle();
  diagnostics.boardTempC = appReadBoardTemperatureC();
  if (controlTaskHandle) {
    diagnostics.controlTaskState = taskStateToString(eTaskGetState(controlTaskHandle));
    diagnostics.controlTaskStackHwm = (int32_t)uxTaskGetStackHighWaterMark(controlTaskHandle);
  }
  if (motorTaskHandle) {
    diagnostics.motorTaskState = taskStateToString(eTaskGetState(motorTaskHandle));
    diagnostics.motorTaskStackHwm = (int32_t)uxTaskGetStackHighWaterMark(motorTaskHandle);
  }
  diagnostics.webTaskState = webTaskHandle ? taskStateToString(eTaskGetState(webTaskHandle)) : "unknown";
  diagnostics.webTaskStackHwm = webTaskHandle ? (int32_t)uxTaskGetStackHighWaterMark(webTaskHandle) : -1;

  portENTER_CRITICAL(&statusMux);
  g_cachedDiagnostics = diagnostics;
  g_statusDiagnosticsBuiltMs = nowMs;
  portEXIT_CRITICAL(&statusMux);
}

static void controlTask(void* parameter) {
  (void)parameter;
  for (;;) {
    uint32_t loopNowUs = micros();
    noteControlLoopTiming(g_artnetTiming, loopNowUs);
    uint32_t busyStartUs = micros();
    pollArtnet();
    recordTaskMetrics(g_controlTaskMetrics, statusMux, micros() - busyStartUs);
    logArtnetTimingWindowIfDue(millis());
    logArtnetHealthIfDue(millis());
    vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
  }
}

static StatusSnapshot readStatusSnapshot() {
  StatusSnapshot snapshot;
  portENTER_CRITICAL(&statusMux);
  snapshot.motorRaw = g_motorRaw;
  snapshot.motorStep = g_motorStep;
  snapshot.lastArtMs = appGetLastArtnetMs();
  snapshot.artDmxRxTotal = artDmxRxTotal;
  snapshot.artDmxUniverseMatchTotal = artDmxUniverseMatchTotal;
  snapshot.artDmxUniverseMismatchTotal = artDmxUniverseMismatchTotal;
  snapshot.artDmxInvalidTotal = artDmxInvalidTotal;
  snapshot.artDmxLastArrivalMs = artDmxLastArrivalMs;
  snapshot.controlTaskLastLoopMs = g_controlTaskMetrics.lastLoopMs;
  snapshot.motorTaskLastLoopMs = g_motorTaskMetrics.lastLoopMs;
  snapshot.controlTaskUtilPermille = g_controlTaskMetrics.utilPermille;
  snapshot.motorTaskUtilPermille = g_motorTaskMetrics.utilPermille;
  portEXIT_CRITICAL(&statusMux);
  return snapshot;
}

static size_t buildStatusJson(char* out, size_t outSize, bool details) {
  StaticJsonDocument<1024> j;
  StatusSnapshot s = readStatusSnapshot();
  uint32_t now = millis();
  uint32_t artAge = (s.lastArtMs == 0) ? 0xFFFFFFFFu : (now - s.lastArtMs);
  AppTaskRuntimeStats webTaskStats = appGetWebTaskRuntimeStats();
  WifiManagerClass& wifiManager = appWifiManager();
  CachedDiagnostics diagnostics = readCachedDiagnostics();
  IPAddress ip = wifiManager.getIP();
  char ipBuf[16];
  char macBuf[18];
  snprintf(ipBuf, sizeof(ipBuf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  wifiManager.getMacAddress(macBuf, sizeof(macBuf));

  j["source"] = (artAge <= SOURCE_HOLD_MS) ? "Art-Net" : "none";
  j["art_age_ms"] = (artAge == 0xFFFFFFFFu) ? -1 : (int32_t)artAge;
  j["motor_value"] = s.motorRaw;
  j["motor_step"] = s.motorStep;
  j["wifi_rssi"] = wifiManager.getRSSI();
  j["hostname"] = wifiManager.getHostnameCStr();
  j["wifi_ip"] = ipBuf;
  j["wifi_mac"] = macBuf;
  j["free_heap"] = ESP.getFreeHeap();
  j["min_free_heap"] = ESP.getMinFreeHeap();
  j["board_temp_c"] = diagnostics.boardTempC;
  j["reset_reason"] = appResetReasonToString(esp_reset_reason());
  j["task_control_state"] = diagnostics.controlTaskState;
  j["task_control_stack_hwm"] = diagnostics.controlTaskStackHwm;
  j["task_control_last_ms_ago"] = (s.controlTaskLastLoopMs == 0) ? -1 : (int32_t)(now - s.controlTaskLastLoopMs);
  j["task_control_util_pct"] = (float)s.controlTaskUtilPermille / 10.0f;
  j["task_motor_state"] = diagnostics.motorTaskState;
  j["task_motor_stack_hwm"] = diagnostics.motorTaskStackHwm;
  j["task_motor_last_ms_ago"] = (s.motorTaskLastLoopMs == 0) ? -1 : (int32_t)(now - s.motorTaskLastLoopMs);
  j["task_motor_util_pct"] = (float)s.motorTaskUtilPermille / 10.0f;
  j["task_web_state"] = diagnostics.webTaskState;
  j["task_web_stack_hwm"] = diagnostics.webTaskStackHwm;
  j["task_web_last_ms_ago"] = (webTaskStats.lastActiveMs == 0) ? -1 : (int32_t)(now - webTaskStats.lastActiveMs);
  j["task_web_util_pct"] = (float)webTaskStats.utilPermille / 10.0f;

  if (details) {
    j["art_rx_total"] = s.artDmxRxTotal;
    j["art_rx_universe_total"] = s.artDmxUniverseMatchTotal;
    j["art_rx_universe_mismatch_total"] = s.artDmxUniverseMismatchTotal;
    j["art_rx_invalid_total"] = s.artDmxInvalidTotal;
    j["art_rx_last_ms_ago"] = (s.artDmxLastArrivalMs == 0) ? -1 : (int32_t)(now - s.artDmxLastArrivalMs);
  }

  return serializeJson(j, out, outSize);
}

static size_t buildHealthSummary(char* out, size_t outSize) {
  refreshStatusDiagnosticsIfDue(millis());
  StatusSnapshot s = readStatusSnapshot();
  uint32_t now = millis();
  int32_t lastMsAgo = (s.artDmxLastArrivalMs == 0) ? -1 : (int32_t)(now - s.artDmxLastArrivalMs);
  uint32_t artAge = (s.lastArtMs == 0) ? 0xFFFFFFFFu : (now - s.lastArtMs);
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
                  "art_src=%s art_rx=%lu match=%lu mismatch=%lu invalid=%lu last=%ldms udp_rx=%lu udp_b=%lu udp_last=%ldms from=%s:%u pkt=%u uni=%u rebind=%lu motor=%u",
                  (artAge <= SOURCE_HOLD_MS) ? "live" : "idle",
                  (unsigned long)s.artDmxRxTotal,
                  (unsigned long)s.artDmxUniverseMatchTotal,
                  (unsigned long)s.artDmxUniverseMismatchTotal,
                  (unsigned long)s.artDmxInvalidTotal,
                  (long)lastMsAgo,
                  (unsigned long)udpRx,
                  (unsigned long)udpBytes,
                  (long)udpLastMsAgo,
                  lastIp.toString().c_str(),
                  (unsigned int)lastPort,
                  (unsigned int)lastSize,
                  (unsigned int)lastUni,
                  (unsigned long)udpRebinds,
                  (unsigned int)s.motorRaw);
}

void setup() {
  appInitializeBaseRuntime();

  if (UART_OE >= 0) {
    pinMode(UART_OE, OUTPUT);
    digitalWrite(UART_OE, LOW);
  }

  g_cfg.dmxStartAddress = appConfig().getDMXAddress();
  g_cfg.artnetUniverse = appConfig().getDMXUniverse();
  g_cfg.startValue = appConfig().getStartValue();

  bldc.begin();
  applyStartValue(g_cfg.startValue);
  refreshStatusDiagnosticsIfDue(millis());

  appInitRuntime({
    "/wifi-manager/index.html",
    "CableCar BLDC",
    buildStatusJson,
    buildHealthSummary,
    nullptr,
  });

  appConnectWifi();
  appStartCommonServices();

  motorQueue = xQueueCreate(1, sizeof(MotorCmd));
  // Keep application work off core 0 so the ESP32 WiFi/lwIP stack can service
  // HTTP and UDP traffic without competing with our control loop.
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 4096, nullptr, 2, &motorTaskHandle, 1);
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, nullptr, 1, &controlTaskHandle, 1);
}

void loop() {
  refreshStatusDiagnosticsIfDue(millis());
  appCommonLoop(WS_STATUS_PUSH_MS);
  delay(1);
}
