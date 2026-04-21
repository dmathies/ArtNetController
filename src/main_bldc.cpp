#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFiUdp.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#include "main_common.h"
#include "bldc_uart.h"

#ifndef BLDC_UART_TX_PIN
#define BLDC_UART_TX_PIN 33
#endif

#ifndef BLDC_UART_RX_PIN
#define BLDC_UART_RX_PIN 32
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
static constexpr uint32_t MAX_ARTNET_PACKETS_PER_LOOP = 64;
static constexpr uint32_t ARTNET_DRAIN_BUDGET_MS = 8;

#ifndef ARTNET_TIMING_LOG_ENABLE
#define ARTNET_TIMING_LOG_ENABLE 0
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

static TaskHandle_t motorTaskHandle = nullptr;
static TaskHandle_t controlTaskHandle = nullptr;
static QueueHandle_t motorQueue = nullptr;

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

  bldc.onFrame = [](const uint8_t* f) {
    Serial.printf("[BLDC RX] %02X %02X %02X %02X %02X %02X %02X\n",
                  f[0], f[1], f[2], f[3], f[4], f[5], f[6]);
  };

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
    recordTaskMetrics(g_motorTaskMetrics, workUs);
  }
}

static void processArtnet(const ArtDmxPacket& a) {
  portENTER_CRITICAL(&statusMux);
  artDmxRxTotal++;
  artLastUniverseFlat = a.universe_flat;
  portEXIT_CRITICAL(&statusMux);

  uint16_t universe = g_cfg.artnetUniverse;
  if (a.universe_flat != universe) {
    portENTER_CRITICAL(&statusMux);
    artDmxUniverseMismatchTotal++;
    portEXIT_CRITICAL(&statusMux);
    return;
  }

  uint16_t channel = g_cfg.dmxStartAddress;
  if (channel < 1 || channel > a.length) return;

  uint8_t raw = a.data[channel - 1];
  enqueueMotor(raw);
  appMarkArtnetActivity();

  portENTER_CRITICAL(&statusMux);
  artDmxUniverseMatchTotal++;
  artDmxLastArrivalMs = millis();
  portEXIT_CRITICAL(&statusMux);
}

static void pollArtnet() {
  static uint8_t artbuf[600];
  static uint32_t lastRebindAttemptMs = 0;
  static constexpr uint32_t ARTNET_IDLE_REBIND_MS = 10000;
  static constexpr uint32_t ARTNET_REBIND_GUARD_MS = 3000;
  WiFiUDP& artudp = appArtnetUdp();

  uint32_t now = millis();
  uint32_t lastUdpMsSnapshot;
  portENTER_CRITICAL(&statusMux);
  lastUdpMsSnapshot = artUdpLastPacketMs;
  portEXIT_CRITICAL(&statusMux);

  if (lastUdpMsSnapshot > 0 &&
      (now - lastUdpMsSnapshot) >= ARTNET_IDLE_REBIND_MS &&
      (now - lastRebindAttemptMs) >= ARTNET_REBIND_GUARD_MS) {
    artudp.stop();
    if (artudp.begin(6454)) {
      portENTER_CRITICAL(&statusMux);
      artUdpRebindTotal++;
      portEXIT_CRITICAL(&statusMux);
      Serial.println("[ARTNET] UDP rebind on idle timeout");
    }
    lastRebindAttemptMs = now;
  }

  uint32_t startMs = millis();
  uint32_t processed = 0;
  int psize = artudp.parsePacket();
  while (psize > 0 && processed < MAX_ARTNET_PACKETS_PER_LOOP) {
    uint32_t packetNowUs = micros();
    IPAddress remoteIp = artudp.remoteIP();
    uint16_t remotePort = artudp.remotePort();

    noteArtnetPacketTiming(packetNowUs);
    noteArtnetSource(remoteIp, remotePort);

    portENTER_CRITICAL(&statusMux);
    artUdpRxTotal++;
    artUdpRxBytesTotal += (uint32_t)psize;
    artUdpLastPacketMs = millis();
    artUdpLastPacketSize = (uint16_t)psize;
    artUdpLastRemoteIp = remoteIp;
    artUdpLastRemotePort = remotePort;
    portEXIT_CRITICAL(&statusMux);

    int readLen = psize;
    if (readLen > (int)sizeof(artbuf)) readLen = sizeof(artbuf);

    int n = artudp.read(artbuf, readLen);
    if (n > 0) {
      noteArtnetSequence(artbuf, n);
      ArtDmxPacket a = appParseArtDmx(artbuf, n);
      if (a.ok) {
        processArtnet(a);
      } else {
        portENTER_CRITICAL(&statusMux);
        artDmxInvalidTotal++;
        portEXIT_CRITICAL(&statusMux);
      }
    }
    psize = artudp.parsePacket();
    processed++;

    if ((millis() - startMs) >= ARTNET_DRAIN_BUDGET_MS) {
      break;
    }
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
  j["motor_value"] = s.motorRaw;
  j["motor_step"] = s.motorStep;
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
  j["task_control_last_ms_ago"] = (s.controlTaskLastLoopMs == 0) ? -1 : (int32_t)(now - s.controlTaskLastLoopMs);
  j["task_control_util_pct"] = (float)s.controlTaskUtilPermille / 10.0f;
  j["task_motor_state"] = taskStateToString(eTaskGetState(motorTaskHandle));
  j["task_motor_stack_hwm"] = (int32_t)uxTaskGetStackHighWaterMark(motorTaskHandle);
  j["task_motor_last_ms_ago"] = (s.motorTaskLastLoopMs == 0) ? -1 : (int32_t)(now - s.motorTaskLastLoopMs);
  j["task_motor_util_pct"] = (float)s.motorTaskUtilPermille / 10.0f;
  j["task_web_state"] = webTaskHandle ? taskStateToString(eTaskGetState(webTaskHandle)) : "unknown";
  j["task_web_stack_hwm"] = webTaskHandle ? (int32_t)uxTaskGetStackHighWaterMark(webTaskHandle) : -1;
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

  if (UART_OE >= 0) {
    pinMode(UART_OE, OUTPUT);
    digitalWrite(UART_OE, LOW);
  }

  g_cfg.dmxStartAddress = appConfig().getDMXAddress();
  g_cfg.artnetUniverse = appConfig().getDMXUniverse();
  g_cfg.startValue = appConfig().getStartValue();

  bldc.begin();
  applyStartValue(g_cfg.startValue);

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
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 4096, nullptr, 2, &motorTaskHandle, 1);
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, nullptr, 1, &controlTaskHandle, 0);
}

void loop() {
  appCommonLoop(WS_STATUS_PUSH_MS);
  delay(1);
}
