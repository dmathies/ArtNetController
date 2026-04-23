#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <AsyncUDP.h>
#include <ArduinoJson.h>

#include "main_common.h"
#include "runtime_metrics.h"

#ifndef LED_PWM_PIN_1
#define LED_PWM_PIN_1 1
#endif

#ifndef LED_PWM_PIN_2
#define LED_PWM_PIN_2 2
#endif

#ifndef LED_PWM_PIN_3
#define LED_PWM_PIN_3 3
#endif

#ifndef LED_PWM_PIN_4
#define LED_PWM_PIN_4 21
#endif

static constexpr int CH1 = LED_PWM_PIN_1;
static constexpr int CH2 = LED_PWM_PIN_2;
static constexpr int CH3 = LED_PWM_PIN_3;
static constexpr int CH4 = LED_PWM_PIN_4;

static constexpr int LEDC_RES_BITS = 12;
static constexpr int LEDC_MAX_DUTY = (1 << LEDC_RES_BITS) - 1;
static constexpr int LEDC_FREQ_HZ = 1000;

static constexpr uint32_t CONTROL_TASK_DELAY_MS = 2;
static constexpr uint32_t CONTROL_TASK_IDLE_WAIT_MS = 20;
static constexpr uint32_t SOURCE_HOLD_MS = 1000;
static constexpr uint32_t WS_STATUS_PUSH_MS = 1000;
static constexpr UBaseType_t CONTROL_TASK_PRIORITY = 2;
static constexpr BaseType_t CONTROL_TASK_CORE = 1;

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
  appInitializeBaseRuntime();

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

  xTaskCreatePinnedToCore(controlTask,
                          "ControlTask",
                          4096,
                          nullptr,
                          CONTROL_TASK_PRIORITY,
                          &controlTaskHandle,
                          CONTROL_TASK_CORE);

}

void loop() {
  appCommonLoop(WS_STATUS_PUSH_MS);
  delay(1);
}
