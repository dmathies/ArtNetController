#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoJson.h>

#include "main_common.h"
#include "RemoteLogBuffer.h"
#include "VariantRuntimeCommon.h"

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
static constexpr uint32_t WS_STATUS_PUSH_MS = 1000;
static constexpr UBaseType_t CONTROL_TASK_PRIORITY = 2;
static constexpr BaseType_t CONTROL_TASK_CORE = 1;
static constexpr uint32_t STATUS_DIAGNOSTICS_REFRESH_MS = 5000;

#ifndef ARTNET_TIMING_LOG_ENABLE
#define ARTNET_TIMING_LOG_ENABLE 0
#endif

static constexpr uint32_t ARTNET_TIMING_LOG_WINDOW_MS = 3000;
static constexpr uint32_t ARTNET_INTERVAL_WARN_US = 40000;
static constexpr uint32_t ARTNET_INTERVAL_FREEZE_US = 100000;

struct StatusSnapshot {
  AppStatusSnapshotBase base;
  uint8_t dmxValue = 0;
  bool relayOn = false;
};

static TaskHandle_t controlTaskHandle = nullptr;
static portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_havePendingRelayValue = false;
static uint8_t g_pendingRelayValue = 0;

static uint8_t g_dmxValue = 0;
static bool g_relayOn = false;
static uint32_t g_lastRelaySwitchMs = 0;
static AppVariantSharedRuntime g_shared;

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

static uint16_t rawToNormalized(uint8_t raw) {
  return relayRawToState(raw) ? 1000 : 0;
}

static void publishVariantStatus() {
  AppVariantStatus status;
  status.variant = AppVariantKind::Relay;
  status.updatedMs = millis();
  portENTER_CRITICAL(&statusMux);
  status.relayOn = g_relayOn;
  status.normalizedValue = rawToNormalized(g_dmxValue);
  portEXIT_CRITICAL(&statusMux);
  appSetVariantStatus(status);
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
    publishVariantStatus();
    return;
  }

  if ((now - g_lastRelaySwitchMs) < RELAY_SWITCH_MIN_INTERVAL_MS) {
    publishVariantStatus();
    return;
  }

  writeRelayOutput(targetState);
  portENTER_CRITICAL(&statusMux);
  g_lastRelaySwitchMs = now;
  portEXIT_CRITICAL(&statusMux);
  publishVariantStatus();
}

static void applyStartValue(float value) {
  applyRelayValue(clampStartRaw(value));
}

static bool consumeDmxPayload(void* ctx, const ArtDmxPacket& packet, uint16_t startAddress, uint32_t nowMs) {
  (void)ctx;
  (void)nowMs;
  if (startAddress < 1 || startAddress > packet.length) {
    return false;
  }

  g_pendingRelayValue = packet.data[startAddress - 1];
  g_havePendingRelayValue = true;
  return true;
}

static void pollArtnet() {
  appEnsureArtnetListener(g_shared);

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
    noteControlLoopTiming(g_shared.artnetTiming, loopNowUs);
    uint32_t busyStartUs = micros();
    pollArtnet();
    recordTaskMetrics(g_shared.controlTaskMetrics, statusMux, micros() - busyStartUs);
    appLogArtnetTimingWindowIfDue(g_shared.artnetTiming,
                                  statusMux,
                                  g_shared.controlTaskMetrics,
                                  nullptr,
                                  millis(),
                                  ARTNET_TIMING_LOG_WINDOW_MS,
                                  ARTNET_TIMING_LOG_ENABLE != 0);
    vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
  }
}

static StatusSnapshot readStatusSnapshot() {
  StatusSnapshot snapshot;
  portENTER_CRITICAL(&statusMux);
  snapshot.dmxValue = g_dmxValue;
  snapshot.relayOn = g_relayOn;
  appFillStatusSnapshotBase(snapshot.base, g_shared, appGetLastArtnetMs());
  portEXIT_CRITICAL(&statusMux);
  return snapshot;
}

static size_t buildStatusJson(char* out, size_t outSize, bool details) {
  StaticJsonDocument<1024> j;
  appRefreshTaskDiagnosticsIfDue(g_shared.diagnostics,
                                 g_shared.statusDiagnosticsBuiltMs,
                                 statusMux,
                                 millis(),
                                 STATUS_DIAGNOSTICS_REFRESH_MS,
                                 controlTaskHandle,
                                 nullptr);
  StatusSnapshot snapshot = readStatusSnapshot();
  uint32_t now = millis();
  AppTaskDiagnosticsCache diagnostics = appReadTaskDiagnostics(g_shared.diagnostics, statusMux);
  appAppendCommonStatusFields(j,
                              now,
                              SOURCE_HOLD_MS,
                              snapshot.base.lastArtMs,
                              diagnostics,
                              snapshot.base.controlTaskLastLoopMs,
                              snapshot.base.controlTaskUtilPermille,
                              nullptr,
                              nullptr);
  j["relay_value"] = snapshot.dmxValue;
  j["relay_on"] = snapshot.relayOn;
  j["relay_state"] = snapshot.relayOn ? "ON" : "OFF";
  j["relay_threshold"] = RELAY_ON_THRESHOLD;
  appAppendArtnetDetailFields(j, g_shared.art, now, details);

  return serializeJson(j, out, outSize);
}

static size_t buildHealthSummary(char* out, size_t outSize) {
  StatusSnapshot snapshot = readStatusSnapshot();
  size_t used = appFormatCommonHealthPrefix(out, outSize, SOURCE_HOLD_MS, snapshot.base.lastArtMs, g_shared.art);
  if (used >= outSize) return used;
  return used + snprintf(out + used,
                         outSize - used,
                         " relay=%s dmx=%u",
                         snapshot.relayOn ? "on" : "off",
                         (unsigned int)snapshot.dmxValue);
}

void setup() {
  appInitializeBaseRuntime();

  pinMode(RELAY_PIN, OUTPUT);

  g_shared.cfg.dmxStartAddress = appConfig().getDMXAddress();
  g_shared.cfg.artnetUniverse = appConfig().getDMXUniverse();
  g_shared.cfg.startValue = appConfig().getStartValue();
  appConfigureArtnetListener(g_shared,
                             statusMux,
                             &controlTaskHandle,
                             consumeDmxPayload,
                             nullptr,
                             ARTNET_TIMING_LOG_ENABLE != 0,
                             ARTNET_INTERVAL_WARN_US,
                             ARTNET_INTERVAL_FREEZE_US);
  g_lastRelaySwitchMs = millis() - RELAY_SWITCH_MIN_INTERVAL_MS;
  appInitRuntime({
    "/wifi-manager/index.html",
    "ArtNetController Relay",
    buildStatusJson,
    buildHealthSummary,
    nullptr,
    AppVariantKind::Relay,
    applyStartValue,
  });

  appApplyVariantStartValue(g_shared.cfg.startValue);

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
  appCommonLoop(WS_STATUS_PUSH_MS);
  delay(1);
}
