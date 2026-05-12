#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoJson.h>

#include "main_common.h"
#include "RemoteLogBuffer.h"
#include "VariantRuntimeCommon.h"

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
static constexpr uint32_t STATUS_DIAGNOSTICS_REFRESH_MS = 5000;

#ifndef ARTNET_TIMING_LOG_ENABLE
#define ARTNET_TIMING_LOG_ENABLE 0
#endif

static constexpr uint32_t ARTNET_TIMING_LOG_WINDOW_MS = 3000;
static constexpr uint32_t ARTNET_INTERVAL_WARN_US = 40000;
static constexpr uint32_t ARTNET_INTERVAL_FREEZE_US = 100000;

static TaskHandle_t controlTaskHandle = nullptr;
static portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_havePendingLedValues = false;
static uint16_t g_pendingLedValues[4] = {0, 0, 0, 0};

static float currentValues[4] = {0, 0, 0, 0};

struct StatusSnapshot {
  AppStatusSnapshotBase base;
  float ledValues[4];
};

static AppVariantSharedRuntime g_shared;

static void setLedFloat(int ledcChannel, float brightness01) {
  if (brightness01 < 0.0f) brightness01 = 0.0f;
  if (brightness01 > 1.0f) brightness01 = 1.0f;

  int duty = (int)(brightness01 * LEDC_MAX_DUTY + 0.5f);
  ledcWrite(ledcChannel, duty);

  portENTER_CRITICAL(&statusMux);
  currentValues[ledcChannel] = brightness01;
  portEXIT_CRITICAL(&statusMux);
}

static uint16_t ledFloatToNormalized(float value) {
  if (value < 0.0f) value = 0.0f;
  if (value > 1.0f) value = 1.0f;
  return (uint16_t)(value * 1000.0f + 0.5f);
}

static void publishVariantStatus() {
  AppVariantStatus status;
  status.variant = AppVariantKind::Led;
  status.updatedMs = millis();
  portENTER_CRITICAL(&statusMux);
  for (int i = 0; i < 4; ++i) status.ledValues[i] = currentValues[i];
  portEXIT_CRITICAL(&statusMux);
  status.normalizedValue = ledFloatToNormalized(status.ledValues[0]);
  appSetVariantStatus(status);
}

static void applyStartValue(float value) {
  for (int i = 0; i < 4; i++) {
    setLedFloat(i, value);
  }
  publishVariantStatus();
}

static bool consumeDmxPayload(void* ctx, const ArtDmxPacket& packet, uint16_t startAddress, uint32_t nowMs) {
  (void)ctx;
  (void)nowMs;
  if (startAddress < 1 || startAddress + 7 > packet.length) {
    return false;
  }

  uint16_t addr = startAddress - 1;
  g_pendingLedValues[0] = ((uint16_t)packet.data[addr] << 8) | packet.data[addr + 1];
  g_pendingLedValues[1] = ((uint16_t)packet.data[addr + 2] << 8) | packet.data[addr + 3];
  g_pendingLedValues[2] = ((uint16_t)packet.data[addr + 4] << 8) | packet.data[addr + 5];
  g_pendingLedValues[3] = ((uint16_t)packet.data[addr + 6] << 8) | packet.data[addr + 7];
  g_havePendingLedValues = true;
  return true;
}

static void pollArtnet() {
  appEnsureArtnetListener(g_shared);

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
    publishVariantStatus();
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
  for (int i = 0; i < 4; i++) snapshot.ledValues[i] = currentValues[i];
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
  j["led_value0"] = snapshot.ledValues[0];
  j["led_value1"] = snapshot.ledValues[1];
  j["led_value2"] = snapshot.ledValues[2];
  j["led_value3"] = snapshot.ledValues[3];
  appAppendArtnetDetailFields(j, g_shared.art, now, details);

  return serializeJson(j, out, outSize);
}

static size_t buildHealthSummary(char* out, size_t outSize) {
  StatusSnapshot snapshot = readStatusSnapshot();
  size_t used = appFormatCommonHealthPrefix(out, outSize, SOURCE_HOLD_MS, snapshot.base.lastArtMs, g_shared.art);
  if (used >= outSize) return used;
  return used + snprintf(out + used, outSize - used, " led0=%.3f", snapshot.ledValues[0]);
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
  appInitRuntime({
    "/wifi-manager/index.html",
    "ArtNetController LED",
    buildStatusJson,
    buildHealthSummary,
    nullptr,
    AppVariantKind::Led,
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
