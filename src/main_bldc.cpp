#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoJson.h>

#include "main_common.h"
#include "bldc_uart.h"
#include "RemoteLogBuffer.h"
#include "VariantRuntimeCommon.h"

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
static constexpr uint32_t BLDC_POWER_PRESENT_TIMEOUT_MS = 2000;
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

struct StatusSnapshot {
  AppStatusSnapshotBase base;
  uint8_t motorRaw = 0;
  uint8_t motorStep = 0;
  uint32_t bldcLastStatusFrameMs = 0;
  uint32_t bldcStatusFrameRxTotal = 0;
};

static TaskHandle_t motorTaskHandle = nullptr;
static TaskHandle_t controlTaskHandle = nullptr;
static QueueHandle_t motorQueue = nullptr;
static bool g_havePendingMotorRaw = false;
static uint8_t g_pendingMotorRaw = 0;

static portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t g_motorRaw = 0;
static uint8_t g_motorStep = 0;
static uint32_t g_bldcLastStatusFrameMs = 0;
static uint32_t g_bldcStatusFrameRxTotal = 0;
static AppVariantSharedRuntime g_shared;

static bool consumeDmxPayload(void* ctx, const ArtDmxPacket& packet, uint16_t startAddress, uint32_t nowMs) {
  (void)ctx;
  (void)nowMs;
  if (startAddress < 1 || startAddress > packet.length) {
    return false;
  }

  g_pendingMotorRaw = packet.data[startAddress - 1];
  g_havePendingMotorRaw = true;
  return true;
}

static void logArtnetHealthIfDue(uint32_t nowMs) {
  AppArtnetHealthSample sample;
  if (!appBuildArtnetHealthSampleIfDue(g_shared,
                                       statusMux,
                                       nowMs,
                                       ARTNET_HEALTH_LOG_INTERVAL_MS,
                                       ARTNET_HEALTH_LOG_ENABLE != 0,
                                       sample)) {
    return;
  }

  uint8_t motorRaw;
  portENTER_CRITICAL(&statusMux);
  motorRaw = g_motorRaw;
  portEXIT_CRITICAL(&statusMux);

  Serial.printf("[ARTHEALTH] up=%lu udp_last=%ldms art_last=%ldms udp_rx=%lu dmx=%lu match=%lu mismatch=%lu invalid=%lu bytes=%lu from=%s:%u pkt=%u uni=%u rebind=%lu motor=%u ctrl=%.1f%%/%ldms motor=%.1f%%/%ldms heap=%lu\n",
                (unsigned long)sample.nowMs,
                (long)(sample.udpLastMsAgo == 0xFFFFFFFFu ? -1 : (int32_t)sample.udpLastMsAgo),
                (long)(sample.artLastMsAgo == 0xFFFFFFFFu ? -1 : (int32_t)sample.artLastMsAgo),
                (unsigned long)sample.udpRx,
                (unsigned long)sample.dmxRx,
                (unsigned long)sample.dmxMatch,
                (unsigned long)sample.dmxMismatch,
                (unsigned long)sample.dmxInvalid,
                (unsigned long)sample.udpBytes,
                sample.lastIp.toString().c_str(),
                (unsigned int)sample.lastPort,
                (unsigned int)sample.lastSize,
                (unsigned int)sample.lastUni,
                (unsigned long)sample.udpRebinds,
                (unsigned int)motorRaw,
                (float)sample.ctrlUtilPermille / 10.0f,
                (long)(sample.ctrlLastMs == 0 ? -1 : (int32_t)(sample.nowMs - sample.ctrlLastMs)),
                (float)sample.motorUtilPermille / 10.0f,
                (long)(sample.motorLastMs == 0 ? -1 : (int32_t)(sample.nowMs - sample.motorLastMs)),
                (unsigned long)ESP.getFreeHeap());
}

static inline uint8_t rawToStep(uint8_t raw) {
  return (uint8_t)((((uint16_t)raw) * 60U + 127U) / 255U);
}

static uint16_t rawToNormalized(uint8_t raw) {
  return (uint16_t)(((uint32_t)raw * 1000U + 127U) / 255U);
}

static void publishVariantStatus() {
  AppVariantStatus status;
  uint32_t nowMs = millis();
  uint32_t lastStatusMs = 0;

  portENTER_CRITICAL(&statusMux);
  status.variant = AppVariantKind::Bldc;
  status.updatedMs = nowMs;
  status.motorValue = g_motorRaw;
  status.motorStep = g_motorStep;
  status.normalizedValue = rawToNormalized(g_motorRaw);
  lastStatusMs = g_bldcLastStatusFrameMs;
  portEXIT_CRITICAL(&statusMux);

  status.controllerPowerOn =
      (lastStatusMs != 0) && ((nowMs - lastStatusMs) <= BLDC_POWER_PRESENT_TIMEOUT_MS);
  status.controllerLastStatusMsAgo =
      (lastStatusMs == 0) ? -1 : (int32_t)(nowMs - lastStatusMs);
  appSetVariantStatus(status);
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
  publishVariantStatus();
}

static void enqueueMotor(uint8_t raw) {
  if (!motorQueue) return;
  MotorCmd cmd{raw};
  xQueueOverwrite(motorQueue, &cmd);
}

static void motorTask(void* parameter) {
  uint8_t currentRaw = 0;

  bldc.onFrame = [](const uint8_t* f) {
    const uint32_t nowMs = millis();
    portENTER_CRITICAL(&statusMux);
    g_bldcLastStatusFrameMs = nowMs;
    g_bldcStatusFrameRxTotal++;
    portEXIT_CRITICAL(&statusMux);
    publishVariantStatus();
#if BLDC_RX_LOG_ENABLE
    Serial.printf("[BLDC RX] %02X %02X %02X %02X %02X %02X %02X\n",
                  f[0], f[1], f[2], f[3], f[4], f[5], f[6]);
#endif
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
      publishVariantStatus();
      workUs += micros() - t0;
    }

    uint32_t t1 = micros();
    bldc.poll();
    workUs += micros() - t1;

    // Queue wait time is idle time and must not be counted as task busy load.
    recordTaskMetrics(g_shared.motorTaskMetrics, statusMux, workUs);
  }
}

static void pollArtnet() {
  appEnsureArtnetListener(g_shared);

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

static void controlTask(void* parameter) {
  (void)parameter;
  for (;;) {
    uint32_t loopNowUs = micros();
    noteControlLoopTiming(g_shared.artnetTiming, loopNowUs);
    uint32_t busyStartUs = micros();
    pollArtnet();
    publishVariantStatus();
    recordTaskMetrics(g_shared.controlTaskMetrics, statusMux, micros() - busyStartUs);
    appLogArtnetTimingWindowIfDue(g_shared.artnetTiming,
                                  statusMux,
                                  g_shared.controlTaskMetrics,
                                  &g_shared.motorTaskMetrics,
                                  millis(),
                                  ARTNET_TIMING_LOG_WINDOW_MS,
                                  ARTNET_TIMING_LOG_ENABLE != 0);
    logArtnetHealthIfDue(millis());
    vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
  }
}

static StatusSnapshot readStatusSnapshot() {
  StatusSnapshot snapshot;
  portENTER_CRITICAL(&statusMux);
  snapshot.motorRaw = g_motorRaw;
  snapshot.motorStep = g_motorStep;
  snapshot.bldcLastStatusFrameMs = g_bldcLastStatusFrameMs;
  snapshot.bldcStatusFrameRxTotal = g_bldcStatusFrameRxTotal;
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
                                 motorTaskHandle);
  StatusSnapshot s = readStatusSnapshot();
  uint32_t now = millis();
  AppTaskDiagnosticsCache diagnostics = appReadTaskDiagnostics(g_shared.diagnostics, statusMux);
  const bool controllerPowerOn =
      (s.bldcLastStatusFrameMs != 0) && ((now - s.bldcLastStatusFrameMs) <= BLDC_POWER_PRESENT_TIMEOUT_MS);
  const int32_t controllerLastStatusMsAgo =
      (s.bldcLastStatusFrameMs == 0) ? -1 : (int32_t)(now - s.bldcLastStatusFrameMs);
  appAppendCommonStatusFields(j,
                              now,
                              SOURCE_HOLD_MS,
                              s.base.lastArtMs,
                              diagnostics,
                              s.base.controlTaskLastLoopMs,
                              s.base.controlTaskUtilPermille,
                              &s.base.motorTaskLastLoopMs,
                              &s.base.motorTaskUtilPermille);
  j["motor_value"] = s.motorRaw;
  j["motor_step"] = s.motorStep;
  j["controller_power_on"] = controllerPowerOn;
  j["controller_last_status_ms_ago"] = controllerLastStatusMsAgo;
  j["controller_status_timeout_ms"] = BLDC_POWER_PRESENT_TIMEOUT_MS;
  if (details) j["controller_status_rx_total"] = s.bldcStatusFrameRxTotal;
  appAppendArtnetDetailFields(j, g_shared.art, now, details);

  return serializeJson(j, out, outSize);
}

static size_t buildHealthSummary(char* out, size_t outSize) {
  StatusSnapshot s = readStatusSnapshot();
  uint32_t now = millis();
  const bool controllerPowerOn =
      (s.bldcLastStatusFrameMs != 0) && ((now - s.bldcLastStatusFrameMs) <= BLDC_POWER_PRESENT_TIMEOUT_MS);
  const int32_t controllerLastStatusMsAgo =
      (s.bldcLastStatusFrameMs == 0) ? -1 : (int32_t)(now - s.bldcLastStatusFrameMs);
  size_t used = appFormatCommonHealthPrefix(out, outSize, SOURCE_HOLD_MS, s.base.lastArtMs, g_shared.art);
  if (used >= outSize) return used;
  return used + snprintf(out + used,
                         outSize - used,
                         " motor=%u ctrl_pwr=%s ctrl_rx_last=%ldms ctrl_rx_total=%lu",
                         (unsigned int)s.motorRaw,
                         controllerPowerOn ? "on" : "off",
                         (long)controllerLastStatusMsAgo,
                         (unsigned long)s.bldcStatusFrameRxTotal);
}

void setup() {
  appInitializeBaseRuntime();

  if (UART_OE >= 0) {
    pinMode(UART_OE, OUTPUT);
    digitalWrite(UART_OE, LOW);
  }

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

  bldc.begin();

  appInitRuntime({
    "/wifi-manager/index.html",
    "ArtNetController BLDC",
    buildStatusJson,
    buildHealthSummary,
    nullptr,
    AppVariantKind::Bldc,
    applyStartValue,
  });

  appApplyVariantStartValue(g_shared.cfg.startValue);
  appRefreshTaskDiagnosticsIfDue(g_shared.diagnostics,
                                 g_shared.statusDiagnosticsBuiltMs,
                                 statusMux,
                                 millis(),
                                 STATUS_DIAGNOSTICS_REFRESH_MS,
                                 controlTaskHandle,
                                 motorTaskHandle);

  appStartCommonServices();
  appConnectWifi();

  motorQueue = xQueueCreate(1, sizeof(MotorCmd));
  // Keep application work off core 0 so the ESP32 WiFi/lwIP stack can service
  // HTTP and UDP traffic without competing with our control loop.
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 4096, nullptr, 2, &motorTaskHandle, 1);
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 4096, nullptr, 1, &controlTaskHandle, 1);
}

void loop() {
  appRefreshTaskDiagnosticsIfDue(g_shared.diagnostics,
                                 g_shared.statusDiagnosticsBuiltMs,
                                 statusMux,
                                 millis(),
                                 STATUS_DIAGNOSTICS_REFRESH_MS,
                                 controlTaskHandle,
                                 motorTaskHandle);
  appCommonLoop(WS_STATUS_PUSH_MS);
  delay(1);
}
