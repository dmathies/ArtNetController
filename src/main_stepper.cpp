#include <Arduino.h>
#include <FastAccelStepper.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <cstdlib>
#include <cstring>

#include "main_common.h"
#include "RemoteLogBuffer.h"
#include "VariantRuntimeCommon.h"

#ifndef DIR_PIN
#define DIR_PIN D0
#endif

#ifndef STEP_PIN
#define STEP_PIN D1
#endif

#ifndef SLP_PIN
#define SLP_PIN D2
#endif

#ifndef RST_PIN
#define RST_PIN D3
#endif

#ifndef MS3_PIN
#define MS3_PIN D4
#endif

#ifndef MS2_PIN
#define MS2_PIN D5
#endif

#ifndef MS1_PIN
#define MS1_PIN D6
#endif

#ifndef EN_PIN
#define EN_PIN D7
#endif

#ifndef HOME_PIN
#define HOME_PIN D8
#endif

#ifndef HOME_PIN2
#define HOME_PIN2 D9
#endif

#ifndef MOTOR_FULL_STEPS_PER_REV
#define MOTOR_FULL_STEPS_PER_REV 400
#endif

static constexpr int DIR_GPIO = DIR_PIN;
static constexpr int STEP_GPIO = STEP_PIN;
static constexpr int SLP_GPIO = SLP_PIN;
static constexpr int RST_GPIO = RST_PIN;
static constexpr int MS3_GPIO = MS3_PIN;
static constexpr int MS2_GPIO = MS2_PIN;
static constexpr int MS1_GPIO = MS1_PIN;
static constexpr int EN_GPIO = EN_PIN;
static constexpr int HOME_GPIO = HOME_PIN;
static constexpr int HOME_GPIO2 = HOME_PIN2;

static constexpr uint8_t MICROSTEP_DIVISOR = 16;
static constexpr int32_t STEPS_PER_REV = MOTOR_FULL_STEPS_PER_REV * MICROSTEP_DIVISOR;
static constexpr uint32_t CONTROL_TASK_DELAY_MS = 2;
static constexpr uint32_t CONTROL_TASK_IDLE_WAIT_MS = 20;
static constexpr uint32_t SOURCE_HOLD_MS = 1000;
static constexpr uint32_t WS_STATUS_PUSH_MS = 1000;
static constexpr UBaseType_t CONTROL_TASK_PRIORITY = 2;
static constexpr BaseType_t CONTROL_TASK_CORE = 1;
static constexpr uint32_t STATUS_DIAGNOSTICS_REFRESH_MS = 5000;
static constexpr uint32_t FAST_SPEED_HZ = 19200;
static constexpr uint32_t FAST_ACCELERATION = 64000;
static constexpr uint32_t HOMING_SPEED_HZ = 2000;
static constexpr uint32_t HOMING_ACCELERATION = 8000;

#ifndef ARTNET_TIMING_LOG_ENABLE
#define ARTNET_TIMING_LOG_ENABLE 0
#endif

static constexpr uint32_t ARTNET_TIMING_LOG_WINDOW_MS = 3000;
static constexpr uint32_t ARTNET_INTERVAL_WARN_US = 40000;
static constexpr uint32_t ARTNET_INTERVAL_FREEZE_US = 100000;

enum class StepperMode : uint8_t {
  Idle = 0,
  Position = 1,
  Homing = 2,
};

struct StatusSnapshot {
  AppStatusSnapshotBase base;
  uint8_t angleRaw = 0;
  uint8_t controlRaw = 0;
  int32_t positionSteps = 0;
  int32_t targetSteps = 0;
  bool homed = false;
  bool homing = false;
  bool homeSensorActive = false;
};

static FastAccelStepperEngine g_engine;
static FastAccelStepper* g_stepper = nullptr;
static TaskHandle_t controlTaskHandle = nullptr;
static portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
static AppVariantSharedRuntime g_shared;

static bool g_havePendingCommand = false;
static uint8_t g_pendingAngleRaw = 0;
static uint8_t g_pendingControlRaw = 0;

static uint8_t g_angleRaw = 0;
static uint8_t g_controlRaw = 0;
static int32_t g_positionSteps = 0;
static int32_t g_targetSteps = 0;
static int32_t g_homingStartSteps = 0;
static bool g_homed = false;
static bool g_homing = false;
static StepperMode g_mode = StepperMode::Idle;
static bool g_holdIdle = false;
static uint32_t g_speedHz = FAST_SPEED_HZ;
static uint32_t g_accel = FAST_ACCELERATION;
static bool g_applyFastMotionSettingsPending = false;
static bool g_persistHoldPending = false;
static bool g_persistSpeedPending = false;
static bool g_persistAccelPending = false;
static bool g_motionCommandActive = false;
static bool g_configWriteActive = false;

// ESP-IDF 4.4 allocates shared-interrupt descriptors with malloc(). On boards
// with PSRAM, those descriptors can otherwise land in external RAM, which is
// inaccessible while a LittleFS/OTA flash operation has the cache disabled.
// Keep the temporary preference scoped to FastAccelStepper initialization so
// the rest of the application can continue using PSRAM normally.
class PreferInternalMallocScope {
 public:
  PreferInternalMallocScope() {
#if defined(BOARD_HAS_PSRAM) && CONFIG_SPIRAM_USE_MALLOC
    heap_caps_malloc_extmem_enable(SIZE_MAX);
#endif
  }

  ~PreferInternalMallocScope() {
#if defined(BOARD_HAS_PSRAM) && CONFIG_SPIRAM_USE_MALLOC
    heap_caps_malloc_extmem_enable(CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL);
#endif
  }
};

static void applyStepperHoldState(bool enabled) {
  g_holdIdle = enabled;
  if (!g_stepper) {
    return;
  }

  g_stepper->setAutoEnable(!enabled);
  if (enabled) {
    g_stepper->enableOutputs();
  } else {
    g_stepper->disableOutputs();
  }
}

static void queueStepperCommand(uint8_t angleRaw, uint8_t controlRaw) {
  portENTER_CRITICAL(&statusMux);
  g_pendingAngleRaw = angleRaw;
  g_pendingControlRaw = controlRaw;
  g_havePendingCommand = true;
  portEXIT_CRITICAL(&statusMux);

  if (controlTaskHandle) {
    xTaskNotifyGive(controlTaskHandle);
  }
}

static int32_t angleRawToSteps(uint8_t raw) {
  return (int32_t)(((uint32_t)raw * (uint32_t)STEPS_PER_REV + 127U) / 255U);
}

static uint16_t rawToNormalized(uint8_t raw) {
  return (uint16_t)(((uint32_t)raw * 1000U + 127U) / 255U);
}

static void configureMotionFast() {
  if (!g_stepper) {
    return;
  }

  uint32_t speedHz, accel;
  portENTER_CRITICAL(&statusMux);
  speedHz = g_speedHz;
  accel = g_accel;
  portEXIT_CRITICAL(&statusMux);

  g_stepper->setSpeedInHz(speedHz);
  g_stepper->setAcceleration(accel);
}

static void configureMotionHoming() {
  if (!g_stepper) {
    return;
  }

  g_stepper->setSpeedInHz(HOMING_SPEED_HZ);
  g_stepper->setAcceleration(HOMING_ACCELERATION);
}

static void applyPendingFastMotionSettings() {
  bool apply = false;

  portENTER_CRITICAL(&statusMux);
  if (!g_configWriteActive && g_applyFastMotionSettingsPending) {
    g_applyFastMotionSettingsPending = false;
    apply = (g_mode != StepperMode::Homing);
  }
  portEXIT_CRITICAL(&statusMux);

  if (apply) {
    configureMotionFast();
  }
}

// LittleFS operations disable the flash cache. FastAccelStepper's interrupt
// path must not be active while that happens, so persist runtime settings only
// after motion and command processing are both idle.
static void serviceDeferredConfigWrites() {
  bool persistHold;
  bool persistSpeed;
  bool persistAccel;
  bool holdIdle;
  uint32_t speedHz;
  uint32_t accel;

  portENTER_CRITICAL(&statusMux);
  bool haveWrites = g_persistHoldPending || g_persistSpeedPending || g_persistAccelPending;
  if (!haveWrites || g_motionCommandActive || g_havePendingCommand || g_configWriteActive) {
    portEXIT_CRITICAL(&statusMux);
    return;
  }
  g_configWriteActive = true;
  portEXIT_CRITICAL(&statusMux);

  if (g_stepper && g_stepper->isRunning()) {
    portENTER_CRITICAL(&statusMux);
    g_configWriteActive = false;
    portEXIT_CRITICAL(&statusMux);
    return;
  }

  portENTER_CRITICAL(&statusMux);
  persistHold = g_persistHoldPending;
  persistSpeed = g_persistSpeedPending;
  persistAccel = g_persistAccelPending;
  holdIdle = g_holdIdle;
  speedHz = g_speedHz;
  accel = g_accel;
  g_persistHoldPending = false;
  g_persistSpeedPending = false;
  g_persistAccelPending = false;
  portEXIT_CRITICAL(&statusMux);

  if (persistHold && !appConfig().writeStepperHoldIdle(holdIdle)) {
    Serial.println("Failed to persist hold setting");
  }
  if (persistSpeed && !appConfig().writeStepperSpeedHz(speedHz)) {
    Serial.println("Failed to persist speed setting");
  }
  if (persistAccel && !appConfig().writeStepperAccel(accel)) {
    Serial.println("Failed to persist accel setting");
  }

  portENTER_CRITICAL(&statusMux);
  g_configWriteActive = false;
  portEXIT_CRITICAL(&statusMux);
}

static void publishVariantStatus() {
  AppVariantStatus status;
  status.variant = AppVariantKind::Stepper;
  status.updatedMs = millis();

  portENTER_CRITICAL(&statusMux);
  status.motorValue = g_angleRaw;
  status.motorStep = (uint8_t)((g_positionSteps < 0) ? 0 : ((g_positionSteps > 255) ? 255 : g_positionSteps));
  status.normalizedValue = rawToNormalized(g_angleRaw);
  status.controllerPowerOn = g_homed;
  status.controllerLastStatusMsAgo = g_homing ? 0 : -1;
  portEXIT_CRITICAL(&statusMux);

  appSetVariantStatus(status);
}

static void applyStepperCommand(uint8_t angleRaw, uint8_t controlRaw) {
  bool homeRequested = controlRaw >= 128;
  bool homeSensorActive = digitalRead(HOME_GPIO) == HIGH;
  int32_t targetSteps = angleRawToSteps(angleRaw);
  int32_t currentPosition = g_stepper ? g_stepper->getCurrentPosition() : g_positionSteps;
  int32_t previousTargetSteps;
  StepperMode previousMode;
  bool previouslyHomed;

  portENTER_CRITICAL(&statusMux);
  previousTargetSteps = g_targetSteps;
  previousMode = g_mode;
  previouslyHomed = g_homed;
  g_angleRaw = angleRaw;
  g_controlRaw = controlRaw;
  g_positionSteps = currentPosition;
  g_homing = homeRequested && !homeSensorActive;
  g_homed = g_homed || (homeRequested && homeSensorActive);
  portEXIT_CRITICAL(&statusMux);

  if (!g_stepper) {
    publishVariantStatus();
    return;
  }

  if (homeRequested) {
    if (homeSensorActive) {
      if (previousMode == StepperMode::Homing || !previouslyHomed) {
        g_stepper->forceStopAndNewPosition(0);
        portENTER_CRITICAL(&statusMux);
        g_positionSteps = 0;
        g_targetSteps = 0;
        g_homed = true;
        g_homing = false;
        g_mode = StepperMode::Idle;
        portEXIT_CRITICAL(&statusMux);
        Serial.println("Homed");
      }
    } else if (previousMode != StepperMode::Homing) {
      configureMotionHoming();
      g_stepper->runBackward();
      portENTER_CRITICAL(&statusMux);
      g_targetSteps = targetSteps;
      g_homingStartSteps = currentPosition;
      g_homing = true;
      g_mode = StepperMode::Homing;
      portEXIT_CRITICAL(&statusMux);
    }
  } else {
    if (previousMode != StepperMode::Position || targetSteps != previousTargetSteps) {
      configureMotionFast();
      g_stepper->moveTo(targetSteps);
      portENTER_CRITICAL(&statusMux);
      g_targetSteps = targetSteps;
      g_homing = false;
      g_mode = StepperMode::Position;
      portEXIT_CRITICAL(&statusMux);
    }
  }

  portENTER_CRITICAL(&statusMux);
  g_positionSteps = g_stepper->getCurrentPosition();
  portEXIT_CRITICAL(&statusMux);
  publishVariantStatus();
}

static void serviceHomingWatchdog() {
  if (!g_stepper) {
    return;
  }

  StepperMode mode;
  int32_t homingStartSteps;
  portENTER_CRITICAL(&statusMux);
  mode = g_mode;
  homingStartSteps = g_homingStartSteps;
  portEXIT_CRITICAL(&statusMux);

  if (mode != StepperMode::Homing) {
    return;
  }

  bool homeSensorActive = digitalRead(HOME_GPIO) == HIGH;
  int32_t currentPosition = g_stepper->getCurrentPosition();

  if (homeSensorActive) {
    g_stepper->forceStopAndNewPosition(0);
    portENTER_CRITICAL(&statusMux);
    g_positionSteps = 0;
    g_targetSteps = 0;
    g_homed = true;
    g_homing = false;
    g_mode = StepperMode::Idle;
    portEXIT_CRITICAL(&statusMux);
    Serial.println("Homed");
    publishVariantStatus();
    return;
  }

  int32_t traveled = currentPosition - homingStartSteps;
  if (traveled < 0) {
    traveled = -traveled;
  }
  if (traveled >= STEPS_PER_REV) {
    g_stepper->forceStopAndNewPosition(currentPosition);
    portENTER_CRITICAL(&statusMux);
    g_positionSteps = currentPosition;
    g_targetSteps = currentPosition;
    g_homing = false;
    g_mode = StepperMode::Idle;
    portEXIT_CRITICAL(&statusMux);
    Serial.println("homing failed");
    publishVariantStatus();
  }
}

static void applyStartValue(float value) {
  if (value < 0.0f) {
    value = 0.0f;
  }
  if (value > 255.0f) {
    value = 255.0f;
  }

  uint8_t raw = (uint8_t)(value + 0.5f);
  applyStepperCommand(raw, 0);
}

static bool runStartupHoming() {
  if (!g_stepper) {
    return false;
  }

  int32_t homingStartSteps = g_stepper->getCurrentPosition();
  bool homeSensorActive = digitalRead(HOME_GPIO) == HIGH;

  portENTER_CRITICAL(&statusMux);
  g_homingStartSteps = homingStartSteps;
  g_homing = !homeSensorActive;
  g_mode = homeSensorActive ? StepperMode::Idle : StepperMode::Homing;
  portEXIT_CRITICAL(&statusMux);

  if (homeSensorActive) {
    g_stepper->forceStopAndNewPosition(0);
    portENTER_CRITICAL(&statusMux);
    g_positionSteps = 0;
    g_targetSteps = 0;
    g_homed = true;
    g_homing = false;
    g_mode = StepperMode::Idle;
    portEXIT_CRITICAL(&statusMux);
    Serial.println("Homed (startup sensor active)");
    publishVariantStatus();
    return true;
  }

  configureMotionHoming();
  g_stepper->runBackward();

  const uint32_t homingTimeoutMs = 15000;
  uint32_t homingStartMs = millis();
  for (;;) {
    int32_t currentPosition = g_stepper->getCurrentPosition();
    int32_t traveled = currentPosition - homingStartSteps;
    if (traveled < 0) {
      traveled = -traveled;
    }

    if (digitalRead(HOME_GPIO) == HIGH) {
      g_stepper->forceStopAndNewPosition(0);
      portENTER_CRITICAL(&statusMux);
      g_positionSteps = 0;
      g_targetSteps = 0;
      g_homed = true;
      g_homing = false;
      g_mode = StepperMode::Idle;
      portEXIT_CRITICAL(&statusMux);
      Serial.println("Homed (startup)");
      publishVariantStatus();
      return true;
    }

    if (traveled >= STEPS_PER_REV || (millis() - homingStartMs) >= homingTimeoutMs) {
      g_stepper->forceStopAndNewPosition(currentPosition);
      portENTER_CRITICAL(&statusMux);
      g_positionSteps = currentPosition;
      g_targetSteps = currentPosition;
      g_homing = false;
      g_mode = StepperMode::Idle;
      portEXIT_CRITICAL(&statusMux);
      Serial.println("startup homing failed");
      publishVariantStatus();
      return false;
    }

    delay(1);
  }
}

static bool handleCliCommand(const char* commandLine) {
  char buffer[64];
  strncpy(buffer, commandLine, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  char* context = nullptr;
  char* command = strtok_r(buffer, " ", &context);
  if (!command) {
    return false;
  }

  if (strcmp(command, "home") == 0) {
    uint8_t currentAngleRaw;
    portENTER_CRITICAL(&statusMux);
    currentAngleRaw = g_angleRaw;
    portEXIT_CRITICAL(&statusMux);
    queueStepperCommand(currentAngleRaw, 255);
    Serial.println("OK home");
    return true;
  }

  if (strcmp(command, "hold") == 0) {
    char* valueText = strtok_r(nullptr, " ", &context);
    if (!valueText) {
      Serial.printf("hold=%s\n", g_holdIdle ? "on" : "off");
      return true;
    }

    String value(valueText);
    value.trim();
    value.toLowerCase();

    bool enabled;
    if (value == "1" || value == "on" || value == "true" || value == "yes") {
      enabled = true;
    } else if (value == "0" || value == "off" || value == "false" || value == "no") {
      enabled = false;
    } else {
      Serial.println("Usage: hold [on|off]");
      return true;
    }

    applyStepperHoldState(enabled);
    portENTER_CRITICAL(&statusMux);
    g_persistHoldPending = true;
    portEXIT_CRITICAL(&statusMux);

    Serial.printf("OK hold=%s\n", enabled ? "on" : "off");
    return true;
  }

  if (strcmp(command, "speed") == 0) {
    char* valueText = strtok_r(nullptr, " ", &context);
    if (!valueText) {
      uint32_t speedHz;
      portENTER_CRITICAL(&statusMux);
      speedHz = g_speedHz;
      portEXIT_CRITICAL(&statusMux);
      Serial.printf("speed=%lu\n", (unsigned long)speedHz);
      return true;
    }
    char* end = nullptr;
    long v = strtol(valueText, &end, 10);
    if (!end || *end != '\0' || v < 100 || v > 200000) {
      Serial.println("Usage: speed [100-200000]");
      return true;
    }
    portENTER_CRITICAL(&statusMux);
    g_speedHz = (uint32_t)v;
    g_applyFastMotionSettingsPending = true;
    g_persistSpeedPending = true;
    portEXIT_CRITICAL(&statusMux);
    if (controlTaskHandle) {
      xTaskNotifyGive(controlTaskHandle);
    }
    Serial.printf("OK speed=%lu\n", (unsigned long)v);
    return true;
  }

  if (strcmp(command, "accel") == 0) {
    char* valueText = strtok_r(nullptr, " ", &context);
    if (!valueText) {
      uint32_t accel;
      portENTER_CRITICAL(&statusMux);
      accel = g_accel;
      portEXIT_CRITICAL(&statusMux);
      Serial.printf("accel=%lu\n", (unsigned long)accel);
      return true;
    }
    char* end = nullptr;
    long v = strtol(valueText, &end, 10);
    if (!end || *end != '\0' || v < 10 || v > 1000000) {
      Serial.println("Usage: accel [10-1000000]");
      return true;
    }
    portENTER_CRITICAL(&statusMux);
    g_accel = (uint32_t)v;
    g_applyFastMotionSettingsPending = true;
    g_persistAccelPending = true;
    portEXIT_CRITICAL(&statusMux);
    if (controlTaskHandle) {
      xTaskNotifyGive(controlTaskHandle);
    }
    Serial.printf("OK accel=%lu\n", (unsigned long)v);
    return true;
  }

  if (strcmp(command, "angle") != 0) {
    return false;
  }

  char* valueText = strtok_r(nullptr, " ", &context);
  if (!valueText) {
    Serial.println("Usage: angle [0-360]");
    return true;
  }

  char* end = nullptr;
  float degrees = strtof(valueText, &end);
  if (!end || *end != '\0' || degrees < 0.0f || degrees > 360.0f) {
    Serial.println("Angle must be a number from 0 to 360");
    return true;
  }

  uint8_t raw = (uint8_t)((degrees * 255.0f / 360.0f) + 0.5f);
  queueStepperCommand(raw, 0);
  Serial.printf("OK angle=%.1f\n", (double)degrees);
  return true;
}

static void printCliHelp() {
  Serial.println("  angle [0-360]");
  Serial.println("  hold [on|off]");
  Serial.println("  home");
  Serial.println("  speed [100-200000]");
  Serial.println("  accel [10-1000000]");
}

static bool consumeDmxPayload(void* ctx, const ArtDmxPacket& packet, uint16_t startAddress, uint32_t nowMs) {
  (void)ctx;
  (void)nowMs;
  if (startAddress < 1 || startAddress + 1 > packet.length) {
    return false;
  }

  // The runtime listener already holds the shared mux while invoking this callback.
  g_pendingAngleRaw = packet.data[startAddress - 1];
  g_pendingControlRaw = packet.data[startAddress];
  g_havePendingCommand = true;
  return true;
}

static void pollArtnet() {
  appEnsureArtnetListener(g_shared);

  bool havePendingCommand = false;
  uint8_t angleRaw = 0;
  uint8_t controlRaw = 0;

  portENTER_CRITICAL(&statusMux);
  if (!g_configWriteActive && g_havePendingCommand) {
    angleRaw = g_pendingAngleRaw;
    controlRaw = g_pendingControlRaw;
    g_havePendingCommand = false;
    g_motionCommandActive = true;
    havePendingCommand = true;
  }
  portEXIT_CRITICAL(&statusMux);

  if (havePendingCommand) {
    applyStepperCommand(angleRaw, controlRaw);
    appMarkArtnetActivity();
    portENTER_CRITICAL(&statusMux);
    g_motionCommandActive = false;
    portEXIT_CRITICAL(&statusMux);
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
    applyPendingFastMotionSettings();
    portENTER_CRITICAL(&statusMux);
    bool configWriteActive = g_configWriteActive;
    portEXIT_CRITICAL(&statusMux);
    if (!configWriteActive) {
      serviceHomingWatchdog();
    }
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
  int32_t currentPosition = g_stepper ? g_stepper->getCurrentPosition() : g_positionSteps;
  bool homeSensorActive = digitalRead(HOME_GPIO) == HIGH;
  portENTER_CRITICAL(&statusMux);
  snapshot.angleRaw = g_angleRaw;
  snapshot.controlRaw = g_controlRaw;
  snapshot.positionSteps = currentPosition;
  snapshot.targetSteps = g_targetSteps;
  snapshot.homed = g_homed;
  snapshot.homing = g_homing;
  snapshot.homeSensorActive = homeSensorActive;
  g_positionSteps = snapshot.positionSteps;
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
  j["stepper_angle"] = snapshot.angleRaw;
  j["stepper_control"] = snapshot.controlRaw;
  j["stepper_target_steps"] = snapshot.targetSteps;
  j["stepper_position_steps"] = snapshot.positionSteps;
  j["stepper_homed"] = snapshot.homed;
  j["stepper_homing"] = snapshot.homing;
  j["stepper_home_sensor"] = snapshot.homeSensorActive;
  j["stepper_hold_idle"] = g_holdIdle;
  uint32_t speedHz, accel;
  portENTER_CRITICAL(&statusMux);
  speedHz = g_speedHz;
  accel = g_accel;
  portEXIT_CRITICAL(&statusMux);
  j["stepper_speed_hz"] = speedHz;
  j["stepper_accel"] = accel;
  j["stepper_steps_per_rev"] = STEPS_PER_REV;
  appAppendArtnetDetailFields(j, g_shared.art, now, details);

  return serializeJson(j, out, outSize);
}

static size_t buildHealthSummary(char* out, size_t outSize) {
  StatusSnapshot snapshot = readStatusSnapshot();
  size_t used = appFormatCommonHealthPrefix(out, outSize, SOURCE_HOLD_MS, snapshot.base.lastArtMs, g_shared.art);
  if (used >= outSize) {
    return used;
  }

  return used + snprintf(out + used,
                         outSize - used,
                         " angle=%u ctrl=%u pos=%ld target=%ld homed=%s homing=%s home=%s hold=%s",
                         (unsigned int)snapshot.angleRaw,
                         (unsigned int)snapshot.controlRaw,
                         (long)snapshot.positionSteps,
                         (long)snapshot.targetSteps,
                         snapshot.homed ? "yes" : "no",
                         snapshot.homing ? "yes" : "no",
                         snapshot.homeSensorActive ? "high" : "low",
                         g_holdIdle ? "on" : "off");
}

static void setupStepperHardware() {
  pinMode(DIR_GPIO, OUTPUT);
  pinMode(STEP_GPIO, OUTPUT);
  pinMode(SLP_GPIO, OUTPUT);
  pinMode(RST_GPIO, OUTPUT);
  pinMode(MS3_GPIO, OUTPUT);
  pinMode(MS2_GPIO, OUTPUT);
  pinMode(MS1_GPIO, OUTPUT);
  pinMode(EN_GPIO, OUTPUT);
  pinMode(HOME_GPIO2, OUTPUT);
  pinMode(HOME_GPIO, INPUT_PULLDOWN);

  digitalWrite(SLP_GPIO, HIGH);
  digitalWrite(RST_GPIO, HIGH);
  digitalWrite(MS3_GPIO, HIGH);
  digitalWrite(MS2_GPIO, HIGH);
  digitalWrite(MS1_GPIO, HIGH);
  digitalWrite(HOME_GPIO2, HIGH);

  {
    PreferInternalMallocScope internalAllocations;
    g_engine.init();
    g_stepper = g_engine.stepperConnectToPin(STEP_GPIO);
    if (!g_stepper) {
      appLogLine("Stepper init failed");
      return;
    }

    g_stepper->setDirectionPin(DIR_GPIO, true);
    g_stepper->setEnablePin(EN_GPIO, true);
    applyStepperHoldState(g_holdIdle);
    configureMotionFast();
  }
}

void setup() {
  appInitializeBaseRuntime();

  g_shared.cfg.dmxStartAddress = appConfig().getDMXAddress();
  g_shared.cfg.artnetUniverse = appConfig().getDMXUniverse();
  g_shared.cfg.startValue = appConfig().getStartValue();
  g_shared.cfg.stepperHoldIdle = appConfig().getStepperHoldIdle();
  g_holdIdle = g_shared.cfg.stepperHoldIdle;
  g_speedHz = appConfig().getStepperSpeedHz();
  g_accel = appConfig().getStepperAccel();
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
    "ArtNetController Stepper",
    buildStatusJson,
    buildHealthSummary,
    nullptr,
    AppVariantKind::Stepper,
    applyStartValue,
    applyStepperHoldState,
    handleCliCommand,
    printCliHelp,
  });

  appStartCommonServices();
  appConnectWifi();
  setupStepperHardware();
  bool homed = runStartupHoming();
  if (homed) {
    appApplyVariantStartValue(g_shared.cfg.startValue);
  } else {
    Serial.println("Skipping startup default position because homing did not complete");
  }

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
  serviceDeferredConfigWrites();
  delay(1);
}
