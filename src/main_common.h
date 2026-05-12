#pragma once

#include <Arduino.h>
#include <esp_http_server.h>

#include "WifiManager.h"

constexpr uint16_t APP_ARTNET_PORT = 6454;

struct ArtDmxPacket {
  bool ok = false;
  uint16_t universe_flat = 0;
  uint16_t length = 0;
  const uint8_t* data = nullptr;
};

enum class AppVariantKind : uint8_t {
  Bldc = 0,
  Relay = 1,
  Led = 2,
  Unknown = 255,
};

struct AppRuntimeHooks {
  const char* indexPagePath;
  const char* deviceName;
  size_t (*buildStatusJson)(char* out, size_t outSize, bool details);
  size_t (*buildHealthSummary)(char* out, size_t outSize);
  void (*pollInputs)();
  AppVariantKind variant;
  void (*applyStartValue)(float value);
};

struct AppVariantStatus {
  AppVariantKind variant = AppVariantKind::Unknown;
  uint32_t updatedMs = 0;
  uint8_t motorValue = 0;
  uint8_t motorStep = 0;
  bool relayOn = false;
  bool controllerPowerOn = false;
  int32_t controllerLastStatusMsAgo = -1;
  uint16_t normalizedValue = 0;
  float ledValues[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};

struct AppTaskRuntimeStats {
  uint32_t lastActiveMs = 0;
  uint16_t utilPermille = 0;
};

struct AppSlowStatusMetrics {
  float boardTempC = NAN;
  uint32_t freeHeap = 0;
  uint32_t minFreeHeap = 0;
  uint32_t largestFreeBlock = 0;
  const char* resetReason = "unknown";
};

Configuration& appConfig();
WifiManagerClass& appWifiManager();
size_t appBuildStatusJson(char* out, size_t outSize, bool details);
const char* appGetDeviceName();
const char* appGetVariantName();
AppVariantKind appGetVariantKind();
const char* appVariantKindToString(AppVariantKind variant);
AppVariantStatus appGetVariantStatus();
void appSetVariantStatus(const AppVariantStatus& status);
void appApplyVariantStartValue(float value);

ArtDmxPacket appParseArtDmx(const uint8_t* p, int len);
void appMarkArtnetActivity();
uint32_t appGetLastArtnetMs();
TaskHandle_t appGetWebServerTaskHandle();
AppTaskRuntimeStats appGetWebTaskRuntimeStats();
AppSlowStatusMetrics appGetSlowStatusMetrics();

const char* appResetReasonToString(esp_reset_reason_t reason);
float appReadBoardTemperatureC();

void appInitializeBaseRuntime();
void appInitRuntime(const AppRuntimeHooks& hooks);
void appStartCommonServices();
void appConnectWifi();
void appStartWebServices();
void appCommonLoop(uint32_t wsStatusPushMs);
