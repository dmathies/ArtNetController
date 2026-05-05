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

struct AppRuntimeHooks {
  const char* indexPagePath;
  const char* deviceName;
  size_t (*buildStatusJson)(char* out, size_t outSize, bool details);
  size_t (*buildHealthSummary)(char* out, size_t outSize);
  void (*pollInputs)();
};

struct AppTaskRuntimeStats {
  uint32_t lastActiveMs = 0;
  uint16_t utilPermille = 0;
};

Configuration& appConfig();
WifiManagerClass& appWifiManager();
size_t appBuildStatusJson(char* out, size_t outSize, bool details);
const char* appGetDeviceName();

ArtDmxPacket appParseArtDmx(const uint8_t* p, int len);
void appMarkArtnetActivity();
uint32_t appGetLastArtnetMs();
TaskHandle_t appGetWebServerTaskHandle();
AppTaskRuntimeStats appGetWebTaskRuntimeStats();

const char* appResetReasonToString(esp_reset_reason_t reason);
float appReadBoardTemperatureC();

void appInitializeBaseRuntime();
void appInitRuntime(const AppRuntimeHooks& hooks);
void appStartCommonServices();
void appConnectWifi();
void appStartWebServices();
void appCommonLoop(uint32_t wsStatusPushMs);
