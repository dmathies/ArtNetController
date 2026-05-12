#pragma once

#include <Arduino.h>
#include <AsyncUDP.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "main_common.h"
#include "runtime_metrics.h"

struct AppConfigCache {
  uint16_t dmxStartAddress = 1;
  uint16_t artnetUniverse = 0;
  float startValue = 0.0f;
};

struct AppArtnetStats {
  uint32_t dmxRxTotal = 0;
  uint32_t dmxUniverseMatchTotal = 0;
  uint32_t dmxUniverseMismatchTotal = 0;
  uint32_t dmxInvalidTotal = 0;
  uint32_t dmxLastArrivalMs = 0;
  uint32_t udpRxTotal = 0;
  uint32_t udpRxBytesTotal = 0;
  uint32_t udpLastPacketMs = 0;
  uint32_t udpRebindTotal = 0;
  uint16_t lastUniverseFlat = 0;
  uint16_t udpLastPacketSize = 0;
  uint16_t udpLastRemotePort = 0;
  IPAddress udpLastRemoteIp = IPAddress(0, 0, 0, 0);
};

struct AppTaskDiagnosticsCache {
  int32_t controlTaskStackHwm = -1;
  int32_t motorTaskStackHwm = -1;
  int32_t webTaskStackHwm = -1;
  const char* controlTaskState = "unknown";
  const char* motorTaskState = "unknown";
  const char* webTaskState = "unknown";
};

struct AppStatusSnapshotBase {
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

using AppConsumeDmxPayloadFn = bool (*)(void* ctx,
                                        const ArtDmxPacket& packet,
                                        uint16_t startAddress,
                                        uint32_t nowMs);

struct AppVariantSharedRuntime {
  AsyncUDP artudp;
  bool artudpListening = false;
  AppConfigCache cfg;
  AppArtnetStats art;
  TaskMetrics controlTaskMetrics;
  TaskMetrics motorTaskMetrics;
  ArtnetTimingWindow artnetTiming;
  AppTaskDiagnosticsCache diagnostics;
  uint32_t statusDiagnosticsBuiltMs = 0;
  uint32_t lastArtnetHealthLogMs = 0;
  portMUX_TYPE* mux = nullptr;
  TaskHandle_t* notifyTaskHandle = nullptr;
  AppConsumeDmxPayloadFn consumeDmxPayload = nullptr;
  void* consumeDmxPayloadContext = nullptr;
  bool timingEnabled = false;
  uint32_t intervalWarnUs = 0;
  uint32_t intervalFreezeUs = 0;
};

struct AppArtnetHealthSample {
  uint32_t nowMs = 0;
  uint32_t artLastMsAgo = 0xFFFFFFFFu;
  uint32_t udpLastMsAgo = 0xFFFFFFFFu;
  uint32_t udpRx = 0;
  uint32_t udpBytes = 0;
  uint32_t udpRebinds = 0;
  uint16_t lastUni = 0;
  uint16_t lastSize = 0;
  uint16_t lastPort = 0;
  IPAddress lastIp = IPAddress(0, 0, 0, 0);
  uint32_t dmxRx = 0;
  uint32_t dmxMatch = 0;
  uint32_t dmxMismatch = 0;
  uint32_t dmxInvalid = 0;
  uint16_t ctrlUtilPermille = 0;
  uint16_t motorUtilPermille = 0;
  uint32_t ctrlLastMs = 0;
  uint32_t motorLastMs = 0;
};

inline void appRecordArtnetUdpPacket(AppArtnetStats& stats,
                                     size_t len,
                                     uint32_t nowMs,
                                     uint16_t packetSize,
                                     const IPAddress& remoteIp,
                                     uint16_t remotePort) {
  stats.udpRxTotal++;
  stats.udpRxBytesTotal += (uint32_t)len;
  stats.udpLastPacketMs = nowMs;
  stats.udpLastPacketSize = packetSize;
  stats.udpLastRemoteIp = remoteIp;
  stats.udpLastRemotePort = remotePort;
}

inline void appRecordArtnetInvalid(AppArtnetStats& stats) { stats.dmxInvalidTotal++; }

inline void appRecordArtnetUniverseMismatch(AppArtnetStats& stats, uint16_t universeFlat) {
  stats.lastUniverseFlat = universeFlat;
  stats.dmxRxTotal++;
  stats.dmxUniverseMismatchTotal++;
}

inline void appRecordArtnetUniverseMatch(AppArtnetStats& stats,
                                         uint16_t universeFlat,
                                         uint32_t nowMs) {
  stats.lastUniverseFlat = universeFlat;
  stats.dmxRxTotal++;
  stats.dmxUniverseMatchTotal++;
  stats.dmxLastArrivalMs = nowMs;
}

inline void appFillStatusSnapshotBase(AppStatusSnapshotBase& base,
                                      const AppVariantSharedRuntime& shared,
                                      uint32_t lastArtMs) {
  base.lastArtMs = lastArtMs;
  base.artDmxRxTotal = shared.art.dmxRxTotal;
  base.artDmxUniverseMatchTotal = shared.art.dmxUniverseMatchTotal;
  base.artDmxUniverseMismatchTotal = shared.art.dmxUniverseMismatchTotal;
  base.artDmxInvalidTotal = shared.art.dmxInvalidTotal;
  base.artDmxLastArrivalMs = shared.art.dmxLastArrivalMs;
  base.controlTaskLastLoopMs = shared.controlTaskMetrics.lastLoopMs;
  base.motorTaskLastLoopMs = shared.motorTaskMetrics.lastLoopMs;
  base.controlTaskUtilPermille = shared.controlTaskMetrics.utilPermille;
  base.motorTaskUtilPermille = shared.motorTaskMetrics.utilPermille;
}

inline bool appBuildArtnetHealthSampleIfDue(AppVariantSharedRuntime& shared,
                                            portMUX_TYPE& mux,
                                            uint32_t nowMs,
                                            uint32_t intervalMs,
                                            bool enabled,
                                            AppArtnetHealthSample& out) {
  if (!enabled) {
    (void)shared;
    (void)mux;
    (void)nowMs;
    (void)intervalMs;
    return false;
  }

  if ((nowMs - shared.lastArtnetHealthLogMs) < intervalMs) {
    return false;
  }

  portENTER_CRITICAL(&mux);
  out.nowMs = nowMs;
  out.udpLastMsAgo = (shared.art.udpLastPacketMs == 0) ? 0xFFFFFFFFu : (nowMs - shared.art.udpLastPacketMs);
  out.udpRx = shared.art.udpRxTotal;
  out.udpBytes = shared.art.udpRxBytesTotal;
  out.udpRebinds = shared.art.udpRebindTotal;
  out.lastUni = shared.art.lastUniverseFlat;
  out.lastSize = shared.art.udpLastPacketSize;
  out.lastPort = shared.art.udpLastRemotePort;
  out.lastIp = shared.art.udpLastRemoteIp;
  out.dmxRx = shared.art.dmxRxTotal;
  out.dmxMatch = shared.art.dmxUniverseMatchTotal;
  out.dmxMismatch = shared.art.dmxUniverseMismatchTotal;
  out.dmxInvalid = shared.art.dmxInvalidTotal;
  out.ctrlUtilPermille = shared.controlTaskMetrics.utilPermille;
  out.motorUtilPermille = shared.motorTaskMetrics.utilPermille;
  out.ctrlLastMs = shared.controlTaskMetrics.lastLoopMs;
  out.motorLastMs = shared.motorTaskMetrics.lastLoopMs;
  portEXIT_CRITICAL(&mux);

  uint32_t artLastMs = appGetLastArtnetMs();
  out.artLastMsAgo = (artLastMs == 0) ? 0xFFFFFFFFu : (nowMs - artLastMs);
  shared.lastArtnetHealthLogMs = nowMs;
  return true;
}

inline void appConfigureArtnetListener(AppVariantSharedRuntime& shared,
                                       portMUX_TYPE& mux,
                                       TaskHandle_t* notifyTaskHandle,
                                       AppConsumeDmxPayloadFn consumeDmxPayload,
                                       void* consumeDmxPayloadContext,
                                       bool timingEnabled,
                                       uint32_t intervalWarnUs,
                                       uint32_t intervalFreezeUs) {
  shared.mux = &mux;
  shared.notifyTaskHandle = notifyTaskHandle;
  shared.consumeDmxPayload = consumeDmxPayload;
  shared.consumeDmxPayloadContext = consumeDmxPayloadContext;
  shared.timingEnabled = timingEnabled;
  shared.intervalWarnUs = intervalWarnUs;
  shared.intervalFreezeUs = intervalFreezeUs;
}

inline void appEnsureArtnetListener(AppVariantSharedRuntime& shared) {
  if (shared.artudpListening) {
    return;
  }

  AppVariantSharedRuntime* sharedPtr = &shared;
  shared.artudp.onPacket([sharedPtr](AsyncUDPPacket& packet) {
    const uint8_t* data = packet.data();
    const size_t len = packet.length();
    if (!data || len == 0 || !sharedPtr->mux) {
      return;
    }

    uint32_t packetNowUs = micros();
    IPAddress remoteIp = packet.remoteIP();
    uint16_t remotePort = packet.remotePort();
    uint32_t nowMs = millis();
    uint16_t packetSize = (uint16_t)(len > 0xFFFFu ? 0xFFFFu : len);

    ArtDmxPacket a = appParseArtDmx(data, (int)len);
    portENTER_CRITICAL(sharedPtr->mux);
    appRecordArtnetUdpPacket(sharedPtr->art, len, nowMs, packetSize, remoteIp, remotePort);

    if (!a.ok) {
      appRecordArtnetInvalid(sharedPtr->art);
      portEXIT_CRITICAL(sharedPtr->mux);
      return;
    }

    if (sharedPtr->timingEnabled) {
      noteArtnetPacketTiming(sharedPtr->artnetTiming,
                             packetNowUs,
                             sharedPtr->intervalWarnUs,
                             sharedPtr->intervalFreezeUs);
      noteArtnetSource(sharedPtr->artnetTiming, remoteIp, remotePort);
      noteArtnetSequence(sharedPtr->artnetTiming, data, (int)len);
    }

    if (a.universe_flat != sharedPtr->cfg.artnetUniverse) {
      appRecordArtnetUniverseMismatch(sharedPtr->art, a.universe_flat);
      portEXIT_CRITICAL(sharedPtr->mux);
      return;
    }

    bool accepted = false;
    if (sharedPtr->consumeDmxPayload) {
      accepted = sharedPtr->consumeDmxPayload(sharedPtr->consumeDmxPayloadContext,
                                              a,
                                              sharedPtr->cfg.dmxStartAddress,
                                              nowMs);
    }

    if (!accepted) {
      portEXIT_CRITICAL(sharedPtr->mux);
      return;
    }

    appRecordArtnetUniverseMatch(sharedPtr->art, a.universe_flat, nowMs);
    portEXIT_CRITICAL(sharedPtr->mux);

    if (sharedPtr->notifyTaskHandle && *sharedPtr->notifyTaskHandle) {
      xTaskNotifyGive(*sharedPtr->notifyTaskHandle);
    }
  });

  shared.artudpListening = shared.artudp.listen(APP_ARTNET_PORT);
  if (shared.artudpListening) {
    appLogPrintf("[ARTNET] AsyncUDP listening on :%u\n", APP_ARTNET_PORT);
  } else {
    appLogPrintf("[ARTNET] AsyncUDP listen failed err=%d\n", (int)shared.artudp.lastErr());
  }
}

inline void appLogArtnetTimingWindowIfDue(ArtnetTimingWindow& timing,
                                          portMUX_TYPE& mux,
                                          const TaskMetrics& controlTaskMetrics,
                                          const TaskMetrics* motorTaskMetrics,
                                          uint32_t nowMs,
                                          uint32_t windowMs,
                                          bool enabled) {
  if (!enabled) {
    (void)timing;
    (void)mux;
    (void)controlTaskMetrics;
    (void)motorTaskMetrics;
    (void)nowMs;
    (void)windowMs;
    return;
  }

  if (timing.windowStartMs == 0) {
    timing.windowStartMs = nowMs;
    return;
  }
  if ((nowMs - timing.windowStartMs) < windowMs) {
    return;
  }

  uint32_t elapsedMs = nowMs - timing.windowStartMs;
  float pktHz = (elapsedMs > 0) ? ((float)timing.packetCount * 1000.0f / (float)elapsedMs) : 0.0f;
  float avgIatMs =
      (timing.packetCount > 1) ? ((float)timing.intervalSumUs / (float)(timing.packetCount - 1) / 1000.0f) : -1.0f;
  float minIatMs = (timing.intervalMinUs == 0xFFFFFFFFu) ? -1.0f : ((float)timing.intervalMinUs / 1000.0f);
  float maxIatMs = (timing.intervalMaxUs == 0) ? -1.0f : ((float)timing.intervalMaxUs / 1000.0f);
  float avgLoopMs =
      (timing.loopSamples > 0) ? ((float)timing.loopPeriodSumUs / (float)timing.loopSamples / 1000.0f) : -1.0f;
  float minLoopMs = (timing.loopPeriodMinUs == 0xFFFFFFFFu) ? -1.0f : ((float)timing.loopPeriodMinUs / 1000.0f);
  float maxLoopMs = (timing.loopPeriodMaxUs == 0) ? -1.0f : ((float)timing.loopPeriodMaxUs / 1000.0f);
  float ctrlUtilPct;
  float motorUtilPct = -1.0f;
  portENTER_CRITICAL(&mux);
  ctrlUtilPct = (float)controlTaskMetrics.utilPermille / 10.0f;
  if (motorTaskMetrics) motorUtilPct = (float)motorTaskMetrics->utilPermille / 10.0f;
  portEXIT_CRITICAL(&mux);
  uint32_t idleGapMs = (timing.lastPacketUs == 0) ? 0xFFFFFFFFu : ((micros() - timing.lastPacketUs) / 1000U);

  if (motorTaskMetrics) {
    Serial.printf("[ARTTIM] win=%lums hz=%.1f iat_ms=%.2f/%.2f/%.2f warn40=%lu freeze100=%lu idle=%lums loop_ms=%.2f/%.2f/%.2f late10=%lu util=%.1f/%.1f\n",
                  (unsigned long)elapsedMs,
                  pktHz,
                  minIatMs,
                  avgIatMs,
                  maxIatMs,
                  (unsigned long)timing.intervalWarnCount,
                  (unsigned long)timing.intervalFreezeCount,
                  (unsigned long)idleGapMs,
                  minLoopMs,
                  avgLoopMs,
                  maxLoopMs,
                  (unsigned long)timing.loopLateCount,
                  ctrlUtilPct,
                  motorUtilPct);
  } else {
    Serial.printf("[ARTTIM] win=%lums hz=%.1f iat_ms=%.2f/%.2f/%.2f warn40=%lu freeze100=%lu idle=%lums loop_ms=%.2f/%.2f/%.2f late10=%lu util=%.1f\n",
                  (unsigned long)elapsedMs,
                  pktHz,
                  minIatMs,
                  avgIatMs,
                  maxIatMs,
                  (unsigned long)timing.intervalWarnCount,
                  (unsigned long)timing.intervalFreezeCount,
                  (unsigned long)idleGapMs,
                  minLoopMs,
                  avgLoopMs,
                  maxLoopMs,
                  (unsigned long)timing.loopLateCount,
                  ctrlUtilPct);
  }

  Serial.printf("[ARTSRC] seq_on=%lu seq_disc=%lu seq_rep=%lu seq_back=%lu src_sw=%lu\n",
                (unsigned long)timing.seqEnabledPackets,
                (unsigned long)timing.seqDiscontCount,
                (unsigned long)timing.seqRepeatCount,
                (unsigned long)timing.seqBackwardCount,
                (unsigned long)timing.sourceSwitchCount);

  resetArtnetTimingWindow(timing, nowMs);
}

inline AppTaskDiagnosticsCache appReadTaskDiagnostics(const AppTaskDiagnosticsCache& diagnostics,
                                                      portMUX_TYPE& mux) {
  AppTaskDiagnosticsCache snapshot;
  portENTER_CRITICAL(&mux);
  snapshot = diagnostics;
  portEXIT_CRITICAL(&mux);
  return snapshot;
}

inline void appRefreshTaskDiagnosticsIfDue(AppTaskDiagnosticsCache& diagnostics,
                                           uint32_t& builtMs,
                                           portMUX_TYPE& mux,
                                           uint32_t nowMs,
                                           uint32_t refreshMs,
                                           TaskHandle_t controlTaskHandle,
                                           TaskHandle_t motorTaskHandle) {
  uint32_t lastBuiltMs;
  portENTER_CRITICAL(&mux);
  lastBuiltMs = builtMs;
  portEXIT_CRITICAL(&mux);

  if (lastBuiltMs != 0 && (nowMs - lastBuiltMs) < refreshMs) {
    return;
  }

  AppTaskDiagnosticsCache next;
  TaskHandle_t webTaskHandle = appGetWebServerTaskHandle();
  if (controlTaskHandle) {
    next.controlTaskState = taskStateToString(eTaskGetState(controlTaskHandle));
    next.controlTaskStackHwm = (int32_t)uxTaskGetStackHighWaterMark(controlTaskHandle);
  }
  if (motorTaskHandle) {
    next.motorTaskState = taskStateToString(eTaskGetState(motorTaskHandle));
    next.motorTaskStackHwm = (int32_t)uxTaskGetStackHighWaterMark(motorTaskHandle);
  }
  next.webTaskState = webTaskHandle ? taskStateToString(eTaskGetState(webTaskHandle)) : "unknown";
  next.webTaskStackHwm = webTaskHandle ? (int32_t)uxTaskGetStackHighWaterMark(webTaskHandle) : -1;

  portENTER_CRITICAL(&mux);
  diagnostics = next;
  builtMs = nowMs;
  portEXIT_CRITICAL(&mux);
}

inline void appAppendCommonStatusFields(JsonDocument& j,
                                        uint32_t nowMs,
                                        uint32_t sourceHoldMs,
                                        uint32_t lastArtMs,
                                        const AppTaskDiagnosticsCache& diagnostics,
                                        uint32_t controlTaskLastLoopMs,
                                        uint16_t controlTaskUtilPermille,
                                        const uint32_t* motorTaskLastLoopMs,
                                        const uint16_t* motorTaskUtilPermille) {
  uint32_t artAge = (lastArtMs == 0) ? 0xFFFFFFFFu : (nowMs - lastArtMs);
  AppTaskRuntimeStats webTaskStats = appGetWebTaskRuntimeStats();
  AppSlowStatusMetrics slowMetrics = appGetSlowStatusMetrics();
  WifiManagerClass& wifiManager = appWifiManager();
  IPAddress ip = wifiManager.getIP();
  char ipBuf[16];
  char macBuf[18];
  snprintf(ipBuf, sizeof(ipBuf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  wifiManager.getMacAddress(macBuf, sizeof(macBuf));

  j["source"] = (artAge <= sourceHoldMs) ? "Art-Net" : "none";
  j["variant"] = appGetVariantName();
  j["art_age_ms"] = (artAge == 0xFFFFFFFFu) ? -1 : (int32_t)artAge;
  j["wifi_rssi"] = wifiManager.getRSSI();
  j["hostname"] = wifiManager.getHostnameCStr();
  j["wifi_ip"] = ipBuf;
  j["wifi_mac"] = macBuf;
  j["free_heap"] = slowMetrics.freeHeap;
  j["min_free_heap"] = slowMetrics.minFreeHeap;
  j["largest_free_block"] = slowMetrics.largestFreeBlock;
  j["board_temp_c"] = slowMetrics.boardTempC;
  j["reset_reason"] = slowMetrics.resetReason;
  j["task_control_state"] = diagnostics.controlTaskState;
  j["task_control_stack_hwm"] = diagnostics.controlTaskStackHwm;
  j["task_control_last_ms_ago"] = (controlTaskLastLoopMs == 0) ? -1 : (int32_t)(nowMs - controlTaskLastLoopMs);
  j["task_control_util_pct"] = (float)controlTaskUtilPermille / 10.0f;
  if (motorTaskLastLoopMs && motorTaskUtilPermille) {
    j["task_motor_state"] = diagnostics.motorTaskState;
    j["task_motor_stack_hwm"] = diagnostics.motorTaskStackHwm;
    j["task_motor_last_ms_ago"] = (*motorTaskLastLoopMs == 0) ? -1 : (int32_t)(nowMs - *motorTaskLastLoopMs);
    j["task_motor_util_pct"] = (float)(*motorTaskUtilPermille) / 10.0f;
  }
  j["task_web_state"] = diagnostics.webTaskState;
  j["task_web_stack_hwm"] = diagnostics.webTaskStackHwm;
  j["task_web_last_ms_ago"] = (webTaskStats.lastActiveMs == 0) ? -1 : (int32_t)(nowMs - webTaskStats.lastActiveMs);
  j["task_web_util_pct"] = (float)webTaskStats.utilPermille / 10.0f;
}

inline void appAppendArtnetDetailFields(JsonDocument& j,
                                        const AppArtnetStats& stats,
                                        uint32_t nowMs,
                                        bool details) {
  if (!details) return;
  j["art_rx_total"] = stats.dmxRxTotal;
  j["art_rx_universe_total"] = stats.dmxUniverseMatchTotal;
  j["art_rx_universe_mismatch_total"] = stats.dmxUniverseMismatchTotal;
  j["art_rx_invalid_total"] = stats.dmxInvalidTotal;
  j["art_rx_last_ms_ago"] = (stats.dmxLastArrivalMs == 0) ? -1 : (int32_t)(nowMs - stats.dmxLastArrivalMs);
}

inline size_t appFormatCommonHealthPrefix(char* out,
                                          size_t outSize,
                                          uint32_t sourceHoldMs,
                                          uint32_t lastArtMs,
                                          const AppArtnetStats& stats) {
  uint32_t nowMs = millis();
  int32_t lastMsAgo = (stats.dmxLastArrivalMs == 0) ? -1 : (int32_t)(nowMs - stats.dmxLastArrivalMs);
  uint32_t artAge = (lastArtMs == 0) ? 0xFFFFFFFFu : (nowMs - lastArtMs);

  return snprintf(out,
                  outSize,
                  "art_src=%s art_rx=%lu match=%lu mismatch=%lu invalid=%lu last=%ldms udp_rx=%lu udp_b=%lu udp_last=%ldms from=%s:%u pkt=%u uni=%u rebind=%lu",
                  (artAge <= sourceHoldMs) ? "live" : "idle",
                  (unsigned long)stats.dmxRxTotal,
                  (unsigned long)stats.dmxUniverseMatchTotal,
                  (unsigned long)stats.dmxUniverseMismatchTotal,
                  (unsigned long)stats.dmxInvalidTotal,
                  (long)lastMsAgo,
                  (unsigned long)stats.udpRxTotal,
                  (unsigned long)stats.udpRxBytesTotal,
                  (long)((stats.udpLastPacketMs == 0) ? -1 : (int32_t)(nowMs - stats.udpLastPacketMs)),
                  stats.udpLastRemoteIp.toString().c_str(),
                  (unsigned int)stats.udpLastRemotePort,
                  (unsigned int)stats.udpLastPacketSize,
                  (unsigned int)stats.lastUniverseFlat,
                  (unsigned long)stats.udpRebindTotal);
}
