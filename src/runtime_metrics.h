#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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

inline void noteArtnetPacketTiming(ArtnetTimingWindow& timing, uint32_t nowUs, uint32_t warnUs, uint32_t freezeUs) {
  if (timing.lastPacketUs != 0) {
    uint32_t dtUs = nowUs - timing.lastPacketUs;
    timing.intervalSumUs += dtUs;
    if (dtUs < timing.intervalMinUs) timing.intervalMinUs = dtUs;
    if (dtUs > timing.intervalMaxUs) timing.intervalMaxUs = dtUs;
    if (dtUs >= warnUs) timing.intervalWarnCount++;
    if (dtUs >= freezeUs) timing.intervalFreezeCount++;
  }
  timing.lastPacketUs = nowUs;
  timing.packetCount++;
}

inline void noteArtnetSource(ArtnetTimingWindow& timing, IPAddress remoteIp, uint16_t remotePort) {
  uint32_t ipRaw = ((uint32_t)remoteIp[0] << 24) |
                   ((uint32_t)remoteIp[1] << 16) |
                   ((uint32_t)remoteIp[2] << 8) |
                   (uint32_t)remoteIp[3];
  uint64_t key = (((uint64_t)ipRaw) << 16) | (uint64_t)remotePort;
  if (timing.lastSourceKey != 0 && timing.lastSourceKey != key) {
    timing.sourceSwitchCount++;
  }
  timing.lastSourceKey = key;
}

inline void noteArtnetSequence(ArtnetTimingWindow& timing, const uint8_t* p, int len) {
  if (len < 18) return;
  if (memcmp(p, "Art-Net\0", 8) != 0) return;
  if (!(p[8] == 0x00 && p[9] == 0x50)) return;

  uint8_t seq = p[12];
  if (seq == 0) return;

  timing.seqEnabledPackets++;
  if (timing.haveLastSeq) {
    uint8_t expected = (uint8_t)(timing.lastSeq + 1);
    if (seq == timing.lastSeq) {
      timing.seqRepeatCount++;
    } else if (seq != expected) {
      timing.seqDiscontCount++;
      if ((uint8_t)(seq - timing.lastSeq) > 128U) {
        timing.seqBackwardCount++;
      }
    }
  }

  timing.lastSeq = seq;
  timing.haveLastSeq = true;
}

inline void noteControlLoopTiming(ArtnetTimingWindow& timing, uint32_t nowUs) {
  if (timing.lastControlLoopUs != 0) {
    uint32_t dtUs = nowUs - timing.lastControlLoopUs;
    timing.loopPeriodSumUs += dtUs;
    if (dtUs < timing.loopPeriodMinUs) timing.loopPeriodMinUs = dtUs;
    if (dtUs > timing.loopPeriodMaxUs) timing.loopPeriodMaxUs = dtUs;
    if (dtUs >= 10000U) timing.loopLateCount++;
    timing.loopSamples++;
  }
  timing.lastControlLoopUs = nowUs;
}

inline void resetArtnetTimingWindow(ArtnetTimingWindow& timing, uint32_t nowMs) {
  timing.windowStartMs = nowMs;
  timing.packetCount = 0;
  timing.intervalSumUs = 0;
  timing.intervalMinUs = 0xFFFFFFFFu;
  timing.intervalMaxUs = 0;
  timing.intervalWarnCount = 0;
  timing.intervalFreezeCount = 0;
  timing.seqEnabledPackets = 0;
  timing.seqDiscontCount = 0;
  timing.seqRepeatCount = 0;
  timing.seqBackwardCount = 0;
  timing.sourceSwitchCount = 0;
  timing.loopSamples = 0;
  timing.loopPeriodSumUs = 0;
  timing.loopPeriodMinUs = 0xFFFFFFFFu;
  timing.loopPeriodMaxUs = 0;
  timing.loopLateCount = 0;
}

inline void recordTaskMetrics(TaskMetrics& metrics, portMUX_TYPE& mux, uint32_t busyUs) {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  portENTER_CRITICAL(&mux);
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
  portEXIT_CRITICAL(&mux);
}

inline const char* taskStateToString(eTaskState state) {
  switch (state) {
    case eRunning: return "running";
    case eReady: return "ready";
    case eBlocked: return "blocked";
    case eSuspended: return "suspended";
    case eDeleted: return "deleted";
    default: return "unknown";
  }
}
