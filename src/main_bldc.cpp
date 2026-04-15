#include <Arduino.h>
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

HardwareSerial BLDCSerial(2);
BLDC::Driver bldc(BLDCSerial, BLDC_RX, BLDC_TX, 9600);

struct MotorCmd {
  uint8_t value;
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
};

static TaskHandle_t motorTaskHandle = nullptr;
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

static inline uint8_t rawToStep(uint8_t raw) {
  return (uint8_t)((((uint16_t)raw) * 60U + 127U) / 255U);
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
    MotorCmd cmd;
    if (xQueueReceive(motorQueue, &cmd, pdMS_TO_TICKS(MOTOR_TASK_DELAY_MS)) == pdPASS) {
      currentRaw = cmd.value;
      uint8_t step = rawToStep(currentRaw);
      bldc.sendSpeed(step);

      portENTER_CRITICAL(&statusMux);
      g_motorRaw = currentRaw;
      g_motorStep = step;
      portEXIT_CRITICAL(&statusMux);
    }

    bldc.poll();
  }
}

static void processArtnet(const ArtDmxPacket& a) {
  portENTER_CRITICAL(&statusMux);
  artDmxRxTotal++;
  artLastUniverseFlat = a.universe_flat;
  portEXIT_CRITICAL(&statusMux);

  uint16_t universe = appConfig().getDMXUniverse();
  if (a.universe_flat != universe) {
    portENTER_CRITICAL(&statusMux);
    artDmxUniverseMismatchTotal++;
    portEXIT_CRITICAL(&statusMux);
    return;
  }

  uint16_t channel = appConfig().getDMXAddress();
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
  static constexpr uint32_t ARTNET_IDLE_REBIND_MS = 3000;
  static constexpr uint32_t ARTNET_REBIND_GUARD_MS = 1000;
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
    IPAddress remoteIp = artudp.remoteIP();
    uint16_t remotePort = artudp.remotePort();

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
  portEXIT_CRITICAL(&statusMux);
  return snapshot;
}

static size_t buildStatusJson(char* out, size_t outSize, bool details) {
  DynamicJsonDocument j(1024);
  StatusSnapshot s = readStatusSnapshot();
  uint32_t now = millis();
  uint32_t artAge = (s.lastArtMs == 0) ? 0xFFFFFFFFu : (now - s.lastArtMs);

  j["source"] = (artAge <= SOURCE_HOLD_MS) ? "Art-Net" : "none";
  j["art_age_ms"] = (artAge == 0xFFFFFFFFu) ? -1 : (int32_t)artAge;
  j["motor_value"] = s.motorRaw;
  j["motor_step"] = s.motorStep;
  j["wifi_rssi"] = appWifiManager().getRSSI();
  j["free_heap"] = ESP.getFreeHeap();
  j["min_free_heap"] = ESP.getMinFreeHeap();
  j["reset_reason"] = appResetReasonToString(esp_reset_reason());

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
  bldc.begin();
  bldc.sendSpeed(0);

  appInitRuntime({
    "/wifi-manager/index.html",
    "CableCar BLDC",
    buildStatusJson,
    buildHealthSummary,
    pollArtnet,
  });

  appConnectWifi();
  appStartCommonServices();

  motorQueue = xQueueCreate(1, sizeof(MotorCmd));
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 4096, nullptr, 2, &motorTaskHandle, 1);
}

void loop() {
  appCommonLoop(WS_STATUS_PUSH_MS);
  delay(1);
}
