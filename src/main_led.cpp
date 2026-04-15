#include <Arduino.h>
#include <WiFiUdp.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#include "main_common.h"

static constexpr int CH1 = 1;
static constexpr int CH2 = 2;
static constexpr int CH3 = 3;
static constexpr int CH4 = 21;

static constexpr int LEDC_RES_BITS = 12;
static constexpr int LEDC_MAX_DUTY = (1 << LEDC_RES_BITS) - 1;
static constexpr int LEDC_FREQ_HZ = 1000;

static constexpr uint32_t CONTROL_TASK_DELAY_MS = 2;
static constexpr uint32_t SOURCE_HOLD_MS = 1000;
static constexpr uint32_t WS_STATUS_PUSH_MS = 1000;
static constexpr uint32_t MAX_ARTNET_PACKETS_PER_LOOP = 64;
static constexpr uint32_t ARTNET_DRAIN_BUDGET_MS = 8;

static portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;

static float currentValues[4] = {0, 0, 0, 0};

struct StatusSnapshot {
  float ledValues[4];
  uint32_t lastArtMs;
  uint32_t artDmxRxTotal;
  uint32_t artDmxUniverseMatchTotal;
  uint32_t artDmxUniverseMismatchTotal;
  uint32_t artDmxInvalidTotal;
  uint32_t artDmxLastArrivalMs;
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

static void setLedFloat(int ledcChannel, float brightness01) {
  if (brightness01 < 0.0f) brightness01 = 0.0f;
  if (brightness01 > 1.0f) brightness01 = 1.0f;

  int duty = (int)(brightness01 * LEDC_MAX_DUTY + 0.5f);
  ledcWrite(ledcChannel, duty);

  portENTER_CRITICAL(&statusMux);
  currentValues[ledcChannel] = brightness01;
  portEXIT_CRITICAL(&statusMux);
}

static inline void writeValue(int ledcChannel, const uint8_t* data, uint16_t addr) {
  uint16_t value = ((uint16_t)data[addr] << 8) | data[addr + 1];
  setLedFloat(ledcChannel, (float)value / 65535.0f);
}

static void processArtnet(const ArtDmxPacket& a) {
  appMarkArtnetActivity();
  portENTER_CRITICAL(&statusMux);
  artLastUniverseFlat = a.universe_flat;
  portEXIT_CRITICAL(&statusMux);

  uint16_t universe = appConfig().getDMXUniverse();
  if (a.universe_flat != universe) {
    portENTER_CRITICAL(&statusMux);
    artDmxUniverseMismatchTotal++;
    portEXIT_CRITICAL(&statusMux);
    return;
  }

  uint16_t addr = appConfig().getDMXAddress();
  if (addr < 1 || addr + 7 > a.length) {
    return;
  }

  addr -= 1;
  writeValue(0, a.data, addr);
  writeValue(1, a.data, addr + 2);
  writeValue(2, a.data, addr + 4);
  writeValue(3, a.data, addr + 6);

  portENTER_CRITICAL(&statusMux);
  artDmxUniverseMatchTotal++;
  artDmxRxTotal++;
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
  for (int i = 0; i < 4; i++) snapshot.ledValues[i] = currentValues[i];
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
  StatusSnapshot snapshot = readStatusSnapshot();
  uint32_t now = millis();
  uint32_t artAge = (snapshot.lastArtMs == 0) ? 0xFFFFFFFFu : (now - snapshot.lastArtMs);

  j["source"] = (artAge <= SOURCE_HOLD_MS) ? "Art-Net" : "none";
  j["art_age_ms"] = (artAge == 0xFFFFFFFFu) ? -1 : (int32_t)artAge;
  j["wifi_rssi"] = appWifiManager().getRSSI();
  j["free_heap"] = ESP.getFreeHeap();
  j["min_free_heap"] = ESP.getMinFreeHeap();
  j["reset_reason"] = appResetReasonToString(esp_reset_reason());

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

  ledcSetup(0, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH1, 0);
  ledcSetup(1, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH2, 1);
  ledcSetup(2, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH3, 2);
  ledcSetup(3, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH4, 3);

  appInitRuntime({
    "/wifi-manager/index_led.html",
    "CableCar LED",
    buildStatusJson,
    buildHealthSummary,
    pollArtnet,
  });

  appConnectWifi();
  appStartCommonServices();

}

void loop() {
  appCommonLoop(WS_STATUS_PUSH_MS);
  delay(1);
}
