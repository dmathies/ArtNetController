#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <esp_http_server.h>
#include "WifiManager.h"     // your manager
#include "esp_dmx.h"         // someweisguy/esp_dmx
#include <ArduinoJson.h>
#include "bldc_uart.h"


Configuration config;
WifiManagerClass WifiManager(config);

// ---------- Pins ----------
static constexpr int CH1 = 1; 
static constexpr int CH2 = 2; 
static constexpr int CH3 = 3; 
static constexpr int CH4 = 21;

// ------ CONSTS -------------
// We want 12-bit resolution -> values 0..4095
const int LEDC_RES_BITS = 12;
const int LEDC_MAX_DUTY = (1 << LEDC_RES_BITS) - 1; // 4095
const int LEDC_FREQ_HZ = 1000;
static constexpr uint32_t CONTROL_TASK_DELAY_MS = 2;

// State
float currentValues[4];
static portMUX_TYPE statusMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t controlTaskHandle = nullptr;

// ---------- DMX (esp_dmx) ----------
static constexpr dmx_port_t DMX_PORT = DMX_NUM_1;  // use UART1 for DMX
static uint8_t dmx_buf[DMX_PACKET_SIZE];          // library's max DMX size

// Your config (pull these from your persisted config if you have it)
struct AppConfig {
  uint16_t dmx_start_address = config.getDMXAddress();  // 1..512
  uint16_t artnet_universe    = config.getDMXUniverse(); // 0..32767 (flat)
} CFG;

// ---------- Art-Net ----------
WiFiUDP artudp;
static constexpr uint16_t ARTNET_PORT = 6454;

struct ArtDmxPacket {
  bool ok = false;
  uint16_t universe_flat = 0;   // 0..32767
  uint16_t length = 0;          // 2..512
  const uint8_t* data = nullptr;
};

static ArtDmxPacket parseArtDmx(const uint8_t* p, int len) {
  ArtDmxPacket r;
  if (len < 18) return r;
  if (memcmp(p, "Art-Net\0", 8) != 0) return r;
  // OpCode ArtDMX 0x5000 (LE)
  if (!(p[8] == 0x00 && p[9] == 0x50)) return r;

  uint8_t subUni = p[14];            // high nibble=subnet, low nibble=universe
  uint8_t net    = p[15];
  uint16_t dlen  = ((uint16_t)p[16] << 8) | p[17];
  if (dlen > 512 || 18 + dlen > len) return r;

  uint8_t subnet = (subUni >> 4) & 0x0F;
  uint8_t uni    = (subUni     ) & 0x0F;
  uint16_t flat  = ((uint16_t)net << 8) | ((uint16_t)subnet << 4) | uni;

  r.ok = true; r.universe_flat = flat; r.length = dlen; r.data = p + 18;
  return r;
}

// ---------- Source arbitration (DMX wins) ----------
enum class Source { None, DMX, Artnet };
static uint32_t lastDmxMs = 0, lastArtMs = 0;
static AsyncWebSocket statusWs("/ws");
static uint32_t lastWsPushMs = 0;
static constexpr uint32_t WS_PUSH_INTERVAL_MS = 250;
static volatile bool controlTaskPaused = false;
static httpd_handle_t abHttpd = nullptr;

static constexpr uint32_t SOURCE_HOLD_MS = 1000;

struct StatusSnapshot {
  float ledValues[4];
  uint32_t lastDmxMs;
  uint32_t lastArtMs;
};

static StatusSnapshot readStatusSnapshot() {
  StatusSnapshot snapshot;
  portENTER_CRITICAL(&statusMux);
  for (int led = 0; led < 4; led++) {
    snapshot.ledValues[led] = currentValues[led];
  }
  snapshot.lastDmxMs = lastDmxMs;
  snapshot.lastArtMs = lastArtMs;
  portEXIT_CRITICAL(&statusMux);
  return snapshot;
}

static void setSourceTimestamp(Source source) {
  uint32_t now = millis();
  portENTER_CRITICAL(&statusMux);
  if (source == Source::DMX) {
    lastDmxMs = now;
  } else if (source == Source::Artnet) {
    lastArtMs = now;
  }
  portEXIT_CRITICAL(&statusMux);
}

static size_t buildStatusJson(char* out, size_t outSize) {
  StaticJsonDocument<256> j;
  uint32_t now = millis();
  char key[16];
  StatusSnapshot snapshot = readStatusSnapshot();

  const char* src =
    ((now - snapshot.lastDmxMs) <= SOURCE_HOLD_MS) ? "DMX" :
    ((now - snapshot.lastArtMs) <= SOURCE_HOLD_MS) ? "Art-Net" : "none";
  j["source"]      = src;
  j["dmx_age_ms"]  = now - snapshot.lastDmxMs;
  j["art_age_ms"]  = now - snapshot.lastArtMs;
  for (int led = 0; led < 4; led++) {
    snprintf(key, sizeof(key), "led_value%d", led);
    j[key] = snapshot.ledValues[led];
  }

  return serializeJson(j, out, outSize);
}

static void onStatusWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                            AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    char out[256];
    size_t outLen = buildStatusJson(out, sizeof(out));
    client->text(out, outLen);
  }
}

static esp_err_t abfastHandler(httpd_req_t* req) {
  uint32_t t0 = micros();
  const char payload[] = "{\"ok\":true}";
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  uint32_t genUs = micros() - t0;
  char genBuf[16];
  snprintf(genBuf, sizeof(genBuf), "%lu", (unsigned long)genUs);
  httpd_resp_set_hdr(req, "X-Gen-Us", genBuf);
  return httpd_resp_send(req, payload, HTTPD_RESP_USE_STRLEN);
}

static void startAbHttpServer() {
  if (abHttpd != nullptr) return;

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 8081;
  config.ctrl_port = 32769;
  config.max_uri_handlers = 4;

  if (httpd_start(&abHttpd, &config) == ESP_OK) {
    httpd_uri_t uri = {};
    uri.uri = "/abfast";
    uri.method = static_cast<http_method>(1);
    uri.handler = abfastHandler;
    uri.user_ctx = nullptr;
    httpd_register_uri_handler(abHttpd, &uri);
    Serial.println("A/B HTTP server started on :8081");
  } else {
    Serial.println("Failed to start A/B HTTP server");
  }
}

static inline Source activeSource() {
  StatusSnapshot snapshot = readStatusSnapshot();
  uint32_t now = millis();
  bool dmx = (now - snapshot.lastDmxMs) <= SOURCE_HOLD_MS;
  bool art = (now - snapshot.lastArtMs) <= SOURCE_HOLD_MS;
  if (dmx) return Source::DMX;
  if (art) return Source::Artnet;
  return Source::None;
}

static inline void processDmx(uint16_t len) {
  setSourceTimestamp(Source::DMX);
  uint16_t addr = CFG.dmx_start_address;
  if (addr < 1 || addr > len) return;
  uint8_t level = dmx_buf[addr];     // DMX buffer is 1-indexed (slot 1 at index 1)

  analogWrite(CH1, dmx_buf[addr]);
  analogWrite(CH2, dmx_buf[addr]+1);
  analogWrite(CH3, dmx_buf[addr]+2);    
  analogWrite(CH4, dmx_buf[addr]+3);
  //Serial.println("DMX received...");
}

// brightness is 0.0 to 1.0 (0%..100%)
void setLedFloat(int ledcChannel, float brightness01) {
  if (brightness01 < 0.0f) brightness01 = 0.0f;
  if (brightness01 > 1.0f) brightness01 = 1.0f;
  
  int duty = (int)(brightness01 * LEDC_MAX_DUTY + 0.5f);
  ledcWrite(ledcChannel, duty);
  portENTER_CRITICAL(&statusMux);
  currentValues[ledcChannel] = brightness01;
  portEXIT_CRITICAL(&statusMux);
}

// brightness is 0..4095 directly
void setLedRaw(int ledcChannel, int value0to4095) {
  if (value0to4095 < 0) value0to4095 = 0;
  if (value0to4095 > LEDC_MAX_DUTY) value0to4095 = LEDC_MAX_DUTY;
  ledcWrite(ledcChannel, value0to4095);
}

static inline void writeValue(int ledcChannel, const uint8_t *data, int addr)
{
  uint16_t value = data[addr] * 256 +data[addr+1];
  float brightness01 = (float)value  / 65535.0;

  setLedFloat(ledcChannel, brightness01);
}

static inline void processArtnet(const ArtDmxPacket& a) {
  setSourceTimestamp(Source::Artnet);
  if (a.universe_flat != CFG.artnet_universe) return;
  uint16_t addr = CFG.dmx_start_address;
  if (addr < 1 || addr > a.length) return;
  
  // Art-Net data is 0-indexed
  addr -=1;
  writeValue(0, a.data, addr);
  addr +=2;
  writeValue(1, a.data, addr);
  addr +=2;
  writeValue(2, a.data, addr);
  addr +=2;
  writeValue(3, a.data, addr);
}

static void controlTask(void* parameter) {
  while (true) {
    if (controlTaskPaused) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    dmx_packet_t packet;
    int size = dmx_receive(DMX_PORT, &packet, 0 /*ticks: 0 = no wait*/);
    if (size > 0) {
      dmx_read(DMX_PORT, dmx_buf, size);
      processDmx((uint16_t)size);
    }

    if (activeSource() != Source::DMX) {
      static uint8_t artbuf[600];
      int psize = artudp.parsePacket();
      if (psize > 0) {
        if (psize > (int)sizeof(artbuf)) psize = sizeof(artbuf);
        int n = artudp.read(artbuf, psize);
        if (n > 0) {
          ArtDmxPacket a = parseArtDmx(artbuf, n);
          if (a.ok) processArtnet(a);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(CONTROL_TASK_DELAY_MS));
  }
}



// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Serial.println("Startup...");

  ledcSetup(0, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH1, 0);
  ledcSetup(1, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH2, 1);
  ledcSetup(2, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH3, 2);
  ledcSetup(3, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(CH4, 3);

  // Wi-Fi via your WifiManager
  bool connected = WifiManager.connectToWifi();
  if (!connected) {
    Serial.println("No WiFi... Starting AP");
    WifiManager.startManagementAP();
  }
  IPAddress ip = WifiManager.getIP();
  Serial.printf("IP Address: %s\n", ip.toString().c_str());

  // WebSocket status stream (push updates)
  statusWs.onEvent(onStatusWsEvent);
  WifiManager.getServer().addHandler(&statusWs);
  startAbHttpServer();

  WifiManager.startManagementServerWeb();

  // Start Art-Net listener
  WiFi.setSleep(false);
  artudp.begin(ARTNET_PORT);

  // ---- DMX driver (esp_dmx) ----
  // 1) default config
  dmx_config_t dmx_config = DMX_CONFIG_DEFAULT;

  // 2) at least one "personality" is required by the driver
  const int personality_count = 1;
  dmx_personality_t personalities[] = {
      {1, "RX Only"}
  };

  // 3) install driver on UART2
  dmx_driver_install(DMX_PORT, &dmx_config, personalities, personality_count);  // API per docs
  // 4) set pins (TX unused; RX=26; RTS=25)

  xTaskCreatePinnedToCore(
    controlTask,
    "ControlTask",
    4096,
    nullptr,
    1,
    &controlTaskHandle,
    1);

    // --- /status endpoint (JSON for UI polling) ---
  WifiManager.getServer().on("/status", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest* req){
    uint32_t t0 = micros();
    char out[256];
    size_t outLen = buildStatusJson(out, sizeof(out));
    auto* res = req->beginResponse(200, "application/json", reinterpret_cast<const uint8_t*>(out), outLen);
    uint32_t genUs = micros() - t0;
    res->addHeader("X-Status-Gen-Us", String(genUs));
    res->addHeader("Cache-Control", "no-store");
    req->send(res);
  });

  // --- diagnostics ---
  WifiManager.getServer().on("/diag/fast", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest* req){
    uint32_t t0 = micros();
    const char payload[] = "{\"ok\":true}";
    auto* res = req->beginResponse(200, "application/json", reinterpret_cast<const uint8_t*>(payload), sizeof(payload) - 1);
    res->addHeader("X-Gen-Us", String(micros() - t0));
    res->addHeader("Cache-Control", "no-store");
    req->send(res);
  });

  WifiManager.getServer().on("/diag/control", AsyncWebRequestMethod::HTTP_GET, [](AsyncWebServerRequest* req){
    if (req->hasParam("pause")) {
      String value = req->getParam("pause")->value();
      controlTaskPaused = (value == "1" || value == "true");
    }

    StaticJsonDocument<96> j;
    j["paused"] = controlTaskPaused;
    j["uptime_ms"] = millis();
    char out[96];
    size_t outLen = serializeJson(j, out, sizeof(out));
    auto* res = req->beginResponse(200, "application/json", reinterpret_cast<const uint8_t*>(out), outLen);
    res->addHeader("Cache-Control", "no-store");
    req->send(res);
  });

}

// ---------- Loop ----------
void loop() {
  // Keep Wi-Fi alive
  WifiManager.check();

  // Push status to websocket clients at a fixed cadence.
  uint32_t now = millis();
  if (now - lastWsPushMs >= WS_PUSH_INTERVAL_MS) {
    char out[256];
    size_t outLen = buildStatusJson(out, sizeof(out));
    statusWs.textAll(out, outLen);
    lastWsPushMs = now;
  }
  statusWs.cleanupClients();

  // Yield briefly so Wi-Fi / async networking tasks stay responsive.
  delay(1);
}
