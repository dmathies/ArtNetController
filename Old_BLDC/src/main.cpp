#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "WifiManager.h"     // your manager
#include "esp_dmx.h"         // someweisguy/esp_dmx
#include <ArduinoJson.h>
#include "bldc_uart.h"


Configuration config;
WifiManagerClass WifiManager(config);

// ---------- Pins ----------
static constexpr int BLDC_TX = 33;   // your BLDC controller: TX on P21
static constexpr int BLDC_RX = 32;   // optional RX from BLDC (if unused, still set)
static constexpr int DMX_RX  = 26;   // RS-485 RO -> ESP32 RX (DMX in)
static constexpr int DMX_TX = 25;   // RS-485 DE & /RE tied together -> direction pin
static constexpr int UART_OE = 19;   

// ---------- BLDC UART ----------
HardwareSerial BLDCSerial(2);  // UART2
BLDC::Driver bldc(BLDCSerial, BLDC_RX, BLDC_TX, /*baud=*/9600);


// -------- Motor queue/task --------
struct MotorCmd { uint16_t value; };
static QueueHandle_t g_motorQ;
static volatile uint16_t g_motorValue = 0;    // last commanded (for /status)
static volatile uint32_t g_motorRxCount = 0;  // optional monitor

// Example BLDC command (replace with your protocol)
static inline void sendBLDCCommand(uint16_t value) {

  bldc.sendSpeed(value);
  Serial.printf("Motor value: %d\n", (int)value);
}

// Dedicated task
void MotorTask(void*) {

    bldc.onFrame = [](const uint8_t* f){
    // f[2]=cmd, f[3]=p1, f[4]=p2, f[5]=p3
    Serial.printf("[BLDC RX] %02X %02X %02X %02X %02X %02X %02X  (cmd=0x%02X p1=%u p2=%u p3=%u)\n",
      f[0],f[1],f[2],f[3],f[4],f[5],f[6], f[2], f[3], f[4], f[5]);
  };

  for (;;) {
    MotorCmd cmd;
    if (xQueueReceive(g_motorQ, &cmd, pdMS_TO_TICKS(200))) {
      g_motorValue = cmd.value;
    }

    sendBLDCCommand(cmd.value);
    bldc.poll();

    // Optional: read telemetry
    // while (BLDC.available() > 0) {
    //   (void)BLDC.read();
    //   g_motorRxCount++;
    // }
  }
}

// Enqueue setpoint (non-blocking overwrite style)
static inline void enqueueMotor(uint16_t value) {
  MotorCmd cmd{value};
  xQueueOverwrite(g_motorQ, &cmd); // single-slot queue: latest wins
}

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

static constexpr uint32_t SOURCE_HOLD_MS = 1000;

static inline Source activeSource() {
  uint32_t now = millis();
  bool dmx = (now - lastDmxMs) <= SOURCE_HOLD_MS;
  bool art = (now - lastArtMs) <= SOURCE_HOLD_MS;
  if (dmx) return Source::DMX;
  if (art) return Source::Artnet;
  return Source::None;
}

static inline void processDmx(uint16_t len) {
  lastDmxMs = millis();
  uint16_t addr = CFG.dmx_start_address;
  if (addr < 1 || addr > len) return;
  uint8_t level = dmx_buf[addr];     // DMX buffer is 1-indexed (slot 1 at index 1)

  Serial.println("DMX received...");

  enqueueMotor(level);
}

static inline void processArtnet(const ArtDmxPacket& a) {
  lastArtMs = millis();
  if (a.universe_flat != CFG.artnet_universe) return;
  uint16_t addr = CFG.dmx_start_address;
  if (addr < 1 || addr > a.length) return;
  uint8_t level = a.data[addr - 1];  // Art-Net data is 0-indexed
  Serial.println("Art-Net received...");

  enqueueMotor(level);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Serial.println("Startup...");
  bldc.begin();
  bldc.sendSpeed(0);
  
  pinMode(DMX_TX, OUTPUT);
  digitalWrite(DMX_TX, LOW);   // keep module in receive

  pinMode(UART_OE, OUTPUT);
  digitalWrite(UART_OE, LOW);   // enable output

  // Wi-Fi via your WifiManager
  bool connected = WifiManager.connectToWifi();
  if (!connected) {
    Serial.println("No WiFi... Starting AP");
    WifiManager.startManagementAP();
  }
  IPAddress ip = WifiManager.getIP();
  Serial.printf("IP Address: %s\n", ip.toString().c_str());
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
  dmx_set_pin(DMX_PORT, DMX_PIN_NO_CHANGE, DMX_RX, DMX_PIN_NO_CHANGE);       
  // Done. We'll call dmx_receive()/dmx_read() in loop.

  // Motor queue & task
  g_motorQ = xQueueCreate(1, sizeof(MotorCmd));         // single-slot "latest wins"
  xTaskCreatePinnedToCore(MotorTask, "MotorTask", 4096, nullptr, 2, nullptr, 0);

    // --- /status endpoint (JSON for UI polling) ---
  WifiManager.getServer().on("/status", HTTP_GET, [](AsyncWebServerRequest* req){
    StaticJsonDocument<256> j;
    uint32_t now = millis();
    const char* src =
      ((now - lastDmxMs) <= SOURCE_HOLD_MS) ? "DMX" :
      ((now - lastArtMs) <= SOURCE_HOLD_MS) ? "Art-Net" : "none";
    j["source"]      = src;
    j["dmx_age_ms"]  = now - lastDmxMs;
    j["art_age_ms"]  = now - lastArtMs;
    j["motor_value"] = g_motorValue;

    String out; serializeJson(j, out);
    auto* res = req->beginResponse(200, "application/json", out);
    res->addHeader("Cache-Control", "no-store");
    req->send(res);
  });

}

// ---------- Loop ----------
void loop() {
  // Keep Wi-Fi alive
  WifiManager.check();

  // ---- DMX (non-blocking) ----
  dmx_packet_t packet;
  int size = dmx_receive(DMX_PORT, &packet, 0 /*ticks: 0 = no wait*/);  // returns >0 when a new frame is ready
  if (size > 0) {
    // Read the received frame into dmx_buf; index 0 is start code, slot1 is at [1]
    dmx_read(DMX_PORT, dmx_buf, size);
    processDmx((uint16_t)size);
  }

  // ---- Art-Net (ignored while DMX is active) ----
  static uint8_t artbuf[600];
  int psize = artudp.parsePacket();
  if (psize > 0) {
    if (psize > (int)sizeof(artbuf)) psize = sizeof(artbuf);
    int n = artudp.read(artbuf, psize);
    if (n > 0 && activeSource() != Source::DMX) {
      ArtDmxPacket a = parseArtDmx(artbuf, n);
      if (a.ok) processArtnet(a);
    }
  }
}
