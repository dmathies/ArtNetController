#include "BleManager.h"

#include <ArduinoJson.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "RemoteLogBuffer.h"
#include "main_common.h"

namespace {
constexpr char BLE_SERVICE_UUID[] = "8f2a2000-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_WIFI_RSSI_UUID[] = "8f2a2002-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_WIFI_IP_UUID[] = "8f2a2003-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_LAST_LOG_UUID[] = "8f2a2004-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_LOG_TAIL_UUID[] = "8f2a2005-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_REBOOT_UUID[] = "8f2a2006-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_RESULT_UUID[] = "8f2a2007-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_SSID_UUID[] = "8f2a2010-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_PASSWORD_UUID[] = "8f2a2011-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_HOSTNAME_UUID[] = "8f2a2012-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_DHCP_UUID[] = "8f2a2013-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_IP_UUID[] = "8f2a2014-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_GATEWAY_UUID[] = "8f2a2015-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_SUBNET_UUID[] = "8f2a2016-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_DNS1_UUID[] = "8f2a2017-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_DNS2_UUID[] = "8f2a2018-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_START_VALUE_UUID[] = "8f2a2019-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_DMX_ADDRESS_UUID[] = "8f2a201a-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_DMX_UNIVERSE_UUID[] = "8f2a201b-1e8e-4f4c-a8c0-6c5b7d019000";

constexpr uint32_t BLE_STATUS_REFRESH_MS = 1000;
constexpr size_t BLE_LOG_TAIL_BYTES = 512;
constexpr uint16_t BLE_SERVICE_HANDLE_COUNT = 64;

enum class ConfigField {
  Ssid,
  Password,
  Hostname,
  Dhcp,
  Ip,
  Gateway,
  Subnet,
  Dns1,
  Dns2,
  StartValue,
  DmxAddress,
  DmxUniverse,
};

BLEServer* g_server = nullptr;
BLECharacteristic* g_wifiRssiChar = nullptr;
BLECharacteristic* g_wifiIpChar = nullptr;
BLECharacteristic* g_lastLogChar = nullptr;
BLECharacteristic* g_logTailChar = nullptr;
BLECharacteristic* g_rebootChar = nullptr;
BLECharacteristic* g_resultChar = nullptr;
BLECharacteristic* g_ssidChar = nullptr;
BLECharacteristic* g_passwordChar = nullptr;
BLECharacteristic* g_hostnameChar = nullptr;
BLECharacteristic* g_dhcpChar = nullptr;
BLECharacteristic* g_ipChar = nullptr;
BLECharacteristic* g_gatewayChar = nullptr;
BLECharacteristic* g_subnetChar = nullptr;
BLECharacteristic* g_dns1Char = nullptr;
BLECharacteristic* g_dns2Char = nullptr;
BLECharacteristic* g_startValueChar = nullptr;
BLECharacteristic* g_dmxAddressChar = nullptr;
BLECharacteristic* g_dmxUniverseChar = nullptr;
bool g_bleStarted = false;
bool g_bleClientConnected = false;
uint32_t g_lastStatusRefreshMs = 0;
bool g_statusRefreshPending = false;

String trimAscii(const std::string& raw) {
  String value(raw.c_str());
  value.trim();
  return value;
}

bool containsInvalidText(const String& value, bool allowEmpty, size_t maxLen) {
  if (value.length() == 0) return !allowEmpty;
  if (value.length() > maxLen) return true;
  for (size_t i = 0; i < value.length(); ++i) {
    unsigned char ch = (unsigned char)value[i];
    if (ch < 32 || ch == 127) return true;
  }
  return false;
}

bool parseBoolText(const String& value, bool& out) {
  String normalized = value;
  normalized.trim();
  normalized.toLowerCase();
  if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on" || normalized == "dhcp") {
    out = true;
    return true;
  }
  if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off" || normalized == "static") {
    out = false;
    return true;
  }
  return false;
}

void setUserDescription(BLECharacteristic* characteristic, const char* label) {
  if (!characteristic || !label) return;
  BLEDescriptor* descriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  descriptor->setValue((uint8_t*)label, strlen(label));
  characteristic->addDescriptor(descriptor);
}

String buildAdvertisedName() {
  WifiManagerClass& wifi = appWifiManager();
  String hostname = wifi.getHostname();
  if (hostname.length() > 0) return hostname;

  char mac[18];
  wifi.getMacAddress(mac, sizeof(mac));
  String suffix = String(mac);
  suffix.replace(":", "");
  if (suffix.length() > 4) suffix = suffix.substring(suffix.length() - 4);
  return String(appGetDeviceName()) + "-" + suffix;
}

void setResult(const String& message) {
  if (!g_resultChar) return;
  g_resultChar->setValue(message.c_str());
  if (g_bleClientConnected) {
    g_resultChar->notify();
  }
}

void refreshConfigCharacteristics() {
  Configuration& config = appConfig();
  if (g_ssidChar) g_ssidChar->setValue(config.getSSID().c_str());
  if (g_passwordChar) g_passwordChar->setValue(config.getPass().c_str());
  if (g_hostnameChar) g_hostnameChar->setValue(config.getHostname().c_str());
  if (g_dhcpChar) g_dhcpChar->setValue(config.getDhcpEnabled() ? "1" : "0");
  if (g_ipChar) g_ipChar->setValue(config.getStaticIP().c_str());
  if (g_gatewayChar) g_gatewayChar->setValue(config.getGateway().c_str());
  if (g_subnetChar) g_subnetChar->setValue(config.getSubnet().c_str());
  if (g_dns1Char) g_dns1Char->setValue(config.getDNS1().c_str());
  if (g_dns2Char) g_dns2Char->setValue(config.getDNS2().c_str());

  char numberBuf[32];
  snprintf(numberBuf, sizeof(numberBuf), "%d", config.getDMXAddress());
  if (g_dmxAddressChar) g_dmxAddressChar->setValue(numberBuf);
  snprintf(numberBuf, sizeof(numberBuf), "%d", config.getDMXUniverse());
  if (g_dmxUniverseChar) g_dmxUniverseChar->setValue(numberBuf);
  snprintf(numberBuf, sizeof(numberBuf), "%.6g", (double)config.getStartValue());
  if (g_startValueChar) g_startValueChar->setValue(numberBuf);
}

void refreshStatusCharacteristics(bool notify) {
  WifiManagerClass& wifi = appWifiManager();
  char ipBuf[16];
  IPAddress ip = wifi.getIP();
  snprintf(ipBuf, sizeof(ipBuf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);

  char rssiBuf[16];
  snprintf(rssiBuf, sizeof(rssiBuf), "%d", (int)wifi.getRSSI());

  String lastLog = appLogGetLastLine();
  String logTail = appLogGetTail(BLE_LOG_TAIL_BYTES);

  if (g_wifiIpChar) g_wifiIpChar->setValue(ipBuf);
  if (g_wifiRssiChar) g_wifiRssiChar->setValue(rssiBuf);
  if (g_lastLogChar) g_lastLogChar->setValue(lastLog.c_str());
  if (g_logTailChar) g_logTailChar->setValue(logTail.c_str());

  if (notify && g_bleClientConnected) {
    if (g_wifiIpChar) g_wifiIpChar->notify();
    if (g_wifiRssiChar) g_wifiRssiChar->notify();
    if (g_lastLogChar) g_lastLogChar->notify();
  }
}

bool applyConfigField(ConfigField field, const String& value, String& message) {
  Configuration& config = appConfig();
  switch (field) {
    case ConfigField::Ssid:
      if (containsInvalidText(value, false, 32)) {
        message = "Invalid SSID";
        return false;
      }
      if (!config.writeSSID(value)) {
        message = "Failed to write SSID";
        return false;
      }
      message = "SSID updated; reboot scheduled";
      return true;
    case ConfigField::Password:
      if (value.length() > 63) {
        message = "Password too long";
        return false;
      }
      if (!config.writePass(value)) {
        message = "Failed to write password";
        return false;
      }
      message = "Password updated; reboot scheduled";
      return true;
    case ConfigField::Hostname:
      if (containsInvalidText(value, true, 63)) {
        message = "Invalid hostname";
        return false;
      }
      if (!config.writeHostname(value)) {
        message = "Failed to write hostname";
        return false;
      }
      message = "Hostname updated; reboot scheduled";
      return true;
    case ConfigField::Dhcp: {
      bool enabled = true;
      if (!parseBoolText(value, enabled)) {
        message = "Invalid DHCP value";
        return false;
      }
      if (!config.writeDhcpEnabled(enabled)) {
        message = "Failed to write DHCP";
        return false;
      }
      message = enabled ? "DHCP enabled; reboot scheduled" : "Static IP mode; reboot scheduled";
      return true;
    }
    case ConfigField::Ip:
      if (!config.writeStaticIP(value)) {
        message = "Failed to write IP";
        return false;
      }
      message = "Static IP updated; reboot scheduled";
      return true;
    case ConfigField::Gateway:
      if (!config.writeGateway(value)) {
        message = "Failed to write gateway";
        return false;
      }
      message = "Gateway updated; reboot scheduled";
      return true;
    case ConfigField::Subnet:
      if (!config.writeSubnet(value)) {
        message = "Failed to write subnet";
        return false;
      }
      message = "Subnet updated; reboot scheduled";
      return true;
    case ConfigField::Dns1:
      if (!config.writeDNS1(value)) {
        message = "Failed to write DNS1";
        return false;
      }
      message = "DNS1 updated; reboot scheduled";
      return true;
    case ConfigField::Dns2:
      if (!config.writeDNS2(value)) {
        message = "Failed to write DNS2";
        return false;
      }
      message = "DNS2 updated; reboot scheduled";
      return true;
    case ConfigField::StartValue: {
      char* end = nullptr;
      float parsed = strtof(value.c_str(), &end);
      if (!end || *end != '\0') {
        message = "Invalid start value";
        return false;
      }
      if (!config.writeStartValue(parsed)) {
        message = "Failed to write start value";
        return false;
      }
      message = "Start value updated; reboot scheduled";
      return true;
    }
    case ConfigField::DmxAddress: {
      char* end = nullptr;
      long parsed = strtol(value.c_str(), &end, 10);
      if (!end || *end != '\0' || parsed < 1 || parsed > 512) {
        message = "DMX address must be 1..512";
        return false;
      }
      if (!config.writeDMXAddress((int)parsed)) {
        message = "Failed to write DMX address";
        return false;
      }
      message = "DMX address updated; reboot scheduled";
      return true;
    }
    case ConfigField::DmxUniverse: {
      char* end = nullptr;
      long parsed = strtol(value.c_str(), &end, 10);
      if (!end || *end != '\0' || parsed < 0 || parsed > 32767) {
        message = "Universe must be 0..32767";
        return false;
      }
      if (!config.writeDMXUniverse((int)parsed)) {
        message = "Failed to write universe";
        return false;
      }
      message = "Universe updated; reboot scheduled";
      return true;
    }
  }

  message = "Unsupported field";
  return false;
}

class ServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer* server) override {
    (void)server;
    g_bleClientConnected = true;
    g_statusRefreshPending = true;
    g_lastStatusRefreshMs = 0;
    setResult("BLE client connected");
  }

  void onDisconnect(BLEServer* server) override {
    (void)server;
    g_bleClientConnected = false;
    g_statusRefreshPending = false;
    BLEDevice::startAdvertising();
  }
};

class ConfigCallbacks : public BLECharacteristicCallbacks {
 public:
  explicit ConfigCallbacks(ConfigField field) : field_(field) {}

  void onWrite(BLECharacteristic* characteristic) override {
    String value = trimAscii(characteristic->getValue());
    String message;
    if (!applyConfigField(field_, value, message)) {
      setResult(message);
      refreshConfigCharacteristics();
      return;
    }

    refreshConfigCharacteristics();
    appLogLine(message.c_str());
    setResult(message);
    appWifiManager().scheduleRestart(1000);
  }

 private:
  ConfigField field_;
};

class RebootCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic) override {
    String value = trimAscii(characteristic->getValue());
    value.toLowerCase();
    if (value != "1" && value != "reboot" && value != "restart") {
      setResult("Write '1' or 'reboot' to restart");
      return;
    }

    appLogLine("BLE reboot requested");
    setResult("Reboot scheduled");
    appWifiManager().scheduleRestart(500);
  }
};

BLECharacteristic* createCharacteristic(BLEService* service,
                                        const char* uuid,
                                        uint32_t properties,
                                        const char* label) {
  BLECharacteristic* characteristic = service->createCharacteristic(uuid, properties);
  setUserDescription(characteristic, label);
  return characteristic;
}

}  // namespace

void appStartBleServices() {
  if (g_bleStarted) return;

  String deviceName = buildAdvertisedName();
  BLEDevice::init(deviceName.c_str());
  BLEDevice::setMTU(247);

  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());

  // The Arduino BLE wrapper defaults services to 15 handles, which is too small
  // once we account for our characteristics plus descriptors.
  BLEService* service = g_server->createService(BLEUUID(BLE_SERVICE_UUID), BLE_SERVICE_HANDLE_COUNT);

  g_wifiRssiChar = createCharacteristic(service,
                                        BLE_CHAR_WIFI_RSSI_UUID,
                                        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY,
                                        "wifi_rssi");
  g_wifiRssiChar->addDescriptor(new BLE2902());
  g_wifiIpChar = createCharacteristic(service,
                                      BLE_CHAR_WIFI_IP_UUID,
                                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY,
                                      "wifi_ip");
  g_wifiIpChar->addDescriptor(new BLE2902());
  g_lastLogChar = createCharacteristic(service,
                                       BLE_CHAR_LAST_LOG_UUID,
                                       BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY,
                                       "last_log_line");
  g_lastLogChar->addDescriptor(new BLE2902());
  g_logTailChar = createCharacteristic(service,
                                       BLE_CHAR_LOG_TAIL_UUID,
                                       BLECharacteristic::PROPERTY_READ,
                                       "log_tail");
  g_rebootChar = createCharacteristic(service,
                                      BLE_CHAR_REBOOT_UUID,
                                      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR,
                                      "reboot");
  g_resultChar = createCharacteristic(service,
                                      BLE_CHAR_RESULT_UUID,
                                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY,
                                      "result");
  g_resultChar->addDescriptor(new BLE2902());

  g_ssidChar = createCharacteristic(service,
                                    BLE_CHAR_SSID_UUID,
                                    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_WRITE_NR,
                                    "ssid");
  g_passwordChar = createCharacteristic(service,
                                        BLE_CHAR_PASSWORD_UUID,
                                        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_WRITE_NR,
                                        "password");
  g_hostnameChar = createCharacteristic(service,
                                        BLE_CHAR_HOSTNAME_UUID,
                                        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_WRITE_NR,
                                        "hostname");
  g_dhcpChar = createCharacteristic(service,
                                    BLE_CHAR_DHCP_UUID,
                                    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_WRITE_NR,
                                    "dhcp");
  g_ipChar = createCharacteristic(service,
                                  BLE_CHAR_IP_UUID,
                                  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                      BLECharacteristic::PROPERTY_WRITE_NR,
                                  "ip");
  g_gatewayChar = createCharacteristic(service,
                                       BLE_CHAR_GATEWAY_UUID,
                                       BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_WRITE_NR,
                                       "gateway");
  g_subnetChar = createCharacteristic(service,
                                      BLE_CHAR_SUBNET_UUID,
                                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                          BLECharacteristic::PROPERTY_WRITE_NR,
                                      "subnet");
  g_dns1Char = createCharacteristic(service,
                                    BLE_CHAR_DNS1_UUID,
                                    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_WRITE_NR,
                                    "dns1");
  g_dns2Char = createCharacteristic(service,
                                    BLE_CHAR_DNS2_UUID,
                                    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                        BLECharacteristic::PROPERTY_WRITE_NR,
                                    "dns2");
  g_startValueChar = createCharacteristic(service,
                                          BLE_CHAR_START_VALUE_UUID,
                                          BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                              BLECharacteristic::PROPERTY_WRITE_NR,
                                          "start_value");
  g_dmxAddressChar = createCharacteristic(service,
                                          BLE_CHAR_DMX_ADDRESS_UUID,
                                          BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                              BLECharacteristic::PROPERTY_WRITE_NR,
                                          "dmx_address");
  g_dmxUniverseChar = createCharacteristic(service,
                                           BLE_CHAR_DMX_UNIVERSE_UUID,
                                           BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
                                               BLECharacteristic::PROPERTY_WRITE_NR,
                                           "dmx_universe");

  g_ssidChar->setCallbacks(new ConfigCallbacks(ConfigField::Ssid));
  g_passwordChar->setCallbacks(new ConfigCallbacks(ConfigField::Password));
  g_hostnameChar->setCallbacks(new ConfigCallbacks(ConfigField::Hostname));
  g_dhcpChar->setCallbacks(new ConfigCallbacks(ConfigField::Dhcp));
  g_ipChar->setCallbacks(new ConfigCallbacks(ConfigField::Ip));
  g_gatewayChar->setCallbacks(new ConfigCallbacks(ConfigField::Gateway));
  g_subnetChar->setCallbacks(new ConfigCallbacks(ConfigField::Subnet));
  g_dns1Char->setCallbacks(new ConfigCallbacks(ConfigField::Dns1));
  g_dns2Char->setCallbacks(new ConfigCallbacks(ConfigField::Dns2));
  g_startValueChar->setCallbacks(new ConfigCallbacks(ConfigField::StartValue));
  g_dmxAddressChar->setCallbacks(new ConfigCallbacks(ConfigField::DmxAddress));
  g_dmxUniverseChar->setCallbacks(new ConfigCallbacks(ConfigField::DmxUniverse));
  g_rebootChar->setCallbacks(new RebootCallbacks());

  refreshConfigCharacteristics();
  refreshStatusCharacteristics(false);
  setResult("ready");

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  appLogPrintf("BLE service started as %s\n", deviceName.c_str());
  g_bleStarted = true;
}

void appBleLoop() {
  if (!g_bleStarted) return;

  uint32_t now = millis();
  if (!g_statusRefreshPending && (now - g_lastStatusRefreshMs) < BLE_STATUS_REFRESH_MS) return;
  g_lastStatusRefreshMs = now;
  bool notify = !g_statusRefreshPending;
  g_statusRefreshPending = false;
  refreshStatusCharacteristics(notify);
}
