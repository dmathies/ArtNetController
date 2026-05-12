#include "BleManager.h"

#ifndef APP_BLE_ENABLE
#define APP_BLE_ENABLE 1
#endif

#ifndef APP_BLE_LOG_TAIL_BYTES
#define APP_BLE_LOG_TAIL_BYTES 512
#endif

#ifndef APP_BLE_LAST_LOG_BYTES
#define APP_BLE_LAST_LOG_BYTES 160
#endif

#ifndef APP_BLE_MTU
#define APP_BLE_MTU 247
#endif

#ifndef APP_BLE_GATT_ENABLE
#define APP_BLE_GATT_ENABLE 1
#endif

#if APP_BLE_ENABLE

#include <cstring>
#include <NimBLEDevice.h>

#include "BuildMetadata.h"
#include "RemoteLogBuffer.h"
#include "main_common.h"

namespace {
constexpr char BLE_SERVICE_UUID[] = "8f2a2000-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_WIFI_RSSI_UUID[] = "8f2a2002-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_WIFI_IP_UUID[] = "8f2a2003-1e8e-4f4c-a8c0-6c5b7d019000";
constexpr char BLE_CHAR_WIFI_MAC_UUID[] = "8f2a2008-1e8e-4f4c-a8c0-6c5b7d019000";
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
constexpr size_t BLE_LOG_TAIL_BYTES = APP_BLE_LOG_TAIL_BYTES;
constexpr size_t BLE_LAST_LOG_BYTES = APP_BLE_LAST_LOG_BYTES;
constexpr uint16_t BLE_ADV_INTERVAL_MIN = 0x320;  // 500 ms
constexpr uint16_t BLE_ADV_INTERVAL_MAX = 0x320;  // 500 ms
constexpr size_t BLE_PENDING_VALUE_MAX = 96;
constexpr uint16_t BLE_MANUFACTURER_ID = 0xFFFF;
constexpr uint8_t BLE_ADV_METADATA_VERSION = 1;
constexpr uint8_t BLE_ADV_FLAG_IP_PRESENT = 0x01;
constexpr uint32_t BLE_ADV_REFRESH_MS = 1000;
constexpr size_t BLE_RSSI_TEXT_BYTES = 16;

using AdvertisedVariant = AppVariantKind;

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

const char* configFieldName(ConfigField field) {
  switch (field) {
    case ConfigField::Ssid:
      return "ssid";
    case ConfigField::Password:
      return "password";
    case ConfigField::Hostname:
      return "hostname";
    case ConfigField::Dhcp:
      return "dhcp";
    case ConfigField::Ip:
      return "ip";
    case ConfigField::Gateway:
      return "gateway";
    case ConfigField::Subnet:
      return "subnet";
    case ConfigField::Dns1:
      return "dns1";
    case ConfigField::Dns2:
      return "dns2";
    case ConfigField::StartValue:
      return "start_value";
    case ConfigField::DmxAddress:
      return "dmx_address";
    case ConfigField::DmxUniverse:
      return "dmx_universe";
  }

  return "unknown";
}

String configValuePreview(ConfigField field, const String& value) {
  if (field == ConfigField::Password) {
    return String("<len=") + value.length() + ">";
  }

  if (value.length() == 0) return "<empty>";
  if (value.length() <= 48) return value;
  return value.substring(0, 48) + "...";
}

BLEServer* g_server = nullptr;
BLECharacteristic* g_wifiRssiChar = nullptr;
BLECharacteristic* g_wifiIpChar = nullptr;
BLECharacteristic* g_wifiMacChar = nullptr;
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
uint32_t g_lastAdvRefreshMs = 0;
char g_lastNotifiedLogLine[BLE_LAST_LOG_BYTES + 1] = {0};
char g_lastNotifiedRssi[BLE_RSSI_TEXT_BYTES] = {0};
std::string g_lastAdvertisingMetadata;
std::string g_lastScanResponseMetadata;
String g_lastAdvertisedName;
portMUX_TYPE g_bleMux = portMUX_INITIALIZER_UNLOCKED;

struct PendingBleWrite {
  bool pending = false;
  bool isReboot = false;
  ConfigField field = ConfigField::Ssid;
  char value[BLE_PENDING_VALUE_MAX] = {0};
};

PendingBleWrite g_pendingBleWrite;

String trimAscii(const std::string& raw) {
  String value(raw.c_str());
  value.trim();
  return value;
}

void queueBleWrite(ConfigField field, const String& value) {
  portENTER_CRITICAL(&g_bleMux);
  g_pendingBleWrite.pending = true;
  g_pendingBleWrite.isReboot = false;
  g_pendingBleWrite.field = field;
  value.toCharArray(g_pendingBleWrite.value, BLE_PENDING_VALUE_MAX);
  portEXIT_CRITICAL(&g_bleMux);
}

void queueBleReboot(const String& value) {
  portENTER_CRITICAL(&g_bleMux);
  g_pendingBleWrite.pending = true;
  g_pendingBleWrite.isReboot = true;
  value.toCharArray(g_pendingBleWrite.value, BLE_PENDING_VALUE_MAX);
  portEXIT_CRITICAL(&g_bleMux);
}

bool takePendingBleWrite(PendingBleWrite& out) {
  bool hasPending = false;
  portENTER_CRITICAL(&g_bleMux);
  if (g_pendingBleWrite.pending) {
    out = g_pendingBleWrite;
    g_pendingBleWrite.pending = false;
    g_pendingBleWrite.value[0] = '\0';
    hasPending = true;
  }
  portEXIT_CRITICAL(&g_bleMux);
  return hasPending;
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
  BLEDescriptor* descriptor = characteristic->createDescriptor("2901", NIMBLE_PROPERTY::READ, strlen(label) + 1);
  if (descriptor) {
    descriptor->setValue(label);
  }
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

bool parseMacAddressBytes(const char* macText, uint8_t out[6]) {
  if (!macText || !out) return false;
  unsigned int parts[6];
  int matched = sscanf(macText, "%2x:%2x:%2x:%2x:%2x:%2x",
                       &parts[0],
                       &parts[1],
                       &parts[2],
                       &parts[3],
                       &parts[4],
                       &parts[5]);
  if (matched != 6) return false;
  for (int i = 0; i < 6; ++i) out[i] = (uint8_t)parts[i];
  return true;
}

void parseVersionTriplet(uint8_t out[3]) {
  if (!out) return;
  out[0] = 0;
  out[1] = 0;
  out[2] = 0;

  unsigned int major = 0;
  unsigned int minor = 0;
  unsigned int patch = 0;
  int matched = sscanf(APP_BUILD_VERSION, "%u.%u.%u", &major, &minor, &patch);
  if (matched >= 1) out[0] = (uint8_t)(major > 255 ? 255 : major);
  if (matched >= 2) out[1] = (uint8_t)(minor > 255 ? 255 : minor);
  if (matched >= 3) out[2] = (uint8_t)(patch > 255 ? 255 : patch);
}

bool readAdvertisedStatusPayload(AdvertisedVariant& variantOut, uint16_t& normalizedValueOut) {
  AppVariantStatus status = appGetVariantStatus();
  variantOut = status.variant;
  normalizedValueOut = status.normalizedValue;
  return variantOut != AdvertisedVariant::Unknown;
}

std::string buildAdvertisingMetadata() {
  WifiManagerClass& wifi = appWifiManager();
  char macBuf[18];
  uint8_t macBytes[6] = {0};
  wifi.getMacAddress(macBuf, sizeof(macBuf));
  parseMacAddressBytes(macBuf, macBytes);
  IPAddress ip = wifi.getIP();
  bool hasIp = ip != IPAddress((uint32_t)0);

  std::string data;
  data.reserve(14);
  data.push_back((char)(BLE_MANUFACTURER_ID & 0xFF));
  data.push_back((char)((BLE_MANUFACTURER_ID >> 8) & 0xFF));
  data.push_back((char)BLE_ADV_METADATA_VERSION);
  data.push_back((char)(hasIp ? BLE_ADV_FLAG_IP_PRESENT : 0));
  data.append((const char*)macBytes, sizeof(macBytes));
  if (hasIp) {
    uint8_t ipBytes[4] = {ip[0], ip[1], ip[2], ip[3]};
    data.append((const char*)ipBytes, sizeof(ipBytes));
  }
  return data;
}

std::string buildScanResponseMetadata() {
  AdvertisedVariant variant = AdvertisedVariant::Unknown;
  uint16_t normalizedValue = 0;
  readAdvertisedStatusPayload(variant, normalizedValue);
  uint8_t version[3];
  parseVersionTriplet(version);

  std::string data;
  data.reserve(9);
  data.push_back((char)(BLE_MANUFACTURER_ID & 0xFF));
  data.push_back((char)((BLE_MANUFACTURER_ID >> 8) & 0xFF));
  data.push_back((char)BLE_ADV_METADATA_VERSION);
  data.push_back((char)version[0]);
  data.push_back((char)version[1]);
  data.push_back((char)version[2]);
  data.push_back((char)variant);
  data.push_back((char)(normalizedValue & 0xFF));
  data.push_back((char)((normalizedValue >> 8) & 0xFF));
  return data;
}

const char* advertisedVariantName(AdvertisedVariant variant) {
  return appVariantKindToString(variant);
}

void logScanResponsePayload(const String& deviceName, const std::string& scanMetadata) {
  if (scanMetadata.size() < 9) {
    appLogPrintf("BLE scan response armed: name=%s invalid_payload_len=%u\n",
                 deviceName.c_str(),
                 (unsigned)scanMetadata.size());
    return;
  }

  uint8_t fwMajor = (uint8_t)scanMetadata[3];
  uint8_t fwMinor = (uint8_t)scanMetadata[4];
  uint8_t fwPatch = (uint8_t)scanMetadata[5];
  AdvertisedVariant variant = (AdvertisedVariant)(uint8_t)scanMetadata[6];
  uint16_t normalizedValue =
      (uint16_t)(uint8_t)scanMetadata[7] | ((uint16_t)(uint8_t)scanMetadata[8] << 8);

  appLogPrintf("BLE scan response armed: name=%s fw=%u.%u.%u variant=%s value=%u\n",
               deviceName.c_str(),
               (unsigned)fwMajor,
               (unsigned)fwMinor,
               (unsigned)fwPatch,
               advertisedVariantName(variant),
               (unsigned)normalizedValue);
}

void configureAdvertisingPayload(BLEAdvertising* advertising,
                                 const String& deviceName,
                                 const std::string& metadata,
                                 const std::string& scanMetadata) {
  if (!advertising) return;

  advertising->setMinInterval(BLE_ADV_INTERVAL_MIN);
  advertising->setMaxInterval(BLE_ADV_INTERVAL_MAX);

  BLEAdvertisementData advData;
  advData.setManufacturerData(metadata);
#if APP_BLE_GATT_ENABLE
  advertising->setConnectableMode(BLE_GAP_CONN_MODE_UND);
  advertising->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
  advData.addServiceUUID(BLE_SERVICE_UUID);
  advertising->setAdvertisementData(advData);

  BLEAdvertisementData scanResponseData;
  scanResponseData.setManufacturerData(scanMetadata);
  scanResponseData.setName(deviceName.c_str());
  advertising->setScanResponseData(scanResponseData);
  advertising->enableScanResponse(true);
#else
  advertising->setConnectableMode(BLE_GAP_CONN_MODE_NON);
  advertising->setDiscoverableMode(BLE_GAP_DISC_MODE_GEN);
  advData.setName(deviceName.c_str());
  advertising->setAdvertisementData(advData);
  advertising->enableScanResponse(false);
#endif
}

void refreshAdvertisingPayloadIfNeeded(bool force) {
  if (!g_bleStarted) return;

  String deviceName = buildAdvertisedName();
  std::string metadata = buildAdvertisingMetadata();
  std::string scanMetadata = buildScanResponseMetadata();
  if (!force &&
      metadata == g_lastAdvertisingMetadata &&
      scanMetadata == g_lastScanResponseMetadata &&
      deviceName == g_lastAdvertisedName) {
    return;
  }

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  if (!advertising) return;

  advertising->stop();
  configureAdvertisingPayload(advertising, deviceName, metadata, scanMetadata);
  advertising->start();
  g_lastAdvertisingMetadata = metadata;
  g_lastScanResponseMetadata = scanMetadata;
  g_lastAdvertisedName = deviceName;
  logScanResponsePayload(deviceName, scanMetadata);
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
  if (g_passwordChar) g_passwordChar->setValue("");
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
  char macBuf[18];
  IPAddress ip = wifi.getIP();
  snprintf(ipBuf, sizeof(ipBuf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  wifi.getMacAddress(macBuf, sizeof(macBuf));

  char rssiBuf[16];
  snprintf(rssiBuf, sizeof(rssiBuf), "%d", (int)wifi.getRSSI());

  char lastLogBuf[BLE_LAST_LOG_BYTES + 1];
  char logTailBuf[BLE_LOG_TAIL_BYTES + 1];
  appLogCopyLastLine(lastLogBuf, sizeof(lastLogBuf), BLE_LAST_LOG_BYTES);
  appLogCopyTail(logTailBuf, sizeof(logTailBuf), BLE_LOG_TAIL_BYTES);
  bool lastLogChanged = strcmp(lastLogBuf, g_lastNotifiedLogLine) != 0;
  bool rssiChanged = strcmp(rssiBuf, g_lastNotifiedRssi) != 0;

  if (g_wifiIpChar) g_wifiIpChar->setValue(ipBuf);
  if (g_wifiMacChar) g_wifiMacChar->setValue(macBuf);
  if (g_wifiRssiChar) g_wifiRssiChar->setValue(rssiBuf);
  if (g_lastLogChar) g_lastLogChar->setValue(lastLogBuf);
  if (g_logTailChar) g_logTailChar->setValue(logTailBuf);

  if (notify && g_bleClientConnected) {
    if (g_wifiRssiChar && rssiChanged) {
      g_wifiRssiChar->notify();
      strncpy(g_lastNotifiedRssi, rssiBuf, sizeof(g_lastNotifiedRssi) - 1);
      g_lastNotifiedRssi[sizeof(g_lastNotifiedRssi) - 1] = '\0';
    }
    if (g_lastLogChar && lastLogChanged) {
      g_lastLogChar->notify();
      strncpy(g_lastNotifiedLogLine, lastLogBuf, sizeof(g_lastNotifiedLogLine) - 1);
      g_lastNotifiedLogLine[sizeof(g_lastNotifiedLogLine) - 1] = '\0';
    }
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
      message = "SSID updated";
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
      message = "Password updated";
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
      message = "Hostname updated";
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
      message = enabled ? "DHCP enabled" : "Static IP mode enabled";
      return true;
    }
    case ConfigField::Ip:
      if (!config.writeStaticIP(value)) {
        message = "Failed to write IP";
        return false;
      }
      message = "Static IP updated";
      return true;
    case ConfigField::Gateway:
      if (!config.writeGateway(value)) {
        message = "Failed to write gateway";
        return false;
      }
      message = "Gateway updated";
      return true;
    case ConfigField::Subnet:
      if (!config.writeSubnet(value)) {
        message = "Failed to write subnet";
        return false;
      }
      message = "Subnet updated";
      return true;
    case ConfigField::Dns1:
      if (!config.writeDNS1(value)) {
        message = "Failed to write DNS1";
        return false;
      }
      message = "DNS1 updated";
      return true;
    case ConfigField::Dns2:
      if (!config.writeDNS2(value)) {
        message = "Failed to write DNS2";
        return false;
      }
      message = "DNS2 updated";
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
      message = "Start value updated";
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
      message = "DMX address updated";
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
      message = "Universe updated";
      return true;
    }
  }

  message = "Unsupported field";
  return false;
}

class ServerCallbacks : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer* server, NimBLEConnInfo& connInfo) override {
    (void)server;
    (void)connInfo;
    g_bleClientConnected = true;
    g_statusRefreshPending = true;
    g_lastStatusRefreshMs = 0;
    setResult("BLE client connected");
  }

  void onDisconnect(BLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    (void)server;
    (void)connInfo;
    (void)reason;
    g_bleClientConnected = false;
    g_statusRefreshPending = false;
    BLEDevice::startAdvertising();
  }
};

class ConfigCallbacks : public BLECharacteristicCallbacks {
 public:
  explicit ConfigCallbacks(ConfigField field) : field_(field) {}

  void onWrite(BLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)characteristic;
    (void)connInfo;
    String value = trimAscii(characteristic->getValue());
    queueBleWrite(field_, value);
  }

 private:
  ConfigField field_;
};

class RebootCallbacks : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    String value = trimAscii(characteristic->getValue());
    queueBleReboot(value);
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
  std::string metadata = buildAdvertisingMetadata();
  std::string scanMetadata = buildScanResponseMetadata();
  BLEDevice::init(deviceName.c_str());
  BLEDevice::setMTU(APP_BLE_MTU);

#if !APP_BLE_GATT_ENABLE
  BLEAdvertising* advertisingOnly = BLEDevice::getAdvertising();
  configureAdvertisingPayload(advertisingOnly, deviceName, metadata, scanMetadata);
  advertisingOnly->start();
  g_lastAdvertisingMetadata = metadata;
  g_lastScanResponseMetadata = scanMetadata;
  g_lastAdvertisedName = deviceName;
  g_lastAdvRefreshMs = millis();
  logScanResponsePayload(deviceName, scanMetadata);
  appLogPrintf("BLE advertising-only started as %s\n", deviceName.c_str());
  g_bleStarted = true;
  return;
#endif

  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());

  // The Arduino BLE wrapper defaults services to 15 handles, which is too small
  // once we account for our characteristics plus descriptors.
  BLEService* service = g_server->createService(BLEUUID(BLE_SERVICE_UUID));

  g_wifiRssiChar = createCharacteristic(service,
                                        BLE_CHAR_WIFI_RSSI_UUID,
                                        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY,
                                        "wifi_rssi");
  g_wifiIpChar = createCharacteristic(service,
                                      BLE_CHAR_WIFI_IP_UUID,
                                      NIMBLE_PROPERTY::READ,
                                      "wifi_ip");
  g_wifiMacChar = createCharacteristic(service,
                                       BLE_CHAR_WIFI_MAC_UUID,
                                       NIMBLE_PROPERTY::READ,
                                       "wifi_mac");
  g_lastLogChar = createCharacteristic(service,
                                       BLE_CHAR_LAST_LOG_UUID,
                                       NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY,
                                       "last_log_line");
  g_logTailChar = createCharacteristic(service,
                                       BLE_CHAR_LOG_TAIL_UUID,
                                       NIMBLE_PROPERTY::READ,
                                       "log_tail");
  g_rebootChar = createCharacteristic(service,
                                      BLE_CHAR_REBOOT_UUID,
                                      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR,
                                      "reboot");
  g_resultChar = createCharacteristic(service,
                                      BLE_CHAR_RESULT_UUID,
                                      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY,
                                      "result");

  g_ssidChar = createCharacteristic(service,
                                    BLE_CHAR_SSID_UUID,
                                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                        NIMBLE_PROPERTY::WRITE_NR,
                                    "ssid");
  g_passwordChar = createCharacteristic(service,
                                        BLE_CHAR_PASSWORD_UUID,
                                        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR,
                                        "password");
  g_hostnameChar = createCharacteristic(service,
                                        BLE_CHAR_HOSTNAME_UUID,
                                        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                            NIMBLE_PROPERTY::WRITE_NR,
                                        "hostname");
  g_dhcpChar = createCharacteristic(service,
                                    BLE_CHAR_DHCP_UUID,
                                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                        NIMBLE_PROPERTY::WRITE_NR,
                                    "dhcp");
  g_ipChar = createCharacteristic(service,
                                  BLE_CHAR_IP_UUID,
                                  NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                      NIMBLE_PROPERTY::WRITE_NR,
                                  "ip");
  g_gatewayChar = createCharacteristic(service,
                                       BLE_CHAR_GATEWAY_UUID,
                                       NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                           NIMBLE_PROPERTY::WRITE_NR,
                                       "gateway");
  g_subnetChar = createCharacteristic(service,
                                      BLE_CHAR_SUBNET_UUID,
                                      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                          NIMBLE_PROPERTY::WRITE_NR,
                                      "subnet");
  g_dns1Char = createCharacteristic(service,
                                    BLE_CHAR_DNS1_UUID,
                                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                        NIMBLE_PROPERTY::WRITE_NR,
                                    "dns1");
  g_dns2Char = createCharacteristic(service,
                                    BLE_CHAR_DNS2_UUID,
                                    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                        NIMBLE_PROPERTY::WRITE_NR,
                                    "dns2");
  g_startValueChar = createCharacteristic(service,
                                          BLE_CHAR_START_VALUE_UUID,
                                          NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                              NIMBLE_PROPERTY::WRITE_NR,
                                          "start_value");
  g_dmxAddressChar = createCharacteristic(service,
                                          BLE_CHAR_DMX_ADDRESS_UUID,
                                          NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                              NIMBLE_PROPERTY::WRITE_NR,
                                          "dmx_address");
  g_dmxUniverseChar = createCharacteristic(service,
                                           BLE_CHAR_DMX_UNIVERSE_UUID,
                                           NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                               NIMBLE_PROPERTY::WRITE_NR,
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

  g_server->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  configureAdvertisingPayload(advertising, deviceName, metadata, scanMetadata);
  advertising->start();
  g_lastAdvertisingMetadata = metadata;
  g_lastScanResponseMetadata = scanMetadata;
  g_lastAdvertisedName = deviceName;
  g_lastAdvRefreshMs = millis();
  logScanResponsePayload(deviceName, scanMetadata);

  appLogPrintf("BLE service started as %s\n", deviceName.c_str());
  g_bleStarted = true;
}

void appRefreshBleAdvertising() {
  if (!g_bleStarted) return;
  refreshAdvertisingPayloadIfNeeded(true);
  g_lastAdvRefreshMs = millis();
  appLogPrintf("BLE advertising refreshed as %s\n", g_lastAdvertisedName.c_str());
}

void appBleLoop() {
  if (!g_bleStarted) return;

  uint32_t now = millis();
  if ((now - g_lastAdvRefreshMs) >= BLE_ADV_REFRESH_MS) {
    refreshAdvertisingPayloadIfNeeded(false);
    g_lastAdvRefreshMs = now;
  }

#if !APP_BLE_GATT_ENABLE
  return;
#endif

  PendingBleWrite pendingWrite;
  if (takePendingBleWrite(pendingWrite)) {
    String value(pendingWrite.value);
    if (pendingWrite.isReboot) {
      appLogPrintf("BLE reboot write: value=%s\n", value.c_str());
      value.toLowerCase();
      if (value != "1" && value != "reboot" && value != "restart") {
        appLogLine("BLE reboot rejected");
        setResult("Write '1' or 'reboot' to restart");
      } else {
        appLogLine("BLE reboot requested");
        setResult("Reboot scheduled");
        appWifiManager().scheduleRestart(500);
      }
    } else {
      String message;
      appLogPrintf("BLE config write: field=%s value=%s\n",
                   configFieldName(pendingWrite.field),
                   configValuePreview(pendingWrite.field, value).c_str());
      if (!applyConfigField(pendingWrite.field, value, message)) {
        appLogPrintf("BLE config rejected: field=%s reason=%s\n",
                     configFieldName(pendingWrite.field),
                     message.c_str());
        setResult(message);
        refreshConfigCharacteristics();
      } else {
        refreshConfigCharacteristics();
        appLogPrintf("BLE config accepted: field=%s result=%s\n",
                     configFieldName(pendingWrite.field),
                     message.c_str());
        appLogLine(message.c_str());
        setResult(message);
      }
    }
  }

  if (!g_bleClientConnected && !g_statusRefreshPending) return;
  if (!g_statusRefreshPending && (now - g_lastStatusRefreshMs) < BLE_STATUS_REFRESH_MS) return;
  g_lastStatusRefreshMs = now;
  bool notify = !g_statusRefreshPending;
  g_statusRefreshPending = false;
  refreshStatusCharacteristics(notify);
}

#else

void appStartBleServices() {}

void appRefreshBleAdvertising() {}

void appBleLoop() {}

#endif
