#include "WifiManager.h"

#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <esp_wifi.h>

#include "Configuration.h"

WifiManagerClass::WifiManagerClass(Configuration& config)
	: _config(config) {
	_reconnectIntervalCheck = 5000;
	_connectionTimeout = 10000;

	_nextReconnectCheck = 0;
	_connected = false;
	_restartAtMs = 0;
	_restartPending = false;
	_managementApActive = false;
	_scanInProgress = false;
	_scanRequested = false;
	_scanHasResult = false;
	_otaInProgress = false;
	_scanTaskHandle = nullptr;
	_reconnectAttempts = 0;
	_reconnectSuccesses = 0;
	_lastReconnectAttemptMs = 0;
	_lastReconnectSuccessMs = 0;

	_networks = "";
	_hostname = "";
	_stationSsid = "";
	_apSsid = "";
}

void WifiManagerClass::check() {
	wl_status_t status = WiFi.status();
	if (status == WL_CONNECTED) {
		_ip = WiFi.localIP();
	}

	if (!_connected && status == WL_CONNECTED) {
		_connected = true;
		_reconnectSuccesses++;
		_lastReconnectSuccessMs = millis();
		Serial.println("WiFi reconnected");
	}

	if (_restartPending && millis() >= _restartAtMs) {
		Serial.println("Restarting device...");
		cleanupBeforeRestart();
		delay(100);  // Give cleanup time to complete
		ESP.restart();
	}

	if (!_otaInProgress && !_managementApActive && millis() > _nextReconnectCheck) {
		if (status != WL_CONNECTED) {
			ensureReconnectAttempt();
		}
		_nextReconnectCheck = millis() + _reconnectIntervalCheck;
	}
}

String WifiManagerClass::getNetworksPayload(bool details, bool refresh) {
	if (!_otaInProgress && refresh) {
		_networks = getAvailableNetworks();
		_scanHasResult = true;
	}

	bool isScanning = false;

	if (details) {
		String payload;
		payload.reserve(_networks.length() + 32);
		payload = "{\"scanning\":";
		payload += isScanning ? "true" : "false";
		payload += ",\"networks\":";
		payload += _networks;
		payload += "}";
		return payload;
	}

	return _networks;
}

void WifiManagerClass::scheduleRestart(unsigned long delayMs) {
	requestRestart(delayMs);
}

void WifiManagerClass::setOtaInProgress(bool inProgress) {
	_otaInProgress = inProgress;
}

void WifiManagerClass::requestRestart(unsigned long delayMs) {
	_restartPending = true;
	_restartAtMs = millis() + delayMs;
}

void WifiManagerClass::cleanupBeforeRestart() {
	Serial.println("Cleaning up WiFi and mDNS before restart...");
	
	// Stop mDNS responder
	MDNS.end();
	
	// Disconnect from WiFi
	WiFi.disconnect(true);  // true = turn off WiFi radio
	
	// Fully deinitialize WiFi driver
	esp_wifi_deinit();

	// Ensure filesystem metadata is committed before the software reset.
	LittleFS.end();
	
	Serial.println("Cleanup complete");
}

void WifiManagerClass::ensureReconnectAttempt() {
	if (_stationSsid.length() == 0) {
		return;
	}

	_connected = false;
	_reconnectAttempts++;
	_lastReconnectAttemptMs = millis();

	wifi_mode_t mode = WiFi.getMode();
	if (mode == WIFI_MODE_AP) {
		WiFi.mode(WIFI_MODE_APSTA);
	} else if (mode == WIFI_MODE_NULL) {
		WiFi.mode(WIFI_MODE_STA);
	}

	WiFi.setSleep(WIFI_PS_NONE);
	esp_wifi_set_ps(WIFI_PS_NONE);

	Serial.println("WiFi not connected. Attempting to reconnect.");
	String currentTarget = WiFi.SSID();
	if (currentTarget == _stationSsid && WiFi.reconnect()) {
		return;
	}

	String pass = _config.getPass();
	WiFi.begin(_stationSsid.c_str(), pass.c_str());
}

void WifiManagerClass::startNetworkScan() {
	if (_scanInProgress || _scanRequested) return;
	_scanHasResult = false;
	_scanRequested = true;
}

void WifiManagerClass::networkScanTaskEntry(void* parameter) {
	WifiManagerClass* self = static_cast<WifiManagerClass*>(parameter);
	self->runNetworkScanTask();
	vTaskDelete(nullptr);
}

void WifiManagerClass::runNetworkScanTask() {
	// _networks = getAvailableNetworks();
	_scanInProgress = false;
	_scanHasResult = true;
	_scanTaskHandle = nullptr;
}

void WifiManagerClass::pollNetworkScan() {
	if (!_scanRequested || _scanInProgress) return;

	_scanRequested = false;
	_scanInProgress = true;
	Serial.println("Starting WiFi scan...");

	if (_scanTaskHandle != nullptr) {
		return;
	}

	BaseType_t taskOk = xTaskCreatePinnedToCore(
		networkScanTaskEntry,
		"WifiScanTask",
		4096,
		this,
		1,
		&_scanTaskHandle,
		1);

	if (taskOk != pdPASS) {
		Serial.println("Failed to start WiFi scan task");
		_scanInProgress = false;
		_scanHasResult = true;
		_scanTaskHandle = nullptr;
	}
}

String WifiManagerClass::getAvailableNetworks() {
	Serial.print("Scanning networks...");

	// Ensure STA is enabled so scanning works in AP management mode.
	wifi_mode_t mode = WiFi.getMode();
	if (mode == WIFI_MODE_AP) {
		WiFi.mode(WIFI_MODE_APSTA);
		WiFi.setSleep(WIFI_PS_NONE);
		esp_wifi_set_ps(WIFI_PS_NONE);
	}

	int networks = WiFi.scanNetworks(false, false);
	if (networks < 0) {
		Serial.printf("scan failed (code %d)\n", networks);
		return "[]";
	}

	String json = "[";
	json.reserve((networks * 34) + 2);
	bool first = true;

	// If negative value is returned from the scan we will
	// just return the empty list as the loop will not
	// run. This is in case of WIFI_SCAN_FAILED or similar.
	for (int i = 0; i < networks; i++) {
		String network = "\"" + WiFi.SSID(i) + "\"";

		if (json.indexOf(network) == -1) {
			if (!first) {
				json += ",";
			}
			json += network;
			first = false;
		}
	}

	json += "]";
	Serial.println("scan complete...\n");

	return json;
}

bool WifiManagerClass::connectToWifi() {
	_stationSsid = _config.getSSID();
	_hostname = _config.getHostname();
	String pass = _config.getPass();
	_managementApActive = false;
	_apSsid = "";

	if (_stationSsid == "") {
		Serial.println("No connection information specified");

		return false;
	}

	WiFi.mode(WIFI_MODE_STA);
	WiFi.setSleep(WIFI_PS_NONE);
	esp_wifi_set_ps(WIFI_PS_NONE);

	// Don't scan networks at startup - it blocks for several seconds
	// Networks will be scanned on-demand when /networks endpoint is called
	_networks = "[]";
	_scanHasResult = false;

	bool useDhcp = _config.getDhcpEnabled();
	if (useDhcp) {
		// Keep DHCP behavior for backward compatibility.
		WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
	} else {
		IPAddress ip;
		IPAddress gateway;
		IPAddress subnet;
		IPAddress dns1;
		IPAddress dns2;
		String ipStr = _config.getStaticIP();
		String gwStr = _config.getGateway();
		String subnetStr = _config.getSubnet();
		String dns1Str = _config.getDNS1();
		String dns2Str = _config.getDNS2();

		bool validRequired = ip.fromString(ipStr) && gateway.fromString(gwStr) && subnet.fromString(subnetStr);
		bool validDns1 = dns1.fromString(dns1Str);
		bool validDns2 = dns2.fromString(dns2Str);
		if (!validDns1) dns1 = INADDR_NONE;
		if (!validDns2) dns2 = INADDR_NONE;

		if (validRequired) {
			if (!WiFi.config(ip, gateway, subnet, dns1, dns2)) {
				Serial.println("Static IP config failed, falling back to DHCP");
				WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
			} else {
				Serial.println("Using static IPv4 configuration");
			}
		} else {
			Serial.println("Invalid static IPv4 settings, falling back to DHCP");
			WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
		}
	}

	if (_hostname != "") {
		WiFi.setHostname(_hostname.c_str());
		Serial.println("Setting hostname " + _hostname);

		if (MDNS.begin(_hostname.c_str())) {
			Serial.println("mDNS responder started");
		} else {
			Serial.println("Unable to start mDNS responder");
		}
	} else {
		Serial.println("No hostname configured");
	}

	Serial.println("Connecting to WiFi...");

	WiFi.begin(_stationSsid.c_str(), pass.c_str());

	_connected = waitForConnection();
	WiFi.setSleep(WIFI_PS_NONE);
	esp_wifi_set_ps(WIFI_PS_NONE);

	return _connected;
}

bool WifiManagerClass::waitForConnection() {
	unsigned long timeout = millis() + _connectionTimeout;

	while (WiFi.status() != WL_CONNECTED) {
		if (millis() > timeout) {
			Serial.println("Unable to connect to WIFI");

			return false;
		}

		delay(20);
	}

	_ip = WiFi.localIP();

	Serial.println("Assigned IP Address:");
	Serial.println(_ip);


	return true;
}

void WifiManagerClass::startManagementAP() {
	const char *ssid="WIFI-MANAGER";

	if (_managementApActive) {
		IPAddress currentApIp = WiFi.softAPIP();
		if (currentApIp != IPAddress((uint32_t)0)) {
			_ip = currentApIp;
			return;
		}
		_managementApActive = false;
	}

	Serial.println("Starting Management AP");
	WiFi.softAPdisconnect(true);
	if (WiFi.getMode() != WIFI_MODE_APSTA) {
		WiFi.mode(WIFI_MODE_APSTA);
	}
	WiFi.setSleep(WIFI_PS_NONE);
	esp_wifi_set_ps(WIFI_PS_NONE);
	delay(100);

	bool apOk = WiFi.softAP(ssid);
	if (!apOk || WiFi.softAPIP() == IPAddress((uint32_t)0)) {
		Serial.println("Initial AP start failed, retrying WiFi AP setup");
		WiFi.mode(WIFI_MODE_NULL);
		delay(100);
		WiFi.mode(WIFI_MODE_APSTA);
		WiFi.setSleep(WIFI_PS_NONE);
		esp_wifi_set_ps(WIFI_PS_NONE);
		delay(100);
		apOk = WiFi.softAP(ssid);
	}

	_managementApActive = apOk && (WiFi.softAPIP() != IPAddress((uint32_t)0));
	_apSsid = _managementApActive ? WiFi.softAPSSID() : "";
	_ip = WiFi.softAPIP();

	if (!_managementApActive) {
		Serial.println("Management AP failed to start");
		return;
	}

	Serial.println("Server IP Address:");
	Serial.println(_ip);
}

String WifiManagerClass::getHostname() {
	return _hostname;
}

const char* WifiManagerClass::getHostnameCStr() const {
	return _hostname.c_str();
}

String WifiManagerClass::getSSID() {
	if (_connected) {
		return _stationSsid;
	}

	if (_managementApActive) {
		return _apSsid;
	}

	return _stationSsid;
}

void WifiManagerClass::getMacAddress(char* out, size_t outSize) const {
	if (!out || outSize == 0) return;

	uint8_t mac[6] = {0};
	esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac);
	if (err != ESP_OK ||
	    (mac[0] == 0 && mac[1] == 0 && mac[2] == 0 && mac[3] == 0 && mac[4] == 0 && mac[5] == 0)) {
		err = esp_wifi_get_mac(WIFI_IF_AP, mac);
	}

	if (err != ESP_OK) {
		out[0] = '\0';
		return;
	}

	snprintf(out,
	         outSize,
	         "%02X:%02X:%02X:%02X:%02X:%02X",
	         mac[0],
	         mac[1],
	         mac[2],
	         mac[3],
	         mac[4],
	         mac[5]);
}

String WifiManagerClass::getMacAddress() {
	char mac[18];
	getMacAddress(mac, sizeof(mac));
	return String(mac);
}

int8_t WifiManagerClass::getRSSI() {
	return WiFi.RSSI();
}

IPAddress WifiManagerClass::getIP() {
	return _ip;
}

bool WifiManagerClass::isConnected() {
	return _connected;
}

uint32_t WifiManagerClass::getReconnectAttempts() const {
	return _reconnectAttempts;
}

uint32_t WifiManagerClass::getReconnectSuccesses() const {
	return _reconnectSuccesses;
}

uint32_t WifiManagerClass::getLastReconnectAttemptMs() const {
	return _lastReconnectAttemptMs;
}

uint32_t WifiManagerClass::getLastReconnectSuccessMs() const {
	return _lastReconnectSuccessMs;
}
