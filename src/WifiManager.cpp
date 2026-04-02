#include "WifiManager.h"

#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WiFi.h>
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
	_scanInProgress = false;
	_scanRequested = false;
	_scanHasResult = false;
	_otaInProgress = false;
	_scanTaskHandle = nullptr;

	_networks = "";
	_hostname = "";
	_ssid = "";
}

void WifiManagerClass::check() {
	if (!_otaInProgress) {
		pollNetworkScan();
	}

	if (_restartPending && millis() >= _restartAtMs) {
		Serial.println("Restarting device...");
		cleanupBeforeRestart();
		delay(100);  // Give cleanup time to complete
		ESP.restart();
	}

	if (!_otaInProgress && _connected && millis() > _nextReconnectCheck) {
		if (WiFi.status() != WL_CONNECTED) {
			_connected = false;

			Serial.println("WiFi connection lost. Attempting to reconnect.");

			WiFi.reconnect();

			waitForConnection();
		}

		_nextReconnectCheck = millis() + _reconnectIntervalCheck;
	}
}

String WifiManagerClass::getNetworksPayload(bool details) {
	if (!_otaInProgress && !_scanInProgress && !_scanRequested && !_scanHasResult) {
		startNetworkScan();
	}
	bool isScanning = _scanInProgress || _scanRequested || !_scanHasResult;

	if (details) {
		return String("{\"scanning\":") + (isScanning ? "true" : "false") + ",\"networks\":" + _networks + "}";
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
	
	Serial.println("Cleanup complete");
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
	_networks = getAvailableNetworks();
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
	String separator = "";

	// If negative value is returned from the scan we will
	// just return the empty list as the loop will not
	// run. This is in case of WIFI_SCAN_FAILED or similar.
	for (int i = 0; i < networks; i++) {
		String network = "\"" + WiFi.SSID(i) + "\"";

		if (json.indexOf(network) == -1) {
			json += separator + network;
			separator = ",";
		}
	}

	json += "]";
	Serial.println("scan complete...\n");

	return json;
}

bool WifiManagerClass::connectToWifi() {
	_ssid = _config.getSSID();
	_hostname = _config.getHostname();
	String pass = _config.getPass();

	if (_ssid == "") {
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

	// Fixes issue with mDNS where hostname was not set (v1.0.1) and mDNS crashed (v1.0.2)
	WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);

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

	WiFi.begin(_ssid.c_str(), pass.c_str());

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
	}

	_ip = WiFi.localIP();

	Serial.println("Assigned IP Address:");
	Serial.println(_ip);


	return true;
}

void WifiManagerClass::startManagementAP() {
	const char *ssid="WIFI-MANAGER";

	Serial.println("Starting Management AP");
	WiFi.mode(WIFI_MODE_APSTA);
	WiFi.setSleep(WIFI_PS_NONE);
	esp_wifi_set_ps(WIFI_PS_NONE);

	WiFi.softAP(ssid);

	_ssid = WiFi.softAPSSID();
	_ip = WiFi.softAPIP();

	Serial.println("Server IP Address:");
	Serial.println(_ip);
}

String WifiManagerClass::getHostname() {
	return _hostname;
}

String WifiManagerClass::getSSID() {
	return _ssid;
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
