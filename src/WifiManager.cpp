#include "WifiManager.h"

#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <Update.h>
#include <esp_partition.h>
#include <esp_wifi.h>

#include "Configuration.h"

WifiManagerClass::WifiManagerClass(Configuration& config)
	: _server(80), 
	  _config(config) {
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

AsyncWebServer& WifiManagerClass::getServer()
{
	return _server;
}

void WifiManagerClass::check() {
	if (!_otaInProgress) {
		pollNetworkScan();
	}

	if (_restartPending && millis() >= _restartAtMs) {
		Serial.println("Restarting device...");
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

void WifiManagerClass::requestRestart(unsigned long delayMs) {
	_restartPending = true;
	_restartAtMs = millis() + delayMs;
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

void WifiManagerClass::startManagementServerWeb() {

	_server
		.serveStatic("/", SPIFFS, "/wifi-manager")
		.setDefaultFile("index.html");

	_server.on("/networks", HTTP_GET, [this](AsyncWebServerRequest *request) {
		if (!_otaInProgress && !_scanInProgress && !_scanRequested && !_scanHasResult) {
			startNetworkScan();
		}
		bool isScanning = _scanInProgress || _scanRequested || !_scanHasResult;

		if (request->hasParam("details")) {
			String payload = String("{\"scanning\":") + (isScanning ? "true" : "false") + ",\"networks\":" + _networks + "}";
			request->send(200, "application/json", payload);
		} else {
			// Legacy response shape for older UI versions.
			request->send(200, "application/json", _networks);
		}
	});

	_server.on("/credentials", HTTP_PUT, [this](AsyncWebServerRequest *request) {
		int params = request->params();

		for (int i = 0; i < params; i++) {
			AsyncWebParameter *param = (AsyncWebParameter*)(request->getParam(i));
			if (!param->isPost()) continue; // body/form-data

			const String &name = param->name();
			const String &value = param->value();

			if ((name == "ssid") && (!value.equals("Loading networks")))   {
				_config.writeSSID(value.c_str());
			} else if (name == "password") {
				_config.writePass(value.c_str());
			} else if (name == "hostname") {
				_config.writeHostname(value.c_str());
			} else if (name == "dmx_address") {
				_config.writeDMXAddress(value.toInt());
			} else if (name == "dmx_universe") {
				_config.writeDMXUniverse(value.toInt());
			}
		}

		request->send(204);
		requestRestart(1000);
	});

	_server.on("/credentials", HTTP_GET, [=](AsyncWebServerRequest *request) {
		StaticJsonDocument<256> doc;
		doc["ssid"] = _config.getSSID();        // const char* or String is fine
		doc["hostname"] = _config.getHostname();

		// For safety, either omit or return empty password.
		// If you really want to return it, uncomment next line:
		doc["password"] = _config.getPass();
//		doc["password"] = ""; // recommended
	    doc["dmx_address"] = _config.getDMXAddress();   // e.g., 1..512
	    doc["dmx_universe"]= _config.getDMXUniverse();  // e.g., 0..32767

		String out;
		serializeJson(doc, out);

		AsyncWebServerResponse *res = request->beginResponse(200, "application/json", out);
		res->addHeader("Cache-Control", "no-store");
		request->send(res);
	});

	// ---- OTA Update endpoints ----
	
	// GET /update - returns OTA page/info
	_server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
		StaticJsonDocument<128> doc;
		doc["version"] = "1.0.0";  // You can set your firmware version here
		doc["device"] = "CableCar";
		doc["ota_ready"] = true;
		
		String out;
		serializeJson(doc, out);
		
		AsyncWebServerResponse *res = request->beginResponse(200, "application/json", out);
		res->addHeader("Cache-Control", "no-store");
		request->send(res);
	});

	// POST /update - handle firmware upload
	_server.on("/update", HTTP_POST, 
		[this](AsyncWebServerRequest *request) {
			// This callback is called when ALL upload data has been received
			bool success = (Update.hasError() == 0);
			_otaInProgress = false;
			AsyncWebServerResponse *res;
			if (success) {
				res = request->beginResponse(200, "application/json", "{\"success\": true, \"message\": \"Firmware updated. Device restarting...\"}");
				res->addHeader("Cache-Control", "no-store");
				request->send(res);
				Serial.println("OTA Update successful, restart scheduled");
				requestRestart(1000);
			} else {
				res = request->beginResponse(400, "application/json", "{\"success\": false, \"message\": \"Update failed\"}");
				res->addHeader("Cache-Control", "no-store");
				request->send(res);
			}
		},
		[this](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
			_otaInProgress = true;
			// Handle the upload
			if (index == 0) {
				// Start of upload
				Serial.printf("Update Start: %s\n", filename.c_str());

				// Begin OTA update - use U_FLASH for firmware
				if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
					Update.printError(Serial);
					Serial.println("OTA: Not enough space or other error");
					_otaInProgress = false;
					return;
				}
			}

			// Write uploaded data
			if (Update.write(data, len) != len) {
				Update.printError(Serial);
				Serial.println("OTA: Write failed");
				_otaInProgress = false;
				return;
			}

			// Check if upload is finished
			if (final) {
				if (Update.end(true)) {  // true = check MD5
					Serial.printf("Update Success: %u bytes\n", index + len);
				} else {
					Update.printError(Serial);
					Serial.println("OTA: Final check failed");
				}
				_otaInProgress = false;
			}
		}
	);

	// ---- SPIFFS Update endpoints ----
	
	// POST /updatefs - handle SPIFFS filesystem upload
	_server.on("/updatefs", HTTP_POST, 
		[this](AsyncWebServerRequest *request) {
			// This callback is called when ALL upload data has been received
			bool success = (Update.hasError() == 0);
			_otaInProgress = false;
			AsyncWebServerResponse *res;
			if (success) {
				res = request->beginResponse(200, "application/json", "{\"success\": true, \"message\": \"Filesystem updated. Device restarting...\"}");
				res->addHeader("Cache-Control", "no-store");
				request->send(res);
				Serial.println("SPIFFS Update successful, restart scheduled");
				requestRestart(1000);
			} else {
				res = request->beginResponse(400, "application/json", "{\"success\": false, \"message\": \"Update failed\"}");
				res->addHeader("Cache-Control", "no-store");
				request->send(res);
			}
		},
		[this](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final) {
			_otaInProgress = true;
			// Handle the upload
			if (index == 0) {
				// Start of upload
				Serial.printf("SPIFFS Update Start: %s\n", filename.c_str());

				const esp_partition_t* spiffsPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
				if (spiffsPart == NULL) {
					Serial.println("SPIFFS OTA: Partition not found");
					_otaInProgress = false;
					return;
				}

				// Begin OTA update - use U_SPIFFS for filesystem
				if (!Update.begin(spiffsPart->size, U_SPIFFS)) {
					Update.printError(Serial);
					Serial.println("SPIFFS OTA: Not enough space or other error");
					_otaInProgress = false;
					return;
				}
			}

			// Write uploaded data
			if (Update.write(data, len) != len) {
				Update.printError(Serial);
				Serial.println("SPIFFS OTA: Write failed");
				_otaInProgress = false;
				return;
			}

			// Check if upload is finished
			if (final) {
				if (Update.end(true)) {  // true = check MD5
					Serial.printf("SPIFFS Update Success: %u bytes\n", index + len);
				} else {
					Update.printError(Serial);
					Serial.println("SPIFFS OTA: Final check failed");
				}
				_otaInProgress = false;
			}
		}
	);

	_server.begin();
}

bool WifiManagerClass::acceptsCompressedResponse(AsyncWebServerRequest *request) {
    if (request->hasHeader("Accept-Encoding")){
        AsyncWebHeader* header = (AsyncWebHeader*)(request->getHeader("Accept-Encoding"));
        String value = header->value();
        bool hasGzip = value.indexOf("gzip") > -1;

        return hasGzip;
    }

    return false;
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
