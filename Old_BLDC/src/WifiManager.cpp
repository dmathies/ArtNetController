#include "WifiManager.h"

#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WiFi.h>

#include "Configuration.h"

WifiManagerClass::WifiManagerClass(Configuration& config)
	: _server(80), 
	  _config(config) {
	_reconnectIntervalCheck = 5000;
	_connectionTimeout = 10000;

	_nextReconnectCheck = 0;
	_connected = false;

	_networks = "";
	_hostname = "";
	_ssid = "";
}

AsyncWebServer& WifiManagerClass::getServer()
{
	return _server;
}

void WifiManagerClass::check() {
	if (_connected && millis() > _nextReconnectCheck) {
		if (WiFi.status() != WL_CONNECTED) {
			_connected = false;

			Serial.println("WiFi connection lost. Attempting to reconnect.");

			WiFi.reconnect();

			waitForConnection();
		}

		_nextReconnectCheck = millis() + _reconnectIntervalCheck;
	}
}

String WifiManagerClass::getAvailableNetworks() {
	Serial.print("Scanning networks...");

	// We need to make sure we are disconnected
	// before trying to scan for networks. We do
	// not care about the return value from it.
	// WiFi.disconnect();  - not needed.

	byte networks = WiFi.scanNetworks();

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

	_networks = getAvailableNetworks();

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

	WiFi.softAP(ssid);

	_ssid = WiFi.softAPSSID();
	_ip = WiFi.softAPIP();

	Serial.println("Server IP Address:");
	Serial.println(_ip);
}

void WifiManagerClass::startManagementServerWeb() {

	_server
		.serveStatic("/", LittleFS, "/wifi-manager")
		.setDefaultFile("index.html");

	_server.on("/networks", HTTP_GET, [=](AsyncWebServerRequest *request) {


		request->send(200, "application/json", _networks);
	});

	_server.on("/credentials", HTTP_PUT, [=](AsyncWebServerRequest *request) {
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

		delay(1000);

		ESP.restart();
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
