#include "WifiManager.h"

#include <ArduinoJson.h>
#include <ESPmDNS.h>

#ifndef APP_MDNS_ENABLE
// XIAO-class builds have plenty of room for mDNS. NodeMCU can also run it,
// but disabling it is the preferred low-memory configuration when heap margin
// matters more than .local discovery convenience.
#define APP_MDNS_ENABLE 1
#endif
#include <WiFi.h>
#include <LittleFS.h>
#include <esp_wifi.h>
#include <esp_err.h>
#include <esp_task_wdt.h>
#include <cctype>

#include "Configuration.h"
#include "main_common.h"
#include "RemoteLogBuffer.h"

namespace {
WifiManagerClass* g_wifiManagerInstance = nullptr;

void applyWifiPowerSaveForCoexistence() {
	WiFi.setSleep(true);
	esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
}

void applyStaTxPowerPreference() {
	if (!WiFi.setTxPower(WIFI_POWER_8_5dBm)) {
		appLogLine("WiFi setTxPower(WIFI_POWER_8_5dBm) failed");
	}
}

void applyStaSecurityCompatibility() {
	wifi_config_t cfg = {};
	esp_err_t getErr = esp_wifi_get_config(WIFI_IF_STA, &cfg);
	if (getErr != ESP_OK) {
		appLogPrintf("WiFi get STA config failed (%d:%s)\n", (int)getErr, esp_err_to_name(getErr));
		return;
	}

	// Prefer WPA2-PSK when APs are in mixed WPA2/WPA3 transition modes.
	cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
	// Keep PMF optional for compatibility with consumer routers.
	cfg.sta.pmf_cfg.capable = true;
	cfg.sta.pmf_cfg.required = false;

	esp_err_t setErr = esp_wifi_set_config(WIFI_IF_STA, &cfg);
	if (setErr != ESP_OK) {
		appLogPrintf("WiFi set STA compat config failed (%d:%s)\n", (int)setErr, esp_err_to_name(setErr));
	}
}

void applyStaPostBeginPreferences() {
	applyStaTxPowerPreference();

	wl_status_t status = WiFi.status();
	if (status == WL_CONNECTED) {
		return;
	}

	if (status == WL_IDLE_STATUS || status == WL_DISCONNECTED || status == WL_NO_SSID_AVAIL ||
	    status == WL_SCAN_COMPLETED || status == WL_CONNECT_FAILED || status == WL_CONNECTION_LOST) {
		applyStaSecurityCompatibility();
	}
}

int hexNibble(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
	if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
	return -1;
}

String normalizeCredentialForAuthRetry(const String& input, bool* changedOut) {
	const bool hasSpace = input.indexOf(' ') >= 0;
	String out;
	out.reserve(input.length());
	bool changed = false;

	const size_t len = input.length();
	for (size_t i = 0; i < len; i++) {
		char ch = input[i];
		if (ch == '%' && (i + 2) < len) {
			int hi = hexNibble(input[i + 1]);
			int lo = hexNibble(input[i + 2]);
			if (hi >= 0 && lo >= 0) {
				out += (char)((hi << 4) | lo);
				i += 2;
				changed = true;
				continue;
			}
		}
		if (ch == '+' && !hasSpace) {
			out += ' ';
			changed = true;
			continue;
		}
		out += ch;
	}

	if (changedOut) {
		*changedOut = changed;
	}
	return changed ? out : input;
}

String buildNetworksJsonFromApRecords(const wifi_ap_record_t* records, uint16_t count) {
	String json = "[";
	json.reserve((count * 34) + 2);
	bool first = true;

	for (uint16_t i = 0; i < count; i++) {
		if (records[i].ssid[0] == '\0') {
			continue;
		}

		String network = "\"" + String((const char*)records[i].ssid) + "\"";
		if (json.indexOf(network) == -1) {
			if (!first) {
				json += ",";
			}
			json += network;
			first = false;
		}
	}

	json += "]";
	return json;
}

void wifiEventRouter(WiFiEvent_t event, WiFiEventInfo_t info) {
	if (g_wifiManagerInstance) {
		g_wifiManagerInstance->handleWifiEvent(event, info);
	}
}
}

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
	_scanStartedAtMs = 0;
	_reconnectAttempts = 0;
	_reconnectSuccesses = 0;
	_lastReconnectAttemptMs = 0;
	_lastReconnectSuccessMs = 0;
	_lastDisconnectReason = 0;
	_lastDisconnectAtMs = 0;
	_preferredBssidValid = false;
	memset(_preferredBssid, 0, sizeof(_preferredBssid));
	_preferredChannel = 0;

	g_wifiManagerInstance = this;
	WiFi.onEvent(wifiEventRouter);

	_networks = "[]";
	_hostname = "";
	_stationSsid = "";
	_apSsid = "";
}

void WifiManagerClass::handleWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
	if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
		_lastDisconnectReason = (uint8_t)info.wifi_sta_disconnected.reason;
		_lastDisconnectAtMs = millis();
		appLogPrintf("WiFi disconnected: reason=%u (%s)\n",
		             (unsigned)_lastDisconnectReason,
		             wifiDisconnectReasonToString(_lastDisconnectReason));
	}
}

const char* WifiManagerClass::wifiDisconnectReasonToString(uint8_t reason) {
	switch (reason) {
		case 2: return "auth_expired";
		case 3: return "auth_leave";
		case 4: return "assoc_expired";
		case 5: return "assoc_toomany";
		case 6: return "not_authed";
		case 7: return "not_assoced";
		case 8: return "assoc_leave";
		case 9: return "assoc_not_authed";
		case 15: return "4way_handshake_timeout";
		case 16: return "group_key_update_timeout";
		case 17: return "ie_in_4way_differs";
		case 23: return "8021x_auth_failed";
		case 200: return "beacon_timeout";
		case 201: return "no_ap_found";
		case 202: return "auth_fail";
		case 203: return "assoc_fail";
		case 204: return "handshake_timeout";
		case 205: return "connection_fail";
		case 206: return "ap_tsf_reset";
		default: return "unknown";
	}
}

void WifiManagerClass::check() {
	esp_task_wdt_reset();
	wl_status_t status = WiFi.status();
	if (status == WL_CONNECTED) {
		_ip = WiFi.localIP();
	}

	pollNetworkScan();

	if (!_connected && status == WL_CONNECTED) {
		_connected = true;
		_reconnectSuccesses++;
		_lastReconnectSuccessMs = millis();
		appLogLine("WiFi reconnected");
	}

	if (_restartPending && millis() >= _restartAtMs) {
		appLogLine("Restarting device...");
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
	yield();
}

String WifiManagerClass::getNetworksPayload(bool details, bool refresh) {
	if (!_otaInProgress && refresh) {
		startNetworkScan();
	}

	pollNetworkScan();

	bool isScanning = _scanRequested || _scanInProgress;

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
	appLogLine("Cleaning up WiFi and mDNS before restart...");
	
	// Stop mDNS responder
	MDNS.end();
	
	// Disconnect from WiFi
	WiFi.disconnect(true);  // true = turn off WiFi radio
	
	// Fully deinitialize WiFi driver
	esp_wifi_deinit();

	// Ensure filesystem metadata is committed before the software reset.
	LittleFS.end();
	
	appLogLine("Cleanup complete");
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

	applyWifiPowerSaveForCoexistence();

	appLogLine("WiFi not connected. Attempting to reconnect.");

	String pass = _config.getPass();
	if (!_preferredBssidValid) {
		selectPreferredBssidForSsid(_stationSsid);
	}
	applyStaTxPowerPreference();
	applyStaSecurityCompatibility();
	if (_preferredBssidValid) {
		WiFi.begin(_stationSsid.c_str(),
		           pass.c_str(),
		           _preferredChannel,
		           _preferredBssid,
		           true);
	} else {
		WiFi.begin(_stationSsid.c_str(), pass.c_str());
	}
}

bool WifiManagerClass::selectPreferredBssidForSsid(const String& ssid) {
	(void)ssid;
	_preferredBssidValid = false;
	_preferredChannel = 0;
	// appLogLine("Skipping pre-connect BSSID scan to avoid startup blocking");
	return false;
}

void WifiManagerClass::startNetworkScan() {
	if (_scanInProgress || _scanRequested || _otaInProgress) return;
	_scanHasResult = false;
	_scanRequested = true;
}

void WifiManagerClass::pollNetworkScan() {
	if (_otaInProgress) {
		return;
	}

	if (_scanRequested && !_scanInProgress) {
		_scanRequested = false;
		_scanStartedAtMs = millis();
		appLogLine("Starting WiFi scan...");
		appLogLine("Scanning networks...");
    appSetBleAdvertisingEnabled(false);

		wifi_mode_t mode = WiFi.getMode();
		if (mode != WIFI_MODE_APSTA) {
			WiFi.mode(WIFI_MODE_APSTA);
			applyWifiPowerSaveForCoexistence();
			yield();
			delay(50);
		}

		// If STA connect is still in progress after a failed join attempt,
		// scan start can fail with ESP_ERR_WIFI_STATE.
		if (_managementApActive && WiFi.status() != WL_CONNECTED) {
			esp_wifi_disconnect();
			yield();
			delay(20);
		}

		int previousScan = WiFi.scanComplete();
		if (previousScan == WIFI_SCAN_RUNNING) {
			WiFi.scanDelete();
			yield();
			delay(20);
		}
		WiFi.scanDelete();
		int startResult = WiFi.scanNetworks(true, false);
		if (startResult == WIFI_SCAN_RUNNING) {
			_scanInProgress = true;
			return;
		}

		if (startResult == WIFI_SCAN_FAILED) {
			appLogLine("WiFi async scan failed, retrying sync scan");
			yield();
			delay(50);
			startResult = WiFi.scanNetworks(false, false);

			if (startResult == WIFI_SCAN_FAILED) {
				appLogLine("WiFi sync scan failed, retrying IDF scan");
				esp_wifi_scan_stop();

				wifi_scan_config_t config = {};
				config.show_hidden = false;
				esp_err_t idfScanErr = esp_wifi_scan_start(&config, true);
				if (idfScanErr == ESP_ERR_WIFI_STATE) {
					appLogLine("WiFi IDF scan in bad state, resetting STA state and retrying");
					esp_wifi_disconnect();
					yield();
					delay(20);
					idfScanErr = esp_wifi_scan_start(&config, true);
				}
				if (idfScanErr == ESP_OK) {
					uint16_t apCount = 0;
					esp_err_t countErr = esp_wifi_scan_get_ap_num(&apCount);
					if (countErr != ESP_OK) {
						appLogPrintf("WiFi IDF scan count failed (%d:%s)\n",
						             (int)countErr,
						             esp_err_to_name(countErr));
						startResult = WIFI_SCAN_FAILED;
					} else if (apCount == 0) {
						_networks = "[]";
						startResult = 0;
					} else {
						wifi_ap_record_t* records =
							(wifi_ap_record_t*)malloc(sizeof(wifi_ap_record_t) * apCount);
						if (!records) {
							appLogLine("WiFi IDF scan allocation failed");
							startResult = WIFI_SCAN_FAILED;
						} else {
							uint16_t recordsToRead = apCount;
							esp_err_t recordsErr =
								esp_wifi_scan_get_ap_records(&recordsToRead, records);
							if (recordsErr != ESP_OK) {
								appLogPrintf("WiFi IDF scan records failed (%d:%s)\n",
								             (int)recordsErr,
								             esp_err_to_name(recordsErr));
								startResult = WIFI_SCAN_FAILED;
							} else {
								_networks = buildNetworksJsonFromApRecords(records, recordsToRead);
								startResult = recordsToRead;
							}
							free(records);
						}
					}
				} else {
					appLogPrintf("WiFi IDF scan start failed (%d:%s)\n",
					             (int)idfScanErr,
					             esp_err_to_name(idfScanErr));
					startResult = WIFI_SCAN_FAILED;
				}
			}
		}

		if (startResult >= 0) {
			_networks = buildNetworksJson(startResult);
			WiFi.scanDelete();
			appLogLine("WiFi scan complete");
		} else {
			appLogPrintf("WiFi scan failed (%d)\n", startResult);
			_networks = "[]";
		}
		appSetBleAdvertisingEnabled(true);

		_scanInProgress = false;
		_scanHasResult = true;
		return;
	}

	if (_scanInProgress) {
		int scanResult = WiFi.scanComplete();
		if (scanResult == WIFI_SCAN_RUNNING) {
			return;
		}

		if (scanResult >= 0) {
			_networks = buildNetworksJson(scanResult);
			WiFi.scanDelete();
			appLogLine("WiFi scan complete");
		} else {
			appLogPrintf("WiFi scan failed (%d)\n", scanResult);
			_networks = "[]";
		}
		appSetBleAdvertisingEnabled(true);

		_scanInProgress = false;
		_scanHasResult = true;
		return;
	}
}

String WifiManagerClass::buildNetworksJson(int networks) {
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
	return json;
}

String WifiManagerClass::getAvailableNetworks() {
	return _networks;
}

bool WifiManagerClass::connectToWifi() {
	_stationSsid = _config.getSSID();
	_hostname = _config.getHostname();
	String pass = _config.getPass();
	_managementApActive = false;
	_apSsid = "";

	if (_stationSsid == "") {
		appLogLine("No connection information specified");

		return false;
	}

	WiFi.mode(WIFI_MODE_STA);
	WiFi.setAutoReconnect(true);
	WiFi.setSleep(false);

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
				appLogLine("Static IP config failed, falling back to DHCP");
				WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
			} else {
				appLogLine("Using static IPv4 configuration");
			}
		} else {
			appLogLine("Invalid static IPv4 settings, falling back to DHCP");
			WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
		}
	}

	if (_hostname != "") {
		WiFi.setHostname(_hostname.c_str());
		appLogPrintf("Setting hostname %s\n", _hostname.c_str());

#if APP_MDNS_ENABLE
		if (MDNS.begin(_hostname.c_str())) {
			appLogLine("mDNS responder started");
		} else {
			appLogLine("Unable to start mDNS responder");
		}
#else
		appLogLine("mDNS responder disabled");
#endif
	} else {
		appLogLine("No hostname configured");
	}

	appLogLine("Connecting to WiFi...");
	applyStaTxPowerPreference();
	applyStaSecurityCompatibility();

	selectPreferredBssidForSsid(_stationSsid);
	if (_preferredBssidValid) {
		WiFi.begin(_stationSsid.c_str(),
		           pass.c_str(),
		           _preferredChannel,
		           _preferredBssid,
		           true);
	} else {
		WiFi.begin(_stationSsid.c_str(), pass.c_str());
	}

	const unsigned long startedAtMs = millis();
	while (millis() - startedAtMs < _connectionTimeout) {
		wl_status_t status = WiFi.status();
		if (status == WL_CONNECTED) {
			_ip = WiFi.localIP();
			_connected = true;
			appLogPrintf("Assigned IP Address: %s\n", _ip.toString().c_str());
			applyWifiPowerSaveForCoexistence();
			return true;
		}

		if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL || status == WL_CONNECTION_LOST) {
			break;
		}

		esp_task_wdt_reset();
		yield();
		delay(50);
	}

	if (!_connected && _lastDisconnectReason == 2) {
		bool changed = false;
		String retryPass = normalizeCredentialForAuthRetry(pass, &changed);
		if (changed && retryPass != pass) {
			WiFi.disconnect(true, true);
			applyStaTxPowerPreference();
			if (_preferredBssidValid) {
				WiFi.begin(_stationSsid.c_str(),
				           retryPass.c_str(),
				           _preferredChannel,
				           _preferredBssid,
				           true);
			} else {
				WiFi.begin(_stationSsid.c_str(), retryPass.c_str());
			}
			const unsigned long retryStartedAtMs = millis();
			while (millis() - retryStartedAtMs < _connectionTimeout) {
				wl_status_t status = WiFi.status();
				if (status == WL_CONNECTED) {
					_ip = WiFi.localIP();
					_connected = true;
					appLogPrintf("Assigned IP Address: %s\n", _ip.toString().c_str());
					_config.writePass(retryPass);
					applyWifiPowerSaveForCoexistence();
					return true;
				}
				esp_task_wdt_reset();
				yield();
				delay(50);
			}
		}
	}

	appLogPrintf("WiFi connect timed out; status=%d ssid=%s\n", (int)WiFi.status(), _stationSsid.c_str());
	applyWifiPowerSaveForCoexistence();

	return false;
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

	appLogLine("Starting Management AP");
	WiFi.softAPdisconnect(true);
	if (WiFi.getMode() != WIFI_MODE_APSTA) {
		WiFi.mode(WIFI_MODE_APSTA);
	}
	applyWifiPowerSaveForCoexistence();
	delay(100);

	bool apOk = WiFi.softAP(ssid);
	if (!apOk || WiFi.softAPIP() == IPAddress((uint32_t)0)) {
		appLogLine("Initial AP start failed, retrying WiFi AP setup");
		WiFi.mode(WIFI_MODE_NULL);
		delay(100);
		WiFi.mode(WIFI_MODE_APSTA);
		applyWifiPowerSaveForCoexistence();
		delay(100);
		apOk = WiFi.softAP(ssid);
	}

	_managementApActive = apOk && (WiFi.softAPIP() != IPAddress((uint32_t)0));
	_apSsid = _managementApActive ? WiFi.softAPSSID() : "";
	_ip = WiFi.softAPIP();

	if (!_managementApActive) {
		appLogLine("Management AP failed to start");
		return;
	}

	appLogPrintf("Server IP Address: %s\n", _ip.toString().c_str());
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
