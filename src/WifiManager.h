#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>

#include "Configuration.h"

class WifiManagerClass {
	public:
		explicit WifiManagerClass(Configuration& config);   // inject by reference

		bool connectToWifi();

		void startManagementAP();

		void check();
		String getNetworksPayload(bool details, bool refresh);
		void scheduleRestart(unsigned long delayMs);
		void setOtaInProgress(bool inProgress);

		String getHostname();
		String getSSID();
		String getMacAddress();
		const char* getHostnameCStr() const;
		void getMacAddress(char* out, size_t outSize) const;
		int8_t getRSSI();
		IPAddress getIP();

		bool isConnected();
		uint32_t getReconnectAttempts() const;
		uint32_t getReconnectSuccesses() const;
		uint32_t getLastReconnectAttemptMs() const;
		uint32_t getLastReconnectSuccessMs() const;
		void handleWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info);

	private:
		Configuration& _config;

		bool _connected;

		int _reconnectIntervalCheck;
		int _connectionTimeout;

		String _networks;
		String _hostname;
		String _stationSsid;
		String _apSsid;

		IPAddress _ip;

		unsigned long _nextReconnectCheck;
		unsigned long _restartAtMs;
		bool _restartPending;
		bool _managementApActive;
		bool _scanInProgress;
		bool _scanRequested;
		bool _scanHasResult;
		bool _otaInProgress;
		unsigned long _scanStartedAtMs;
		uint32_t _reconnectAttempts;
		uint32_t _reconnectSuccesses;
		uint32_t _lastReconnectAttemptMs;
		uint32_t _lastReconnectSuccessMs;
		uint8_t _lastDisconnectReason;
		uint32_t _lastDisconnectAtMs;
		bool _preferredBssidValid;
		uint8_t _preferredBssid[6];
		int32_t _preferredChannel;

		String getAvailableNetworks();
		String buildNetworksJson(int networks);
		bool selectPreferredBssidForSsid(const String& ssid);
		static const char* wifiDisconnectReasonToString(uint8_t reason);
		void requestRestart(unsigned long delayMs);
		void cleanupBeforeRestart();
		void ensureReconnectAttempt();
		void pollNetworkScan();
		void startNetworkScan();

		bool waitForConnection();
};
#endif
