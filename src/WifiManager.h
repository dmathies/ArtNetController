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
		int8_t getRSSI();
		IPAddress getIP();

		bool isConnected();
		uint32_t getReconnectAttempts() const;
		uint32_t getReconnectSuccesses() const;
		uint32_t getLastReconnectAttemptMs() const;
		uint32_t getLastReconnectSuccessMs() const;

	private:
		Configuration& _config;

		bool _connected;

		int _reconnectIntervalCheck;
		int _connectionTimeout;

		String _networks;
		String _hostname;
		String _ssid;

		IPAddress _ip;

		unsigned long _nextReconnectCheck;
		unsigned long _restartAtMs;
		bool _restartPending;
		bool _scanInProgress;
		bool _scanRequested;
		bool _scanHasResult;
		bool _otaInProgress;
		TaskHandle_t _scanTaskHandle;
		uint32_t _reconnectAttempts;
		uint32_t _reconnectSuccesses;
		uint32_t _lastReconnectAttemptMs;
		uint32_t _lastReconnectSuccessMs;

		String getAvailableNetworks();
		void requestRestart(unsigned long delayMs);
		void cleanupBeforeRestart();
		void pollNetworkScan();
		void startNetworkScan();
		void runNetworkScanTask();
		static void networkScanTaskEntry(void* parameter);

		bool waitForConnection();
};
#endif
