#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Configuration.h"

class WifiManagerClass {
	public:
		explicit WifiManagerClass(Configuration& config);   // inject by reference

		bool connectToWifi();

		void startManagementAP();
        void startManagementServerWeb();
		AsyncWebServer& getServer();

		void check();

		String getHostname();
		String getSSID();
		int8_t getRSSI();
		IPAddress getIP();

		bool isConnected();

	private:
		AsyncWebServer _server;
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

		String getAvailableNetworks();
		void requestRestart(unsigned long delayMs);
		void pollNetworkScan();
		void startNetworkScan();
		void runNetworkScanTask();
		static void networkScanTaskEntry(void* parameter);

		bool waitForConnection();
        bool acceptsCompressedResponse(AsyncWebServerRequest *request);
};
#endif
