#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <ESPAsyncWebServer.h>

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

		String getAvailableNetworks();

		bool waitForConnection();
        bool acceptsCompressedResponse(AsyncWebServerRequest *request);
};
#endif
