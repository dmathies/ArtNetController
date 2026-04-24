#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <LittleFS.h>

class Configuration {
	protected:
		void initFileSystem();
		void ensureDefaultFile(const char *path, const char *defaultValue);

		bool writeFile(fs::FS &fs, const char *path, const char *message);
		String readFile(fs::FS &fs, const char *path);

		const char *ssidPath;
		const char *passPath;
		const char *hostnamePath;
		const char *dmx_addrPath;
		const char *dmx_uniPath;
		const char *dhcpPath;
		const char *ipPath;
		const char *gatewayPath;
		const char *subnetPath;
		const char *dns1Path;
		const char *dns2Path;
		const char *startValuePath;

		bool fsInitialized;

	public:
		Configuration();

		bool writeSSID(const String& ssid);
		bool writePass(const String& pass);
        bool writeDMXAddress(int value);
        bool writeDMXUniverse(int value);
        bool writeHostname(const String& hostname);
		bool writeDhcpEnabled(bool enabled);
		bool writeStaticIP(const String& ip);
		bool writeGateway(const String& gateway);
		bool writeSubnet(const String& subnet);
		bool writeDNS1(const String& dns1);
		bool writeDNS2(const String& dns2);
		bool writeStartValue(float value);

        String getSSID();
		String getPass();
		String getHostname();
        int getDMXAddress();
        int getDMXUniverse();
		bool getDhcpEnabled();
		String getStaticIP();
		String getGateway();
		String getSubnet();
		String getDNS1();
		String getDNS2();
		float getStartValue();
};

#endif
