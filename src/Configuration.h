#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <LittleFS.h>

class Configuration {
	protected:
		void initFileSystem();
		void ensureDefaultFile(const char *path, const char *defaultValue);

		void writeFile(fs::FS &fs, const char *path, const char *message);
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

		void writeSSID(const char *ssid);
		void writePass(const char *pass);
        void writeDMXAddress(int value);
        void writeDMXUniverse(int value);
        void writeHostname(const char *hostname);
		void writeDhcpEnabled(bool enabled);
		void writeStaticIP(const char *ip);
		void writeGateway(const char *gateway);
		void writeSubnet(const char *subnet);
		void writeDNS1(const char *dns1);
		void writeDNS2(const char *dns2);
		void writeStartValue(float value);

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
