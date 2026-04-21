#include "Configuration.h"

#include <LittleFS.h>
#include <cstdio>
#include <unistd.h>

Configuration::Configuration() {
	// File paths to save input values permanently
	ssidPath = "/ssid.txt";
	passPath = "/pass.txt";
	hostnamePath = "/hostname.txt";
	dmx_addrPath = "/address.txt";
	dmx_uniPath = "/universe.txt";
	dhcpPath = "/dhcp.txt";
	ipPath = "/ip.txt";
	gatewayPath = "/gateway.txt";
	subnetPath = "/subnet.txt";
	dns1Path = "/dns1.txt";
	dns2Path = "/dns2.txt";
	startValuePath = "/start_value.txt";

	fsInitialized = false;
}

void Configuration::initFileSystem() {
	if (fsInitialized) {
		return;
	}

	fsInitialized = true;

	if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) {
		Serial.println("An error has occurred while mounting LittleFS");
		return;
	}

	ensureDefaultFile(dhcpPath, "1");
	ensureDefaultFile(ipPath, "");
	ensureDefaultFile(gatewayPath, "");
	ensureDefaultFile(subnetPath, "");
	ensureDefaultFile(dns1Path, "");
	ensureDefaultFile(dns2Path, "");
	ensureDefaultFile(startValuePath, "0");

	Serial.println("LittleFS mounted successfully");
}

void Configuration::ensureDefaultFile(const char *path, const char *defaultValue) {
	if (LittleFS.exists(path)) {
		return;
	}
	writeFile(LittleFS, path, defaultValue ? defaultValue : "");
}

void Configuration::writeSSID(const char *ssid) {
	writeFile(LittleFS, ssidPath, ssid);
}

void Configuration::writePass(const char *pass) {
	writeFile(LittleFS, passPath, pass);
}

void Configuration::writeHostname(const char *hostname) {
	writeFile(LittleFS, hostnamePath, hostname);
}

void Configuration::writeDMXAddress(int value) {
	writeFile(LittleFS, dmx_addrPath, String(value).c_str());
}

void Configuration::writeDMXUniverse(int value) {
	writeFile(LittleFS, dmx_uniPath, String(value).c_str());
}

void Configuration::writeDhcpEnabled(bool enabled) {
	writeFile(LittleFS, dhcpPath, enabled ? "1" : "0");
}

void Configuration::writeStaticIP(const char *ip) {
	writeFile(LittleFS, ipPath, ip ? ip : "");
}

void Configuration::writeGateway(const char *gateway) {
	writeFile(LittleFS, gatewayPath, gateway ? gateway : "");
}

void Configuration::writeSubnet(const char *subnet) {
	writeFile(LittleFS, subnetPath, subnet ? subnet : "");
}

void Configuration::writeDNS1(const char *dns1) {
	writeFile(LittleFS, dns1Path, dns1 ? dns1 : "");
}

void Configuration::writeDNS2(const char *dns2) {
	writeFile(LittleFS, dns2Path, dns2 ? dns2 : "");
}

void Configuration::writeStartValue(float value) {
	char buf[24];
	snprintf(buf, sizeof(buf), "%.6g", (double)value);
	writeFile(LittleFS, startValuePath, buf);
}

String Configuration::getSSID() {
	return readFile(LittleFS, ssidPath);
}

String Configuration::getPass() {
	String pass = readFile(LittleFS, passPath);
	return pass;
}

String Configuration::getHostname() {
	return readFile(LittleFS, hostnamePath);
}

int Configuration::getDMXAddress() {
	String value = readFile(LittleFS, dmx_addrPath);
	return atoi(value.c_str());
}

int Configuration::getDMXUniverse() {
	String value = readFile(LittleFS, dmx_uniPath);
	return atoi(value.c_str());
}

bool Configuration::getDhcpEnabled() {
	String value = readFile(LittleFS, dhcpPath);
	value.trim();
	value.toLowerCase();
	if (value.length() == 0) {
		return true;
	}
	return !(value == "0" || value == "false" || value == "static");
}

String Configuration::getStaticIP() {
	return readFile(LittleFS, ipPath);
}

String Configuration::getGateway() {
	return readFile(LittleFS, gatewayPath);
}

String Configuration::getSubnet() {
	return readFile(LittleFS, subnetPath);
}

String Configuration::getDNS1() {
	return readFile(LittleFS, dns1Path);
}

String Configuration::getDNS2() {
	return readFile(LittleFS, dns2Path);
}

float Configuration::getStartValue() {
	String value = readFile(LittleFS, startValuePath);
	if (value.length() == 0) {
		return 0.0f;
	}
	return value.toFloat();
}

// Read file from LittleFS
String Configuration::readFile(fs::FS &fs, const char *path) {
	initFileSystem();

	if (!fs.exists(path)) {
		return String();
	}

	File file = fs.open(path);
	if (!file || file.isDirectory()) {
		return String();
	}

	String fileContent = file.readStringUntil('\n');
	file.close();
	return fileContent;
}

// Write file to LittleFS
void Configuration::writeFile(fs::FS &fs, const char *path, const char *message) {
	initFileSystem();
	(void)fs;

	const char *value = message ? message : "";
	size_t len = strlen(value);

	char tmpPath[64];
	snprintf(tmpPath, sizeof(tmpPath), "%s.tmp", path);
	char fullTmpPath[96];
	char fullPath[96];
	snprintf(fullTmpPath, sizeof(fullTmpPath), "/littlefs%s", tmpPath);
	snprintf(fullPath, sizeof(fullPath), "/littlefs%s", path);

	remove(fullTmpPath);

	FILE *file = fopen(fullTmpPath, "w");
	if (!file) {
		Serial.printf("Failed to open temp config file for write: %s\n", fullTmpPath);
		return;
	}

	size_t written = 0;
	if (len > 0) {
		written = fwrite(value, 1, len, file);
		if (written != len) {
			Serial.printf("Failed to write config file: %s (%u/%u bytes)\n",
			              path,
			              (unsigned int)written,
			              (unsigned int)len);
			fclose(file);
			remove(fullTmpPath);
			return;
		}
	}
	fflush(file);
	fsync(fileno(file));
	fclose(file);

	if (LittleFS.exists(path) && remove(fullPath) != 0) {
		Serial.printf("Failed to remove old config file: %s\n", path);
		remove(fullTmpPath);
		return;
	}

	if (rename(fullTmpPath, fullPath) != 0) {
		Serial.printf("Failed to rename temp config file: %s -> %s\n", fullTmpPath, fullPath);
		remove(fullTmpPath);
	}
}
