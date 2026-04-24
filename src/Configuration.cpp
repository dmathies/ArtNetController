#include "Configuration.h"

#include <LittleFS.h>
#include <cstdio>

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

	if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) {
		Serial.println("An error has occurred while mounting LittleFS");
		return;
	}

	fsInitialized = true;

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

bool Configuration::writeSSID(const String& ssid) {
	return writeFile(LittleFS, ssidPath, ssid.c_str());
}

bool Configuration::writePass(const String& pass) {
	return writeFile(LittleFS, passPath, pass.c_str());
}

bool Configuration::writeHostname(const String& hostname) {
	return writeFile(LittleFS, hostnamePath, hostname.c_str());
}

bool Configuration::writeDMXAddress(int value) {
	String text = String(value);
	return writeFile(LittleFS, dmx_addrPath, text.c_str());
}

bool Configuration::writeDMXUniverse(int value) {
	String text = String(value);
	return writeFile(LittleFS, dmx_uniPath, text.c_str());
}

bool Configuration::writeDhcpEnabled(bool enabled) {
	return writeFile(LittleFS, dhcpPath, enabled ? "1" : "0");
}

bool Configuration::writeStaticIP(const String& ip) {
	return writeFile(LittleFS, ipPath, ip.c_str());
}

bool Configuration::writeGateway(const String& gateway) {
	return writeFile(LittleFS, gatewayPath, gateway.c_str());
}

bool Configuration::writeSubnet(const String& subnet) {
	return writeFile(LittleFS, subnetPath, subnet.c_str());
}

bool Configuration::writeDNS1(const String& dns1) {
	return writeFile(LittleFS, dns1Path, dns1.c_str());
}

bool Configuration::writeDNS2(const String& dns2) {
	return writeFile(LittleFS, dns2Path, dns2.c_str());
}

bool Configuration::writeStartValue(float value) {
	char buf[24];
	snprintf(buf, sizeof(buf), "%.6g", (double)value);
	return writeFile(LittleFS, startValuePath, buf);
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
	String fileContent = "";
	initFileSystem();

	if (fs.exists(path)) {
		File file = fs.open(path);
		if (file && !file.isDirectory()) {
			fileContent = file.readStringUntil('\n');
			file.close();
		}
	}

	return fileContent;
}

// Write file to LittleFS
bool Configuration::writeFile(fs::FS &fs, const char *path, const char *message) {
	initFileSystem();

	const char *value = message ? message : "";
	size_t len = strlen(value);

	if (fs.exists(path) && !fs.remove(path)) {
		Serial.printf("Failed to remove config file: %s\n", path);
		return false;
	}

	File file = fs.open(path, FILE_WRITE, true);
	if (!file || file.isDirectory()) {
		Serial.printf("Failed to open config file for write: %s\n", path);
		return false;
	}

	size_t written = file.print(value);
	file.flush();
	file.close();

	if (written != len) {
		Serial.printf("Failed to write config file: %s (%u/%u bytes)\n",
		              path,
		              (unsigned int)written,
		              (unsigned int)len);
		fs.remove(path);
		return false;
	}

	File verifyFile = fs.open(path, FILE_READ);
	if (!verifyFile || verifyFile.isDirectory()) {
		Serial.printf("Failed to reopen config file for verify: %s\n", path);
		fs.remove(path);
		return false;
	}

	String verifyValue = verifyFile.readString();
	verifyFile.close();
	if (verifyValue != value) {
		Serial.printf("Config verify mismatch for %s\n", path);
		fs.remove(path);
		return false;
	}

	return true;
}
