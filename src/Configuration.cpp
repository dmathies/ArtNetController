#include "Configuration.h"

#include <LittleFS.h>

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

	Serial.println("LittleFS mounted successfully");
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
	return fileContent;
}

// Write file to LittleFS
void Configuration::writeFile(fs::FS &fs, const char *path, const char *message) {
	initFileSystem();

	File file = fs.open(path, FILE_WRITE);
	if (!file) {
		return;
	}
	file.print(message);
}
