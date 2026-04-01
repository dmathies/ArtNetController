#include "Configuration.h"

#include <SPIFFS.h>

Configuration::Configuration() {
	// File paths to save input values permanently
	ssidPath = "/ssid.txt";
	passPath = "/pass.txt";
	hostnamePath = "/hostname.txt";
	dmx_addrPath = "/address.txt";
	dmx_uniPath = "/universe.txt";

	fsInitialized = false;
}

void Configuration::initSPIFFS() {
	if (fsInitialized) {
		return;
	}

	fsInitialized = true;

	if (!SPIFFS.begin(true)) {
		Serial.println("An error has occurred while mounting SPIFFS");
		return;
	}

	Serial.println("SPIFFS mounted successfully");
}

void Configuration::writeSSID(const char *ssid) {
	writeFile(SPIFFS, ssidPath, ssid);
}

void Configuration::writePass(const char *pass) {
	writeFile(SPIFFS, passPath, pass);
}

void Configuration::writeHostname(const char *hostname) {
	writeFile(SPIFFS, hostnamePath, hostname);
}

void Configuration::writeDMXAddress(int value) {
	writeFile(SPIFFS, dmx_addrPath, String(value).c_str());
}

void Configuration::writeDMXUniverse(int value) {
	writeFile(SPIFFS, dmx_uniPath, String(value).c_str());
}

String Configuration::getSSID() {
	return readFile(SPIFFS, ssidPath);
}

String Configuration::getPass() {
	String pass = readFile(SPIFFS, passPath);
	return pass;
}

String Configuration::getHostname() {
	return readFile(SPIFFS, hostnamePath);
}

int Configuration::getDMXAddress() {
	String value = readFile(SPIFFS, dmx_addrPath);
	return atoi(value.c_str());
}

int Configuration::getDMXUniverse() {
	String value = readFile(SPIFFS, dmx_uniPath);
	return atoi(value.c_str());
}

// Read File from SPIFFS
String Configuration::readFile(fs::FS &fs, const char *path) {
	initSPIFFS();

	File file = fs.open(path);
	if (!file || file.isDirectory()) {
		return String();
	}

	String fileContent = file.readStringUntil('\n');
	return fileContent;
}

// Write file to SPIFFS
void Configuration::writeFile(fs::FS &fs, const char *path, const char *message) {
	initSPIFFS();

	File file = fs.open(path, FILE_WRITE);
	if (!file) {
		return;
	}
	file.print(message);
}
