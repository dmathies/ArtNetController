#include "Configuration.h"

#include <LittleFS.h>

Configuration::Configuration() {
	// File paths to save input values permanently
	ssidPath = "/ssid.txt";
	passPath = "/pass.txt";
	hostnamePath = "/hostname.txt";
	dmx_addrPath = "/address.txt";
	dmx_uniPath = "/universe.txt";

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

// Read file from LittleFS
String Configuration::readFile(fs::FS &fs, const char *path) {
	initFileSystem();

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
