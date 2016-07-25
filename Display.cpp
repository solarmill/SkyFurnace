#include "Display.h"

void Display::init() {
	lcd = new LiquidCrystal_I2C(0x27, 20, 4);
	lcd->init();
	lcd->backlight();

	lcd->createChar(0, bell);
	lcd->createChar(1, note);
	lcd->createChar(2, clock);
	lcd->createChar(3, heart);
	lcd->createChar(4, duck);
	lcd->createChar(5, check);
	lcd->createChar(6, cross);
	lcd->createChar(7, retarrow);
	lcd->home();
}

void Display::message(String line1, String line2) {
	lcd->clear();
	lcd->setCursor((20 - line1.length()) / 2, 1);
	lcd->print(line1);
	lcd->setCursor((20 - line2.length()) / 2, 2);
	lcd->print(line2);
	Serial.println("Message: " + line1 + " " + line2);
}

void Display::setCursor(byte column, byte row) {
	lcd->setCursor(column, row);
}

void Display::print(String line) {
	lcd->print(line);
}

void Display::message(String line1) {
	message(line1, "");
}

void Display::alert(String line1, String line2) {
	lcd->noBacklight();
	message(line1, line2);
	lcd->backlight();
}

void Display::alert(String line1) {
	alert(line1, "");
}

void Display::banner(String firmwareVersion) {
	lcd->clear();
	lcd->setCursor(5, 0);
	lcd->print("SolarMill");
	lcd->setCursor(3, 1);
	lcd->print("Solar Furnace");
	lcd->setCursor(3, 3);
	lcd->print("firmware " + firmwareVersion);
}

void Display::clear() {
	lcd->clear();
}
