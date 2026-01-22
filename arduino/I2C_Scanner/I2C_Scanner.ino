/*
  I2C Scanner
  Scans for I2C devices and reports their addresses
*/

#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  delay(2000);
  Serial.println("\nI2C Scanner Starting...");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning I2C bus...");

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found!");
    Serial.println("Check:");
    Serial.println("  - SDA/SCL connections");
    Serial.println("  - Power (VCC = 3.3V, GND)");
    Serial.println("  - SDO pins (0x68 needs SDO->GND, 0x69 needs SDO->3.3V)");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)");
  }
  
  Serial.println();
  delay(5000);
}
