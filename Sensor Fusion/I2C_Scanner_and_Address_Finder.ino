#include <Wire.h>

// Change these if needed
#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n🔍 ESP32 I2C Scanner Starting...");
  
  // Initialize I2C with explicit pins
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.print("Using SDA = GPIO ");
  Serial.print(SDA_PIN);
  Serial.print(", SCL = GPIO ");
  Serial.println(SCL_PIN);
}

void loop() {
  byte error, address;
  int devicesFound = 0;

  Serial.println("\nScanning I2C bus...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("✅ I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devicesFound++;
    }
    else if (error == 4) {
      Serial.print("⚠️ Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (devicesFound == 0) {
    Serial.println("❌ No I2C devices found");
    Serial.println("Check wiring, power, and pull-ups");
  } else {
    Serial.print("🎯 Total devices found: ");
    Serial.println(devicesFound);
  }

  delay(3000);  // scan every 3 seconds
}