// error 5 at address 0x3B

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Serial.println("SETUP");
  
  Wire.setClock(100000);
  delay(50);
  Wire.begin(21, 22, 100000); // sda, sdl, freq
  delay(50);
  Wire.setTimeout(50000); //us
}

void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address <= 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission(true);
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 2) {
      // ignore
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    } else {
      Serial.print("error ");
      Serial.print(error);
      Serial.print(" at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(1000);
}
