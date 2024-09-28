/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h>
#include "MMA7455.h"

const bool DEBUG = false;

char debug_string_buffer[20];
// sprintf + serial of 20 bytes takes ~200us
// sprintf + serial of 10 bytes takes ~144us
// sprintf + serial of  5 bytes takes ~108us
#define debug(formato, valor) \
  sprintf(debug_string_buffer, formato, valor); \
  Serial.print(debug_string_buffer);

MMA7455 accel;

// sender (this ESP32) MAC Address: 08:3a:f2:8e:2e:64
// receiver MAC Address: 08:d1:f9:e1:89:ac
const uint8_t receiverAddress[] = {0x08, 0xd1, 0xf9, 0xe1, 0x89, 0xac};

// Structure example to send data
// Must match the receiver structure
typedef struct accel_packet_t {
  float x;
  float y;
  float z;
} accel_packet_t;

unsigned long timestamp = 0; // for debug

void setup() {

  esp_now_peer_info_t peerInfo;

  // MMA7455 init
  uint8_t c;

  Serial.begin(115200);
  Serial.println("Freescale MMA7455 accelerometer");
  Serial.println("May 2012");

  // for esp8266 and esp32
  accel.begin(21, 22);

  // Read the Status Register
  accel.read(MMA7455_STATUS, &c, 1);
  Serial.print("Status: "); Serial.println(c);

  // Read the "Who am I" value
  accel.read(MMA7455_WHOAMI, &c, 1);
  Serial.print("WhoAmI: "); Serial.println(c);

  // Read the optional temperature output value (I always read zero)
  accel.read(MMA7455_TOUT, &c, 1);
  Serial.print("Temp.: "); Serial.println(c);
   
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    while (true) {
      Serial.println("Error initializing ESP-NOW");
      delay(1000);
    }
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  if (DEBUG) {
    esp_now_register_send_cb(OnDataSent);
  }
  
  // Register peer
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    while (true) {
      Serial.println("Failed to add peer");
      delay(1000);
    }
  }
}

void loop() {
  // Create a accel_packet_t called accelPacket
  accel_packet_t accelPacket;

  // The function MMA7455_xyz returns the 'g'-force
  // as an integer in 64 per 'g'.
  uint16_t x, y, z;
  int error;
  double dX, dY, dZ;

  // set x,y,z to zero (they are not written in case of an error).
  x = y = z = 0;
  error = accel.xyz(&x, &y, &z); // get the accelerometer values.

  // Set values to send (g values)
  accelPacket.x = (int16_t) x / 64.0;
  accelPacket.y = (int16_t) y / 64.0;
  accelPacket.z = (int16_t) z / 64.0;

  if (DEBUG) {
    debug("E:%.5i ", error);
    debug("X:% 01.3f ", accelPacket.x);
    debug("Y:% 01.3f ", accelPacket.y);
    debug("Z:% 01.3f ", accelPacket.z);
    Serial.print("\n");
  }

  if (DEBUG) {
    timestamp = micros();
  }
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &accelPacket, sizeof(accelPacket));
   
  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
    if (!DEBUG) delay(1000);
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (DEBUG) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
    Serial.print(micros() - timestamp);
    Serial.println("ms");
  }
}
