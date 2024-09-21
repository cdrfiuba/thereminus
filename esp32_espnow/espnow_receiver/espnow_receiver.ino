/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

const bool DEBUG = true;

char debug_string_buffer[20];
// sprintf + serial of 20 bytes takes ~200us
// sprintf + serial of 10 bytes takes ~144us
// sprintf + serial of  5 bytes takes ~108us
#define debug(formato, valor) \
  sprintf(debug_string_buffer, formato, valor); \
  Serial.print(debug_string_buffer)

// Structure example to receive data
// Must match the sender structure
typedef struct accel_packet_t {
    float x;
    float y;
    float z;
} accel_packet_t;

// Create a accel_packet_t called accelPacket
accel_packet_t accelPacket;

unsigned long timestamp = 0;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&accelPacket, incomingData, sizeof(accelPacket));
  //debug("X:% 01.3f ", accelPacket.x);
  //debug("Y:% 01.3f ", accelPacket.y);
  //debug("Z:% 01.3f ", accelPacket.z);
  Serial.print(accelPacket.x); Serial.print(" ");
  Serial.print(accelPacket.y); Serial.print(" ");
  Serial.print(accelPacket.z); Serial.print("\n");
  if (DEBUG) {
    Serial.print(micros() - timestamp); Serial.print("\n");
    timestamp = micros();
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    while (true) {
      Serial.println("Error initializing ESP-NOW");
      delay(1000);
    }
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {

}
