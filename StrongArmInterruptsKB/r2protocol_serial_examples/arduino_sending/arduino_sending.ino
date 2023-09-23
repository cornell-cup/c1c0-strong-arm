// Example File for Sending R2Protocol messages to Jetson 
// In coordination with jetson_receiving.py

// See the FAQ on the Cornell Cup Google Drive, located Here: 
// https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit


#include "R2Protocol.h"

#define MAX_BUFFER_SIZE 2048
uint32_t counter = 0;

char msg[9] = "baseball";
uint8_t msg_data_buffer[1];
uint8_t msg_send_buffer[MAX_BUFFER_SIZE];

void setup() {
  Serial.begin(9600); // Serial monitor
  Serial1.begin(38400); // TX1/RX1 

  while (Serial1.available() > 0) {
    Serial1.read();
  }
  delay(100);

  for(int i = 0; i < 8; i++) {
    msg_data_buffer[i] = (uint8_t) msg[i];
  }
}

void send(char type[5], const uint8_t* msg, uint32_t msg_len, uint8_t* send_buffer) {
  uint32_t written = r2p_encode(type, msg, msg_len, send_buffer, MAX_BUFFER_SIZE);
  Serial1.write(send_buffer, written);
  //Serial.println("NUMBER OF BYTES WRITTEN: " + String(written));
}

void loop() {
  send("sprt", msg_data_buffer, 8, msg_send_buffer);
  Serial.println(String(++counter));
  //delay(250);
}
