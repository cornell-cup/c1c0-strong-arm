// Example File for Recieving R2Protocol Messages from Jetson 
// In coordination with jetson_sending.py

// See the FAQ on the Cornell Cup Google Drive, located Here: 
// https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit

#include "R2Protocol.h"

// parameters that r2p decode funcion takes in
uint8_t msg_buffer[22];
uint32_t msg_buffer_len = 22;
uint16_t checksum;
char type[5];
uint8_t msg[6];
uint16_t msg_final[3];
uint32_t msg_len;


void setup() {
  Serial.begin(9600); // Serial monitor
  Serial1.begin(38400); // TX1/RX1 

  Serial.println("SETUP");

  while (Serial1.available() > 0) {
    Serial1.read();
    delay(100);
  }

  //Serial.println("DATA BEGIN:"); // might mess up input data
}


void loop() {

  if (Serial1.available() > 0) {
    Serial.println("Waiting");
    Serial1.readBytes(msg_buffer, msg_buffer_len);
    
    //r2p_decode(msg_buffer, msg_buffer_len, &checksum, type, msg, &msg_len);
    r2p_decode(msg_buffer, msg_buffer_len, &checksum, type, msg, &msg_len);

    Serial.println("Data:");
    convert_b8_to_b16(msg, msg_final, 6);
    for(int i = 0; i < 3; i++) {
      Serial.println(msg_final[i]);
    }
  }

}

void convert_b8_to_b16(uint8_t *databuffer, uint16_t *data, int len) {
  int data_idx;
  for (int i=0; i < len; i++) {
    data_idx = i / 2;
    if ( (i & 1) == 0) {
      // even
      data[data_idx] = databuffer[i] << 8;
    } else {
      // odd
      data[data_idx] |= databuffer[i];
    }
  }
}

void convert_b16_to_b8(int *databuffer, uint8_t *data, int len) {
  int data_idx1;
  int data_idx2;
  for (int i = 0; i < 2*len; i+=2) {
    data[i] = (databuffer[i/2] >> 8) & 255;
    data[i+1] = (databuffer[i/2]) & 255;
  }
}

