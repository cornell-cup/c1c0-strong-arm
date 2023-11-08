#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <Servo.h>

// This file is used to adjust the position of the elbow stepper before zeroing out the encoder. 
// Set the direction of the hand through the Serial monitor by inputting a '1' or '2', and 
// then type '0' when you want to zero the encoder at that position.

char input = 0;

// Stepper motor (elbow) encoder
int s0 = 4;     // step pin
int d0 = 10;    // direction pin
int c0 = 35;    // chip select pin 

MovingSteppersLib motors[1] {{s0,d0,c0}};     

void setup() {
  Serial.begin(9600);  
  pinMode(d0, OUTPUT);
  pinMode(s0, OUTPUT);
}

void loop() {
  while (Serial.available() > 0) {      // checks if data has been sent to the Serial monitor by the user
    input = Serial.read(); 
  }     
  
  // char input is controlled by typing into Serial monitor. Set to '0' by default.
  // '0' --> no motor movement
  // '1' --> bend out 
  // '2' --> bend in
  if (input == '0') {
    digitalWrite(s0, LOW);
    motors[0].encoder.setZeroSPI(c0);
  } else if (input == '1') {  
    digitalWrite(d0, LOW);
    digitalWrite(s0, HIGH);
    delay(1);
    digitalWrite(s0, LOW);
    delay(2);
  } else if (input == '2') {
    digitalWrite(d0, HIGH);
    digitalWrite(s0, HIGH);
    delay(1);
    digitalWrite(s0, LOW);
    delay(2);
  }
}
