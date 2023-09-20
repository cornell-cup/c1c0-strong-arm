#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <Servo.h>

// This file is used to adjust the position of the spin servo. Set the direction of the hand through 
// the Serial monitor by inputting a '1' or '2', and then type '0' when you want to zero the encoder 
// at that position.

Servo spin_servo;
char input = 0; 

void setup() {
  Serial.begin(9600);     
  spin_servo.attach(6);
}

void loop() {
  while (Serial.available() > 0) {      // checks if data has been sent to the Serial monitor by the user
    input = Serial.read(); 
  }     
  
  // char input is controlled by typing into Serial monitor. Set to '0' by default.
  // '0' --> no motor movement
  // '1' --> opens hand
  // '2' --> closes hand
  if (input == '0') {
    spin_servo.write(93);
  } else if (input == '1') {
    spin_servo.write(113);
  } else if (input == '2') {
    spin_servo.write(73);
  }
}
