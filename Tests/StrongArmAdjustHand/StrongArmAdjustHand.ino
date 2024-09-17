#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <Servo.h>

// This file is used to adjust the position of the hand before zeroing out the encoder. This can be done by hand for the 
// elbow joint, but less easily for the hand. So, set the direction of the hand through the Serial monitor by inputting
// a '1' or '2', and then type '0' when you want to zero the encoder at that position.

Servo hand_servo;
char input = 0;

// Continuous hand encoder pins - step and direction are arbitrary
int s1 = 30;    // step pin - useless
int d1 = 31;    // direction pin - useless
int c1 = 4;     // chip select pin - useful 

MovingSteppersLib motors[1] {{s1,d1,c1}};     

void setup() {
  Serial.begin(9600);     
  hand_servo.attach(3);
  Serial.print("start");
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
    hand_servo.write(93);
    motors[0].encoder.setZeroSPI(c1);
  } else if (input == '1') {
    Serial.print("Bruh");
    hand_servo.write(113);
  } else if (input == '2') {
    hand_servo.write(73);
  }
}
