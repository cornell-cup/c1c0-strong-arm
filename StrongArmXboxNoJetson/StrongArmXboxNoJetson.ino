/*
File for how motors will move later on; going to write functions for each type
of motor used on the strong arm, then build into old file and take boolean array data
from locomotion file
*/

#include <Servo.h>
#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

#define MAX_ENCODER_VAL 16383

// Hand continous servo
Servo hand_servo;
int hand_cs = 40; // chip select pin
float hand_angle = 0;

// Wrist spin continous servo
Servo cont_servo;

// Elbow stepper motor
int elbow_pul = 8;  // step pin
int elbow_dir = 10; // direction pin
int elbow_cs = 35;  // chip select pin
float elbow_angle = 0;

// Shoulder stepper motor
int shoulder_pul = 11; // step pin 
int shoulder_dir = 12; // direction pin
int shoulder_cs = ;    // chip select pin
float shoulder_angle = 0;

// Storing pins and states for the hand servo (index 0), elbow motor (index 1), and shoulder motor (index 2)
MovingSteppersLib motors[3] {{30,31,hand_cs}, {elbow_pul, elbow_dir, elbow_cs}, {shoulder_pul, shoulder_dir, shoudler_cs}};
int stepPin[3] = {30, elbow_pul, shoulder_pul};
int directionPin[3] = {31, elbow_dir, shoulder_dir};
volatile int move [3];             
volatile int state [3];

// Storing encoder values
float encoderPos[3];              // units of encoder steps

// Movement and Direction Array (1 if move, 0 if don't move):
// Position 0: Hand, 
//          1: 
//          2:
//          3:
//          4:
//          5:
//          6:
//          7: Shoulder, Dir
//          8: Shoulder, CCW movement
move[8] = [0,0,0,0,0,0,0,0];
dir[8] = [0,0,0,0,0,0,0,0];

void setup() {  
  // Setup for wrist spin servo
  spin_servo.attach(6);
  spin_servo.write(92);

  // Setup for servo that controls the hand
  hand_servo.attach(5);
  encoderPos[0] = motors[0].encoder.getPositionSPI(14);
  hand_angle = encoderPos[0]/45.51111;

  // Elbow motor setup
  pinMode(directionPin[1], OUTPUT);
  pinMode(stepPin[1], OUTPUT);
  encoderPos[1] = motors[1].encoder.getPositionSPI(14);
  elbow_angle = encoderPos[1]/45.51111;

  // Shoulder motor setup
  pinMode(directionPin[2], OUTPUT);
  pinMode(stepPin[2], OUTPUT);
  encoderPos[2] = motors[2].encoder.getPositionSPI(14);
  shoulder_angle = encoderPos[2]/45.51111;

  // ENCODERS - first for hand, second for elbow, third for shoulder  (true = zero)
  redefine_encoder_zero_position(false, false, false); 

  // ISR TIMER
  // Initialize interrupt timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 65518;            // preload timer 65536-(16MHz/256/4Hz)

  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER1_OVF_vect) {        // ISR to pulse pins of moving motors
  TCNT1 = 65518;              // preload timer to 300 us          
  fill_serial_buffer = true;  // check

  // HAND CONTINUOUS ISR
  hand_angle = encoderPos[0]/45.51111;

  // WRIST SPIN ISR

  // ELBOW  ISR
  elbow_angle = encoderPos[1]/45.51111;
  if move[1]

  // SHOULDER ISR
  shoulder_angle = encoderPos[2]/45.51111;
  if move[2]

}
void loop() {

}

void elbow_movement(float elbow_encoder_val, bool elbow_move, bool elbow_dir) {
  // Direction is 1 to move inward (towards 135), 0 to move outward (towards 0)

}

// Probably do not need encoder for the wrist spin, although wires wrapping around the upper
// part of the arm is still a concern. Come back to this later.
void wrist_spin_movement(bool spin_move, bool spin_dir) {
  // Direction is 1 to move CW, 0 to move CCW
  if (spin_move) {
    if (spin_dir) { 
      spin_servo.write(140); // Spin CW
    } else { 
      spin_servo.write(22); // Spin CCW
    }
  } else {
    spin_servo.write(92); // STOP
  }
}

// angle limits - 0 to 80
void hand_movement(float hand_encoder_val, bool hand_move, bool hand_dir) {
  if (hand_move) {
    if (hand_dir == 1) and (hand_encoder_val < 85) {
      hand_servo.write(22); // close hand 
    } else if (hand_dir == 0) and (hand_encoder_val > 0) {
      hand_servo.write(132); // open hand
    } else {
      hand_servo.write(92); // stop if encoder val not satis
    }
  } else {
    hand_servo.write(92); // stop
  }
}

// 
void shoulder_movement(bool shoulder_move, bool shoulder_dir) {
  
}

// Call this function to reset the encoder's zero position to be the current position of the motor
void redefine_encoder_zero_position(bool rezeroHand, bool rezeroElbow, bool rezeroShoulder) {
  if (rezeroHand) {
    motors[0].encoder.setZeroSPI(c0); 
  }
  if (rezeroElbow) {
    motors[1].encoder.setZeroSPI(c1);
  }
  if (rezeroShoulder) {
    motors[2].encoder.setZeroSPI(c1);
  }
}