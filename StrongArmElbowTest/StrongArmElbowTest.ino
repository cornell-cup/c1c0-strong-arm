#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <Servo.h>

#define MAX_ENCODER_VAL 16383

// Stepper motor (elbow) encoder
int s0 = 8;     // step pin
int d0 = 10;    // direction pin
int c0 = 35;    // chip select pin (had this on 5, changing for now)

volatile int fill_serial_buffer = false;

// Storing pins and states for the elbow stepper motor
MovingSteppersLib motors[1] {{s0,d0,c0}};     
int stepPin[1] = {s0}; 
int directionPin[1] = {d0};  
volatile int move[1];              
volatile int state[1];             

int reversed[1] = {0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal) only for the elbow 

// Storing encoder values
volatile float encoderDiff[1];    // units of encoder steps
volatile float encoderTarget[1];  // units of encoder steps
volatile float targetAngle[1];    // units of degrees
float encoderPos[1];              // units of encoder steps

// Represent if motor is not at desired position
volatile int not_tolerant_elbow;         

void setup() {
  Serial.begin(9600);   // Serial monitor

  // Elbow stepper motor setup
  targetAngle[0] = 0;   // max is roughly 135 if zeroed correctly
  pinMode(directionPin[0], OUTPUT);
  pinMode(stepPin[0], OUTPUT);
  encoderTarget[0] = targetAngle[0] * 45.51111; // * 360/255;
  encoderPos[0] = motors[0].encoder.getPositionSPI(c0);
  encoderDiff[0] = encoderTarget[0] - encoderPos[0];
  move[0] = 1; 

  // ENCODERS
  redefine_encoder_zero_position(false); 

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
  
  // ELBOW STEPPER ISR
  not_tolerant_elbow = abs(encoderDiff[0]) > 10 && ((abs(encoderDiff[0]) + 10) < (MAX_ENCODER_VAL + encoderTarget[0])); // 2nd condition to check if 359 degrees is close enough to 0
  if (move[0]) {                            // if motor should move
    if (not_tolerant_elbow) {               // if not within tolerance
      state[0] = !state[0];                 // toggle state
      digitalWrite(stepPin[0], state[0]);   // write to step pin
    } else {
      move[0] = 0;    // stop moving motor if location reached
    }
  }
}

void loop() {
  checkDirLongWay(0);
  Serial.print("Elbow Encoder Current Position: ");
  Serial.println(encoderPos[0]);
  Serial.print("Elbow Encoder Difference: ");
  Serial.println(encoderDiff[0]);
}

// Checks that motor is moving in right direction and switches if not
void checkDirLongWay(int motorNum) { 
  encoderPos[motorNum] = motors[motorNum].encoder.getPositionSPI(c0);
  if (encoderPos[motorNum] == 65535){
    move[motorNum] = 0;   // stop moving if encoder reads error message
  }
  
  if ((MAX_ENCODER_VAL - 1000) < encoderPos[motorNum]) {  // if motor goes past zero incorrectly, we want to make sure it moves back in the correct direction
    encoderPos[motorNum] = 0;
  }
  
  encoderDiff[motorNum] = encoderTarget[motorNum] - encoderPos[motorNum];

  if (encoderDiff[motorNum] > 0) {
    digitalWrite(directionPin[motorNum], !reversed[motorNum]); 
  } else {
    digitalWrite(directionPin[motorNum], reversed[motorNum]);
  }
}

// Call this function to reset the encoder's zero position to be the current position of the motor
void redefine_encoder_zero_position(bool rezeroStepper) {
  if (rezeroStepper) {
    motors[0].encoder.setZeroSPI(c0); 
    move[0] = 0;
  }
}
