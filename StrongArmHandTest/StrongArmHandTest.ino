#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <Servo.h>

#define MAX_ENCODER_VAL 16383

// Create Servo object
Servo hand_servo;

// Continuous hand encoder pins - step and direction are arbitrary
int s1 = 30;    // step pin - useless
int d1 = 31;    // direction pin - useless
int c1 = 4;     // chip select pin - useful

volatile int fill_serial_buffer = false;
volatile int cont_wait = 0;

// Storing pins and states for continuous Servo
MovingSteppersLib motors[1] {{s1,d1,c1}};     
int stepPin[1] = {s1}; 
int directionPin[1] = {d1};  
volatile int move[1];              
volatile int state[1];             

// Storing encoder values
volatile float encoderDiff[1];    // units of encoder steps
volatile float encoderTarget[1];  // units of encoder steps
volatile float targetAngle[1];    // units of degrees
float encoderPos[1];              // units of encoder steps

// Represent if motor is not at desired position
volatile int not_tolerant_hand;

void setup() {
  Serial.begin(9600);   // Serial monitor
  
  // Setup for the servo that controls the hand 
  hand_servo.attach(5);
  targetAngle[0] = 1;   // Hand encoder setup: 80 is closed and 1 is open
  encoderTarget[0] = targetAngle[0] * 45.51111;
  encoderPos[0] = motors[0].encoder.getPositionSPI(c1);
  encoderDiff[0] = encoderTarget[0] - encoderPos[0];
  move[0] = 1; 

  // Set argument to true to redefine zero position for hand encoder; false if not
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
  
  // HAND CONTINUOUS ISR
  cont_wait += 1;
  if (cont_wait == 100) {
    not_tolerant_hand = abs(encoderDiff[0]) > 10 && ((abs(encoderDiff[0]) + 10) < (MAX_ENCODER_VAL + encoderTarget[0]));
    if (move[0]) { 
      if (not_tolerant_hand) {    // move continuous servo
        cont_check_dir(0);
      } else {
        move[0] = 0;              // stop moving motor if location reached
        hand_servo.write(92);
      }
    }
    cont_wait = 0;
  }
}

void loop() {
  // checkDirLongWay(0);
  Serial.print("Encoder Current Position: ");
  Serial.println(encoderPos[0]);
  Serial.print("Encoder Difference: ");
  Serial.println(encoderDiff[0]);
}

void cont_check_dir(int contNum) {
  // 92/93 - zero rotation
  encoderPos[contNum] = MAX_ENCODER_VAL - motors[contNum].encoder.getPositionSPI(c1);
  
  if (encoderPos[contNum] == 65535){
    move[contNum] = 0;    // stop moving if encoder reads error message
  }

  if ((MAX_ENCODER_VAL - 1000) < encoderPos[contNum]) {  // if servo goes past zero incorrectly, we want to make sure it moves back in the correct direction
    encoderPos[contNum] = 0;
  }

  encoderDiff[contNum] = encoderTarget[contNum] - encoderPos[contNum];

  if (encoderDiff[contNum] > 0) {
    // Close the hand
    hand_servo.write(22); 
  } else {
    // Open hand
    hand_servo.write(132); 
  }
}

// Call this function to reset the encoder's zero position to be the current position of the motor
void redefine_encoder_zero_position(bool rezeroHand) {
  if (rezeroHand) {
    motors[0].encoder.setZeroSPI(c1);
  }
}
