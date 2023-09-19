#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>

// This file is used for testing purposes
// You manually set the target angles in the setup() instead of reading values from object detection

float mult = 6;

#define MAX_ENCODER_VAL 16383


// step (pulse) pins 
int s0 = 8;

// direction pins
int d0 = 10;

//chip select pins
int c0 = 9;

int i = 0; 
volatile int counter = 0;
volatile int fill_serial_buffer = false;

volatile int revolutions = 0;
volatile int remainTarget;

//Storing pins and states for each motor
MovingSteppersLib motors[1] {{s0,d0,c0}};  //Instantiate Motors (StepPin, DirectionPin, EncoderChipSelectPin)
int stepPin[1] = {s0}; 
int directionPin[1] = {d0};  
volatile int move [1]; //volatile because changed in ISR
volatile int state [1]; //volatile because changed in ISR

int reversed[1] = {0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal)

//Storing encoder values
volatile float encoderDiff[1];  // units of encoder steps
volatile float encoderTarget[1];  // units of encoder steps
volatile float targetAngle[1];   // units of degrees
float encoderPos[1];   // units of encoder steps

volatile int nottolerant; // motor not within expected position

void reset_input_buffer() {
  while (Serial1.available() > 0 ) Serial1.read();
  delay(100);
}

void redefine_encoder_zero_position(){
  // call this function to reset the encoder's zero postion to be the current position of the motor
  motors[0].encoder.setZeroSPI(c0); 
}

void setup()
{
  Serial.begin(115200); //Baud Rate

  // stepper motor setup
  reset_input_buffer();
  //redefine_encoder_zero_position(); // uncomment this whenever you want to set zero position
  //targetAngle[0] = -1;
  targetAngle[0] = 0;
    
  pinMode(directionPin[0], OUTPUT); //set direction and step pins as outputs
  pinMode(stepPin[0], OUTPUT);
  move[0] = 1; //enable j1 // send move to the jetson and recieve the encoder directions from the jetson
  encoderTarget[0] = targetAngle[0] * 45.51111; //map degree to encoder steps
  //encoderTarget[0] = encoderTarget[0] * mult; //multiplier for encoder in the wrong place
  encoderPos[0] = motors[0].encoder.getPositionSPI(14); //get starting encoder position
  encoderDiff[0] = encoderTarget[0] - encoderPos[0]; //calculate difference between target and current
  
  
  // initialize interrupt timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 65518;            // preload timer 65536-(16MHz/256/4Hz)
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER1_OVF_vect) //ISR to pulse pins of moving motors
{
  TCNT1 = 65518;   // preload timer to 300 us          
  fill_serial_buffer = true; //check
  
  nottolerant = abs(encoderDiff[0]) > 10 && ((abs(encoderDiff[0]) + 10) < (MAX_ENCODER_VAL + encoderTarget[0])); // 2nd condition to check if 359degrees is close enough to 0
  //nottolerant = abs(encoderDiff[i]) > 10; // we dont need the extra condition above bc we never pass through zero
  
  // // account for revolutions
  // if (revolutions >= 1) {
  //   encoderTarget[0] = MAX_ENCODER_VAL - 1000;
  //   revolutions -= 1;
  // }
  // else {
  //   encoderTarget[0] = remainTarget;
  // }

  if (move[0]) { //if motor should move
    if (nottolerant){ //if not within tolerance
      state[0] = !state[0]; //toggle state
      digitalWrite(stepPin[0], state[0]); //write to step pin
    }
    else {
      move[0] = 0; //stop moving motor if location reached
    }
  }
}


void loop() {
  Serial.println(motors[0].encoder.getPositionSPI(14));
  Serial.println(encoderTarget[0]);
  Serial.println(move[0]);

  //targetLimit();
  checkDirLongWay(0);

}

void checkDirLongWay(int motorNum){ //checks that motor is moving in right direction and switches if not

  encoderPos[motorNum] = motors[motorNum].encoder.getPositionSPI(14);
  if (encoderPos[motorNum] == 65535){
    move[motorNum] = 0; //stop moving if encoder reads error message
  }
  
  if ((MAX_ENCODER_VAL - 1000) < encoderPos[motorNum]) {  // if motor goes past zero incorrectly, we want to make sure it moves back in the correct direction
    encoderPos[motorNum] = 0;
  }
  
  encoderDiff[motorNum] = encoderTarget[motorNum] - encoderPos[motorNum];

  if(encoderDiff[motorNum] > 0){
      digitalWrite(directionPin[motorNum], !reversed[motorNum]); 
  }
  else {
      digitalWrite(directionPin[motorNum], reversed[motorNum]);
  }
}
