#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <Servo.h>

#define MAX_ENCODER_VAL 16383

// Continuous Servo variables
Servo hand_servo;

// Elbow Encoder Pins - step and direction (motor pins) are arbitrary values
int s0 = 30;    // step pin
int d0 = 31;    // direction pin
int c0 = 4;     // chip select pin

volatile int fill_serial_buffer = false;
volatile int servo_wait = 0;

// storing pins and states for the stepper motor
MovingSteppersLib motors[1] {{s0,d0,c0}};     // instantiate motor (StepPin, DirectionPin, EncoderChipSelectPin)
int stepPin[1] = {s0}; 
int directionPin[1] = {d0};  
volatile int move [1];    //volatile because changed in ISR
volatile int state [1];   //volatile because changed in ISR

int reversed[1] = {0}; // motors that have encoders facing the wrong way must pick direction changes slightly differently (opposite of normal)

// storing encoder values
volatile float encoderDiff[1];    // units of encoder steps
volatile float encoderTarget[1];  // units of encoder steps
volatile float targetAngle[1];    // units of degrees
float encoderPos[1];    // units of encoder steps

volatile int nottolerant; // motor not within expected position

uint16_t encoder_angles[] = {0};

void redefine_encoder_zero_position() {
  // call this function to reset the encoder's zero position to be the current position of the motor
  motors[0].encoder.setZeroSPI(c0);
}

Servo other;

void setup() {
  Serial.begin(9600); // Serial monitor
  hand_servo.attach(5);

  other.attach(7);

  redefine_encoder_zero_position(); // uncomment this whenever you want to set zero position
  targetAngle[0] = 1; // max is roughly 135 if zeroed correctly 
  encoderTarget[0] = targetAngle[0] * 45.51111; //map degree to encoder steps
  encoderPos[0] = MAX_ENCODER_VAL - motors[0].encoder.getPositionSPI(14); //get starting encoder position
  encoderDiff[0] = encoderTarget[0] - encoderPos[0]; //calculate difference between target and current
  move[0] = 1;
  
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
  
  nottolerant = abs(encoderDiff[0]) > 10 && ((abs(encoderDiff[0]) + 10) < (MAX_ENCODER_VAL + encoderTarget[0])); // 2nd condition to check if 359 degrees is close enough to 0
  if (move[0]) { //if motor should move
    if (nottolerant){ //if not within tolerance
      hand_servo.write(130);
      //contCheckDir(0);
    }
    else {
      move[0] = 0; //stop moving motor if location reached
      hand_servo.write(92);
    }
  }

  // servo_wait += 1;

  // if (servo_wait == 150) {
  //   positional_servo_ISR(other, 10);
  //   servo_wait = 0;
  // }
}

void update_encoder_angles(){
  encoder_angles[0] =  motors[0].encoder.getPositionSPI(14) / 45.1111;  // how to convert to char and how many digits to round to
}

void loop() {
  update_encoder_angles(); 
  encoderPos[0] = motors[0].encoder.getPositionSPI(14);

  Serial.println(encoderTarget[0]);
  Serial.println(encoderPos[0]);

  //other.write(30); 

  //hand_servo.write(122);
  // delay(3000);   
  // hand_servo.write(92);
  // delay(1000);
  // hand_servo.write(122);
  // delay(3000); 
  // hand_servo.write(92);              
}

void contCheckDir(int contNum) {
  // 92/93 - zero rotation
  encoderPos[contNum] = MAX_ENCODER_VAL - motors[contNum].encoder.getPositionSPI(14);
  
  if (encoderPos[contNum] == 65535){
    move[contNum] = 0; //stop moving if encoder reads error message
  }

  if ((MAX_ENCODER_VAL - 1000) < encoderPos[contNum]) {  // if servo goes past zero incorrectly, we want to make sure it moves back in the correct direction
    encoderPos[contNum] = 0;
  }

  encoderDiff[contNum] = encoderTarget[contNum] - encoderPos[contNum];

  if(encoderDiff[contNum] > 0){
    // close the hand
    hand_servo.write(22); 
    }
  else {
    // open hand
    hand_servo.write(132); 
  }
}

void positional_servo_ISR(Servo servo, int desired_pos) {
    volatile int current_pos = servo.read();
    if (abs(desired_pos - current_pos) < 1) {
      //servo.detach();
    }
    else if (abs(desired_pos - current_pos) >= 1) {
      if ((desired_pos - current_pos) < 0){
        servo.write(current_pos - 1);
      }  
      else if ((desired_pos - current_pos) > 0) {
        servo.write(current_pos + 1);
      }
    }
}