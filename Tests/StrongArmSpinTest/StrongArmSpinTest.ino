#include <MovingSteppersLib.h>
#include <MotorEncoderLib.h>
#include <Servo.h>

#define MAX_ENCODER_VAL 16383

// Create Servo object
Servo spin_servo;
volatile int spin_wait = 0;
volatile int spin_pos = 0;
volatile int spin_target = 0;     

void setup() {
  Serial.begin(9600);   // Serial monitor
  
  // Setup for the servo that controls the hand 
  spin_servo.attach(6);
  spin_target = 180;

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

  spin_wait++;

  // SPIN SERVO
  if (spin_wait < spin_target) {
    if (spin_target > spin_pos) { // turn right
      spin_servo.write(150);
    } elif (spin_target < spin_pos) { // turn left
      spin_servo.write(23);
    }
  } else {
    spin_pos = spin_target;
    spin_servo.write(92);
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
