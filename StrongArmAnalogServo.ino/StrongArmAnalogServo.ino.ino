#include <Servo.h>

Servo myservo;  
int pos = 0;    

void setup() {
  // servo library only supports pins 9 and 10 
  // write(n) controls the speed (forward 0 to 89, backward 91 to 180)
  Serial.begin(9600);
  myservo.attach(7);  // attaches the servo on pin 9 to the servo object
  Serial.println(myservo.attached()); 
  myservo.writeMicroseconds(1500);

 // TODO: how to get the arm to move to calculate specific angles to go tos
 // minor issue: repeating the same angles moves the servo more and more in one direction over time
 // may need to add 'high precision resistors in order to rectify imprecise movements
  myservo.write(90);
}

void loop() {
  // for continuous servo, write(n) sets the speed
  myservo.write(150);
  delay(200);
  myservo.write(90);
  delay(2000);
  myservo.write(50);
  delay(200); 
  myservo.write(90);
  delay(2000);
}
