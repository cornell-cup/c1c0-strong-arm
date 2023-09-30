#define driverPUL 11
#define driverDIR 9

void setup()
{
  Serial.begin(115200);
  Serial.println("setup");
  pinMode(driverPUL, OUTPUT);
  pinMode(driverDIR, OUTPUT);
  digitalWrite(driverPUL, LOW);
  digitalWrite(driverDIR, LOW);
}

void loop()
{
  digitalWrite(driverDIR, LOW);
  digitalWrite(driverPUL, HIGH);
  delay(1);
  digitalWrite(driverPUL, LOW);
  delay(1);
}
