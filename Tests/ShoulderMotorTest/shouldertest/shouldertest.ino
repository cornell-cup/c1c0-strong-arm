#define driverPUL 8
#define driverDIR 10
#define EB 3
#define EA 2
int stepPerRev = 200;
int pd = 1000 * 1000;
int wait = 2;
volatile bool setdir = LOW;
int encoderCnt = 0;
int ea_on = 0;
int eb_on = 0;

void ea()
{
  ea_on = 1;
  Serial.println('Inside');
  if (eb_on == 0) encoderCnt++;
}

void eaFall()
{
  ea_on = 0;
}

void eb()
{
  eb_on = 1;
  if (ea_on == 0) encoderCnt--;
}

void ebFall()
{
  eb_on = 0;
}

void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(driverPUL, OUTPUT);
  pinMode(driverDIR, OUTPUT);
  pinMode(EA, INPUT);
  pinMode(EB, INPUT);
  Serial.println("setup");
  attachInterrupt(EA,ea,RISING);
  attachInterrupt(EA,eaFall,FALLING);
  // attachInterrupt(EB,eb,RISING);
  // attachInterrupt(EB,ebFall,FALLING);
  digitalWrite(driverPUL, LOW);
  digitalWrite(driverDIR, HIGH);
  encoderCnt = 0;
}

void loop()
{
  digitalWrite(driverDIR, !setdir);
  digitalWrite(driverPUL, HIGH);
  delay(10);
  digitalWrite(driverPUL, LOW);
  delay(100);
}
