#include <PID_v1.h>
#include <analogWrite.h>

int LPWM_Output1 = 14;
int RPWM_Output1 = 12;
uint8_t encoder1 = 25;

int LPWM_Output2 = 26;
int RPWM_Output2 = 27;
uint8_t encoder2 = 33;

volatile unsigned int counter1, counter2;
double Setpoint, Input1, Input2, Output1, Output2;

double Kp=0.15, Ki=0.2, Kd=0.0125;
 
void setup()
{
  Setpoint = 400;

  Serial.begin(9600);
  pinMode(encoder1, INPUT);
  pinMode(encoder2, INPUT);
  pinMode(RPWM_Output1, OUTPUT);
  pinMode(LPWM_Output1, OUTPUT);
  pinMode(RPWM_Output2, OUTPUT);
  pinMode(LPWM_Output2, OUTPUT);

  digitalWrite(encoder1, HIGH);
  digitalWrite(encoder2, HIGH);

  attachInterrupt(encoder1, countpulse1, RISING);
  attachInterrupt(encoder2, countpulse2, RISING);
}

void countpulse1()
{
  counter1++;
}

void countpulse2()
{
  counter2++;
}
 
double preRound = 0;
double roundPerMin(volatile unsigned int &counter)
{
  static uint32_t previousMillis;
  double round;
  if (millis() - previousMillis >= 1000 )
  {
    round = (counter/40) * 60;
    counter = 0;
    previousMillis += 1000;
    if (round <= 900)
    {
      preRound = round;
      return round;
    }
    else return preRound;
  }
}

double inputB = 0;

void loop()
{
  if (Serial.available() > 0)
  {
    inputB = (double(char(Serial.read()) - '0') * 0.1);
    Kd = inputB;
  }

  Input1 = roundPerMin(counter1);
  Input2 = roundPerMin(counter2);

  PID myPID1(&Input1, &Output1, &Setpoint, Kp, Ki, Kd, DIRECT);
  PID myPID2(&Input2, &Output2, &Setpoint, Kp, Ki, Kd, DIRECT);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID1.Compute();
  myPID2.Compute();
  // Serial.println(myPID.GetKp());

  analogWrite(LPWM_Output1, Output1);
  analogWrite(RPWM_Output1, 0);
  analogWrite(LPWM_Output2, 0);
  analogWrite(RPWM_Output2, Output2);
  Serial.println(String(Setpoint)+" "+String(Input1)+" "+String(Output1) + " " + String(abs(Input1 - Setpoint)) + " " + String(myPID1.GetKd()));
  delay(100);
}