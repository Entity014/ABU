#include <PID_v1.h>
#include <analogWrite.h>
#include <SimpleKalmanFilter.h>

int LPWM_Output1 = 12;
int RPWM_Output1 = 14;
uint8_t encoder1 = 25;

int LPWM_Output2 = 26;
int RPWM_Output2 = 27;
uint8_t encoder2 = 33;

int counter1, counter2;
double Setpoint, Input1, Input2, Output1, Output2;
double Infil1, Infil2;

// double Kp = 5e-4, Ki = 0.02, Kd = 1.0;
double Kp = 5e-4, Ki = 0.035, Kd = 1.0;

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.001);

void countpulse1() {
  counter1++;
}

void countpulse2() {
  counter2++;
}

double roundPerMin(int &counter) {
  double round;
  round = (counter / 40) * 600;
  counter = 0;
  return round;
}

void setup() {

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

  Setpoint = 5000;
}

void loop() {
  static uint32_t previousMillis1;

  while (millis() - previousMillis1 >= 100) {
    Input1 = roundPerMin(counter1);
    Input2 = roundPerMin(counter2);
    Infil1 = simpleKalmanFilter.updateEstimate(Input1);
    Infil2 = simpleKalmanFilter.updateEstimate(Input2);
    previousMillis1 += 100;
  }

  PID myPID1(&Infil1, &Output1, &Setpoint, Kp, Ki, Kd, DIRECT);
  PID myPID2(&Infil2, &Output2, &Setpoint, Kp, Ki, Kd, DIRECT);
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID1.Compute();
  myPID2.Compute();
  // Serial.println(myPID.GetKp());

  analogWrite(LPWM_Output1, Output1);
  // analogWrite(LPWM_Output1, 255);
  analogWrite(RPWM_Output1, 0);
  analogWrite(LPWM_Output2, 0);
  analogWrite(RPWM_Output2, Output2);
  Serial.println(String(Setpoint) + " " + String(Infil1) + " " + String(Infil2) + " " + String(Output1) + " " + String(abs(Infil1 - Setpoint)) + " " + String(myPID1.GetKp()) + " " + String(myPID1.GetKi()) + " " + String(myPID1.GetKd()));
}
