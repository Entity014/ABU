#include <analogWrite.h>
int LPWM_Output1 = 14;
uint8_t encoder1 = 25;

// int LPWM_Output2 = 27;
uint8_t encoder2 = 34;

int counter1 = 0, counter2 = 0, rpm1 = 0, rpm2 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(encoder1, INPUT);
  pinMode(encoder2, INPUT);
  pinMode(LPWM_Output1, OUTPUT);
  // pinMode(LPWM_Output2, OUTPUT);

  digitalWrite(encoder1, HIGH);
  digitalWrite(encoder2, HIGH);

  attachInterrupt(encoder1, countpulse1, RISING);
  attachInterrupt(encoder2, countpulse2, RISING);
}

void countpulse1() {
  counter1++;
}

void countpulse2() {
  counter2++;
}

double preRound = 0;
double roundPerMin(int &counter) {
  double round;
  round = (counter / 40) * 60;
  counter = 0;
  if (round <= 10000) {
    preRound = round;
    return round;
  } else return preRound;
}

int PWMM = 0;
void loop() {
  static uint32_t previousMillis1;
  static uint32_t previousMillis2;
  while (millis() - previousMillis2 >= 5000)
  {
    if (PWMM < 250) PWMM += 25;
    else PWMM = 0;
    previousMillis2 += 5000;
  }
  while (millis() - previousMillis1 >= 100) {
    rpm1 = roundPerMin(counter1);
    rpm2 = roundPerMin(counter2);
    analogWrite(LPWM_Output1, PWMM);
    // analogWrite(LPWM_Output2, PWMM);
    Serial.print(rpm1);
    Serial.print(" ");
    Serial.print(rpm2);
    Serial.print(" ");
    Serial.println(PWMM);
    // Serial.print(" ");
    previousMillis1 += 100;
  }
}
