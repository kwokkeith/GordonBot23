#define EN 5
#define R_PWM 2
#define L_PWM 4
#define CONTROLLER_PPM 13
#define PPM_CHANNELS 10

#include <PPMReader.h>

PPMReader ppm(CONTROLLER_PPM, PPM_CHANNELS);

void setup() {
  pinMode(EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  float duty_cycle = (ppm.latestValidChannelValue(2, 0) - 1500.0)/500.0;
  bool dir = (duty_cycle < 0);
  Serial.println(ppm.latestValidChannelValue(2, 0));
  float power = abs(duty_cycle)*255.0;
  if(duty_cycle < 0) duty_cycle *= -1;
  analogWrite(EN, power);
  digitalWrite(R_PWM, dir);
  digitalWrite(L_PWM, !dir);
}
