#define R_EN 5
#define L_EN 5
#define R_PWM 22
#define L_PWM 24

void setup() {
  // put your setup code here, to run once:
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(R_PWM, 1);
  digitalWrite(L_PWM, 0);
  delay(300);
  for(int p=0; p<=255; ++p){
    analogWrite(L_EN,p);
    Serial.println("Motor Duty Cycle: " + String(p*100.0/255.0) + "%");
    delay(10);
  }
  delay(300);

  for(int p=255; p>=0; --p){
    analogWrite(L_EN,p);
    Serial.println("Motor Duty Cycle: " + String(p*100.0/255.0) + "%");
    delay(10);
  }
  digitalWrite(R_PWM, 0);
  digitalWrite(L_PWM, 1);
  delay(300);
  for(int p=0; p<=255; ++p){
    analogWrite(L_EN,p);
    Serial.println("Motor Duty Cycle: " + String(p*100.0/255.0) + "%");
    delay(10);
  }
  delay(300);

  for(int p=255; p>=0; --p){
    analogWrite(L_EN,p);
    Serial.println("Motor Duty Cycle: " + String(p*100.0/255.0) + "%");
    delay(10);
  }
}
