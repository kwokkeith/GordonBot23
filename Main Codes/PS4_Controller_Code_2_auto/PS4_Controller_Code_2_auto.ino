#include <PS4Controller.h>// Library for the PS4 controller

//declaring all the variables used   

bool LR_button_state[4] = {0}; //L1, R1, L3, R3

/* The enable pins control the speed of the motor using pulse width modulation
 * The Left and right PWM pins controls the direction of the motors
 */
const int c1_en = 12;//storage box motor enable pin
const int c1_pwmL = 27;//storage box pwm left pin
const int c1_pwmR = 14 ;//storage box pwm right pin
const int c2_en = 33;//vacuum motor enable pin
const int c2_pwmL = 25;//vacuum motor pwm left pin
const int c2_pwmR = 26;//vacuum motor pwm right pin
const int c3_en = 19;//telescope motor enable pin
const int c3_pwmL = 18;//telescope motor pwm left pin
const int c3_pwmR = 5;//telescope motor pwm right pin
const int max_speed_container = 180;//(0-255) for trash container
const int max_speed_vacuum = 255;//(0-255) for trash container

const int suction_motor =17;// turn on/off turbo
const int trash_container =16;// extend/retract vacuum

//Setting PWM properties
const int freq_container = 40000;//variable used to store the frequency value, easier to change by changing here
const int freq_vacuum = 10000;
const int pwmCh_c1 = 0;//PWM channel 0
const int pwmCh_c2 = 1;//PWM channel 1
const int pwmCh_c3 = 2;//PWM channel 2
const int resolution = 8;//8 bits

//Function to throw all the balls into the scoring bin. The speed value is based on the right joystick y position. The max Speed is pre set using a variable when we first declared.
void throw_trash(float speed_value){
    digitalWrite(c1_pwmL,HIGH);
    digitalWrite(c1_pwmR,LOW);
    ledcWrite(pwmCh_c1, max_speed_container*speed_value);
}

//Function to bring up the trash collector into collecting position. The speed value is based on the right joystick y position. The max Speed is pre set using a variable when we first declared.
void collect_trash(float speed_value){

    digitalWrite(c1_pwmL,LOW);
    digitalWrite(c1_pwmR,HIGH);
    ledcWrite(pwmCh_c1, max_speed_container*speed_value);
}

//Function which controls the blow speed of the vacuum motor. this fuction can be used when a trash is stuck in the vacuum chamber. The speed value obtained by sebtracting left and right trigger button in ps4 controller.
void blower(float speed_value){

    digitalWrite(c2_pwmL,LOW);
    digitalWrite(c2_pwmR,HIGH);
    ledcWrite(pwmCh_c2,speed_value);
}

//Function controls the suction power of the vacuum motor. The speed value obtained by sebtracting left and right trigger button in ps4 controller.
void sucker(float speed_value){

    digitalWrite(c2_pwmL,HIGH);
    digitalWrite(c2_pwmR,LOW);
    ledcWrite(pwmCh_c2, speed_value);
}

//Function which sets the vacuum motor speed to 0 when no input from the operator
void vacuum_motor_stop(){
    digitalWrite(c2_pwmL,LOW);
    digitalWrite(c2_pwmR,HIGH);
    ledcWrite(pwmCh_c2, 0);
}

void tele_extend(float speed_value){

    digitalWrite(c3_pwmL,HIGH);
    digitalWrite(c3_pwmR,LOW);
    ledcWrite(pwmCh_c3, max_speed_container*speed_value);
}

//Function to bring up the trash collector into collecting position. The speed value is based on the right joystick y position. The max Speed is pre set using a variable when we first declared.
void tele_retract(float speed_value){

    digitalWrite(c3_pwmL,LOW);
    digitalWrite(c3_pwmR,HIGH);
    ledcWrite(pwmCh_c3, max_speed_container*speed_value);
}

void auto_throw(){
  
  
}

//Setup function (only runs one time after the ESP boot up)
void setup() {
  
  Serial.begin(115200);//setting the buadrate for serial monitor which can be used to read the print statements sent by ESP
  Serial.println("Connecting to PS4.");// print statement to notify that the ESP is up and running
  PS4.begin("5c:96:56:b4:9e:2a");// setting the bluetooth address of the ESP32 which will be stored inside the ps4 controller as master using SixAxisPair tool software. PS4 will try to connect to this adress when the home button is presses. 
  Serial.println("Ready.");//after connecting resdy is printed in the serial monitor 
  
  // configure LED PWM functionalitites
  ledcAttachPin(c1_en, pwmCh_c1);
  ledcSetup(pwmCh_c1, freq_container, resolution);

  ledcAttachPin(c2_en, pwmCh_c2);
  ledcSetup(pwmCh_c2, freq_vacuum, resolution);

  ledcAttachPin(c3_en, pwmCh_c3);
  ledcSetup(pwmCh_c3, freq_container, resolution);
  
  // configure pin modes
  pinMode(c1_pwmL,OUTPUT);
  pinMode(c1_pwmR,OUTPUT);
  pinMode(c2_pwmL,OUTPUT);
  pinMode(c2_pwmR,OUTPUT);
  pinMode(c3_pwmL,OUTPUT);
  pinMode(c3_pwmR,OUTPUT);
  pinMode(suction_motor,INPUT);
  pinMode(trash_container,INPUT);
  

}

void loop() {
   //the if statemnent makes sure the motors only runs when the PS4 is connected.
  

        if (digitalRead(suction_motor) == HIGH){
          blower(255);
        }    

         else{
          blower(0);
         }
      

      
    
  }
  




































































































/*//Calculate joystick position
    float left_stick_mag = pow( pow(PS4.LStickX()/128.0, 2) + pow(PS4.LStickY()/128.0, 2), 0.5);
    float right_stick_mag = pow( pow(PS4.RStickX()/128.0, 2) + pow(PS4.RStickY()/128.0, 2), 0.5);

    if (left_stick_mag>1) {
      left_stick_mag = 1;
      }
    if (right_stick_mag>1) {
      right_stick_mag = 1;
    }

    //Changing button states
    if (PS4.L1()) {
      LR_button_state[0] = !LR_button_state[0];
      Serial.println("L1 Button pressed.\n");
    }
    if (PS4.R1()) {
      LR_button_state[1] = !LR_button_state[1];
      Serial.println("R1 Button pressed.\n");
    }
    if (PS4.L3()) {
      LR_button_state[2] = !LR_button_state[2];
      Serial.println("L3 Button pressed.\n");
    }
    if (PS4.R3()) {
      LR_button_state[3] = !LR_button_state[3];
      Serial.println("R3 Button pressed.\n");
    }

    //Button Press
    if (PS4.Right()) {
      Serial.println("Right Button");
    }
    if (PS4.Down()) {
      Serial.println("Down Button");
    }
    if (PS4.Up()) {
      Serial.println("Up Button");
    }
    if (PS4.Left()) {
      Serial.println("Left Button");
    }
    if (PS4.Square()) {
      Serial.println("Square Button");
    }
    if (PS4.Cross()) {
      Serial.println("Cross Button");
    }
    if (PS4.Circle()) {
      Serial.println("Circle Button");
    }
    if (PS4.Triangle()) {
      Serial.println("Triangle Button");
    }

    //Button Hold
    if (PS4.L2()) {
      Serial.printf("Holding L2 button at %d\n", PS4.L2Value());
    }
    if (PS4.R2()) {
      Serial.printf("Holding R2 button at %d\n", PS4.R2Value());
    }

    //Joystick output
    if (left_stick_mag > joystick_deadzone) {
      Serial.printf("Left Stick Magnitude: %f\n", left_stick_mag);
    }
    if (right_stick_mag > joystick_deadzone) {
      Serial.printf("Right Stick Magnitude: %f\n", right_stick_mag);
    }*/


     /*else if (PS4.RStickY()>0.1){
        digitalWrite(c1_en1,LOW);
        digitalWrite(c1_en2,HIGH);
        digitalWrite(c2_en1,LOW);
        digitalWrite(c2_en2,HIGH);
        
        ledcWrite(pwmCh_c1, abs(PS4.RStickY())/128.0*255);
        ledcWrite(pwmCh_c2, abs(PS4.RStickY())/128.0*255);
        Serial.printf("Right Stick Down: %f\n",(PS4.RStickY()/128.0) );
    }

   if (PS4.L2()<0.1){
        digitalWrite(pu_en1,HIGH);
        digitalWrite(pu_en2,LOW);
        
        ledcWrite(pwmCh_c3, abs(PS4.L2())/128.0*255);
        Serial.printf("L2 pressed: %f\n",(PS4.L2()/128.0) );
    }
    
    else if (PS4.L2()>0.1){
        digitalWrite(pu_en1,HIGH);
        digitalWrite(pu_en2,LOW);
        
        ledcWrite(pwmCh_c3, 0);
        Serial.printf("L2 pressed: %f\n",(PS4.L2()/128.0) );
    }*/
