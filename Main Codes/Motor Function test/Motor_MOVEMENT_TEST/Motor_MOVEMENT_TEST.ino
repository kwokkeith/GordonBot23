#include <math.h>// Library for math formulas

#include <PPMReader.h>// Library to read the data received by the receiver from the controller

#define INT_PIN 13 // inturrupt pin used by ESP to receive PPM data
#define PPM_CHANNELS 8// declaring the no of channels reading from the receiver
PPMReader ppm(INT_PIN, PPM_CHANNELS);// declaring the variables into the PPM function

//Declare all the variables used for storing data and pin numbers

float yaxis = 0; //controller Y axis position of right joystick
float xaxis = 0;//controller X asis position of right joystick
float casual_turn = 0;//controller var A position which will be used for gradual turn of the robot while moving forward or backward
float turn = 0;// controller var B position which will be used to turn the robot to either left or right 
float control_speed = 0;// controller Y axis position of left joystick
float safety = 0;// controller switch 
int auto_mode = 0;

/* The enable pins control the speed of the motor using pulse width modulation
 * The Left and right PWM pins controls the direction of the motors
 */

const int M1_EN = 32;//Front Left motor enable pin
const int M1L_PWM = 33;//Front Left motor pwm left pin
const int M1R_PWM =25 ;//Front Left motor pwm right pin
const int M3_EN = 27;//Back Left motor enable pin
const int M3L_PWM = 14;//Back Left motor pwm left pin
const int M3R_PWM =12;//Back Left motor pwm right pin
const int M2_EN = 19;//Front Right motor enable pin
const int M2L_PWM = 18;//Front Right motor pwm left pin
const int M2R_PWM =5;//Front Right motor pwm right pin
const int M4_EN = 4;//Back Right motor enable pin
const int M4L_PWM =2;//Back Right motor pwm left pin
const int M4R_PWM =15;//Back Right motor pwm right pin

const int suction_motor =16;// turn on/off turbo
const int trash_container =17;// extend/retract vacuum


int for_speed = 150;
int bac_speed = 150;
int side_speed = 200;




//Setting PWM properties
const int freq = 40000;//variable used to store the frequency value, easier to change by changing here
const int pwmCh_M1 = 0;//PWM channel 0
const int pwmCh_M2 = 1;//PWM channel 1
const int pwmCh_M3 = 2;//PWM channel 2
const int pwmCh_M4 = 3;//PWM channel 3
const int resolution = 8;//8 bits


void forward() {
  digitalWrite(M2L_PWM,HIGH);
  digitalWrite(M2R_PWM,LOW);
  ledcWrite(pwmCh_M2, for_speed );
  digitalWrite(M3L_PWM,LOW);
  digitalWrite(M3R_PWM,HIGH);
  ledcWrite(pwmCh_M3, for_speed);
  digitalWrite(M1R_PWM,HIGH);
  digitalWrite(M1L_PWM,LOW);
  ledcWrite(pwmCh_M1, for_speed);
  digitalWrite(M4R_PWM,LOW);
  digitalWrite(M4L_PWM,HIGH);
  ledcWrite(pwmCh_M4, for_speed);
}


void backward() {
  digitalWrite(M2R_PWM,HIGH);
  digitalWrite(M2L_PWM,LOW);
  ledcWrite(pwmCh_M2, bac_speed );
  digitalWrite(M3R_PWM,LOW);
  digitalWrite(M3L_PWM,HIGH);
  ledcWrite(pwmCh_M3, bac_speed);
  digitalWrite(M1L_PWM,HIGH);
  digitalWrite(M1R_PWM,LOW);
  ledcWrite(pwmCh_M1, bac_speed);
  digitalWrite(M4L_PWM,LOW);
  digitalWrite(M4R_PWM,HIGH);
  ledcWrite(pwmCh_M4, bac_speed);
}


void move_left() {
  digitalWrite(M2L_PWM,HIGH);
  digitalWrite(M2R_PWM,LOW);
  ledcWrite(pwmCh_M2, side_speed );
  digitalWrite(M3L_PWM,LOW);
  digitalWrite(M3R_PWM,HIGH);
  ledcWrite(pwmCh_M3, side_speed);
  digitalWrite(M1R_PWM,HIGH);
  digitalWrite(M1L_PWM,LOW);
  ledcWrite(pwmCh_M1, side_speed);
  digitalWrite(M4R_PWM,LOW);
  digitalWrite(M4L_PWM,HIGH);
  ledcWrite(pwmCh_M4, side_speed);
}

void move_right() {
  digitalWrite(M2L_PWM,HIGH);
  digitalWrite(M2R_PWM,LOW);
  ledcWrite(pwmCh_M2, side_speed );
  digitalWrite(M3L_PWM,LOW);
  digitalWrite(M3R_PWM,HIGH);
  ledcWrite(pwmCh_M3, side_speed);
  digitalWrite(M1R_PWM,HIGH);
  digitalWrite(M1L_PWM,LOW);
  ledcWrite(pwmCh_M1, side_speed);
  digitalWrite(M4R_PWM,LOW);
  digitalWrite(M4L_PWM,HIGH);
  ledcWrite(pwmCh_M4, side_speed);
}

void all_stop(){


        digitalWrite(M1L_PWM,HIGH);
        digitalWrite(M1R_PWM,HIGH);
        ledcWrite(pwmCh_M1, (0));
        digitalWrite(M2R_PWM,HIGH);
        digitalWrite(M2L_PWM,HIGH);
        ledcWrite(pwmCh_M2, (0));
        digitalWrite(M3R_PWM,HIGH);
        digitalWrite(M3L_PWM,HIGH);
        ledcWrite(pwmCh_M3, (0));
        digitalWrite(M4L_PWM,HIGH);
        digitalWrite(M4R_PWM,HIGH);
        ledcWrite(pwmCh_M4, (0));
        

}


void setup() {
  
  Serial.begin(115200); //setting the buadrate for serial monitor which can be used to read the print statements sent by ESP

  

      
 // configure LED PWM functionalitites
  ledcAttachPin(M1_EN, pwmCh_M1);
  ledcSetup(pwmCh_M1, freq, resolution);

  ledcAttachPin(M2_EN, pwmCh_M2);
  ledcSetup(pwmCh_M2, freq, resolution);

  ledcAttachPin(M3_EN, pwmCh_M3);
  ledcSetup(pwmCh_M3, freq, resolution);

  ledcAttachPin(M4_EN, pwmCh_M4);
  ledcSetup(pwmCh_M4, freq, resolution);

  // configure pin modes
  pinMode(M1L_PWM,OUTPUT);
  pinMode(M2L_PWM,OUTPUT);
  pinMode(M3L_PWM,OUTPUT);
  pinMode(M4L_PWM,OUTPUT);
  pinMode(M1R_PWM,OUTPUT);
  pinMode(M2R_PWM,OUTPUT);
  pinMode(M3R_PWM,OUTPUT);
  pinMode(M4R_PWM,OUTPUT);
  pinMode(suction_motor,OUTPUT);
  pinMode(trash_container,OUTPUT);


  digitalWrite(trash_container,LOW);
  digitalWrite(suction_motor,LOW);

}

void loop() {

    float safety = ((ppm.latestValidChannelValue(5, 1500))/500.0)-3;
    float turbo = ((ppm.latestValidChannelValue(6, 1500))/500.0)-3;

if (safety >0 and auto_mode ==0){
 
   
    //digitalWrite(suction_motor,LOW);
    forward();
    delay(4100);
    all_stop();
     digitalWrite(suction_motor,HIGH);
     delay(500);
     backward();
     delay(200);
     all_stop();
    
    auto_mode =1;    
 
  
}
else{
  all_stop();
}


  
}
