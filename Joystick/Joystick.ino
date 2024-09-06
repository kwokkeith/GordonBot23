#include <math.h> 
#include <PPMReader.h>

// Initialize a PPMReader on digital pin 3 with 6 expected channels.
byte interruptPin = 3;
byte channelAmount = 10;
PPMReader ppm(interruptPin, channelAmount);


const int enA = 6;//fL
const int in1 = 7;//fL
const int enB = 5;//fR
const int in2 = 4;//fR
const int enC = 10;//bL
const int in3 = 12;//bL
const int enD = 9;//bR
const int in4 = 8;//bR

const int enG = 11;//grip
const int in5 = 13;//grip

const int in6 = 2;//ball releaser




struct motorspeeds {
  double a; 
  double b;
};


bool store_load = 0;
bool store_state = 0;

void setup() {
  Serial.begin(115200);
  pinMode(enA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(enB,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(enC,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(enD,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(enG, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);



  digitalWrite(in1,HIGH);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,HIGH);
  digitalWrite(in5, LOW);


  //griper
  digitalWrite(in5,HIGH);
  analogWrite(enG, 100);
  delay(800);
  analogWrite(enG, 10);


  //motor
  analogWrite(enA, 255);
  delay(2000);
  analogWrite(enA, 0);
  analogWrite(enB, 255);
  delay(2000);
  analogWrite(enB, 0);
  analogWrite(enC, 255);
  delay(2000);
  analogWrite(enC, 0);
  analogWrite(enD, 255);
  delay(2000);
  analogWrite(enD, 0);


  //ball releaser
  digitalWrite(in6,HIGH);
  delay(100);
  digitalWrite(in6, LOW);
  

}

void loop() {
  
 double yaxis = (ppm.latestValidChannelValue(2, 0) - 1500.0)/500.0;
 double xaxis = (ppm.latestValidChannelValue(4, 0) - 1500.0)/500.0;
 double turn = (ppm.latestValidChannelValue(5, 0) - 1500.0)/500.0;
 bool grip_load = (ppm.latestValidChannelValue(6, 0))>1500;
 bool grip_state = (ppm.latestValidChannelValue(7, 0)) > 1500.0;
 bool ball_release = (ppm.latestValidChannelValue(8, 0))>1500;
 
  

 Serial.println("state ="+ String(grip_state));
 Serial.println("load ="+ String(grip_load));

if(ball_release){
  digitalWrite(in6, HIGH);
  delay(1500);
  digitalWrite(in6, LOW);
}


if ((store_state != grip_state) or (store_load != grip_load)){
      grip_load = (ppm.latestValidChannelValue(6, 0))>1500;
      grip_state = (ppm.latestValidChannelValue(7, 0)) > 1500.0;
      if ((store_state != grip_state) or (store_load != grip_load)){
        Serial.println("gripper!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        store_state = grip_state;
        store_load = grip_load;
        gripper(store_state, store_load);
      }
     
    }
 
 motor_set(turn, xaxis, yaxis);
 
 
}


struct motorspeeds motor_speed(double x_axis,double y_axis){
    struct motorspeeds result;
    double angle = atan2(y_axis,x_axis);
     if (angle <0){
      angle = 3.14159 + (3.14159 + angle);
     }
     double magnitude = sqrt(pow(x_axis,2)+pow(y_axis,2));
     if (magnitude>1){
      magnitude = 1;
     }
     //Serial.print(xaxis);
     //Serial.print(" ");
     //Serial.print(yaxis);
     //Serial.print(" ");
     //Serial.print(magnitude);
     //Serial.print(", ");
     //Serial.println(angle);
     double spd_a = sin(angle + 0.7854)*magnitude;//fl,br
     double spd_b = sin(angle - 0.7854)*magnitude;//fr,bl
     //Serial.println(spd_a);
     //Serial.println(spd_b);
     result.a = spd_a;
     result.b = spd_b;
     return result;      
}

void motor_set(double dir,double x,double y){

    
  
    if((0.08>dir) and (-0.08<dir)){
      //Serial.println("run" + String(dir));
      
      if (motor_speed(x, y).a>0){
        //Serial.println("motor speed = " + String(abs((motor_speed(x, y).a*255))+10));
        digitalWrite(in1,HIGH);
        analogWrite(enA, abs(motor_speed(x, y).a*255));
        digitalWrite(in4,HIGH);
        analogWrite(enD, abs(motor_speed(x, y).a*255));
      }
      else{ 
        //Serial.println("motor speed = " + String(abs((motor_speed(x, y).a*255))+10));
        digitalWrite(in1,LOW);
        analogWrite(enA, abs(motor_speed(x, y).a*255));
        digitalWrite(in4,LOW);
        analogWrite(enD, abs(motor_speed(x, y).a*255));
      }

      if (motor_speed(x, y).b>0){
        //Serial.println("motor speed = " + String(abs((motor_speed(x, y).b*255))+10));
        digitalWrite(in2,HIGH);
        analogWrite(enB, abs(motor_speed(x, y).b*255));
        digitalWrite(in3,HIGH);
        analogWrite(enC, abs(motor_speed(x, y).b*255));
      }
      
      else{
        //Serial.println("motor speed = " + String(abs((motor_speed(x, y).b*255))+10));
        digitalWrite(in2,LOW);
        analogWrite(enB, abs(motor_speed(x, y).b*255));
        digitalWrite(in3,LOW);
        analogWrite(enC, abs(motor_speed(x, y).b*255));
      }
      
       
    }

    else{

      //Serial.println("turn" + String(dir*255));
      
      if (dir<0){
        digitalWrite(in1,LOW);
        analogWrite(enA, abs(dir*255));
        digitalWrite(in2,HIGH);
        analogWrite(enB, abs(dir*255));
        digitalWrite(in3,LOW);
        analogWrite(enC, abs(dir*255));
        digitalWrite(in4,HIGH);
        analogWrite(enD, abs(dir*255));
      }
      else{
        digitalWrite(in1,HIGH);
        analogWrite(enA, abs(dir*255));
        digitalWrite(in2,LOW);
        analogWrite(enB, abs(dir*255));
        digitalWrite(in3,HIGH);
        analogWrite(enC, abs(dir*255));
        digitalWrite(in4,LOW);
        analogWrite(enD, abs(dir*255));
      }
    }


    


  
}



void gripper(double state, double load){


 if (state == 0) {
  digitalWrite(in5,HIGH);
  analogWrite(enG, 100);
  delay(900);
  analogWrite(enG, 10);
 }

 /*if ((state == 0) and (load == 1)){
  digitalWrite(in5, LOW);
  analogWrite(enG, 50);
  delay(900);
  analogWrite(enG, 10);
  delay(200);
 

  digitalWrite(in5,HIGH);
  analogWrite(enG, 100);
  delay(900);
  analogWrite(enG, 10);
 }*/

 else if ((state == 1) and (load == 1)) {
   digitalWrite(in5,HIGH);
   analogWrite(enG, 100);
   delay(900);
   analogWrite(enG, 10);
   delay(50);
  }

 else if ((state == 1) and (load == 0)){
  digitalWrite(in5, LOW);
  analogWrite(enG, 50);
  delay(800);
  analogWrite(enG, 10);
 }

 


  
}
