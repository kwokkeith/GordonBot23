/*
 * This code controls a four-wheeled robot using pulse width modulation (PWM).
 * The robot's movements are determined by user input from a joystick, which is read
 * in through a PPM signal. The code defines several constants for the pins and control
 * variables, as well as two functions for calculating the speeds of the motors based
 * on the joystick input.
 */


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




//Setting PWM properties
const int freq = 40000;//variable used to store the frequency value, easier to change by changing here
const int pwmCh_M1 = 0;//PWM channel 0
const int pwmCh_M2 = 1;//PWM channel 1
const int pwmCh_M3 = 2;//PWM channel 2
const int pwmCh_M4 = 3;//PWM channel 3
const int resolution = 8;//8 bits


//Declare motorspeeds structure members
struct motorspeeds {
  float a; 
  float b;
};
// motorspeeds structure is used to find the speed of the motors by calculating angle and magnitude (math library is used in this structure. This structure takes in the joystick positions and returns the individual motor speeds.
struct motorspeeds motor_speed(float x_axis,float y_axis){
    struct motorspeeds result; // initialise structure variable
    float angle = atan2(y_axis,x_axis);
     if (angle <0){
      angle = 3.14159 + (3.14159 + angle);
     }
     float magnitude = sqrt(pow(x_axis,2)+pow(y_axis,2));
     if (magnitude>1){
      magnitude = 1;
     }
   
     float spd_a = sin(angle + 0.7854)*magnitude;// 45 degree added to found angle for angle correction received form joystick and multiplied with magnitude to find speed, Front Left motor and Back Right motor use this speed value
     float spd_b = sin(angle - 0.7854)*magnitude;//45 degree substracted to found angle for angle correction received form joystick and multiplied with magnitude to find speedFront Right motor and Back Left motor use this value


     //the speed is multiplied with 1.45 to achieve the max speed in forward and reverse direction. Without the multiplication the max speed forward and reverse is only close to 70%
     spd_a = spd_a*1.45;
     spd_b = spd_b*1.45;

     //if statement used to remove the values exceeding 1 since it will cause error in assigning the speed in the Motor_Speed_set function  
     if (spd_a>1){
      spd_a = 1;
     }
     else if (spd_a<-1){
      spd_a = -1;
     }
     if (spd_b>1){
      spd_b = 1;
     }
     else if (spd_b<-1){
      spd_b = -1;
     }

     
     //the calculated speeds are returned using the structure variables
     result.a = spd_a;
     result.b = spd_b;
     return result;      
}



//function to set the motor speed based on joystick position. This function takes in the joystick x and y position along with the maxspeed set by the operator.
void Motor_Speed_set(float x, float y,int maxspeed){
  
    Serial.println(String(x)+"  "+String(y)+"  ");
    //speed is checked using if statements for positive or reverse to assign the direction of the motors, maxspeed variable is data receive from the controller to set the speed requirements as needed by the operator 
    if (motor_speed(x, y).b>0.02){
          digitalWrite(M2L_PWM,HIGH);
          digitalWrite(M2R_PWM,LOW);
          ledcWrite(pwmCh_M2, abs(motor_speed(x, y).b*maxspeed));
          digitalWrite(M3L_PWM,LOW);
          digitalWrite(M3R_PWM,HIGH);
          ledcWrite(pwmCh_M3, abs(motor_speed(x, y).b*maxspeed));
  
    }
    
    else if (motor_speed(x, y).b<-0.02){
          digitalWrite(M2L_PWM,LOW);
          digitalWrite(M2R_PWM,HIGH);
          ledcWrite(pwmCh_M2, abs(motor_speed(x, y).b*maxspeed));
          digitalWrite(M3L_PWM,HIGH);
          digitalWrite(M3R_PWM,LOW);
          ledcWrite(pwmCh_M3, abs(motor_speed(x, y).b*maxspeed));
    }

    else{
          digitalWrite(M2L_PWM,LOW);
          digitalWrite(M2R_PWM,HIGH);
          ledcWrite(pwmCh_M2, 0);
          digitalWrite(M3L_PWM,HIGH);
          digitalWrite(M3R_PWM,LOW);
          ledcWrite(pwmCh_M3, 0);
    }
  
     if (motor_speed(x, y).a>0.02){
          digitalWrite(M1R_PWM,HIGH);
          digitalWrite(M1L_PWM,LOW);
          ledcWrite(pwmCh_M1, abs(motor_speed(x, y).a*maxspeed));
          digitalWrite(M4R_PWM,LOW);
          digitalWrite(M4L_PWM,HIGH);
          ledcWrite(pwmCh_M4, abs(motor_speed(x, y).a*maxspeed));
  
    }
    
    else if (motor_speed(x, y).a<-0.02){
          digitalWrite(M1R_PWM,LOW);
          digitalWrite(M1L_PWM,HIGH);
          ledcWrite(pwmCh_M1, abs(motor_speed(x, y).a*maxspeed));
          digitalWrite(M4R_PWM,HIGH);
          digitalWrite(M4L_PWM,LOW);
          ledcWrite(pwmCh_M4, abs(motor_speed(x, y).a*maxspeed));
    }

    else{
          digitalWrite(M1R_PWM,LOW);
          digitalWrite(M1L_PWM,HIGH);
          ledcWrite(pwmCh_M1, 0);
          digitalWrite(M4R_PWM,HIGH);
          digitalWrite(M4L_PWM,LOW);
          ledcWrite(pwmCh_M4, 0);
    }
  


}

// Function to turn the robot left 
void turn_left(float dir, int maxspeed){


        digitalWrite(M1L_PWM,HIGH);
        digitalWrite(M1R_PWM,LOW);
        ledcWrite(pwmCh_M1, (dir*maxspeed));
        digitalWrite(M2R_PWM,LOW);
        digitalWrite(M2L_PWM,HIGH);
        ledcWrite(pwmCh_M2, (dir*maxspeed));
        digitalWrite(M3R_PWM,LOW);
        digitalWrite(M3L_PWM,HIGH);
        ledcWrite(pwmCh_M3, (dir*maxspeed));
        digitalWrite(M4L_PWM,HIGH);
        digitalWrite(M4R_PWM,LOW);
        ledcWrite(pwmCh_M4, (dir*maxspeed));
        

    

}

//Function to turn the robot right
void turn_right(float dir, int maxspeed){

   
        digitalWrite(M1R_PWM,HIGH);
        digitalWrite(M1L_PWM,LOW);
        ledcWrite(pwmCh_M1, (dir*maxspeed));
        digitalWrite(M2L_PWM,LOW);
        digitalWrite(M2R_PWM,HIGH);
        ledcWrite(pwmCh_M2, (dir*maxspeed));
        digitalWrite(M3L_PWM,LOW);
        digitalWrite(M3R_PWM,HIGH);
        ledcWrite(pwmCh_M3, (dir*maxspeed));
        digitalWrite(M4R_PWM,HIGH);
        digitalWrite(M4L_PWM,LOW);
        ledcWrite(pwmCh_M4, (dir*maxspeed));
        

}


//Safety function to set all the motors to zero when safety switch is on
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


//Setup function (only runs one time after the ESP boot up)
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

}

//Loop function (runs infinitely after the setup function until ESP is rebooted or powerd off)
void loop() {

    //reads all the channels using the PPM library and assign it to variables
    yaxis = ((ppm.latestValidChannelValue(2, 1500))/500.0)-3;
    xaxis = ((ppm.latestValidChannelValue(4, 1500))/500.0)-3;
    turn = ((ppm.latestValidChannelValue(8, 1500))/500.0)-3;
    
    //Few channels are read twice to avoid errors
   float casual_turn1 = ((ppm.latestValidChannelValue(7, 1500))/500.0)-3;
   float control_speed1 = ((ppm.latestValidChannelValue(3, 0))/1000.0)-1;
   float safety1 = ((ppm.latestValidChannelValue(5, 1500))/500.0)-3;
  
   float casual_turn2 = ((ppm.latestValidChannelValue(7, 1500))/500.0)-3;
   float control_speed2 = ((ppm.latestValidChannelValue(3, 0))/1000.0)-1;
   float safety2 = ((ppm.latestValidChannelValue(5, 1500))/500.0)-3;

    // If statements are used to compare the changes in channels to avoid errors 
   if (casual_turn1 == casual_turn2){
      casual_turn = casual_turn1;
   }
  
   if (control_speed1 == control_speed2){
      control_speed = control_speed1;
   }
   
   if (safety1 == safety2){
      safety = safety1;
   }
   
   int max_speed = (155+control_speed*100);// control speed is the value set by the operator. max speed helps to map the received controller speed (0.0-1.0) to 155 - 255. 
  
   
  
  // The safety button is checked using if statement to assign the speed of the motor by calling the Motor_Speed_set and turn function based on the inputs from the controller.
   if (safety >0){
      Serial.println(String(yaxis)+"  "+String(xaxis)+"  "+String(turn)+"  "+String(safety)+"  "+String(control_speed));
   if (turn>=-0.1 && turn<=0.1){
      Motor_Speed_set(xaxis, yaxis, max_speed);
   }
  
   else if(turn<-0.1 || turn>0.1){
   
      if(turn>0.2){
        turn_left(abs(turn), max_speed );
      }
      else if (turn<-0.2) {
        turn_right(abs(turn), max_speed);
     }
  
   }
    else{
      all_stop;// set all motors to 0 speed by calling the all_stop function
     }
        
  }
   else{
    all_stop;// set all motors to 0 speed by calling the all_stop function
     }
         
}
