#include <WiFi.h>
#include <AsyncUDP.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <PPMReader.h>

#define INT_PIN 13
#define PPM_CHANNELS 8

#define FL_EN 33
#define FL_PWM 26
#define FR_EN 32
#define FR_PWM 14
#define BL_EN 25
#define BL_PWM 27
#define BR_EN 18
#define BR_PWM 12

// Setting PWM properties
const int freq = 10000;
const int PWM_FL = 0;
const int PWM_FR = 1;
const int PWM_BL = 2;
const int PWM_BR = 3;
const int resolution = 8;

//Calibration Settings
const float initial_deadzone = 0.02;
const int initial_min = 140;
float trim_value = 0;
float turn_const = 0.3;
float deadzone = initial_deadzone;
int min_to_start = initial_min;
String current_mode;
float trim_right = 1;
float trim_left = 1;

PPMReader ppm(INT_PIN, PPM_CHANNELS);

const char * ssid = "raisins";
const char * password = "illuminatingdarkness";

AsyncUDP udp;

void move_robot(float power, float Dir_power_FL, float Dir_power_FR, float Dir_power_BL, float Dir_power_BR);
void rotate_robot(float power_CCW);

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }
  if (udp.connect(IPAddress(192, 168, 0, 69), 9999)) {
    Serial.println("UDP connected");
    udp.onPacket([](AsyncUDPPacket packet) {
      Serial.print("UDP Packet Type: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
      Serial.print(", From: ");
      Serial.print(packet.remoteIP());
      Serial.print(":");
      Serial.print(packet.remotePort());
      Serial.print(", To: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Length: ");
      Serial.print(packet.length());
      Serial.print(", Data: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();
      //reply to the client
      packet.printf("Got %u bytes of data", packet.length());
    });
  }

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  // configure LED PWM functionalitites
  ledcAttachPin(FL_EN, PWM_FL);
  ledcSetup(PWM_FL, freq, resolution);

  ledcAttachPin(FR_EN, PWM_FR);
  ledcSetup(PWM_FR, freq, resolution);

  ledcAttachPin(BL_EN, PWM_BL);
  ledcSetup(PWM_BL, freq, resolution);

  ledcAttachPin(BR_EN, PWM_BR);
  ledcSetup(PWM_BR, freq, resolution);

  pinMode(FL_PWM, OUTPUT);
  pinMode(FR_PWM, OUTPUT);
  pinMode(BL_PWM, OUTPUT);
  pinMode(BR_PWM, OUTPUT);
}

void loop()
{
  ArduinoOTA.handle();
  //Send broadcast on port 1234

  //Runs data very 10 millisecs
  if (millis() % 10 == 0 ) {
    //Getting values from all channels
    int channel_values[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 1; i <= 8; ++i) {
      channel_values[i - 1] = ppm.latestValidChannelValue(i, 1500);
    }


    //Moving Robot Motor Power Calculation
    float angle = atan2((channel_values[1] - 1500.0), (channel_values[3] - 1500.0)) - PI / 2;
    float convert_to_deg = 180 * angle / PI;
    float FR = pow(2, 0.5) * cos(angle - PI / 4);
    float FL = pow(2, 0.5) * -sin(angle - PI / 4);
    float power = pow( pow((channel_values[1] - 1500.0) / 500.0, 2) + pow((channel_values[3] - 1500.0) / 500.0 , 2), 0.5);
    if (FR > 1) {
      FR = 1;
    }
    else if (FR < -1) {
      FR = -1;
    }
    if (FL > 1) {
      FL = 1;
    }
    else if (FL < -1) {
      FL = -1;
    }
    if (power>1) {
      power = 1;
      }
      
    //Rotating Robot Motor Power Calculation
    float rotationpower_CW, rotationpower_CCW;
    rotationpower_CCW = (channel_values[7] - 1500) / 500;

    //Moving Robot
    if (power > deadzone || abs(rotationpower_CCW) > deadzone) {
      move_robot(power, FL, FR, -FR, -FL, rotationpower_CCW);
    }
    else {
      ledcWrite(PWM_FL, 0);
      ledcWrite(PWM_FR, 0);
      ledcWrite(PWM_BL, 0);
      ledcWrite(PWM_BR, 0);
    }

    //Changing Channel values to states 0->Low, 1->Center, 2->High
    for (int i = 4; i <= 6; ++i) {
      if (channel_values[i] > 1800) {
        channel_values[i] = 2;
      }
      else if (channel_values[i] < 1200) {
        channel_values[i] = 0;
      }
      else {
        channel_values[i] = 1;
      }
    }

    //Trim Value Adjustment
    if (channel_values[6] == 2) {
      trim_value += 0.00001;
    }
    else if (channel_values[6] == 0) {
      trim_value -= 0.00001;
    }

    //Modes Switching
    if (channel_values[5] == 1) {
      current_mode = "PUSHING MODE.";
      min_to_start = 255;
      deadzone = 0.2;
    }
    else if (channel_values[5] == 2) {
      current_mode = "TURBO MODE.";
      min_to_start = initial_min;
      deadzone = initial_deadzone;
    }
    else {
      current_mode = "NORMAL MODE.";
      min_to_start = initial_min;
      deadzone = initial_deadzone;
    }

    //Printing Data
    String msg;
    msg = "\n\nCurrent Direction is " + String(convert_to_deg) + ".\n" + "Current Mode: " + current_mode + "\t" + "Switch 1 State: " + String(channel_values[4]) + "\t" + "Trim: " + String(trim_value) + "\t" + "Rotation Power: " + String(rotationpower_CCW);
    udp.broadcastTo(msg.c_str(), 9999);
  }
}

void move_robot(float power, float Dir_power_FL, float Dir_power_FR, float Dir_power_BL, float Dir_power_BR, float power_CCW) {
  float adj_power;
  String motorpower_msg;

  //Calculating for Trim Values
  if (trim_value > 0) {
    trim_left = 1;
    trim_right = 1 - trim_value;
  }
  else {
    trim_right = 1;
    trim_left = 1 + trim_value;
  }
  if (power > deadzone) {
    //Setting Motor Power of Forward Left Motor
    if (Dir_power_FL > 0) {
      adj_power = min_to_start + power * Dir_power_FL * (255 - min_to_start) - power_CCW * (255 - min_to_start) * turn_const;
      if (adj_power > 255) {
        adj_power = 255;
      }
      motorpower_msg = "\nFL Motor Power: " + String(adj_power) + "\t";

      digitalWrite(FL_PWM, HIGH);
      ledcWrite(PWM_FL, adj_power * trim_left);
    }
    else if (Dir_power_FL < 0) {
      adj_power =  min_to_start + abs(power * Dir_power_FL * (255 - min_to_start)) - power_CCW * (255 - min_to_start) * turn_const;
      if (adj_power > 255) {
        adj_power = 255;
      }
      motorpower_msg = "\nFL Motor Power: " + String(adj_power) + "\t";

      digitalWrite(FL_PWM, LOW);
      ledcWrite(PWM_FL, adj_power * trim_left);
    }

    //Setting Motor Power of Forward Right Motor
    if (Dir_power_FR > 0) {
      adj_power = min_to_start + power * Dir_power_FR * (255 - min_to_start) + power_CCW * (255 - min_to_start) * turn_const;
      if (adj_power > 255) {
        adj_power = 255;
      }
      motorpower_msg += "FR Motor Power: " + String(adj_power) + "\t";

      digitalWrite(FR_PWM, HIGH);
      ledcWrite(PWM_FR, adj_power * trim_right);
    }
    else if (Dir_power_FR < 0) {
      adj_power = min_to_start + abs(power * Dir_power_FR * (255 - min_to_start)) + power_CCW * (255 - min_to_start) * turn_const;
      if (adj_power > 255) {
        adj_power = 255;
      }
      motorpower_msg += "FR Motor Power: " + String(adj_power) + "\t";

      digitalWrite(FR_PWM, LOW);
      ledcWrite(PWM_FR, adj_power * trim_right);
    }

    //Setting Motor Power of Backward Left Motor
    if (Dir_power_BL > 0) {
      adj_power = min_to_start + power * Dir_power_BL * (255 - min_to_start) - power_CCW * (255 - min_to_start) * turn_const;
      if (adj_power > 255) {
        adj_power = 255;
      }
      motorpower_msg += "BL Motor Power: " + String(adj_power) + "\t";

      digitalWrite(BL_PWM, HIGH);
      ledcWrite(PWM_BL, adj_power * trim_left);
    }
    else if (Dir_power_BL < 0) {
      adj_power = min_to_start + abs(power * Dir_power_BL * (255 - min_to_start)) - power_CCW * (255 - min_to_start) * turn_const;
      if (adj_power > 255) {
        adj_power = 255;
      }
      motorpower_msg += "BL Motor Power: " + String(adj_power) + "\t";

      digitalWrite(BL_PWM, LOW);
      ledcWrite(PWM_BL, adj_power * trim_left);
    }

    //Setting Motor Power of Backward Right Motor
    if (Dir_power_BR > 0) {
      adj_power = min_to_start + power * Dir_power_BR * (255 - min_to_start) + power_CCW * (255 - min_to_start) * turn_const;
      if (adj_power > 255) {
        adj_power = 255;
      }
      motorpower_msg += "BR Motor Power: " + String(adj_power);

      digitalWrite(BR_PWM, HIGH);
      ledcWrite(PWM_BR, adj_power * trim_right);
    }
    else if (Dir_power_BR < 0) {
      adj_power = min_to_start + abs(power * Dir_power_BR * (255 - min_to_start)) + power_CCW * (255 - min_to_start) * turn_const;
      if (adj_power > 255) {
        adj_power = 255;
      }
      motorpower_msg += "BR Motor Power: " + String(adj_power);

      digitalWrite(BR_PWM, LOW);
      ledcWrite(PWM_BR, adj_power * trim_right);
    }
  }
  else {
    if (power_CCW > 0) {
      //Rotation Counter-Clockwise
      digitalWrite(FR_PWM, HIGH);
      ledcWrite(PWM_FR, min_to_start + power_CCW * (255 - min_to_start) * trim_right);
      digitalWrite(BR_PWM, LOW);
      ledcWrite(PWM_BR, min_to_start + power_CCW * (255 - min_to_start) * trim_right);

      digitalWrite(FL_PWM, LOW);
      ledcWrite(PWM_FL, min_to_start + power_CCW * (255 - min_to_start) * trim_left);
      digitalWrite(BL_PWM, HIGH);
      ledcWrite(PWM_BL, min_to_start + power_CCW * (255 - min_to_start) * trim_left);
    }
    else {
      //Rotation Clockwise
      digitalWrite(FL_PWM, HIGH);
      ledcWrite(PWM_FL, power_CCW * 255 * trim_left);
      digitalWrite(BL_PWM, LOW);
      ledcWrite(PWM_BL, power_CCW * 255 * trim_left);

      digitalWrite(FR_PWM, LOW);
      ledcWrite(PWM_FR, power_CCW * 255 * trim_right);
      digitalWrite(BR_PWM, HIGH);
      ledcWrite(PWM_BR, power_CCW * 255 * trim_right);
    }
  }

  //Printing Data
  udp.broadcastTo(motorpower_msg.c_str(), 9999);

}
