#include <PS4Controller.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

const float joystick_deadzone = 0.04;
bool LR_button_state[4] = {0}; //L1, R1, L3, R3

const char * ssid = "raisins";
const char * password = "illuminatingdarkness";




void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

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

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  Serial.println("Connecting to PS4.");
  PS4.begin("8c:55:4a:6f:ca:4c");
  Serial.println("Ready.");
}

void loop() {

   ArduinoOTA.handle();
   
  if (millis() % 10 == 0 && PS4.isConnected()) {
    //Calculate joystick position
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
    }
  }
  
}
