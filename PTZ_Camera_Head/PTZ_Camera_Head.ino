#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <AccelStepper.h>
#include <LANCControl.h>
#include <ESP32Servo.h>
#include <Preferences.h>

Preferences prefs;

Servo servo1;


#define dirPin1 22
#define stepPin1 21
#define dirPin2 27
#define stepPin2 25

#define servoPin 15


#define PAN_HOMING_OFFSET 2.5    // Adjust PanStepper offset in steps
#define TILT_HOMING_OFFSET -190  // Adjust TiltStepper offset in steps


unsigned long buttonPressStart[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // Store the press start time for each button
const unsigned long holdDuration = 1000;
const unsigned long holdDurationShort = 500;


AccelStepper PanStepper(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper TiltStepper(AccelStepper::DRIVER, stepPin2, dirPin2);

// MAC address for the Ethernet shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// Create a server that listens on port 8080
EthernetServer server(8080);

// Variables to receive from the client (all integers)
int variables[15];
int postionPan[8];
int postionTilt[8];

float moveSpeed = 1;
float panSpeedMulti = 1;
float moveSpeedMultiplier = 1;
float moveSpeedMultiplierOld = 1;
int knob1Old, knob1New, knob2Old, knob2New, knob3Old, knob3New;
float AccelerationPan = 0.14;
float AccelerationTilt = 0.14;
bool zooming = 0;



void movesteppers(void *prameters) {

  float delayedSpeedPan = 0;
  int speedPan = 0;
  int speedJoyPan = 0;
  int speedPan1 = 0;
  int lastSpeedPan = 0;
  bool panLeftLogic = 0;
  bool panRightLogic = 0;

  float delayedSpeedTilt = 0;
  int speedTilt = 0;
  int speedJoyTilt = 0;
  int speedTilt1 = 0;
  int lastSpeedTilt = 0;

  int notPressedButton2 = 0;
  int notPressedButton3 = 0;


  while (true) {
    AccelerationPan = (1900 * moveSpeed * moveSpeedMultiplier) / 35000;
    AccelerationTilt = (1900 * moveSpeed * moveSpeedMultiplier) / 35000;


    speedJoyPan = variables[10] - 1900;

    speedJoyPan = speedJoyPan * moveSpeed * moveSpeedMultiplier;


    //Left continuous pan
    if (variables[1] == 1 && panLeftLogic == 0 && notPressedButton2 == 0) {
      panLeftLogic = 1;
      panRightLogic = 0;
      notPressedButton2 = 1;
    }

    if (variables[1] == 0) {
      notPressedButton2 = 0;
    }

    if (panLeftLogic == 1) {
      if (((variables[10] - 1900) > 1200 || (variables[10] - 1900) < -1200 || variables[1] == 1) && notPressedButton2 == 0) {
        panLeftLogic = 0;
        speedJoyPan = 0;
        notPressedButton2 = 1;
      } else {
        speedJoyPan = 1000 * panSpeedMulti;
      }
    }


    //right continuous pan
    if (variables[2] == 1 && panRightLogic == 0 && notPressedButton3 == 0) {
      panRightLogic = 1;
      panLeftLogic = 0;
      notPressedButton3 = 1;
    }

    if (variables[2] == 0) {
      notPressedButton3 = 0;
    }

    if (panRightLogic == 1) {
      if (((variables[10] - 1900) > 1200 || (variables[10] - 1900) < -1200 || variables[2] == 1) && notPressedButton3 == 0) {
        panRightLogic = 0;
        speedJoyPan = 0;
        notPressedButton3 = 1;
      } else {
        speedJoyPan = -1000 * panSpeedMulti;
      }
    }




    if (speedJoyPan > delayedSpeedPan) {
      delayedSpeedPan = delayedSpeedPan + AccelerationPan;
    } else if (speedJoyPan < delayedSpeedPan) {
      delayedSpeedPan = delayedSpeedPan - AccelerationPan;
    } else {
    }

    speedPan = delayedSpeedPan;

    speedPan1 = speedPan;

    if (speedPan != 0) {
      speedPan = speedPan / 100;
      speedPan = speedPan * -speedPan;
      speedPan = speedPan * (speedPan1 / abs(speedPan1));
    }

    if (speedPan < 22 && speedPan > -22) {
      speedPan = 0;
    }

    if (PanStepper.currentPosition() < -2850) {
      if (speedPan < 0) {  // If motor is moving towards the negative limit, stop it
        speedPan = 0;
        delayedSpeedPan = 0;
      }
    }
    if (PanStepper.currentPosition() > 2850) {
      if (speedPan > 0) {  // If motor is moving towards the positive limit, stop it
        speedPan = 0;
        delayedSpeedPan = 0;
      }
    }

    if (speedPan != lastSpeedPan) {
      PanStepper.setSpeed(speedPan / 5);
      lastSpeedPan = speedPan;
    }

    if (speedPan != 0) {
      PanStepper.runSpeed();
    }

    speedJoyTilt = variables[11] - 1900;

    speedJoyTilt = speedJoyTilt * moveSpeed * moveSpeedMultiplier;

    if (speedJoyTilt > delayedSpeedTilt) {
      delayedSpeedTilt = delayedSpeedTilt + AccelerationTilt;
    } else if (speedJoyTilt < delayedSpeedTilt) {
      delayedSpeedTilt = delayedSpeedTilt - AccelerationTilt;
    } else {
    }

    speedTilt = delayedSpeedTilt;

    speedTilt1 = speedTilt;

    if (speedTilt != 0) {
      speedTilt = speedTilt / 100;
      speedTilt = speedTilt * -speedTilt;
      speedTilt = speedTilt * (speedTilt1 / abs(speedTilt1));
    }

    if (speedTilt < 20 && speedTilt > -20) {
      speedTilt = 0;
    }
    if (TiltStepper.currentPosition() < -1000) {
      if (speedTilt < 0) {  // If motor is moving towards the negative limit, stop it
        speedTilt = 0;
        delayedSpeedTilt = 0;
      }
    }
    if (TiltStepper.currentPosition() > 1000) {
      if (speedTilt > 0) {  // If motor is moving towards the positive limit, stop it
        speedTilt = 0;
        delayedSpeedTilt = 0;
      }
    }

    if (speedTilt != lastSpeedTilt) {
      TiltStepper.setSpeed(speedTilt / 16);
      lastSpeedTilt = speedTilt;
    }

    if (speedTilt != 0) {
      TiltStepper.runSpeed();
    }



    //Trigger Shutdown when button 4 is held for holdDuration Seconds
    unsigned long currentTime3 = millis();
    if (variables[3] == 1) {
      if (buttonPressStart[3] == 0) {
        buttonPressStart[3] = currentTime3;  // Record the time when the button was first pressed
      } else if (currentTime3 - buttonPressStart[3] >= holdDuration) {
        Serial.println("Button 4 held for holdDuration");
        ESP.restart();
        buttonPressStart[3] = 0;  // Reset to detect the next long press
      }
    } else if (currentTime3 - buttonPressStart[3] <= holdDurationShort) {  //if short press
      runToPostion(-2000, 900);

      // Save moveSpeed to non-volatile storage
      prefs.begin("my-app", false);
      prefs.putFloat("moveSpeed", moveSpeed);
      prefs.putFloat("panSpeedMulti", panSpeedMulti);
      prefs.end();
      panLeftLogic = 0;
      panRightLogic = 0;
      delayedSpeedPan = 0;
      Serial.println("Shutdown complete, moveSpeed and panSpeedMulti saved, and motors returned to home position.");
      panLeftLogic = 0;
      buttonPressStart[3] = 0;  // Reset to detect the next press
    } else {
      buttonPressStart[3] = 0;  // Reset if the button is released
    }


    // Button 5
    unsigned long currentTime4 = millis();
    if (variables[4] == 1) {
      if (buttonPressStart[4] == 0) {
        buttonPressStart[4] = currentTime4;                             // Record the time when the button was first pressed
      } else if (currentTime4 - buttonPressStart[4] >= holdDuration) {  //if long press
        Serial.println("Button 5 held for holdDuration");

        postionPan[4] = PanStepper.currentPosition();
        postionTilt[4] = TiltStepper.currentPosition();
        Serial.println("set postion button 5");

        // Save varibles to non-volatile storage
        prefs.begin("my-app", false);
        prefs.putInt("postionPan4", postionPan[4]);
        prefs.putInt("postionTilt4", postionTilt[4]);
        prefs.end();

        buttonPressStart[4] = 0;  // Reset to detect the next long press
      }
    } else if (currentTime4 - buttonPressStart[4] <= holdDurationShort) {  //if short press
      runToPostion(postionPan[4], postionTilt[4]);
      panLeftLogic = 0;
      panRightLogic = 0;
      delayedSpeedPan = 0;
      Serial.println("short press 5");
      buttonPressStart[4] = 0;  // Reset to detect the next press
    } else {
      buttonPressStart[4] = 0;  // Reset if the button is released
    }


    // Button 6
    unsigned long currentTime5 = millis();
    if (variables[5] == 1) {
      if (buttonPressStart[5] == 0) {
        buttonPressStart[5] = currentTime5;                             // Record the time when the button was first pressed
      } else if (currentTime5 - buttonPressStart[5] >= holdDuration) {  //if long press
        Serial.println("Button 6 held for holdDuration");

        postionPan[5] = PanStepper.currentPosition();
        postionTilt[5] = TiltStepper.currentPosition();
        Serial.println("set postion button 6");

        // Save varibles to non-volatile storage
        prefs.begin("my-app", false);
        prefs.putInt("postionPan5", postionPan[5]);
        prefs.putInt("postionTilt5", postionTilt[5]);
        prefs.end();

        buttonPressStart[5] = 0;  // Reset to detect the next long press
      }
    } else if (currentTime5 - buttonPressStart[5] <= holdDurationShort) {  //if short press
      runToPostion(postionPan[5], postionTilt[5]);
      panLeftLogic = 0;
      panRightLogic = 0;
      delayedSpeedPan = 0;
      Serial.println("short press 6");
      buttonPressStart[5] = 0;  // Reset to detect the next press
    } else {
      buttonPressStart[5] = 0;  // Reset if the button is released
    }


    // Button 7
    unsigned long currentTime6 = millis();
    if (variables[6] == 1) {
      if (buttonPressStart[6] == 0) {
        buttonPressStart[6] = currentTime6;                             // Record the time when the button was first pressed
      } else if (currentTime6 - buttonPressStart[6] >= holdDuration) {  //if long press
        Serial.println("Button 7 held for holdDuration");

        postionPan[6] = PanStepper.currentPosition();
        postionTilt[6] = TiltStepper.currentPosition();
        Serial.println("set postion button 7");

        // Save varibles to non-volatile storage
        prefs.begin("my-app", false);
        prefs.putInt("postionPan6", postionPan[6]);
        prefs.putInt("postionTilt6", postionTilt[6]);
        prefs.end();

        buttonPressStart[6] = 0;  // Reset to detect the next long press
      }
    } else if (currentTime6 - buttonPressStart[6] <= holdDurationShort) {  //if short press
      runToPostion(postionPan[6], postionTilt[6]);
      panLeftLogic = 0;
      panRightLogic = 0;
      delayedSpeedPan = 0;
      Serial.println("short press 7");
      buttonPressStart[6] = 0;  // Reset to detect the next press
    } else {
      buttonPressStart[6] = 0;  // Reset if the button is released
    }


    // Button 8
    unsigned long currentTime7 = millis();
    if (variables[7] == 1) {
      if (buttonPressStart[7] == 0) {
        buttonPressStart[7] = currentTime7;                             // Record the time when the button was first pressed
      } else if (currentTime7 - buttonPressStart[7] >= holdDuration) {  //if long press
        Serial.println("Button 8 held for holdDuration");

        postionPan[7] = PanStepper.currentPosition();
        postionTilt[7] = TiltStepper.currentPosition();
        Serial.println("set postion button 8");

        // Save varibles to non-volatile storage
        prefs.begin("my-app", false);

        ("postionPan7", postionPan[7]);
        prefs.putInt("postionTilt7", postionTilt[7]);
        prefs.end();

        buttonPressStart[7] = 0;  // Reset to detect the next long press
      }
    } else if (currentTime7 - buttonPressStart[7] <= holdDurationShort) {  //if short press
      runToPostion(postionPan[7], postionTilt[7]);
      panLeftLogic = 0;
      panRightLogic = 0;
      delayedSpeedPan = 0;
      Serial.println("short press 8");
      buttonPressStart[7] = 0;  // Reset to detect the next press
    } else {
      buttonPressStart[7] = 0;  // Reset if the button is released
    }
  }
}


void zoom(void *prameters) {
  int zoom = 0;
  while (true) {
    zoom = variables[9] - 1980;
    zoom = zoom / 20;
    if (zoom < 20 && zoom > -20) {
      zoom = 0;
      zooming == 0;
    } else {
      zooming == 1;
    }

    if (zoom > 0) {
      int zoomInSpeed = map(zoom, 0, 100, 0, 7);  // Map to zoom in states (0-7)
      ZoomIn(zoomInSpeed);                        // Call ZoomIn function
    } else if (zoom < 0) {
      int zoomOutSpeed = map(zoom, 0, -100, 0, 7);  // Map to zoom out states (0-7)
      ZoomOut(zoomOutSpeed);                        // Call ZoomOut function
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}


void controlServo(void *parameters) {
  vTaskDelay(150 / portTICK_PERIOD_MS);
  knob1Old = variables[12];
  while (true) {
    knob1New = variables[12];

    if (knob1New >= knob1Old + 1) {
      for (int i = 0; i < 250; i++) {
        servo1.write(110);
        vTaskDelay(1.5 / portTICK_PERIOD_MS);
      }
    } else if (knob1New <= knob1Old - 1) {
      for (int i = 0; i < 250; i++) {
        servo1.write(70);
        vTaskDelay(1.5 / portTICK_PERIOD_MS);
      }
    } else {
      servo1.write(90);
    }
    knob1Old = knob1New;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}



void setup() {
  Serial.begin(115200);

  delay(500);

  setupLANC();

  // Allow allocation of all timers for ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach the servo
  servo1.setPeriodHertz(50);            // Standard 50 Hz servo
  servo1.attach(servoPin, 1000, 2000);  // Attach the servo to the pin with min/max pulse widths


  delay(500);
  Serial.println("Initializing Ethernet...");

  Ethernet.init(5);  // Set the CS pin (5 for W5500)

  // Start Ethernet with DHCP
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    while (true)
      ;  // Halt execution if DHCP fails
  } else {
    Serial.println("DHCP configured successfully");
  }
  delay(2000);

  // Print local IP
  Serial.println("Server is at IP: ");
  Serial.println(Ethernet.localIP());

  // Start listening for clients
  server.begin();

  pinMode(4, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(16, OUTPUT);

  digitalWrite(4, LOW);
  digitalWrite(13, LOW);
  digitalWrite(16, LOW);

  PanStepper.setMaxSpeed(2000);
  TiltStepper.setMaxSpeed(2000);
  PanStepper.setAcceleration(1000);
  TiltStepper.setAcceleration(1000);

  // Recalling Values from memory
  prefs.begin("my-app", false);

  // Load the saved moveSpeed value if it exists
  if (prefs.isKey("moveSpeed")) {
    moveSpeed = prefs.getFloat("moveSpeed", 2.0);
    Serial.println("Loaded moveSpeed: ");
    Serial.println(moveSpeed);
  }

  if (prefs.isKey("panSpeedMulti")) {
    panSpeedMulti = prefs.getFloat("panSpeedMulti", 2.0);
    Serial.println("Loaded panSpeedMulti: ");
    Serial.println(panSpeedMulti);
  }

  if (prefs.isKey("postionTilt4")) {
    postionTilt[4] = prefs.getInt("postionTilt4", 2.0);
    Serial.println("Loaded postionTilt4: ");
    Serial.println(postionTilt[4]);
  }
  if (prefs.isKey("postionPan4")) {
    postionPan[4] = prefs.getInt("postionPan4", 2.0);
    Serial.println("Loaded postionPan4: ");
    Serial.println(postionPan[4]);
  }

  if (prefs.isKey("postionTilt5")) {
    postionTilt[5] = prefs.getInt("postionTilt5", 2.0);
    Serial.println("Loaded postionTilt5: ");
    Serial.println(postionTilt[5]);
  }
  if (prefs.isKey("postionPan5")) {
    postionPan[5] = prefs.getInt("postionPan5", 2.0);
    Serial.println("Loaded postionPan5: ");
    Serial.println(postionPan[5]);
  }

  if (prefs.isKey("postionTilt6")) {
    postionTilt[6] = prefs.getInt("postionTilt6", 2.0);
    Serial.println("Loaded postionTilt6: ");
    Serial.println(postionTilt[6]);
  }
  if (prefs.isKey("postionPan6")) {
    postionPan[6] = prefs.getInt("postionPan6", 2.0);
    Serial.println("Loaded postionPan6: ");
    Serial.println(postionPan[6]);
  }

  if (prefs.isKey("postionTilt7")) {
    postionTilt[7] = prefs.getInt("postionTilt7", 2.0);
    Serial.println("Loaded postionTilt7: ");
    Serial.println(postionTilt[7]);
  }
  if (prefs.isKey("postionPan7")) {
    postionPan[7] = prefs.getInt("postionPan7", 2.0);
    Serial.println("Loaded postionPan7: ");
    Serial.println(postionPan[7]);
  }
  // End of recalling moveSpeed from memory

  prefs.end();


  // Homing PanStepper
  pinMode(26, INPUT);
  PanStepper.setSpeed(1000);
  int counter = 0;
  while (digitalRead(26) == 1) {
    PanStepper.runSpeed();
    delay(1.5);
    counter++;
    if (counter >= 3500) {
      break;  // Exit loop after 3500 iterations
    }
  }
  PanStepper.setSpeed(-1000);
  while (digitalRead(26) == 1) {
    PanStepper.runSpeed();
    delay(1.5);
  }

  PanStepper.move(-750);  // Move 500 steps forward
  PanStepper.runToPosition();

  PanStepper.setSpeed(250);
  while (digitalRead(26) == 1) {
    PanStepper.runSpeed();
    delay(1.5);
  }

  // Move PanStepper by offset after homing
  PanStepper.move(PAN_HOMING_OFFSET);
  PanStepper.runToPosition();  // Ensure it reaches the offset position
  Serial.println("PanStepper homing done");

  // Homing TiltStepper
  pinMode(35, INPUT);
  TiltStepper.setSpeed(-1000);
  counter = 0;
  while (digitalRead(35) == 1) {
    TiltStepper.runSpeed();
    delay(1);
    counter++;
    if (counter >= 2500) {
      break;  // Exit loop after 2500 iterations
    }
  }
  TiltStepper.setSpeed(1000);
  while (digitalRead(35) == 1) {
    TiltStepper.runSpeed();
    delay(1.5);
  }

  TiltStepper.move(500);  // Move 500 steps forward
  TiltStepper.runToPosition();

  TiltStepper.setSpeed(-250);
  while (digitalRead(35) == 1) {
    TiltStepper.runSpeed();
    delay(1.5);
  }

  TiltStepper.move(105);
  TiltStepper.runToPosition();

  // Move TiltStepper by offset after homing
  TiltStepper.move(TILT_HOMING_OFFSET);
  TiltStepper.runToPosition();  // Ensure it reaches the offset position
  Serial.println("TiltStepper homing done");

  PanStepper.setCurrentPosition(0);
  TiltStepper.setCurrentPosition(0);

  xTaskCreate(
    movesteppers,     // Funcction name
    "Move Steppers",  // Task Name
    10000,            // stack size
    NULL,             // task parameters
    1,                // task priority
    NULL              // task handel
  );

  xTaskCreate(
    zoom,    // Funcction name
    "Zoom",  // Task Name
    4000,    // stack size
    NULL,    // task parameters
    1,       // task priority
    NULL     // task handel
  );

  xTaskCreate(
    controlServo,     // Function to run the task
    "Control Servo",  // Task name
    4000,             // Stack size
    NULL,             // Task parameters
    1,                // Task priority
    NULL              // Task handle
  );
}


String readStringUntil(EthernetClient &client, char terminator) {
  String value = "";
  char c;
  while (client.available()) {
    c = client.read();
    if (c == terminator) break;
    value += c;
  }
  return value;
}

int readIntVariable(EthernetClient &client) {
  String value = "";
  char c;
  while (client.available()) {
    c = client.read();
    if (c == '\n') break;  // Stop reading at newline
    value += c;
  }
  return value.toInt();
}

void printReceivedVariables() {
  for (int i = 0; i < 15; i++) {
    Serial.println("Variable ");
    Serial.println(i + 1);
    Serial.println(": ");
    Serial.println(variables[i]);
  }
}

void runToPostion(int PanPostion, int TiltPostion) {
  // Action for button 2 - Move both motors to target positions
  PanStepper.moveTo(PanPostion);
  TiltStepper.moveTo(TiltPostion);

  // Calculate the distances to travel
  long panDistance = PanStepper.distanceToGo();
  long tiltDistance = TiltStepper.distanceToGo();

  // Calculate the speed ratio to synchronize arrival times
  float maxDistance = max(abs(panDistance), abs(tiltDistance));
  float panSpeed = 500.0 * (abs(panDistance) / maxDistance);
  float tiltSpeed = 500.0 * (abs(tiltDistance) / maxDistance);

  // Set the speeds with appropriate direction
  PanStepper.setSpeed(panSpeed * (panDistance < 0 ? -1 : 1));
  TiltStepper.setSpeed(tiltSpeed * (tiltDistance < 0 ? -1 : 1));

  // Run both motors until they reach their target positions
  while (PanStepper.distanceToGo() != 0 || TiltStepper.distanceToGo() != 0) {
    PanStepper.run();
    TiltStepper.run();
  }
}

unsigned long lastCommunicationTime = 0;
const unsigned long timeoutDuration = 200;  // 5 seconds timeout

void resetNonEncoderVariables() {
  // Reset button and joystick variables
  for (int i = 0; i < 8; i++) {
    variables[i] = 0;  // Reset buttons to 0
  }
  for (int i = 8; i < 12; i++) {
    variables[i] = 1900;  // Reset joysticks to 1900
  }
}








String dataBuffer = "";  // Buffer to accumulate incoming data

void receive_variables() {
  EthernetClient client = server.available();  // Check if a client is trying to connect
  if (client) {
    if (client.connected()) {
      // Client is connected, reset the timeout
      lastCommunicationTime = millis();

      // Read incoming data and add to the buffer
      while (client.available()) {
        char c = client.read();
        dataBuffer += c;

        // Process each complete message
        if (c == '\n') {
          // Split message by prefix, then assign the variables
          int equalPos = dataBuffer.indexOf('=');
          if (equalPos > 0) {
            String prefix = dataBuffer.substring(0, equalPos);
            int variableIndex = prefix.substring(1).toInt() - 1;
            if (variableIndex >= 0 && variableIndex < 15) {
              variables[variableIndex] = dataBuffer.substring(equalPos + 1).toInt();
            }
          }

          dataBuffer = "";  // Clear the buffer after processing
        }
      }
    }

    // Disconnect the client if no longer connected
    if (!client.connected()) {
      Serial.println("Client disconnected");
      client.stop();  // Stop the client and release resources
      resetNonEncoderVariables();
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }



/*
void receive_variables() {
  EthernetClient client = server.available();  // Check if a client is trying to connect
  if (client) {
    if (client.connected()) {
      // Client is connected, reset the timeout
      lastCommunicationTime = millis();

      // Client is sending data
      while (client.available()) {
        String prefix = readStringUntil(client, '=');
        int variableIndex = prefix.substring(1).toInt() - 1;
        if (variableIndex >= 0 && variableIndex < 15) {
          variables[variableIndex] = readIntVariable(client);
        }
        //vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    }

    // Check if the client has disconnected
    if (!client.connected()) {
      Serial.println("Client disconnected");
      client.stop();  // Stop the client and release resources
      resetNonEncoderVariables();
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}
*/
}







/*
void receive_variables() {
  EthernetClient client = server.available();  // Check if a client is trying to connect
  if (client) {
    if (client.connected()) {
      // Client is connected, reset the timeout
      lastCommunicationTime =5 millis();

      // Client is .5sending data
      while (clie.5nt.available()) {
        String prefix = readStringUntil(client, '=');
        int variab4eIndex = prefix.substring(1).toInt() - 1;
        if (vari4bleIndex >= 0 && variableIndex < 15) {
          variables[variableIndex] = readIntVariable(client);
        }
        //vTaskDelay(10 / portTICK_PERIOD_MS);
      }
    }

    // Check if the client has disconnected
    if (!client.connected()) {
      Serial.println("Client disconnected");
      client.stop();  // Stop the client and release resources
      resetNonEncoderVariables();
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}
*/

void connectionTimeout() {
  if (millis() - lastCommunicationTime > timeoutDuration) {
    Serial.println("Client timeout, resetting variables...");
    resetNonEncoderVariables();
    lastCommunicationTime = millis();  // Reset the time to avoid repeating the reset every loop
  }
}


void adjustSpeed() {
  knob2New = variables[13];
  int knob2Difference = knob2Old - knob2New;
  knob2Old = knob2New;
  moveSpeed = moveSpeed + 0.1 * knob2Difference;

  if (moveSpeed < 1) {
    moveSpeed = 1;
  }
  if (moveSpeed > 2.5) {
    moveSpeed = 2.5;
  }

  if (variables[0] == 1) {
    moveSpeedMultiplier = 1.85;
  } else {
    moveSpeedMultiplier = 1;
  }
  if (moveSpeed * moveSpeedMultiplier > 4.5) {
    moveSpeedMultiplier = 4.5 / moveSpeed;
  }


  knob3New = variables[14];
  int knob3Difference = knob3Old - knob3New;
  knob3Old = knob3New;
  panSpeedMulti = panSpeedMulti + 0.15 * knob3Difference;

  if (panSpeedMulti < 1) {
    panSpeedMulti = 1;
  }
  if (panSpeedMulti > 2.5) {
    panSpeedMulti = 2.5;
  }
}


void loop() {
  receive_variables();
  vTaskDelay(150 / portTICK_PERIOD_MS);
  knob2Old = variables[13];
  while (true) {
    receive_variables();
    connectionTimeout();
    adjustSpeed();
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}