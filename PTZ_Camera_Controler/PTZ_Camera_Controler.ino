#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ESP_Knob.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PS4Controller.h>


#define LedPin 16
#define Buttonpin1 4
#define Buttonpin2 13
#define Buttonpin3 14
#define Buttonpin4 36
#define Buttonpin5 39
#define Buttonpin6 15
#define Buttonpin7 2
#define Buttonpin8 12
#define Joypin1x 33
#define Joypin1y 32
#define Joypin2x 35
#define Joypin2y 34
#define RE_1_1 17

#define RE_1_2 21
#define RE_2_1 22
#define RE_2_2 25
#define RE_3_1 26
#define RE_3_2 27

#define NUM_LEDS 7

//use the SixaxisPairTool to fiugure the Mac Address of the PS4 and put it in the PS4.begin in the startup

// Server IP and port configuration
IPAddress server(10, 69, 2, 122);  //change the Server IP Address to the cameras IP Address    10, 69, 2, 122
const int port = 8080;

// MAC address for the Ethernet shield
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE };

EthernetClient client;



int variables[15];


unsigned long buttonPressStart[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };  // Store the press start time for each button
const unsigned long holdDuration = 1000;
const unsigned long holdDurationShort = 500;


Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LedPin, NEO_GRB + NEO_KHZ800);


ESP_Knob knob1(RE_1_1, RE_1_2);
ESP_Knob knob2(RE_2_1, RE_2_2);
ESP_Knob knob3(RE_3_1, RE_3_2);

int knob1Count = 0;
int knob2Count = 0;
int knob3Count = 0;
bool R1Bool = 1;

int knob1Old, knob1New, knob2Old, knob2New, knob3Old, knob3New;


void setColorAll(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);  // Set the color for each pixel
  }
  strip.show();  // Update the strip with the new color
}


void lightsForKnobs(void *prameters) {
  while (true) {
    knob1New = knob1Count;

    if (knob1New == knob1Old + 1) {
      strip.setPixelColor(0, strip.Color(0, 0, 255));
      strip.show();
    }

    if (knob1New == knob1Old - 1) {
      strip.setPixelColor(0, strip.Color(0, 0, 255));
      strip.show();
    }
    knob1Old = knob1New;


    knob2New = knob2Count;

    if (knob2New == knob2Old + 1) {
      strip.setPixelColor(1, strip.Color(0, 0, 255));
      strip.show();
    }

    if (knob2New == knob2Old - 1) {
      strip.setPixelColor(1, strip.Color(0, 0, 255));
      strip.show();
    }
    knob2Old = knob2New;


    knob3New = knob3Count;

    if (knob3New == knob3Old + 1) {
      strip.setPixelColor(2, strip.Color(0, 0, 255));
      strip.show();
    }

    if (knob3New == knob3Old - 1) {
      strip.setPixelColor(2, strip.Color(0, 0, 255));
      strip.show();
    }
    knob3Old = knob3New;
    vTaskDelay(1.5 / portTICK_PERIOD_MS);
  }
}


void lightsForButtons(void *prameters) {
  while (true) {
    // Button 1
    if (digitalRead(Buttonpin1) == 1) {
      strip.setPixelColor(3, strip.Color(0, 0, 255));
      strip.show();
    }

    // Button 2
    unsigned long currentTime1 = millis();
    if (digitalRead(Buttonpin2) == 1) {
      if (buttonPressStart[1] == 0) {
        buttonPressStart[1] = currentTime1;                             // Record the time when the button was first pressed
      } else if (currentTime1 - buttonPressStart[1] >= holdDuration) {  //if long press
                                                                        //Action if long
        buttonPressStart[1] = 0;                                        // Reset to detect the next long press
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    } else if (currentTime1 - buttonPressStart[1] <= holdDurationShort) {  //if short press
      for (int i = 0; i < 100; i++) {
        strip.setPixelColor(4, strip.Color(0, 0, 255));
        strip.show();
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      //Serial.println("short press 2");
      buttonPressStart[1] = 0;  // Reset to detect the next press
    } else {
      buttonPressStart[1] = 0;  // Reset if the button is released
    }


    // Button 3
    unsigned long currentTime2 = millis();
    if (digitalRead(Buttonpin3) == 1) {
      if (buttonPressStart[2] == 0) {
        buttonPressStart[2] = currentTime2;                             // Record the time when the button was first pressed
      } else if (currentTime2 - buttonPressStart[2] >= holdDuration) {  //if long press
                                                                        //Action if long
        buttonPressStart[2] = 0;                                        // Reset to detect the next long press
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    } else if (currentTime2 - buttonPressStart[2] <= holdDurationShort) {  //if short press
      for (int i = 0; i < 100; i++) {
        strip.setPixelColor(5, strip.Color(0, 0, 255));
        strip.show();
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      //Serial.println("short press 3");
    } else {
      buttonPressStart[2] = 0;  // Reset if the button is released
    }


    // Button 4
    unsigned long currentTime3 = millis();
    if (digitalRead(Buttonpin4) == 1) {
      if (buttonPressStart[3] == 0) {
        buttonPressStart[3] = currentTime3;                             // Record the time when the button was first pressed
      } else if (currentTime3 - buttonPressStart[3] >= holdDuration) {  //if long press
                                                                        //Serial.println("Button 5 held for holdDuration");
        for (int i = 0; i < 100; i++) {
          strip.setPixelColor(6, strip.Color(255, 0, 0));
          strip.show();
          vTaskDelay(2 / portTICK_PERIOD_MS);
        }
        buttonPressStart[3] = 0;  // Reset to detect the next long press
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    } else if (currentTime3 - buttonPressStart[3] <= holdDurationShort) {  //if short press

      for (int i = 0; i < 100; i++) {
        strip.setPixelColor(6, strip.Color(0, 0, 255));
        strip.show();
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }

      //Serial.println("short press 5");
      buttonPressStart[3] = 0;  // Reset to detect the next press
    } else {
      buttonPressStart[3] = 0;  // Reset if the button is released
    }


    // Button 5
    unsigned long currentTime4 = millis();
    if (digitalRead(Buttonpin5) == 1) {
      if (buttonPressStart[4] == 0) {
        buttonPressStart[4] = currentTime4;                             // Record the time when the button was first pressed
      } else if (currentTime4 - buttonPressStart[4] >= holdDuration) {  //if long press
                                                                        //Serial.println("Button 5 held for holdDuration");
        for (int i = 0; i < 100; i++) {
          strip.setPixelColor(3, strip.Color(255, 0, 0));
          strip.show();
          vTaskDelay(2 / portTICK_PERIOD_MS);
        }
        buttonPressStart[4] = 0;  // Reset to detect the next long press
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    } else if (currentTime4 - buttonPressStart[4] <= holdDurationShort) {  //if short press
      for (int i = 0; i < 100; i++) {
        strip.setPixelColor(3, strip.Color(0, 0, 255));
        strip.show();
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      //Serial.println("short press 5");
      buttonPressStart[4] = 0;  // Reset to detect the next press
    } else {
      buttonPressStart[4] = 0;  // Reset if the button is released
    }


    // Button 6
    unsigned long currentTime5 = millis();
    if (digitalRead(Buttonpin6) == 1) {
      if (buttonPressStart[5] == 0) {
        buttonPressStart[5] = currentTime5;                             // Record the time when the button was first pressed
      } else if (currentTime5 - buttonPressStart[5] >= holdDuration) {  //if long press
                                                                        //Serial.println("Button 6 held for holdDuration");
        for (int i = 0; i < 100; i++) {
          strip.setPixelColor(4, strip.Color(255, 0, 0));
          strip.show();
          vTaskDelay(2 / portTICK_PERIOD_MS);
        }
        buttonPressStart[5] = 0;  // Reset to detect the next long press
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    } else if (currentTime5 - buttonPressStart[5] <= holdDurationShort) {  //if short press
      for (int i = 0; i < 100; i++) {
        strip.setPixelColor(4, strip.Color(0, 0, 255));
        strip.show();
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      //Serial.println("short press 6");
      buttonPressStart[5] = 0;  // Reset to detect the next press
    } else {
      buttonPressStart[5] = 0;  // Reset if the button is released
    }


    // Button 7
    unsigned long currentTime6 = millis();
    if (digitalRead(Buttonpin7) == 1) {
      if (buttonPressStart[6] == 0) {
        buttonPressStart[6] = currentTime6;                             // Record the time when the button was first pressed
      } else if (currentTime6 - buttonPressStart[6] >= holdDuration) {  //if long press
                                                                        //Serial.println("Button 7 held for holdDuration");
        for (int i = 0; i < 100; i++) {
          strip.setPixelColor(5, strip.Color(255, 0, 0));
          strip.show();
          vTaskDelay(2 / portTICK_PERIOD_MS);
        }
        buttonPressStart[6] = 0;  // Reset to detect the next long press
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    } else if (currentTime6 - buttonPressStart[6] <= holdDurationShort) {  //if short press
      for (int i = 0; i < 100; i++) {
        strip.setPixelColor(5, strip.Color(0, 0, 255));
        strip.show();
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      //Serial.println("short press 7");
      buttonPressStart[6] = 0;  // Reset to detect the next press
    } else {
      buttonPressStart[6] = 0;  // Reset if the button is released
    }


    // Button 8
    unsigned long currentTime7 = millis();
    if (digitalRead(Buttonpin8) == 1) {
      if (buttonPressStart[7] == 0) {
        buttonPressStart[7] = currentTime7;                             // Record the time when the button was first pressed
      } else if (currentTime7 - buttonPressStart[7] >= holdDuration) {  //if long press
                                                                        //Serial.println("Button 8 held for holdDuration");
        for (int i = 0; i < 100; i++) {
          strip.setPixelColor(6, strip.Color(255, 0, 0));
          strip.show();
          vTaskDelay(2 / portTICK_PERIOD_MS);
        }
        buttonPressStart[7] = 0;  // Reset to detect the next long press
      }
      vTaskDelay(200 / portTICK_PERIOD_MS);
    } else if (currentTime7 - buttonPressStart[7] <= holdDurationShort) {  //if short press
      for (int i = 0; i < 100; i++) {
        strip.setPixelColor(6, strip.Color(0, 0, 255));
        strip.show();
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
      //Serial.println("short press 8");
      buttonPressStart[7] = 0;  // Reset to detect the next press
    } else {
      buttonPressStart[7] = 0;  // Reset if the button is released
    }
    vTaskDelay(2.5 / portTICK_PERIOD_MS);
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(Buttonpin1, INPUT);
  pinMode(Buttonpin2, INPUT);
  pinMode(Buttonpin3, INPUT);
  pinMode(Buttonpin4, INPUT);
  pinMode(Buttonpin5, INPUT);
  pinMode(Buttonpin6, INPUT);
  pinMode(Buttonpin7, INPUT);
  pinMode(Buttonpin8, INPUT);
  pinMode(Joypin1x, INPUT);
  pinMode(Joypin1y, INPUT);
  pinMode(Joypin2x, INPUT);
  pinMode(Joypin2y, INPUT);

  knob1.begin();
  knob2.begin();
  knob3.begin();


  // Initialize PS4 Controller
  PS4.begin("00:1a:7d:da:71:13");  // Replace with your PS4 controller's MAC address

  strip.begin();
  strip.show();

  setColorAll(strip.Color(255, 0, 0));

  knob1.attachLeftEventCallback([](int count, void *) {
    knob1Count = count;
  });
  knob1.attachRightEventCallback([](int count, void *) {
    knob1Count = count;
  });
  knob1.attachHighLimitEventCallback([](int count, void *) {
    knob1Count = count;
  });
  knob1.attachLowLimitEventCallback([](int count, void *) {
    knob1Count = count;
  });
  knob1.attachZeroEventCallback([](int count, void *) {
    knob1Count = count;
  });
  knob2.attachLeftEventCallback([](int count, void *) {
    knob2Count = count;
  });
  knob2.attachRightEventCallback([](int count, void *) {
    knob2Count = count;
  });
  knob2.attachHighLimitEventCallback([](int count, void *) {
    knob2Count = count;
  });
  knob2.attachLowLimitEventCallback([](int count, void *) {
    knob2Count = count;
  });
  knob2.attachZeroEventCallback([](int count, void *) {
    knob2Count = count;
  });
  knob3.attachLeftEventCallback([](int count, void *) {
    knob3Count = count;
  });
  knob3.attachRightEventCallback([](int count, void *) {
    knob3Count = count;
  });
  knob3.attachHighLimitEventCallback([](int count, void *) {
    knob3Count = count;
  });
  knob3.attachLowLimitEventCallback([](int count, void *) {
    knob3Count = count;
  });
  knob3.attachZeroEventCallback([](int count, void *) {
    knob3Count = count;
  });


  delay(1000);
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
  Serial.print("Local IP: ");
  Serial.println(Ethernet.localIP());


  xTaskCreate(
    lightsForKnobs,    // Function name
    "lightsForKnobs",  // Task Name
    10000,             // stack size
    NULL,              // task parameters
    1,                 // task priority
    NULL               // task handle
  );
  xTaskCreate(
    lightsForButtons,    // Function name
    "lightsForButtons",  // Task Name
    10000,               // stack size
    NULL,                // task parameters
    1,                   // task priority
    NULL                 // task handle
  );
}

void connectToServer() {
  while (!client.connect(server, port)) {
    setColorAll(strip.Color(255, 0, 0));
    Serial.println("Connection failed, retrying...");
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  setColorAll(strip.Color(0, 255, 0));
  //Serial.println("Connected to server");
}


void send_variables() {

  int variables[15] = {
    digitalRead(Buttonpin1) || PS4.L2(),        // Button 1 prioritized over PS4.L1
    digitalRead(Buttonpin2) || PS4.Left(),      // Button 2 prioritized over PS4.L2
    digitalRead(Buttonpin3) || PS4.Right(),     // Button 3 prioritized over PS4.R1
    digitalRead(Buttonpin4) || PS4.Share(),     // Button 4 prioritized over PS4.R2
    digitalRead(Buttonpin5) || PS4.Square(),    // Button 5 prioritized over PS4 Triangle
    digitalRead(Buttonpin6) || PS4.Triangle(),  // Button 6 prioritized over PS4 Circle
    digitalRead(Buttonpin7) || PS4.Circle(),    // Button 7 prioritized over PS4 Cross
    digitalRead(Buttonpin8) || PS4.Cross(),     // Button 8 prioritized over PS4 Square
    1900,
    1900,
    1900,
    1900,
    knob1Count,
    knob2Count,
    knob3Count
  };

  int joy1x = analogRead(Joypin1x);
  int joy1y = analogRead(Joypin1y);
  int joy2x = analogRead(Joypin2x);
  int joy2y = analogRead(Joypin2y);


  // Use physical joystick if it moves, otherwise use PS4 values
  variables[8] = (joy1x > 2000 || joy1x < 1800) ? joy1x : (PS4.LStickX() + 128) * 15;
  variables[9] = (joy1y > 2000 || joy1y < 1800) ? joy1y : (PS4.LStickY() + 128) * 15;
  variables[10] = (joy2x > 2000 || joy2x < 1800) ? joy2x : (-PS4.RStickX() + 128) * 15;



  if (PS4.R2() == 1) {
    R1Bool = 0;
  }


  if (PS4.R1() == 1) {
    R1Bool = 1;
  }

  // Update variables[10] only if PS4.R1 is not pressed
  if (R1Bool == 1) {
    variables[11] = (joy2y > 2000 || joy2y < 1800) ? joy2y : (-PS4.RStickY() + 128) * 15;
  } else {
    variables[11] = 1900;
  }

  //int variables[15] = { digitalRead(Buttonpin1), digitalRead(Buttonpin2), digitalRead(Buttonpin3), digitalRead(Buttonpin4), digitalRead(Buttonpin5), digitalRead(Buttonpin6), digitalRead(Buttonpin7), digitalRead(Buttonpin8), analogRead(Joypin1x), analogRead(Joypin1y), analogRead(Joypin2x), analogRead(Joypin2y), knob1Count, knob2Count, knob3Count };
  if (!client.connected()) {
    Serial.println("Lost connection. Reconnecting...");
    connectToServer();
  } else {
    // Send each variable with a prefix
    for (int i = 0; i < 15; i++) {
      client.print("V");
      client.print(i + 1);
      client.print("=");
      client.print(variables[i]);
      client.println();
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}



void loop() {
  if (PS4.isConnected()) {
    if (PS4.Battery() <= 2) {
      setColorAll(strip.Color(255, 75, 255));
    }
  }
  send_variables();
  connectToServer();
}