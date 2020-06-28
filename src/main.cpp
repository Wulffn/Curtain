#include <Arduino.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>
#include <Stepper.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

enum State_enum {CALIBRATING, READY};

#define MQTT_SERV "io.adafruit.com"
#define MQTT_PORT 1883
#define MQTT_NAME "wulffn"
#define MQTT_PASS "aio_tPil960uLO81gwJO0uXm9vrtfr8k"

const uint32_t deviceId = ESP.getChipId();
const int stepsPerRevolution = 2048;
int buttonPinUp = D3;
int buttonPinDown = D2;
int stepperPosition = 0;
int maxPosition = 0;
int minPosition = 0;
int mqttPositions[20];

uint8_t state;


Stepper myStepper = Stepper(stepsPerRevolution, D5, D7, D6, D8);

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERV, MQTT_PORT, MQTT_NAME, MQTT_PASS);

Adafruit_MQTT_Subscribe curtain = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/Curtain");

void runStepper(int, int);
void checkButtonClicks();
void checkMqtt();
void MQTT_connect();
void stateMachine();
void calibrate();
void blinkPattern(int, long);
void setMqttPositions(int, int);

void setup() {
    Serial.begin(9600);
    WiFiManager wifiManager;
    wifiManager.autoConnect("AutoCurtain");
    Serial.println("Connected to WiFi as AutoCurtain");
    mqtt.subscribe(&curtain);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(buttonPinUp, INPUT_PULLUP);
    pinMode(buttonPinDown, INPUT_PULLUP);
    myStepper.setSpeed(15);
    state = CALIBRATING;
}

void loop() {
  stateMachine();
}

void stateMachine() {
  switch(state) {
    case CALIBRATING:
      calibrate();
      break;
    case READY:
      //checkButtonClicks();
      checkMqtt();
      break;
  }
}

void setMqttPositions(int min, int max) {
  Serial.print("min: ");
  Serial.println(min);
    Serial.print("max: ");
  Serial.println(max);
  int stepsPrIteration = min / 19;
  mqttPositions[0] = 0;
  Serial.print("pos 0: ");
  Serial.println(mqttPositions[0]);
  for (int i = 1; i < 20; i++)
  {
    mqttPositions[i] = (mqttPositions[i-1] + stepsPrIteration);
    Serial.print("pos ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(mqttPositions[i]);
  }
  
}

void calibrate() {
  Serial.println("calibrating..");
  blinkPattern(5, 200);

  bool maxCalibrated = false;
  bool minCalibrated = false;
    while(!maxCalibrated) {
      if(digitalRead(buttonPinUp) == LOW && digitalRead(buttonPinDown) == LOW) {
        maxPosition = 0;
        stepperPosition = 0;
        maxCalibrated = true;
      } else if(digitalRead(buttonPinUp) == LOW) {
        runStepper(0, 20);
      } else if(digitalRead(buttonPinDown) == LOW) {
        //runStepper(1, 20);
      }
      yield();
    }

    blinkPattern(3, 200);

    while(!minCalibrated) {
      if(digitalRead(buttonPinUp) == LOW && digitalRead(buttonPinDown) == LOW) {
        if(stepperPosition > maxPosition ) {
        minPosition = stepperPosition;
        minCalibrated = true;
        }
      } else if(digitalRead(buttonPinDown) == LOW) {
        runStepper(1, 20);
       }else if(digitalRead(buttonPinUp) == LOW) {
        runStepper(0, 20);
      }
      yield();
    }

    blinkPattern(3, 200);

    while(stepperPosition >= maxPosition) {
      runStepper(0, 20);
      yield();
    }

    setMqttPositions(minPosition, maxPosition);

    blinkPattern(10, 100);

    state = READY;
}

void blinkPattern(int numOfTimes, long de) {
  for (int i = 0; i < numOfTimes; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(de);
    digitalWrite(LED_BUILTIN, LOW);
    delay(de);
  }
}


void checkMqtt() {
   MQTT_connect();

  //Read from our subscription queue until we run out, or
  //wait up to 5 seconds for subscription to update
  Adafruit_MQTT_Subscribe * subscription;
  while ((subscription = mqtt.readSubscription(5000)))
  {
    //If we're in here, a subscription updated...
    if (subscription == &curtain)
    {
      //Get the percentage from the subscription
      int mqttReading = atoi((const char*) curtain.lastread);
      if(mqttReading == 0) {
        if(stepperPosition != 0) {
          while(stepperPosition >= 0) {
            runStepper(0, 5);
            yield();
          }
        }
      } else {
          uint8_t index = mqttReading / 5;
          if(mqttPositions[index] < stepperPosition) {
            while(stepperPosition >= mqttPositions[index]) {
              runStepper(0, 5);
              yield();
            }
          } else if(mqttPositions[index] > stepperPosition) {
              while(stepperPosition <= mqttPositions[index]) {
                runStepper(1, 5);
                yield();
            }
          }
        }
    }
  }
}

void checkButtonClicks() {
  if(digitalRead(buttonPinUp) == LOW && digitalRead(buttonPinDown) == LOW) {
      state = CALIBRATING;
    } else {
      if(digitalRead(buttonPinUp) == LOW) {
        runStepper(0, 5);
      } else if(digitalRead(buttonPinDown) == LOW) {
        runStepper(1, 5);
      }
    }
}

void runStepper(int direction, int steps) {
  if(direction == 0) {
    stepperPosition = stepperPosition - steps;
    myStepper.step(-steps);
  } else {
    stepperPosition = stepperPosition + steps;
    myStepper.step(steps);
  }
}

void MQTT_connect() 
{
  int8_t ret;
  // Stop if already connected
  if (mqtt.connected())
  {
    return;
  }

  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) // connect will return 0 for connected
  { 
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) 
    {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

