#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_TSL2561_U.h>
#include <pgmspace.h>
#include "RTClib.h"

/*
 Debounce

 Each time the input pin goes from LOW to HIGH (e.g. because of a push-button
 press), the output pin is toggled from LOW to HIGH or HIGH to LOW.  There's
 a minimum delay between toggles to debounce the circuit (i.e. to ignore
 noise).

 The circuit:
 * LED attached from pin 13 to ground
 * pushbutton attached from pin 2 to +5V
 * 10K resistor attached from pin 2 to ground

 * Note: On most Arduino boards, there is already an LED on the board
 connected to pin 13, so you don't need any extra components for this example.

 */
 Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
 RTC_DS1307 rtc;
 
#include <SoftwareSerial.h>
// constants won't change. They're used here to
// set pin numbers:
const int buttonPin = 9;    // the number of the pushbutton pin
const int ledPin = 13;      // the number of the LED pin
const int relayPin = 10;
const int lightTriggerPin = 11;
const int buttonTriggerPin = 8;
const int shutDownPin = 12;

bool fridgeOpened = false;
  int openCounter = 0;
  int closeCounter = 0;
bool shutDownReceived = false;

DateTime startTime;

// Variables will change:
int ledState = LOW;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
  long randNumber;
  
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(relayPin,OUTPUT);
  pinMode(buttonTriggerPin, OUTPUT);
  pinMode(lightTriggerPin, OUTPUT);
  pinMode(shutDownPin, INPUT);
  Serial.begin(9600);

  

  digitalWrite(relayPin,LOW);
  digitalWrite(buttonTriggerPin, LOW);
  digitalWrite(lightTriggerPin, LOW);
  digitalWrite(shutDownPin, LOW);
  
  // set initial LED state
  digitalWrite(ledPin, ledState);

  Serial.begin(57600);
  Serial.println("Light Sensor Test"); Serial.println("");
  
  
  /* Initialise the sensor */
  if(!tsl.begin())
  {
  /* There was a problem detecting the TSL2561 ... check your connections */
  Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("");
  
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  startTime = rtc.now();
  
  
}

void loop() {
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the relay if the new button state is HIGH
      if (buttonState == HIGH) {
        digitalWrite(relayPin,HIGH);
        digitalWrite(buttonTriggerPin,HIGH);
        digitalWrite(lightTriggerPin,LOW);
        Serial.println("Button pressed");
      }
    }
  }

  lastButtonState = reading;
  
  /* Get a new sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);
  DateTime now = rtc.now();

  //after testing make this 4 hours 
  if(now.unixtime() - startTime.unixtime() >= 900)
  {
      digitalWrite(relayPin,HIGH);
      digitalWrite(lightTriggerPin,HIGH);
      digitalWrite(buttonTriggerPin,HIGH);
      startTime = now;
  }

  /* Display the results (light is measured in lux) */
  if (event.light)
  {
    Serial.print(event.light); Serial.println(" lux"); 
    if(event.light > 60)
    {
      //open
      closeCounter =  0;
      openCounter++;
      if(openCounter > 5) {
        fridgeOpened = true; 
        Serial.println("Fridge open long enough. fridgeOpened = true");
      }
      if (openCounter > 100000) {
        openCounter = 5;
      }
    }
    if(event.light < 55)
    {
      //close 
      openCounter = 0;
      closeCounter++;
      //after testing make this 150
      if(closeCounter > 30 && fridgeOpened){
        //turn on pi!
        digitalWrite(relayPin,HIGH);
        digitalWrite(lightTriggerPin,HIGH);
        digitalWrite(buttonTriggerPin,LOW);
        fridgeOpened = false;
        Serial.println("Light trigger conditions satisfied. Turning on pi");
      }
      if(closeCounter > 1000000) {
        closeCounter = 30; 
      }
      
    }
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }
  
  if(digitalRead(shutDownPin)==0 && shutDownReceived)
  {
    digitalWrite(relayPin,LOW);
    shutDownReceived = false;
    digitalWrite(lightTriggerPin,LOW);
    digitalWrite(buttonTriggerPin,LOW);
    Serial.println("Shutting down pi.");
  }
  
  if(digitalRead(shutDownPin)==1)
  {
    shutDownReceived = true;
    Serial.println("Shutdown received.");
  }

  
  delay(250);


}
