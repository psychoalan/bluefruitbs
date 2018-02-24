#include <Adafruit_ATParser.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>

/*********************************************************************
  This is an example based on nRF51822 based Bluefruit LE modules

********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

/*=========================================================================
       -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/


Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();



  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  //Give module a new name
  ble.println("AT+GAPDEVNAME=LED-LAMP"); // named LONE

  // Check response status
  ble.waitForOK();

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
  pinMode(led, OUTPUT);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Check for user input
      
  char n, inputs[BUFSIZE + 1];
  char ble_data[50];    // an array for storing data sent from the application
  int led = 9;          // the PWM pin the LED is attached to
  int brightness = 0;   // how bright the LED is
  int fadeAmount = 5;   // how many points to fade the LED by
  int fadeAmount2;      // how many points to fade the LED by
  int timeleft = 0;     // time left in seconds until the alarm goes off
  int x;
  int c;
  
  if (Serial.available())
  {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
  }
  
  if (ble.available()) {
    Serial.print("* "); 
    Serial.print(ble.available()); 
    Serial.println(F(" bytes available from BTLE"));
  }
  // Echo received data
  
  if (ble.available())
  {
    int i = 0;
    while (ble.available()) {
      //Serial.println("Read one byte");
      c = ble.read();
      ble_data[i++] = c;
    }
    ble_data[i] = 0; // reset
    
    //Serial.println("Data = ");
    //Serial.println(ble_data);
    
    //Convert Str it Int
    int timetowakeup = atoi(ble_data);
    
    x = -c;
    if(x==-49){
      digitalWrite(led, HIGH);
    } else if(x==-48) {
      digitalWrite(led, LOW);
    } else if(x==-50){
      fadeAmount2 = 51; //255:5=51
      analogWrite(led, brightness);
      brightness = brightness + fadeAmount2;
      if (brightness <= 0 || brightness >= 255) {
        fadeAmount2 = -fadeAmount2;
      }
    } else {
      timeleft = timetowakeup - 255; //lamp should go off before actual time so it has time to adjust brightness 
      do{
        delay(1000); // delay of 1 second = timeleft - 1
        timeleft--;
        Serial.println("time: "); Serial.println(timetowakeup);
        Serial.println("timeleft: "); Serial.println(timeleft);
      
        if (timeleft == 0){
          Serial.println("Time to wake up soon"); 
          for(brightness=0; brightness<=255; brightness++){ //this for loop will last roughly 4,5 min before led's brightness is set to max
            analogWrite(led, brightness);
            delay(1000);
          }
        }
      }while(timeleft>0);
    } 
  }
}
