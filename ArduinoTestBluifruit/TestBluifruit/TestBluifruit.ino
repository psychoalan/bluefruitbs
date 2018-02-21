#include <Adafruit_ATParser.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <time.h>

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

int led = 9; // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
int time1 = 60;
int timeleft = 0;
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
  ble.println("AT+GAPDEVNAME=PLEASEGODHELPME"); // named LONE

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
  
  //Serial.print("Led On");
  //digitalWrite(led, HIGH);
  //delay(1000);
  //Serial.print("Led Off");
  //digitalWrite(led, LOW);
  int a = 1;
  
  char n, inputs[BUFSIZE + 1];
  char ble_data[50];
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
  //Serial.print("I am hier ");
  if (ble.available()) {
    Serial.print("* "); Serial.print(ble.available()); Serial.println(F(" bytes available from BTLE"));
  }
  // Echo received data
  //while
  if ( ble.available() )
  {
    int i = 0;
    while (ble.available()) {
      //Serial.println("Read one byte");
      c = ble.read();
      ble_data[i++] = c;
    }
    ble_data[i] = 0;
    Serial.println(ble_data);
    //Serial.print(c);
    //1 in html is 49
    //Serial.println(c);
    if(c==49){
    
      //Serial.print("c=1");
      digitalWrite(led, HIGH);
      delay(400);
    } else if(c==48)
    {
      digitalWrite(led, LOW);
    }
//    else if(c==50){
//      for(int i=0;i<1000;i++){
//        analogWrite(led, brightness);
//        brightness = brightness + fadeAmount;
//        
//        if (brightness <= 0 || brightness >= 255) {
//          fadeAmount = -fadeAmount;
//        }
//      }
//      // wait for 30 milliseconds to see the dimming effect
//      delay(50);  
//     }
     else if(c==50){
      timeleft = time1;
      do{
      Serial.println("I'm here "); 
      Serial.println("");
      //ble.read();
      timeleft--;
      
      delay(1000);
      Serial.println("time: ");
      Serial.println(time1);
      Serial.println("timeleft: ");
      Serial.println(timeleft);
      
      //int timeleft = time - 1;
      if (timeleft <= 0){
        for(int i=0;i<100;i++){
          analogWrite(led, brightness);
          brightness++;
          delay(100);
        }
      }
      
      //int a = 0;
      }while(c==50);
  } 
  //delay(1000);
}}
