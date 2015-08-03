/*
 *  Whatever it receives from the Serial2 (usually commands and queries) 
 *  is sent to the Dynamixel bus, and what it receives from the Dynamixel bus
 *  is sent to the Serial2 (usually answers). Essentially this code makes the
 *  OpenCM work like a USB2Dynamixel but with the OpenCM serial2 port instead.
 *
 *  This code is a modified version of the Dxl_Tosser example program provided with the ROBOTIS OpenCM IDE.
 *  Dxl_Tosser was created by aprendiendo:
 *  http://softwaresouls.com/softwaresouls/2013/06/12/robotis-cm-900-as-a-tosser-for-dynamixel-commands/
 *  ...and modified on 2014-05-22 by ROBOITIS CO,,LTD.
 *
 *  Most recent modification by Tim Peskett on 2015-05-13.
 */

  /* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

#define SERIAL2_BAUD 115200

Dynamixel Dxl(DXL_BUS_SERIAL1);


void blinkOnce()
{
  digitalWrite(BOARD_LED_PIN, LOW);
  delay_us(100);
  digitalWrite(BOARD_LED_PIN, HIGH);
}

void setup()
{  
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  Serial2.begin(SERIAL2_BAUD);
  
  pinMode(BOARD_LED_PIN, OUTPUT);
}

byte aByte=0;
uint8 aUint8;

void loop() 
{    
  if (Serial2.available())
  {
    aUint8=Serial2.read();
    blinkOnce();
    Dxl.writeRaw(aUint8);
  }   

  if (Dxl.available())
  {
    aByte=Dxl.readRaw();
    blinkOnce();
    Serial2.write(aByte);
  } 
}
