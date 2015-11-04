# OpenCM #

> OpenCM9.04 is a microcontroller board based on 32bit ARM Cortex-M3. The
> OpenCM9.04’s schematics and source codes are open-source.

 - [Robotis
   Website](http://support.robotis.com/en/product/auxdevice/controller/opencm9.04.htm)

The OpenCM is used in the OARK to provide communication between the Raspberry
Pi and the AX12 Dynamixel servomotors. The OpenCM is cheap and reliable,
making it great for the OARK.

## Software ##

The OpenCM is an Arduino-compatible platform. As such, the software for the
OpenCM looks very much like software written for an Arduino device. A 'sketch'
is a piece of software that can be run on an Arduino (or OpenCM) device.  As
an embedded system, the code written for the OpenCM is the *only* code running
on the device.

While the OpenCM can be programmed on its own to usefully control AX12 motors,
the OARK only uses the OpenCM to relay messages from the Raspberry Pi. This is
done because

 *   Pushing this functionality higher up in the system makes it more flexible
    and reliable. 

 *   The software on the Pi that synthesises and handles raw AX12 messages is a
ROS package created by Antons Rebguns. Any updates to this package will
indirectly benefit the OARK. If we were to duplicate any of this functionality
ourselves, then it would be hard to benefit from these updates.

 *   Software on the Raspberry Pi can be written in higher level languages and
    it is usually much easier to test.

The OARK sketch built for the OpenCM is very simple, as it just has to relay
messages. The sketch itself is shown below. It can also be found in the OARK
repository in the opencm directory.

    /*
     *  Whatever it receives from the Serial2 (usually commands and queries) 
     *  is sent to the Dynamixel bus, and what it receives from the Dynamixel bus
     *  is sent to the Serial2 (usually answers). Essentially this code makes the
     *  OpenCM work like a USB2Dynamixel but with the OpenCM serial2 port instead.
     *
     *  This code is a modified version of the Dxl_Tosser example program provided
        with the ROBOTIS OpenCM IDE.
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


This sketch is a slightly modified version of the DxlTosser example sketch
that comes included with the Robotis OpenCM IDE v1.0.2. Essentially this
sketch causes the OpenCM board to act like the USB2Dynamixel. It forwards
messages from the Serial2 port to the Dynamixel bus and vice versa. Notably,
the serial port must uses a lower baud rate because whatever is talking to it
probably only supports lower baud rates. This particular sketch uses 115200
baud for the serial port. This means that the Raspberry Pi serial port must
also be set to communicate at 115200 baud.

The code is simple to follow if the reader is at all familiar with Arduino
programming. If there are bytes available on the Dynamixel bus, then they will
be read and sent out on the second serial port. If there are bytes available
on the serial port, then they will be read and sent out on the Dynamixel bus.

The blinking LED is not strictly necessary here, but has frequently proved to
be invaluable for debugging purposes.


## SETUP ##

To setup the OpenCM once it is wired in, one need only flash the above sketch
to its memory. 

Flashing the OpenCM:

1.  Navigate to the [OpenCM section of the Robotis Website](http://support.robotis.com/en/software/robotis_opencm.htm) and
download the appropriate Robotis OpenCM Integrated Development Environment for
your operating system.

2.  Obtain the OARK OpenCM sketch. This can be done by saving the code above
    into a file, or downloading it from the OARK repository. Using the
repository is recommended as a more robust and reliable method of obtaining
the sketch.

3.  Install/Run the software and open the saved sketch. Flash the sketch on to
    the OpenCM by following the instructions at the Robotis Website linked in
step 1.
