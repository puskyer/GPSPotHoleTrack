// Alpha code for GPS modules for pothole tracking.
//
//  August 15th 2016
//

/*
for NMEA 0183 version 3.00 active the Mode indicator field is added

     $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a,m*hh
     
Field #
1    = UTC time of fix
2    = Data status (A=Valid position, V=navigation receiver warning)
3    = Latitude of fix
4    = N or S of longitude
5    = Longitude of fix
6    = E or W of longitude
7    = Speed over ground in knots
8    = Track made good in degrees True
9    = UTC date of fix
10   = Magnetic variation degrees (Easterly var. subtracts from true course)
11   = E or W of magnetic variation
12   = Mode indicator, (A=Autonomous, D=Differential, E=Estimated, N=Data not valid)
13   = Checksum

example 

$GPRMC,171033.800,A,4503.0912,N,07523.7578,W,0.05,82.23,260716,,,A*4F

const byte GPGGA_string_start=1;
const byte GPGGA_string_end=6;
const byte GPGGA_time_start=8;
const byte GPGGA_time_end=17;
const byte GPGGA_latitude_start=18;
const byte GPGGA_latitude_end=19;
const byte GPGGA_latitude_M_start=20;
const byte GPGGA_latitude_M_end=25;
const byte GPGGA_latitude_hemisphere=27;
const byte GPGGA_longitude_start=29;
const byte GPGGA_longitude_end=37;
const byte GPGGA_longitude_hemisphere=39;
const byte GPGGA_GPS_Quality=41;

Courtesy of Brian McClure, N8PQI.

Global Positioning System Fix Data. Time, position and fix related data for a GPS receiver.

eg2. $GPGGA,hhmmss.ss,ddmm.mmm,a,dddmm.mmm,b,q,xx,p.p,a.b,M,c.d,M,x.x,nnnn

hhmmss.ss = UTC of position 
ddmm.mmm = latitude of position
a = N or S, latitutde hemisphere
dddmm.mmm = longitude of position
b = E or W, longitude hemisphere 
q = GPS Quality indicator (0=No fix, 1=Non-differential GPS fix, 2=Differential GPS fix, 6=Estimated fix) 
xx = number of satellites in use 
p.p = horizontal dilution of precision 
a.b = Antenna altitude above mean-sea-level
M = units of antenna altitude, meters 
c.d = Geoidal height
M = units of geoidal height, meters 
x.x = Age of Differential GPS data (seconds since last valid RTCM transmission) 
nnnn = Differential reference station ID, 0000 to 1023 

example 

$GPGGA,171033.800,4503.0912,N,07523.7578,W,1,06,1.40,52.2,M,-33.9,M,,*65
$GPGGA,023900.000,4503.1057,N,07523.7231,W,1,07,1.65,80.6,M,-33.9,M,,*6C

 * 30 51.8007N, 100 35.9989W
 * The challenge however is that if you want to log data from your GPS and then show a track of where you have been, you will have to create a KML or KMZ file.
 * These files are even more particular about the format of the coordinate data. 
 * These files want both latitude and longitude as decimal numbers, so we must convert degrees, minutes to decimal degrees. 
 * You can do this simply by realizing that there are sixty minutes in a degree. 
 * So, for the case above, 51.8007 Minutes = 51.8007/60 degrees = .86335 degrees. 
 * So, to make a KML file that Google Earth will like, 30 51.8007N should be converted to 30.86335. 
 * We still have to deal with the N and W. KML files do not know what to do with the N, S, E, W. So, you must do the conversion. 
 * On Latitude, if you have a N, leave Latitude positive, if you have a S make your Latitude negative. 
 * On Longitude, if you have an E leave your number positive. If you have a W make your Longitude negative. Following these rules:
 * - See more at: http://www.toptechboy.com/arduino/lesson-24-understanding-gps-nmea-sentences/#sthash.QZVvBbk4.dpuf
 */


#include <Adafruit_GPS.h>
// #include <SoftwareSerial.h>
#include <Wire.h>    
#include "Adafruit_FRAM_I2C.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

/* Example code for the Adafruit I2C FRAM breakout */

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 5.0V DC
   Connect GROUND to common ground */
   
Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
uint16_t          framAddr = 0;

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#if !defined(ARDUINO_ARCH_SAM) && !defined(ARDUINO_ARCH_SAMD) && !defined(ESP8266) && !defined(ARDUINO_ARCH_STM32F2)
 #include <util/delay.h>
#endif
 
#define disk1 0x50        //Address of 24C02 eeprom chip
#define eepromSize 255   //24C02 eeprom chip is 256 bytes or 32 pages  x 8 bytes or 2K bits
#define eepromreserver 10 // reserve 6 bytes for our own use..
#define GPSpageSize 15  // number of bytes we will use for data structure 

byte Weepromaddress = 0;   // points to the address being used
byte Reepromaddress = 0;   // points to the address being used
byte eepromptr = 0;       // points to the next availble address
byte eepromLastAddress = 0;  //points to last address written to.

// turn on GGA only
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

// constants won't change. They're used here to
// set pin numbers:
const byte First_buttonPin = 6;     // the number of the pushbutton pin
const byte First_ledPin = 8;      // the number of the LED pin
const byte Second_buttonPin = 12;     // the number of the pushbutton pin
const byte Second_ledPin = 9;      // the number of the LED pin
//const byte Third_buttonPin = 9;     // the number of the pushbutton pin
//const byte Third_ledPin = 13;      // the number of the LED pin

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      1

// variables will change:
boolean  First_buttonState = false;     // variable for first pushbutton status
boolean Second_buttonState = false;     // variable for reading the pushbutton status
//boolean Third_buttonState = false;      // variable for reading the pushbutton status

unsigned long StartButtonPress = 0;          // start of button press time
unsigned long StopButtonPress = 0;           // end of button press time
unsigned long First_buttonPressDelay = 0;

String inputString;         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

typedef struct {
    byte SysHour;
    byte SysMinutes;
    byte SysSeconds;
    byte SysDay;
    byte SysMonth;
    byte SysYear;
    byte LastAddress[4];
       
} SYSdata ;

typedef struct {
    byte GPSHour;
    byte GPSMinutes;
    byte GPSSeconds;
    byte GPSDay;
    byte GPSMonth;
    byte GPSYear;
    byte LATArrayBytes[4];
    byte LONArrayBytes[4];
   
} GPSdata ;

byte GPSdataCRC;

union LATdata {
  float GPSLatitudeDegrees;
  byte LATArrayOfFourBytes[4];
};

union LONdata {
  float GPSLongitudeDegrees;
  byte LONArrayOfFourBytes[4];
};


// to Compute the MODBUS RTU CRC
//#define UInt16 uint16_t
// CRC should be DD18
// char *t = (char *)"DEADBEEF";

// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3
// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
// SoftwareSerial mySerial(3, 2);
// Adafruit_GPS GPS(&mySerial);

Adafruit_GPS GPS(&Serial1);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = true;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

/*************************************************** 
  This is a library for our I2C LED Backpacks

  Designed specifically to work with the Adafruit 16x8 LED Matrix backpacks 
  ----> http://www.adafruit.com/products/2035
  ----> http://www.adafruit.com/products/2036
  ----> http://www.adafruit.com/products/2037
  ----> http://www.adafruit.com/products/2038
  ----> http://www.adafruit.com/products/2039
  ----> http://www.adafruit.com/products/2040
  ----> http://www.adafruit.com/products/2041
  ----> http://www.adafruit.com/products/2042
  ----> http://www.adafruit.com/products/2043
  ----> http://www.adafruit.com/products/2044
  ----> http://www.adafruit.com/products/2052

  These displays use I2C to communicate, 2 pins are required to 
  interface. There are multiple selectable I2C addresses. For backpacks
  with 2 Address Select pins: 0x70, 0x71, 0x72 or 0x73. For backpacks
  with 3 Address Select pins: 0x70 thru 0x77

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_LEDBackpack.h"
#include "Adafruit_GFX.h"

Adafruit_8x16matrix matrix = Adafruit_8x16matrix();

static const uint8_t PROGMEM
  smile_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10100101,
    B10011001,
    B01000010,
    B00111100 },
  neutral_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10111101,
    B10000001,
    B01000010,
    B00111100 },
  frown_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10011001,
    B10100101,
    B01000010,
    B00111100 };

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, First_ledPin, NEO_GRB + NEO_KHZ800);

void setup()  
{

  matrix.begin(0x70);  // pass in the address
  pixels.begin(); // This initializes the NeoPixel library.


// reserve 10 bytes for the inputString:
  inputString.reserve(5);

// initialize the LED pin as an output:
  pinMode(First_ledPin, OUTPUT);
  pinMode(Second_ledPin, OUTPUT);  
//  pinMode(Third_ledPin, OUTPUT);  
    
// initialize the pushbutton pin as an input:
  pinMode(First_buttonPin, INPUT_PULLUP);
  pinMode(Second_buttonPin, INPUT_PULLUP);
//  pinMode(Third_buttonPin, INPUT);
  
// connect at 115200 so we can read the GPS fast enough and echo without dropping chars
// also spit it out
// Wait for Mo usb port to switch from bootloader to Serial on the MO ****** CRITICAL TO DO THIS *********

  while ( ! Serial ) { delay ( 1 ); }
  Serial.begin(115200);

// set up for eeprom 
//  Wire.begin();  

// read the eeprom to see if this is a restart or if it is the first time
// if sysdata.LastAddress[0] is zero then this is a first time if a non-zero then it is a restart
/*  
  eepromLastAddress = fram.read8((eepromSize-eepromreserver)+7);
  if ( eepromLastAddress != 0 ) {
  Weepromaddress = eepromLastAddress ;
  } else {
  eepromLastAddress = Weepromaddress ;   
  }
*/

  Serial.println("Gather Location!");
  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGA);
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);

//  delay(500);

// Ask for firmware version
Serial.println(Serial.println(PMTK_Q_RELEASE));

/* 

 if (GPS.LOCUS_ReadStatus()) {
  
      if (GPS.LOCUS_status) digitalWrite(First_ledPin, HIGH);  // turn LED on:
      if (GPS.LOCUS_status) digitalWrite(First_ledPin, LOW);  // turn LED off
 }
 */

//  void writeEEPROM(byte, byte, byte);
//  byte readEEPROM(int, int);
  boolean W_Date2eeprom();
  int R_DataFeeprom();
  byte i2ccrc8(byte *, byte);
  pixels.show(); // This sends the updated pixel color to the hardware.
  

}

void loop()                     // run over and over again
{

  // check if the pushbutton is pressed.
  // if it is, the buttonState is true:

  if (First_buttonState) {

          // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      pixels.setPixelColor(NUMPIXELS, pixels.Color(0,150,0)); // Moderately bright green color.
      pixels.show(); // This sends the updated pixel color to the hardware.
      Serial.println("button 1");
      //W_Date2eeprom();
      First_buttonState = false;
      
  } else if (Second_buttonState) {
         // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      pixels.setPixelColor(NUMPIXELS, pixels.Color(0,100,0)); // Moderately bright green color.
      pixels.show(); // This sends the updated pixel color to the hardware.

      Second_buttonState = false;
      
 //     matrix.clear();
 //     matrix.drawBitmap(0, 0, smile_bmp, 8, 8, LED_ON);
 //     matrix.writeDisplay();
 //     delay(250);
 //     matrix.clear();
 //     matrix.drawBitmap(8, 8, frown_bmp, 8, 16, LED_ON);
 //     matrix.writeDisplay();
      

  } else if (stringComplete) {
    Serial.print(inputString);
    if  ( inputString == "E" )
    {
      Serial.println("Erase Command Recieved! This will ERASE the data log stored in the FLASH - Permanently!");
      Serial.println("\nERASING!");
      GPS.sendCommand(PMTK_LOCUS_ERASE_FLASH);
      Serial.println("Erased");
    } else if ( inputString == "D" ) {
      
      R_DataFeeprom();
            
    } else if ( inputString == "R" ) {

        // the following will zero out the eeprom
        for (byte i=0; i < eepromSize ; i++) {
          Serial.print("Writing 0 to Address ");
          Serial.println(i, HEX);
          fram.write8(disk1,i);
          Serial.print("Reading Address ");
          Serial.println(i, HEX);
          Serial.print(fram.read8(i), HEX);
        }

    } else if ( inputString == "S" ) {
      if (GPS.LOCUS_ReadStatus()) {
       Serial.print("\n\nLog #"); 
       Serial.print(GPS.LOCUS_serial, DEC);
       if (GPS.LOCUS_type == LOCUS_OVERLAP)
          Serial.print(", Overlap, ");
       else if (GPS.LOCUS_type == LOCUS_FULLSTOP)
          Serial.print(", Full Stop, Logging");
       
       if (GPS.LOCUS_mode & 0x1) Serial.print(" AlwaysLocate");
       if (GPS.LOCUS_mode & 0x2) Serial.print(" FixOnly");
       if (GPS.LOCUS_mode & 0x4) Serial.print(" Normal");
       if (GPS.LOCUS_mode & 0x8) Serial.print(" Interval");
       if (GPS.LOCUS_mode & 0x10) Serial.print(" Distance");
       if (GPS.LOCUS_mode & 0x20) Serial.print(" Speed");
        
       Serial.print(", Content "); Serial.print((int)GPS.LOCUS_config);
       Serial.print(", Interval "); Serial.print((int)GPS.LOCUS_interval);
       Serial.print(" sec, Distance "); Serial.print((int)GPS.LOCUS_distance);
       Serial.print(" m, Speed "); Serial.print((int)GPS.LOCUS_speed);
       Serial.print(" m/s, Status "); 
       if (GPS.LOCUS_status) 
          Serial.print("LOGGING, ");
       else 
          Serial.print("OFF, ");
       Serial.print((int)GPS.LOCUS_records); Serial.print(" Records, ");
       Serial.print((int)GPS.LOCUS_percent); Serial.print("% Used "); 
       Serial.println();
       
//       digitalWrite(Second_ledPin, LOW);   
      }       
    } else if ( inputString == "STL" ) {
        if (GPS.LOCUS_ReadStatus()) {
              if (GPS.LOCUS_status)
              {
                   Serial.println("Already Running");  
              }
              else
              {
                  GPS.LOCUS_StartLogger();
                  // turn LED on:
//                  digitalWrite(First_ledPin, HIGH);
                  Serial.println(" STARTED!");

              }
        }      
        else
       {

                 Serial.println(" no response :("); 
       }
     } else if ( inputString == "SL" ) {
       if (GPS.LOCUS_ReadStatus()) {
               if (GPS.LOCUS_status)
              {
                  GPS.LOCUS_StopLogger();
                  // turn LED off:
//                  digitalWrite(First_ledPin, LOW);
                  Serial.println("Log Stopped!");
              }
              else
              {
                 Serial.println("\nAlready Stopped.");
              }         
        } 
        else
       {

                 Serial.println(" no response :("); 
       }
    } else {
      Serial.print("Unknown command! -");
      Serial.println(inputString);
      Serial.println();
      Serial.println("commands are! D (Download)");
      Serial.println("              E (Erase)");
      Serial.println("              R (Reset EEprom)");
      Serial.println("              E (Erase Internal DataLogger Data)");      
      Serial.println("              S (Status)");
      Serial.println("              STL (Start Internal DataLogger)");
      Serial.println("              SL (Stop Internal DataLogger)");
      
    }
          // clear the string and the flag...
    inputString = "";
    stringComplete = false; 
  }  

 //   digitalWrite(First_ledPin, HIGH);
 //   delay(1000);
 //   digitalWrite(First_ledPin, LOW);

}

/******************************************************************/
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
//  usingInterrupt = false;
  char c = GPS.read();

  // if you want to debug, this is a good time to do it!
  if (GPSECHO && c) {
     if (c) Serial.print(c);  
     
//#ifdef UDR0
//   UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
//#endif
  }
  if (digitalRead(First_buttonPin) == LOW) {
      First_buttonState = true;
  } else if (digitalRead(Second_buttonPin) == LOW) {
      Second_buttonState = true;
  }
//  usingInterrupt = true;
}


void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


void writeEEPROM(byte deviceaddress, byte eeaddress, byte data) 
{

  Wire.beginTransmission(deviceaddress);
  Wire.write(eeaddress);
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
}

byte readEEPROM(int deviceaddress, int eeaddress ) 
{
  byte rdata = 0xFF;
 
  Wire.beginTransmission(deviceaddress);
  Wire.write(eeaddress); 
  Wire.endTransmission();
 
  Wire.requestFrom(deviceaddress,1);
 
  if (Wire.available()) rdata = Wire.read();

  delay(5);
  return rdata;
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } else {
    // add it to the inputString:
    inputString += inChar;
    }
  }
  Serial.println(inputString); 
}

/*
 From: http://www.ccontrolsys.com/w/How_to_Compute_the_Modbus_RTU_Message_CRC
 
https://www.lammertbies.nl/comm/info/crc-calculation.html
Using the 8 character ASCII input DEADBEEF (upper case)
the checksum is 0xDD18
The code below agrees with the online calculator here:
http://www.lammertbies.nl/comm/info/crc-calculation.html

*/

/*
//#define UInt16 uint16_t
// CRC should be DD18
//char *t = (char *)"DEADBEEF";
// Compute the MODBUS RTU CRC
byte ModRTU_CRC(byte * buf, byte len)
{
  byte crc = 0xFF;
 
  for (byte pos = 0; pos < len; pos++) {
    crc ^= (byte)buf[pos];          // XOR byte into least sig. byte of crc
 
    for (byte i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x01) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0x01;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;  
}

*/

  
// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"

/*
uint8_t 1-wirecrc8( uint8_t *addr, uint8_t len)
{
     uint8_t crc=0;
     
     for (uint8_t i=0; i<len;i++) 
     {
           uint8_t inbyte = addr[i];
           for (uint8_t j=0;j<8;j++) 
           {
                 uint8_t mix = (crc ^ inbyte) & 0x01;
                 crc >>= 1;
                 if (mix) 
                       crc ^= 0x8C;
                 
                 inbyte >>= 1;
           }
     }
     return crc;
}

*/

// calculate 8-bit CRC
byte i2ccrc8 (byte* addr, byte len)
{
  byte crc = 0;
  while (len--) 
    {
    byte inbyte = *addr++;
    for (byte i = 8; i; i--)
      {
      byte mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) 
        crc ^= 0x8C;
      inbyte >>= 1;
      }  // end of for
    }  // end of while
  return crc;
}  // end of crc8


extern "C" char *sbrk(int i);

int FreeRam () {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}


boolean W_Date2eeprom() {

  union LATdata latdata;
  union LONdata londata;
  GPSdata gpsdata ;
  SYSdata sysdata ;
  
    // turn LED on:
    digitalWrite(First_ledPin, HIGH);
    // delay(250);  


       //while (!GPS.parse(GPS.lastNMEA())) { Serial.println("Missed GPS Last NMEA, will try again"); }  // this also sets the newNMEAreceived() flag to false
            Serial.println("Writting to EEPROM!");
            
            gpsdata.GPSHour = GPS.hour;
            gpsdata.GPSMinutes = GPS.minute;
            gpsdata.GPSSeconds = GPS.seconds;
            gpsdata.GPSDay = GPS.day;
            gpsdata.GPSMonth = GPS.month;
            gpsdata.GPSYear = GPS.year;
            latdata.GPSLatitudeDegrees = GPS.latitudeDegrees;
            londata.GPSLongitudeDegrees = GPS.longitudeDegrees;
            byte GPSDataCRC=i2ccrc8(&gpsdata.GPSHour,15);
            

            fram.write8(Weepromaddress+0,gpsdata.GPSHour);
            fram.write8(Weepromaddress+1,gpsdata.GPSMinutes);
            fram.write8(Weepromaddress+2,gpsdata.GPSSeconds);
            fram.write8(Weepromaddress+3,gpsdata.GPSDay);
            fram.write8(Weepromaddress+4,gpsdata.GPSMonth);
            fram.write8(Weepromaddress+5,gpsdata.GPSYear);
            fram.write8(Weepromaddress+6,latdata.LATArrayOfFourBytes[0]);
            fram.write8(Weepromaddress+7,latdata.LATArrayOfFourBytes[1]);
            fram.write8(Weepromaddress+8,latdata.LATArrayOfFourBytes[2]);
            fram.write8(Weepromaddress+9,latdata.LATArrayOfFourBytes[3]);
            fram.write8(Weepromaddress+10,londata.LONArrayOfFourBytes[0]);
            fram.write8(Weepromaddress+11,londata.LONArrayOfFourBytes[1]);
            fram.write8(Weepromaddress+12,londata.LONArrayOfFourBytes[2]);
            fram.write8(Weepromaddress+13,londata.LONArrayOfFourBytes[3]);
            fram.write8(Weepromaddress+14,GPSDataCRC);            
            
            eepromLastAddress = Weepromaddress;

            // this is incase of power failure or device failure  we know when the last eepro entry was done.
            sysdata.LastAddress[0] = eepromLastAddress;
            fram.write8((eepromSize-eepromreserver)+0,sysdata.SysHour);
            fram.write8((eepromSize-eepromreserver)+1,sysdata.SysMinutes);
            fram.write8((eepromSize-eepromreserver)+2,sysdata.SysSeconds);
            fram.write8((eepromSize-eepromreserver)+3,sysdata.SysDay);
            fram.write8((eepromSize-eepromreserver)+4,sysdata.SysMonth);
            fram.write8((eepromSize-eepromreserver)+5,sysdata.SysYear);
            fram.write8((eepromSize-eepromreserver)+6,sysdata.SysMonth);
            fram.write8((eepromSize-eepromreserver)+7,sysdata.LastAddress[0]);

            Weepromaddress +=GPSpageSize;

            digitalWrite(First_ledPin, LOW);
}

int R_DataFeeprom() {

  union LATdata latdata;
  union LONdata londata;
  GPSdata gpsdata ;
  SYSdata sysdata ;

  while  ( Reepromaddress <= eepromLastAddress ) {
                gpsdata.GPSHour=fram.read8(Reepromaddress+0);
                gpsdata.GPSMinutes=fram.read8(Reepromaddress+1);
                gpsdata.GPSSeconds=fram.read8(Reepromaddress+2);
                gpsdata.GPSDay=fram.read8(Reepromaddress+3);
                gpsdata.GPSMonth=fram.read8(Reepromaddress+4);
                gpsdata.GPSYear=fram.read8(Reepromaddress+5);
                
                latdata.LATArrayOfFourBytes[0]=fram.read8(Reepromaddress+6);
                latdata.LATArrayOfFourBytes[1]=fram.read8(Reepromaddress+7);
                latdata.LATArrayOfFourBytes[2]=fram.read8(Reepromaddress+8);
                latdata.LATArrayOfFourBytes[3]=fram.read8(Reepromaddress+9);
                londata.LONArrayOfFourBytes[0]=fram.read8(Reepromaddress+10);
                londata.LONArrayOfFourBytes[1]=fram.read8(Reepromaddress+11);
                londata.LONArrayOfFourBytes[2]=fram.read8(Reepromaddress+12);
                londata.LONArrayOfFourBytes[3]=fram.read8(Reepromaddress+13);
                byte GPSDataCRC=fram.read8(Reepromaddress+14);

    
                Serial.println("Hours;Minutes;Seconds;Day;Moth;Year;LatitudeDegrees;GPSLongitudeDegrees;CRC");
                Serial.print(gpsdata.GPSHour);
                Serial.print(";");
                Serial.print(gpsdata.GPSMinutes);
                Serial.print(";");
                Serial.print(gpsdata.GPSSeconds);
                Serial.print(";");
                Serial.print(gpsdata.GPSDay);
                Serial.print(";");
                Serial.print(gpsdata.GPSMonth);
                Serial.print(";");
                Serial.print(gpsdata.GPSYear);
                Serial.print(";");
                Serial.print(latdata.GPSLatitudeDegrees);
                Serial.print(";");
                Serial.println(londata.GPSLongitudeDegrees);
                Serial.print(";");
                Serial.println(GPSDataCRC, HEX);  
                Serial.print("Calculated CrC is:  ");              
                Serial.println(i2ccrc8(&gpsdata.GPSHour,15),HEX);               

                Reepromaddress +=GPSpageSize;
   }
}            
