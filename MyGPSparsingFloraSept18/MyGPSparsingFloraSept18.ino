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
#include <Time.h>
#include <TimeLib.h>

// Offset hours from gps time (UTC)
//const int offset = 1;   // Central European Time
//const int offset = -5;  // Eastern Standard Time (USA)
const int offset = -4;  // Eastern Daylight Time (USA)
//const int offset = -8;  // Pacific Standard Time (USA)
//const int offset = -7;  // Pacific Daylight Time (USA)


      
#include "Adafruit_FRAM_I2C.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#if !defined(ARDUINO_ARCH_SAM) && !defined(ARDUINO_ARCH_SAMD) && !defined(ESP8266) && !defined(ARDUINO_ARCH_STM32F2)
 #include <util/delay.h>
#endif

/* Example code for the Adafruit I2C FRAM breakout */

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 5.0V DC
   Connect GROUND to common ground */
   
Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C();
#define I2C_framAddr 0x50        //I2C Address for eeprom chip 
#define FramSize 32768  // 32K Fram

/*
Last 15 bytes are for saving state information

ReserveFramAddr+0 is sysdata.SysHour);
ReserveFramAddr+1 is sysdata.SysMinutes);
ReserveFramAddr+2 is sysdata.SysSeconds);
ReserveFramAddr+3 is sysdata.SysDay);
ReserveFramAddr+4 is sysdata.SysMonth);
ReserveFramAddr+5 is sysdata.SysYear);
ReserveFramAddr+6 is sysdata.SysMonth);
ReserveFramAddr+7 is sysdata.LastAddress[0]);
*/
#define ReserveFramAddr 15 // reserve 15 bytes for our own use..
#define GPSpageSize 15  // number of bytes we will use for data structure 

uint16_t WrFramAddr = 0;   // points to the address being used
uint16_t RdFramAddr = 0;   // points to the address being used
uint16_t NextFramAddr = 0;       // points to the next availble address
uint16_t LastFramAddr = 0;  //points to last address written to.

// turn on GGA only
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

// constants won't change. They're used here to
// set pin numbers:
const byte First_buttonPin = 9;     // the number of the pushbutton pin
const byte Second_buttonPin = 10;     // the number of the pushbutton pin
const byte NeoPixelPin = 8;      // the number of the LED pin
// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      1

// variables will change:
volatile boolean First_buttonState = false;     // variable for first pushbutton status
volatile boolean Second_buttonState = false;     // variable for reading the pushbutton status

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

SYSdata sysdata ;

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


// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

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

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, NeoPixelPin, NEO_GRB + NEO_KHZ800);

extern "C" char *sbrk(int i);

int FreeRam () {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

void First_buttonPress() {
      First_buttonState = true;
      }
  
void Second_buttonPress() {
      Second_buttonState = true;
      }



#if not defined(ARDUINO_SAMD_ZERO) && not defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
/******************************************************************/
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO && c) {
     if (c) Serial.print(c);  
  }
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

#endif



void serialEventRun(void) {
  if (Serial.available()) serialEvent();
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
}

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


//extern "C" char *sbrk(int i);

//int FreeRam () {
//  char stack_dummy = 0;
 // return &stack_dummy - sbrk(0);
//}


boolean W_Date2eeprom() {

  union LATdata latdata;
  union LONdata londata;
  GPSdata gpsdata ;
 // SYSdata sysdata ;

          //Loop until you have a good NMEA sentence
          while (!GPS.parse(GPS.lastNMEA())) {    // this also sets the newNMEAreceived() flag to false
           char c=GPS.read();
         // if you want to debug, this is a good time to do it!
            if (GPSECHO)
            if (c) Serial.print(c);
          }

            Serial.println("Start writting to EEPROM!");
            
            gpsdata.GPSHour = GPS.hour;
            gpsdata.GPSMinutes = GPS.minute;
            gpsdata.GPSSeconds = GPS.seconds;
            gpsdata.GPSDay = GPS.day;
            gpsdata.GPSMonth = GPS.month;
            gpsdata.GPSYear = GPS.year;
            latdata.GPSLatitudeDegrees = GPS.latitudeDegrees;
            londata.GPSLongitudeDegrees = GPS.longitudeDegrees;
            byte GPSDataCRC=i2ccrc8(&gpsdata.GPSHour,15);

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
                Serial.println();   

            

            fram.write8(WrFramAddr+0,gpsdata.GPSHour);
            fram.write8(WrFramAddr+1,gpsdata.GPSMinutes);
            fram.write8(WrFramAddr+2,gpsdata.GPSSeconds);
            fram.write8(WrFramAddr+3,gpsdata.GPSDay);
            fram.write8(WrFramAddr+4,gpsdata.GPSMonth);
            fram.write8(WrFramAddr+5,gpsdata.GPSYear);
            fram.write8(WrFramAddr+6,latdata.LATArrayOfFourBytes[0]);
            fram.write8(WrFramAddr+7,latdata.LATArrayOfFourBytes[1]);
            fram.write8(WrFramAddr+8,latdata.LATArrayOfFourBytes[2]);
            fram.write8(WrFramAddr+9,latdata.LATArrayOfFourBytes[3]);
            fram.write8(WrFramAddr+10,londata.LONArrayOfFourBytes[0]);
            fram.write8(WrFramAddr+11,londata.LONArrayOfFourBytes[1]);
            fram.write8(WrFramAddr+12,londata.LONArrayOfFourBytes[2]);
            fram.write8(WrFramAddr+13,londata.LONArrayOfFourBytes[3]);
            fram.write8(WrFramAddr+14,GPSDataCRC);            
            
            LastFramAddr = WrFramAddr;

            // this is incase of power failure or device failure  we know when the last eepro entry was done.
            sysdata.LastAddress[0] = LastFramAddr;
            fram.write8((FramSize-ReserveFramAddr)+0,sysdata.SysHour);
            fram.write8((FramSize-ReserveFramAddr)+1,sysdata.SysMinutes);
            fram.write8((FramSize-ReserveFramAddr)+2,sysdata.SysSeconds);
            fram.write8((FramSize-ReserveFramAddr)+3,sysdata.SysDay);
            fram.write8((FramSize-ReserveFramAddr)+4,sysdata.SysMonth);
            fram.write8((FramSize-ReserveFramAddr)+5,sysdata.SysYear);
            fram.write8((FramSize-ReserveFramAddr)+6,sysdata.SysMonth);
            fram.write8((FramSize-ReserveFramAddr)+7,sysdata.LastAddress[0]);
            fram.write8((FramSize-ReserveFramAddr)+8,sysdata.LastAddress[1]);
            fram.write8((FramSize-ReserveFramAddr)+9,sysdata.LastAddress[2]);            
            fram.write8((FramSize-ReserveFramAddr)+10,sysdata.LastAddress[3]);
            
            WrFramAddr +=GPSpageSize;
            Serial.println("Finish writting to EEPROM!");
}

void R_DataFeeprom() {

  union LATdata latdata;
  union LONdata londata;
  GPSdata gpsdata ;
//  SYSdata sysdata ;

  while  (RdFramAddr <= LastFramAddr) {
                gpsdata.GPSHour=fram.read8(RdFramAddr+0);
                gpsdata.GPSMinutes=fram.read8(RdFramAddr+1);
                gpsdata.GPSSeconds=fram.read8(RdFramAddr+2);
                gpsdata.GPSDay=fram.read8(RdFramAddr+3);
                gpsdata.GPSMonth=fram.read8(RdFramAddr+4);
                gpsdata.GPSYear=fram.read8(RdFramAddr+5);
                
                latdata.LATArrayOfFourBytes[0]=fram.read8(RdFramAddr+6);
                latdata.LATArrayOfFourBytes[1]=fram.read8(RdFramAddr+7);
                latdata.LATArrayOfFourBytes[2]=fram.read8(RdFramAddr+8);
                latdata.LATArrayOfFourBytes[3]=fram.read8(RdFramAddr+9);
                londata.LONArrayOfFourBytes[0]=fram.read8(RdFramAddr+10);
                londata.LONArrayOfFourBytes[1]=fram.read8(RdFramAddr+11);
                londata.LONArrayOfFourBytes[2]=fram.read8(RdFramAddr+12);
                londata.LONArrayOfFourBytes[3]=fram.read8(RdFramAddr+13);
                byte GPSDataCRC=fram.read8(RdFramAddr+14);

    
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

                RdFramAddr +=GPSpageSize;
   }

}


   /*
 hour();            // The hour now  (0-23)
 minute();          // The minute now (0-59)
 second();          // The second now (0-59)
 day();             // The day now (1-31)
 weekday();         // Day of the week, Sunday is day 1
 month();           // The month now (1-12)
 year();            // The full four digit year: (2009,
                    //  2010 etc)

*/

void DateTime() {
    Serial.print("The Date is ");
    Serial.print(day()); 
    Serial.print("/");
    Serial.print(month());
    Serial.print("/");
    Serial.println(year());
    Serial.print("The time is ");
    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());
    Serial.print(":");
    Serial.println(second());    
}


void setup()  
{

//  matrix.begin(0x70);  // pass in the address
  pixels.begin(); // This initializes the NeoPixel library.

//  pixels.show(); // This sends the updated pixel color to the hardware.

// reserve 10 bytes for the inputString:
  inputString.reserve(5);

// initialize the LED pin as an output:
//  pinMode(First_ledPin, OUTPUT);

// initialize the pushbutton pin as an input:
  pinMode(First_buttonPin, INPUT_PULLUP);
  pinMode(Second_buttonPin, INPUT_PULLUP);

  attachInterrupt(First_buttonPin, First_buttonPress, LOW);
  attachInterrupt(Second_buttonPin, Second_buttonPress, LOW);
  
// connect at 115200 so we can read the GPS fast enough and echo without dropping chars
// also spit it out
// Wait for Mo usb port to switch from bootloader to Serial on the MO ****** CRITICAL TO DO THIS *********

  while ( ! Serial ) { delay ( 1 ); }
  Serial.begin(115200);

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

  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);
  GPS.sendCommand(PGCMD_NOANTENNA);              //Turn off antenna update nuisance data

// Ask for firmware version
Serial.println(GPSSerial.print(PMTK_Q_RELEASE));


// read the eeprom to see if this is a restart or if it is the first time
// if sysdata.LastAddress[0] is zero then this is a first time if a non-zero then it is a restart

 fram.begin();

Serial.print("Checking EEPROM for Last Address used! ");
  sysdata.LastAddress[0] = fram.read8((FramSize-ReserveFramAddr)+7);
  sysdata.LastAddress[1] = fram.read8((FramSize-ReserveFramAddr)+8); 
  sysdata.LastAddress[2] = fram.read8((FramSize-ReserveFramAddr)+9);  
  sysdata.LastAddress[3] = fram.read8((FramSize-ReserveFramAddr)+10);  
  LastFramAddr = sysdata.LastAddress[0];
Serial.print(" LastAddress is ");  
Serial.println(LastFramAddr, DEC);
  
  if ( LastFramAddr != 0 ) {
  WrFramAddr = LastFramAddr + GPSpageSize ;
  } else {
  LastFramAddr = WrFramAddr ;   
  }

  //Loop until you have a good NMEA sentence
  while (!GPS.parse(GPS.lastNMEA())) {    // this also sets the newNMEAreceived() flag to false
   char c=GPS.read();
 // if you want to debug, this is a good time to do it!
    if (GPSECHO)
    if (c) Serial.print(c);
  }    
    setTime(GPS.hour,GPS.minute,GPS.seconds,GPS.day,GPS.month,GPS.year);
    adjustTime(offset * SECS_PER_HOUR);


  DateTime();
}


void loop()                     // run over and over again
{

  boolean W_Date2eeprom();
  int R_DataFeeprom();
  byte i2ccrc8(byte *, byte);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is true:


  if (First_buttonState) {
      delay(250);
        // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      pixels.setPixelColor(NUMPIXELS, pixels.Color(0,150,0)); // Moderately bright green color.
      pixels.show(); // This sends the updated pixel color to the hardware
      W_Date2eeprom();
      First_buttonState = false;
  } 

   /*
      0x0a (ASCII newline)
      0x0d (ASCII carriage return)
      CRLF (0x0d0a)
    */
    
char c;
  
  if (Second_buttonState) {
          Second_buttonState = false;
/*
      matrix.clear();
      matrix.drawBitmap(0, 0, smile_bmp, 8, 8, LED_ON);
      matrix.writeDisplay();
      delay(250);
      matrix.clear();
      matrix.drawBitmap(8, 8, frown_bmp, 8, 16, LED_ON);
      matrix.writeDisplay();

*/  
         // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
      pixels.setPixelColor(NUMPIXELS, pixels.Color(0,150,0)); // Moderately bright green color.
      pixels.show(); // This sends the updated pixel color to the hardware.
      delay (250);
      Serial.println("Removing last GPS Entry! ");
      WrFramAddr = LastFramAddr;
      // this is incase of power failure or device failure  we know when the last eepro entry was done.
      sysdata.LastAddress[0] = LastFramAddr;
      fram.write8((FramSize-ReserveFramAddr)+7,sysdata.LastAddress[0]);
  }
  
  if (stringComplete) {
      if ( inputString == "D" ) {
          
          R_DataFeeprom();
                
        } else if ( inputString == "R" ) {

            DateTime();
            Serial.println("First Set Saved FramAddress to 0!");
            fram.write8((FramSize-ReserveFramAddr)+7,0);
            fram.write8((FramSize-ReserveFramAddr)+8,0);
            fram.write8((FramSize-ReserveFramAddr)+9,0);
            fram.write8((FramSize-ReserveFramAddr)+10,0);
            
            DateTime();
            Serial.print("Zero out the FRAM!");          
            // the following will zero out the eeprom
            for (byte i=0; i < FramSize ; i++) {
              fram.write8(i,0);
              if (fram.read8(i) != 0) Serial.println("Memory Error!");;
            }

            DateTime();
            Serial.print("FRAM as been Erased!");
            
        } else if ( inputString == "S" ) {
    
          //Loop until you have a good NMEA sentence
          while (!GPS.parse(GPS.lastNMEA())) {    // this also sets the newNMEAreceived() flag to false
           char c=GPS.read();
         // if you want to debug, this is a good time to do it!
            if (GPSECHO)
            if (c) Serial.print(c);
          }
                Serial.println();
                Serial.print(GPS.hour);
                Serial.print(":");
                Serial.print(GPS.minute);
                Serial.print(":");            
                Serial.println(GPS.seconds);
                
                Serial.print(GPS.day);
                Serial.print("/");              
                Serial.print(GPS.month);
                Serial.print("/");              
                Serial.println(GPS.year);
                Serial.print("GPSLatitudeDegrees: ");
                Serial.println(GPS.latitudeDegrees);
                Serial.print("GPSLongitudeDegrees: ");
                Serial.println(GPS.longitudeDegrees); 
                Serial.println();
        } else {
          Serial.print("Unknown command! -");
          Serial.println(inputString);
          Serial.println();
          Serial.println("commands are! D (Download)");
          Serial.println("              R (Reset EEprom)");   
          Serial.println("              S (Status)");
        }
    // clear the string and the flag...
    inputString = "";
    stringComplete = false; 
  }  
}

