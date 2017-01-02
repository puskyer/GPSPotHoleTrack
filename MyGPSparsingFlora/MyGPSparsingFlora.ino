// Alpha code for GPS modules for pothole tracking.
//
//  December 31st 2016
//
// put together by Pasquale Riccio

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
c.d = Geoidal heightM = units of geoidal height, meters 
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
#include <Wire.h> 
#include <Time.h>
#include <TimeLib.h>
#include <fmtDouble.h>
#include <crc8.h>

//#include <EnableInterrupt.h>

// Offset hours from gps time (UTC)

//volatile int offset = -4;  // Eastern Daylight Time (USA)
//volatile int offset = -5;  // Eastern Standard Time (USA)
volatile int offset = -6;  // Centril Standard Time (USA)
//volatile int offset = -8;  // Pacific Standard Time (USA)
//volatile int offset = -7;  // Pacific Daylight Time (USA)
      
#include "Adafruit_FRAM_I2C.h"
#include <Adafruit_NeoPixel.h>

/* Adafruit I2C FRAM breakout
   Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 5.0V DC
   Connect GROUND to common ground */
   
Adafruit_FRAM_I2C fram  = Adafruit_FRAM_I2C();
#define I2C_framAddr 0x50   //  I2C Address for FRAM chip 
#define FramSize 32768      // 32K Fram

/*
Last 15 bytes are for saving state information

ReserveFramAddr+0 is sysdata.SysHour);
ReserveFramAddr+1 is sysdata.SysMinutes);
ReserveFramAddr+2 is sysdata.SysSeconds);
ReserveFramAddr+3 is sysdata.SysDay);
ReserveFramAddr+4 is sysdata.SysMonth);ReserveFramAddr+5 is sysdata.SysYear);
ReserveFramAddr+6 is sysdata.SysMonth);
ReserveFramAddr+7 is sysdata.LastAddress[0]);
*/
#define ReserveFramAddr 24 // reserve 24 bytes for our own use..
#define GPSpageSize 24  // number of bytes we will use for data structure 

uint16_t WrFramAddr = 0;   // points to the address being used
uint16_t RdFramAddr = 0;   // points to the address being used
uint16_t NextFramAddr = 0; // points to the next availble address
uint16_t LastFramAddr = 0; //points to last address written to.

// turn on GGA only
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

// constants won't change. They're used here to

// set pin numbers:
const int First_buttonPin = 9;     // the number of the pushbutton pin
const int Second_buttonPin = 10;   // the number of the pushbutton pin
const int NeoPixelPin = 6;         // the number of the LED pin
const int FLORAled = 8;            // Pin D7 has an LED connected on FLORA. give it a name:

#define NUMPIXEL  0           // How many NeoPixels are attached
boolean NoGPSFix;

// variables will change:
volatile boolean First_buttonState = false;     // variable for first pushbutton status
volatile boolean Second_buttonState = false;    // variable for reading the pushbutton status
volatile int buttonState;
unsigned long StartButtonPress = 0;             // start of button press time
unsigned long StopButtonPress = 0;              // end of button press time
unsigned long First_buttonPressDelay = 0;

typedef struct {
    byte SysHour;
    byte SysMinutes;
    byte SysSeconds;
    byte SysDay;
    byte SysMonth;
    byte SysYear;
    byte LastAddress[4];
       
} SYSdata ;

int LangthGPSdata = 22;

char GPSStringLAT[16];
char GPSStringLON[16];


typedef struct {
    byte GPSHour;
    byte GPSMinutes;
    byte GPSSeconds;
    byte GPSDay;
    byte GPSMonth;
    byte GPSYear;
    byte LATArrayBytes[8];
    byte LONArrayBytes[8];
} GPSdata ;

union CRCdata {
  word  GPSDataCRC;
  byte GPSArrayCRC[2];  
};


union LATdata {
  double GPSLatitudeDegrees;
  byte LATArrayOfFourBytes[8];
};

union LONdata {
  double GPSLongitudeDegrees;
  byte LONArrayOfFourBytes[8];
};

const int inputStringSize = 10;
boolean stringComplete = false;     // whether the string is complete
volatile char inputChar[inputStringSize];

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

  int indx = 0;
  while ((Serial.available()) && (!stringComplete)) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      inputChar[indx] = '\0';
      stringComplete = true;
    } else {
    // add it to the inputChar:
    inputChar[indx] += inChar;
    indx += 1;
    }
  }
}

SYSdata sysdata ;

// For the GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
//  We are using the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
boolean GPSECHO = false;

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = true;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

extern "C" char *sbrk(int i);

int FreeRam () {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
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

void W_Date2eeprom() {

  union LATdata latdata;
  union LONdata londata;
  union CRCdata crcdata;
  GPSdata gpsdata ;
 // SYSdata sysdata ;

          //Loop until you have a good NMEA sentence
          while (!GPS.parse(GPS.lastNMEA())) {    // this also sets the newNMEAreceived() flag to false
           delay(10);
          }
            GPS.parse(GPS.lastNMEA());
            Serial.print("Start writting to FRAM at Address! ");
            Serial.println(WrFramAddr, DEC);            
            gpsdata.GPSHour = hour();
            gpsdata.GPSMinutes = minute();
            gpsdata.GPSSeconds = second();
            gpsdata.GPSDay = day();
            gpsdata.GPSMonth = month();
            gpsdata.GPSYear = year();

            latdata.GPSLatitudeDegrees = GPS.latitudeDegrees;
            for (int i=0 ; i < 8 ; i++) gpsdata.LATArrayBytes[i] = latdata.LATArrayOfFourBytes[i];
          
            londata.GPSLongitudeDegrees = GPS.longitudeDegrees;
            for (int i=0 ; i < 8 ; i++) gpsdata.LONArrayBytes[i] = londata.LONArrayOfFourBytes[i];            

            crcdata.GPSDataCRC=i2ccrc8(&gpsdata.GPSHour,LangthGPSdata);

            fram.write8(WrFramAddr+0,gpsdata.GPSHour);
            fram.write8(WrFramAddr+1,gpsdata.GPSMinutes);
            fram.write8(WrFramAddr+2,gpsdata.GPSSeconds);
            fram.write8(WrFramAddr+3,gpsdata.GPSDay);
            fram.write8(WrFramAddr+4,gpsdata.GPSMonth);
            fram.write8(WrFramAddr+5,gpsdata.GPSYear);

            fram.write8(WrFramAddr+6,gpsdata.LATArrayBytes[0]);
            fram.write8(WrFramAddr+7,gpsdata.LATArrayBytes[1]);
            fram.write8(WrFramAddr+8,gpsdata.LATArrayBytes[2]);
            fram.write8(WrFramAddr+9,gpsdata.LATArrayBytes[3]);
            fram.write8(WrFramAddr+10,gpsdata.LATArrayBytes[4]);
            fram.write8(WrFramAddr+11,gpsdata.LATArrayBytes[5]);
            fram.write8(WrFramAddr+12,gpsdata.LATArrayBytes[6]);            
            fram.write8(WrFramAddr+13,gpsdata.LATArrayBytes[7]);
            
            fram.write8(WrFramAddr+14,gpsdata.LONArrayBytes[0]);
            fram.write8(WrFramAddr+15,gpsdata.LONArrayBytes[1]);
            fram.write8(WrFramAddr+16,gpsdata.LONArrayBytes[2]);
            fram.write8(WrFramAddr+17,gpsdata.LONArrayBytes[3]);
            fram.write8(WrFramAddr+18,gpsdata.LONArrayBytes[4]);
            fram.write8(WrFramAddr+19,gpsdata.LONArrayBytes[5]);
            fram.write8(WrFramAddr+20,gpsdata.LONArrayBytes[6]);
            fram.write8(WrFramAddr+21,gpsdata.LONArrayBytes[7]);
                                                
            fram.write8(WrFramAddr+22,crcdata.GPSArrayCRC[0]);             
            fram.write8(WrFramAddr+23,crcdata.GPSArrayCRC[1]);            
            
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

//            latdata.GPSLatitudeDegrees= GPS.latitudeDegrees;
//            londata.GPSLongitudeDegrees = GPS.longitudeDegrees;

            fmtDouble(latdata.GPSLatitudeDegrees, 6, GPSStringLAT,16);
            fmtDouble(londata.GPSLongitudeDegrees, 6, GPSStringLON,16); 

            Serial.println("Hours;Minutes;Seconds;Day;Moth;Year;LatitudeDegrees;GPSLongitudeDegrees;CRC;Calculated CrC");
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
            if (latdata.GPSLatitudeDegrees < 0) {Serial.print("-");}
            Serial.print(GPSStringLAT);
            Serial.print(";");
            if (londata.GPSLongitudeDegrees < 0) {Serial.print("-");}               
            Serial.print(GPSStringLON);
            Serial.print(";");
            Serial.println(crcdata.GPSDataCRC, DEC);
            Serial.print("Finish writting to FRAM next address will be! ");
            Serial.println(WrFramAddr, DEC);    
//            Serial.print(GPS.latitudeDegrees);
//            Serial.print(";");   
//            Serial.println(GPS.longitudeDegrees);            
}

void R_DataFeeprom() {

  union LATdata latdata;
  union LONdata londata;
  union CRCdata crcdata;
  GPSdata gpsdata;
  RdFramAddr = 0;

      Serial.println();
      Serial.print("Dumpingt from FRAM address! ");
      Serial.println(0x0, DEC);    
      Serial.print("Dumpingt To FRAM address! ");
      Serial.println(LastFramAddr, DEC);      
      Serial.println("Hours;Minutes;Seconds;Day;Moth;Year;LatitudeDegrees;GPSLongitudeDegrees;CRC;Calculated CrC");
      
  while  (RdFramAddr < LastFramAddr) {
                gpsdata.GPSHour=fram.read8(RdFramAddr+0);
                gpsdata.GPSMinutes=fram.read8(RdFramAddr+1);
                gpsdata.GPSSeconds=fram.read8(RdFramAddr+2);
                gpsdata.GPSDay=fram.read8(RdFramAddr+3);
                gpsdata.GPSMonth=fram.read8(RdFramAddr+4);
                gpsdata.GPSYear=fram.read8(RdFramAddr+5);
                gpsdata.LATArrayBytes[0]=fram.read8(RdFramAddr+6);
                gpsdata.LATArrayBytes[1]=fram.read8(RdFramAddr+7);
                gpsdata.LATArrayBytes[2]=fram.read8(RdFramAddr+8);
                gpsdata.LATArrayBytes[3]=fram.read8(RdFramAddr+9);
                gpsdata.LATArrayBytes[4]=fram.read8(RdFramAddr+10);
                gpsdata.LATArrayBytes[5]=fram.read8(RdFramAddr+11);
                gpsdata.LATArrayBytes[6]=fram.read8(RdFramAddr+12);
                gpsdata.LATArrayBytes[7]=fram.read8(RdFramAddr+13);
                
                gpsdata.LONArrayBytes[0]=fram.read8(RdFramAddr+14);
                gpsdata.LONArrayBytes[1]=fram.read8(RdFramAddr+15);
                gpsdata.LONArrayBytes[2]=fram.read8(RdFramAddr+16);
                gpsdata.LONArrayBytes[3]=fram.read8(RdFramAddr+17);
                gpsdata.LONArrayBytes[4]=fram.read8(RdFramAddr+18);
                gpsdata.LONArrayBytes[5]=fram.read8(RdFramAddr+19);
                gpsdata.LONArrayBytes[6]=fram.read8(RdFramAddr+20);
                gpsdata.LONArrayBytes[7]=fram.read8(RdFramAddr+21);
                
                crcdata.GPSArrayCRC[0]=fram.read8(RdFramAddr+22);
                crcdata.GPSArrayCRC[1]=fram.read8(RdFramAddr+23);

                for (int i=0 ; i < 8 ; i++) latdata.LATArrayOfFourBytes[i] = gpsdata.LATArrayBytes[i];
                for (int i=0 ; i < 8 ; i++) londata.LONArrayOfFourBytes[i] = gpsdata.LONArrayBytes[i];

                fmtDouble(latdata.GPSLatitudeDegrees, 6, GPSStringLAT);
                fmtDouble(londata.GPSLongitudeDegrees, 6, GPSStringLON);
                
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
                if (latdata.GPSLatitudeDegrees < 0) {Serial.print("-");}
                Serial.print(GPSStringLAT);
                Serial.print(";");
                if (londata.GPSLongitudeDegrees < 0) {Serial.print("-");}               
                Serial.print(GPSStringLON);
                Serial.print(";");
                Serial.print(crcdata.GPSDataCRC, DEC);
                Serial.print(";");           
                Serial.println(i2ccrc8(&gpsdata.GPSHour,LangthGPSdata),DEC);               
//                Serial.print(GPS.latitudeDegrees);
//                Serial.print(";");   
//                Serial.println(GPS.longitudeDegrees);  

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
    Serial.print("GPS Date/Time     ");
    Serial.print(GPS.day);
    Serial.print("/");              
    Serial.print(GPS.month);
    Serial.print("/");              
    Serial.print(GPS.year);                     
    Serial.print("  ");
    Serial.print(GPS.hour);
    Serial.print(":");
    Serial.print(GPS.minute);
    Serial.print(":");            
    Serial.println(GPS.seconds);
 
    Serial.print("Flora Date/Time  ");
    Serial.print(day());
    Serial.print("/");              
    Serial.print(month());
    Serial.print("/");              
    Serial.print(year());                     
    Serial.print("  ");
    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());
    Serial.print(":");            
    Serial.println(second());
}

uint8_t debounceRead(int pin)
{
  
  uint8_t pinState = digitalRead(pin);
  uint32_t timeout = millis();
  while ((millis() - timeout) < 100)
  {
    if (digitalRead(pin) != pinState)
    {
      pinState = digitalRead(pin);
      timeout = millis();
    }
  }

  return pinState;
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR(PCINT0_vect)
{
  cli();
  First_buttonState = digitalRead(First_buttonPin);
  Second_buttonState = digitalRead(Second_buttonPin);
  sei();

}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.write(c);  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
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


Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXEL, NeoPixelPin, NEO_GRB + NEO_KHZ800);

void setup()  
{

/*
Pin Change Mask Register 0 – PCMSK0

PCINT7 PCINT6 PCINT5 PCINT4 PCINT3 PCINT2 PCINT1 PCINT0 PCMSK0
bit7   bit6   bit5   bit4   bit3   bit2   bit1   bit0
• Bit 7..0 – PCINT7..0: Pin Change Enable Mask 7..0
Each PCINT7..0 bit selects whether pin change interrupt is enabled on the corresponding I/O pin. If PCINT7..0
is set and the PCIE0 bit in PCICR is set, pin change interrupt is enabled on the corresponding I/O pin. If
PCINT7..0 is cleared, pin change interrupt on the corresponding I/O pin is disabled.

// There are three interrupt vectors:
//ISR(PCINT0_vect){} // for pins PCINT0-PCINT7   (PB0-PB7)  
//ISR(PCINT1_vect){} // for pins PCINT8-PCINT14  (PC0-PC6)
//ISR(PCINT2_vect){} // for pins PCINT16-PCINT23 (PD0-PD7)
*/

//    PCMSK0 |= _BV(PCINT5) | _BV(PCINT6);    //select Pin-Change Interrupt 6 on Port B
//    PCIFR |= _BV(PCIF0);                    //Clear any pending pin-change interrupts
//    PCICR |= _BV(PCIE0);                    //Enable pin-change interrupt

//    pciSetup(9);
//    pciSetup(10);


  pixels.begin(); // This initializes the NeoPixel library.
  // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
  pixels.setPixelColor(NUMPIXEL, pixels.Color(0,0,0)); // Moderately bright green color.   
  pixels.show(); // This sends the updated pixel color to the hardware.

// initialize the LED pin as an output:
  pinMode(FLORAled, OUTPUT); 

// initialize the pushbutton pin as an input:
  pinMode(First_buttonPin, INPUT_PULLUP);
  pinMode(Second_buttonPin, INPUT_PULLUP);
  
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

Serial.print("GPS firmware version!  ");
// Ask for firmware version
Serial.println(GPSSerial.print(PMTK_Q_RELEASE));

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  
// read the eeprom to see if this is a restart or if it is the first time
// if sysdata.LastAddress[0] is zero then this is a first time if a non-zero then it is a restart

 fram.begin();

Serial.print("Checking EEPROM for Last Address used! LastAddress is ");
  sysdata.LastAddress[0] = fram.read8((FramSize-ReserveFramAddr)+7);
  sysdata.LastAddress[1] = fram.read8((FramSize-ReserveFramAddr)+8); 
  sysdata.LastAddress[2] = fram.read8((FramSize-ReserveFramAddr)+9);  
  sysdata.LastAddress[3] = fram.read8((FramSize-ReserveFramAddr)+10);  
  LastFramAddr = sysdata.LastAddress[0];
Serial.println(LastFramAddr, DEC);
  
  if ( LastFramAddr != 0 ) {
  WrFramAddr = LastFramAddr + GPSpageSize ;
  } else {
  LastFramAddr = WrFramAddr ;   
  }

/*
  //Loop until you have a good NMEA sentence
  while (!GPS.newNMEAreceived())  {    // this also sets the newNMEAreceived() flag to false
    delay(5); 
  }

  while (!GPS.parse(GPS.lastNMEA()))  {    // this also sets the newNMEAreceived() flag to false
    delay(5); 
  }
 */
   Serial.print("Here"); 
   setTime(GPS.hour,GPS.minute,GPS.seconds,GPS.day,GPS.month,GPS.year);
   adjustTime(offset * SECS_PER_HOUR);

   Serial.print("Number of GPS Satellites: ");
   Serial.println(GPS.satellites);
   Serial.print("GPS.fix is : ");
   Serial.println(GPS.fix);
   
   if (GPS.fix == 0) { 
     Serial.println("No GPS Fix!");
     NoGPSFix = true;
     // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
     pixels.setPixelColor(NUMPIXEL, pixels.Color(150,0,0)); // Moderately bright green color. 
     pixels.show(); // This sends the updated pixel color to the hardware.
     delay(250);
   } else {
     Serial.println("Got GPS Fix!");
   }
   DateTime();   
   // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
   pixels.setPixelColor(NUMPIXEL, pixels.Color(0,0,0)); // Moderately bright green color. 
   pixels.show(); // This sends the updated pixel color to the hardware.
   digitalWrite(FLORAled, LOW);
  
}

void loop()                     // run over and over again
{


//  boolean W_Date2eeprom();
  int R_DataFeeprom();
  byte i2ccrc8(byte *, byte);
  
  // check if the pushbutton is pressed.
  // if it is, the _buttonPin is true:

  //Loop until you have a good NMEA sentence
  while (!GPS.newNMEAreceived())  {    // this also sets the newNMEAreceived() flag to false
    delay(5); 
  }
  while (!GPS.parse(GPS.lastNMEA()))  {    // this also sets the newNMEAreceived() flag to false
    delay(5); 
  }
//  Serial.print(GPS.fix);

//turn off Neopixel Magenta
  if ((GPS.fix == 1) && NoGPSFix == true) { 
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(NUMPIXEL, pixels.Color(64,0,64)); // Moderately bright green color. 
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(250);
    NoGPSFix = false;
  }

 
  if (debounceRead(First_buttonPin) == LOW)  {

      if (GPS.fix == 1) {
          digitalWrite(FLORAled, HIGH);
          Serial.print("Fram Address to be writtent to is!  ");
          Serial.println(WrFramAddr, DEC);
          if (WrFramAddr + GPSpageSize + ReserveFramAddr < FramSize) {
            
            W_Date2eeprom();
            
          } else {
            Serial.println("Fram full, please download and reset! ");
          }
          digitalWrite(FLORAled, LOW);
      } else {
          Serial.println("We do not have a fix yet! ");
      }
  }   
  
  if (debounceRead(Second_buttonPin) == LOW) {

      digitalWrite(FLORAled, HIGH);
      
      if (LastFramAddr - GPSpageSize != 0 ) {
        Serial.print("Removing last GPS Entry at FRAM address! ");
        Serial.println(WrFramAddr, DEC);
        WrFramAddr = LastFramAddr;        
        LastFramAddr = WrFramAddr - GPSpageSize;   
        
        // this is incase of power failure or device failure  we know when the last eepro entry was done.
        sysdata.LastAddress[0] = LastFramAddr;
        fram.write8((FramSize-ReserveFramAddr)+7,sysdata.LastAddress[0]);
        fram.write8((FramSize-ReserveFramAddr)+8,sysdata.LastAddress[1]);
        fram.write8((FramSize-ReserveFramAddr)+9,sysdata.LastAddress[2]);
        fram.write8((FramSize-ReserveFramAddr)+10,sysdata.LastAddress[3]);
        Serial.print("We are now at FRAM address! ");
        Serial.println(WrFramAddr, DEC); 
      } else {
        Serial.print("We are staying at FRAM address! ");
        Serial.print(WrFramAddr, DEC);
        Serial.println(" We are at the start of the FRAM! ");       
      }
      digitalWrite(FLORAled, LOW);
  }
  
   /*
      0x0a (ASCII newline)
      0x0d (ASCII carriage return)
      CRLF (0x0d0a)
    */
  
  if (stringComplete) {
    
// Dump all stored data    
      if ((inputChar[0] == 'D') || (inputChar[0] == 'd')) {
          
          R_DataFeeprom();
          
// enable/disable Antenna reporting 
      } else if ((inputChar[0] == 'A') || (inputChar[0] == 'a')) {
          int indx=1;
          boolean done = true;
          while (done) {
            if (inputChar[indx] != '+' || inputChar[indx] != '-' ) {
               indx++;
 //              Serial.println(indx);
            } else if (inputChar[indx] == '-') {
               Serial.println("1");              
                GPS.sendCommand(PGCMD_NOANTENNA);  //Turn off antenna update nuisance data  
                done = !done;
            } else {
                // Request updates on antenna status, comment out to keep quiet
                GPS.sendCommand(PGCMD_ANTENNA);
                done = !done;                 
            }
          }
          
// check if we have GPS Fix          
        } else if ((inputChar[0] == 'F') || (inputChar[0] == 'f')) {
          
          Serial.print("GPS.Fix is;  ");
          Serial.println(GPS.fix);

// reset storage 
        } else if ((inputChar[0] == 'R') || (inputChar[0] == 'r')) {

            DateTime();
            Serial.println("First Set Saved FramAddress to 0!");
            fram.write8((FramSize-ReserveFramAddr)+7,0);
            fram.write8((FramSize-ReserveFramAddr)+8,0);
            fram.write8((FramSize-ReserveFramAddr)+9,0);
            fram.write8((FramSize-ReserveFramAddr)+10,0);
            LastFramAddr = 0;
            Serial.println();

            if (WrFramAddr != 0) {
                DateTime();
                Serial.print("Zero out the FRAM up to Last address Write!  ");
                Serial.println(WrFramAddr, DEC);
                // the following will zero out the eeprom
                for (unsigned int i=0; i < WrFramAddr ; i++) {
                  fram.write8(i,0);
                  if (fram.read8(i) != 0) {
                    Serial.print("Memory Error @ !");                  
                    Serial.println(i, DEC);                      
                  } else {
                    Serial.print("Erased FRAM Address!  ");
                    Serial.println(i, DEC);              
                  }
                }
                DateTime();
                Serial.print("FRAM has been Erased!");
                WrFramAddr = 0; 
                LastFramAddr = 0;
            } else {
                DateTime();
                Serial.print("Adddress is already at ");
                Serial.println(WrFramAddr, DEC);               
            }
// display status of GPS, and Lat and Lon            
        } else if ((inputChar[0] == 'S') || (inputChar[0] == 's')) {

          union LATdata latdata;
          union LONdata londata;
          //Loop until you have a good NMEA sentence
          while (!GPS.parse(GPS.lastNMEA())) {    // this also sets the newNMEAreceived() flag to false
           delay(5);
          }
                DateTime();
                Serial.print("Number of GPS Satellites: ");
                Serial.println(GPS.satellites);
                if (GPS.fix == 1) {
                   latdata.GPSLatitudeDegrees= GPS.latitudeDegrees;
                   londata.GPSLongitudeDegrees = GPS.longitudeDegrees;         
                   fmtDouble(latdata.GPSLatitudeDegrees, 6, GPSStringLAT,16);
                   fmtDouble(londata.GPSLongitudeDegrees, 6, GPSStringLON,16);
                   Serial.print("GPSLatitudeDegrees: ");
                   if (latdata.GPSLatitudeDegrees < 0) {Serial.print("-");}
                   Serial.println(GPSStringLAT);
                   Serial.print("GPSLongitudeDegrees: ");
                   if (londata.GPSLongitudeDegrees < 0) Serial.print("-");               
                   Serial.println(GPSStringLON);

                } else {
                   Serial.println("We do not have a fix yet! ");
                }

                Serial.println();

// Enable/disable echo
        } else if ((inputChar[0] == 'E') || (inputChar[0] == 'e')){
           GPSECHO = !GPSECHO;
           if (GPSECHO)  { 
            Serial.println("Echo On!");
           } else {
            Serial.println("Echo Off!");
           }

// Get time from GPS and set Flora data/time           
        } else if ((inputChar[0] == 'T') || (inputChar[0] == 't')){

          //Loop until you have a good NMEA sentence
          while (!GPS.parse(GPS.lastNMEA())) {    // this also sets the newNMEAreceived() flag to false
           delay(5);
          }
           setTime(GPS.hour,GPS.minute,GPS.seconds,GPS.day,GPS.month,GPS.year);
           adjustTime(offset * SECS_PER_HOUR);
           DateTime();

 // change time zone           
 /*       } else if ((inputChar[0] == 'Z') || (inputChar[0] == 'z')) {
          // Offset hours from gps time (UTC)
          //volatile int offset = -4;  // Eastern Daylight Time (USA)
          //volatile int offset = -5;  // Eastern Standard Time (USA)
          //volatile int offset = -6;  // Centril Standard Time (USA)
          //volatile int offset = -8;  // Pacific Standard Time (USA)
          //volatile int offset = -7;  // Pacific Daylight Time (USA)
          int indx=1;
          boolean done = true;
//          char mystring[4];
          int myoffset;
                    Serial.println(inputChar[indx]);   
          while (done) {
            if ((inputChar[indx] != '+') || (inputChar[indx] != '-')) {
               indx++;
                    Serial.println(inputChar[indx]);    

            } else if (inputChar[indx] == '-') {
                Serial.println("2");             
                myoffset = int(inputChar[indx+1]*-1);
                Serial.println(myoffset);  
                done = !done;
            } else {
                Serial.println("3");    
                myoffset = int(inputChar[indx+1]);
                Serial.println(myoffset);  
                done = !done;                 
            } 
          }
          Serial.println(inputChar[indx]);           
          Serial.println(inputChar[indx+1]);           
          adjustTime(myoffset * SECS_PER_HOUR);  */
        } else {
          Serial.print("Unknown command! [");
          Serial.print(inputChar[0]);
          Serial.println("]!");
          Serial.println();
          Serial.println("commands are! D (Download)");
          Serial.println("              R (Reset EEprom)");   
          Serial.println("              S (Status)");
          Serial.println("              E (GPSECHO)");
          Serial.println("              T (Set time)");
          Serial.println("              A<+-> (Enable/Disable Antenna)");
          Serial.println("              F (Check if we have GPS fix)");
       /*   Serial.println("              Z<+-># ");
          // Offset hours from gps time (UTC)
          Serial.print("                Z-4; Atlantic Standard Time (USA)");
          Serial.print("                Z-5; Eastern  Standard Time (USA)");
          Serial.print("                Z-6; Central  Standard Time (USA)");
          Serial.print("                Z-7; Pacific  Standard Time (USA)");          
         */
        }
    // clear the string and the flag...
    for (int i=0;i < inputStringSize; i++) {
      inputChar[i] = '\0'; 
      }
    stringComplete = false; 
  }  
}


/*
buttonpress() debounces a pushbutton and returns
  0 if the button was not depressed,
  1 if a short button press was detected, or
  2 if a long button press was detected

int buttonpress() 
{
  #define debounceDelay 50
  #define longPressTime 1000
  
  int retVal = 0;
  static int button;
  static int lastButtonState = HIGH;
  static int buttonState = HIGH;
  static long lastDebounceTime;
  static long buttonPressStartTime;
  
  button = digitalRead(buttonPin);  // read the button pin
  if (button != lastButtonState)   // if the button changed
  {
    lastDebounceTime = millis();   // reset the debounce timer
  }  
  if ((millis() - lastDebounceTime) > debounceDelay)  // check if button has been stable for debounceDelay mS
  {
    if (button != buttonState)      // this starts the processing of a debounced state change
    {
      buttonState = button;
      if (buttonState == HIGH) // the button has just been released
      {
        if (millis() - buttonPressStartTime >= longPressTime) 
        { 
          retVal = 2;  // a long press has been detected
        }
        else 
        {
          retVal = 1;  // otherwise it was a short press
        }
      }
      else   // the button has just been pressed
      {
        buttonPressStartTime = millis();  // start timing how long the button is depressed
      }
    }    
  }  // This ends processing of a debounced state change
  lastButtonState = button;
  return(retVal);
}

#define debounceDelay 50
#define longPressTime 1000

int retVal = 0;
static int button;
static int lastButtonState = HIGH;
static int buttonState = HIGH;
static long lastDebounceTime;
static long buttonPressStartTime;

  digitalWrite(FLORAled,button);
  // read the state of the pushbutton value:
  int First_buttonState = digitalRead(First_buttonPin);
  if (First_buttonState == HIGH) { button = First_buttonState; }
        
  int Second_buttonState = digitalRead(Second_buttonPin);
  if (Second_buttonState == HIGH) { button = Second_buttonState; }
    
  if (button != lastButtonState)    // if the button changed
  {
    lastDebounceTime = millis();   // reset the debounce timer
  }  
  if ((millis() - lastDebounceTime) > debounceDelay)  // check if button has been stable for debounceDelay mS
  {
    if (button != buttonState)   // this starts the processing of a debounced state change
    {
      buttonState = button;
      if (buttonState == HIGH) // the button has just been released
      {
        if (millis() - buttonPressStartTime >= longPressTime) 
        { 
          retVal = 2;  // a long press has been detected
        }
        else 
        {
          retVal = 1;  // otherwise it was a short press
        }
      }
      else   // the button has just been pressed
      {
        buttonPressStartTime = millis();  // start timing how long the button is depressed
      }
    }    
  }  // This ends processing of a debounced state change
  lastButtonState = button;
  
  digitalWrite(FLORAled,!button);


//required for fmod()
#include <math.h>;
 
// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
double convertDegMinToDecDeg (float degMin) {
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}


  if (GPS.newNMEAreceived())
  {
    if (GPS.parse(GPS.lastNMEA()))
    {
      if (GPS.fix != 0)
      {
        if ((previous_latitude != GPS.latitudeDegrees) || 
            (previous_longitudeDegrees != GPS.longitudeDegrees))
        {
          double lonA = M_PI * previous_longitudeDegrees / 180.0;
          double lonB = M_PI * GPS.longitudeDegrees / 180.0;
          double latA = M_PI * previous_latitude / 180.0;
          double latB = M_PI * GPS.latitudeDegrees / 180.0;

          double a = sin((lonA - lonB) / 2.0);
          double b = cos(lonA) * cos(lonB);
          double c = sin((latB - latA) / 2.0);
          double d = sqrt(a * a + b * c * c);
          double e = asin(d);
          double f = 12734890.0 * e;

          Serial.print("Time: ");
          Serial.print(GPS.hour, DEC); Serial.print(':');
          Serial.print(GPS.minute, DEC); Serial.print(':');
          Serial.print(GPS.seconds, DEC); Serial.print('.');
          Serial.print(GPS.milliseconds);
          Serial.print(" - Location (in degrees): ");
          Serial.print(GPS.latitudeDegrees, 9);
          Serial.print(", "); 
          Serial.print(GPS.longitudeDegrees, 9);
          Serial.print(" ("); 
          Serial.print(f, 0);
          Serial.println(")"); 
          previous_latitude = GPS.latitudeDegrees;
          previous_longitudeDegrees = GPS.longitudeDegrees;
        }
      }
    }
  }



*/

