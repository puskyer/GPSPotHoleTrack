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
#include <SoftwareSerial.h>
#include <Wire.h>    
 
#define disk1 0x50        //Address of 24C02 eeprom chip
#define eepromSize 256   //24C02 eeprom chip is 256 bytes or 32 pages  x 8 bytes or 2K bits
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
const byte First_buttonPin = 4;     // the number of the pushbutton pin
const byte First_ledPin = 11;      // the number of the LED pin
const byte Second_buttonPin = 5;     // the number of the pushbutton pin
const byte Second_ledPin = 12;      // the number of the LED pin
const byte Third_buttonPin = 6;     // the number of the pushbutton pin
const byte Third_ledPin = 13;      // the number of the LED pin

// variables will change:
boolean  First_buttonState = false;     // variable for first pushbutton status
boolean Second_buttonState = false;     // variable for reading the pushbutton status
boolean Third_buttonState = false;      // variable for reading the pushbutton status

unsigned long StartButtonPress = 0;          // start of button press time
unsigned long StopButtonPress = 0;           // end of button press time
unsigned long First_buttonPressDelay = 0;

String inputString = "";         // a string to hold incoming data
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
#define UInt16 uint16_t
// CRC should be DD18
char *t = (char *)"DEADBEEF";


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
SoftwareSerial mySerial(3, 2);

Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


void setup()  
{
 
  // initialize the LED pin as an output:
  pinMode(First_ledPin, OUTPUT);
  pinMode(Second_ledPin, OUTPUT);  
  pinMode(Third_ledPin, OUTPUT);  
    
  // initialize the pushbutton pin as an input:
  pinMode(First_buttonPin, INPUT);
  pinMode(Second_buttonPin, INPUT);
  pinMode(Third_buttonPin, INPUT);

  // set up for eeprom 
  Wire.begin();  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);

    // reserve 10 bytes for the inputString:
  inputString.reserve(10);

  // read the eeprom to see if this is a restart or if it is the first time
  // if sysdata.LastAddress[0] is zero then this is a first time if a non-zero then it is a restart
  
  eepromLastAddress = readEEPROM(disk1,(eepromSize-eepromreserver)+7);
  if ( eepromLastAddress != 0 ) {
  Weepromaddress = eepromLastAddress ;
  } else {
  eepromLastAddress = Weepromaddress ;   
  }

  
  delay(250);
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

  delay(500);
  // Ask for firmware version
  // Serial.println(mySerial.println(PMTK_Q_RELEASE));

  

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false
/*  
 if (GPS.LOCUS_ReadStatus()) {
    if (GPS.LOCUS_status) digitalWrite(First_ledPin, HIGH);  // turn LED off:
 }
*/ 

}

void loop()                     // run over and over again
{

  union LATdata latdata;
  union LONdata londata;
  
  GPSdata gpsdata ;
  SYSdata sysdata ;
  

  void writeEEPROM(byte, byte, byte);
  byte readEEPROM(int, int);
  byte i2ccrc8(byte *, byte);
 
      
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:

  if (digitalRead(First_buttonPin) == HIGH) {

    // turn LED on:
    digitalWrite(First_ledPin, HIGH);
    delay(250);
    


       while (!GPS.parse(GPS.lastNMEA())) { Serial.println("Missed GPS Last NMEA, will try again"); }  // this also sets the newNMEAreceived() flag to false
            Serial.println("Writting to EEPROM!");
            
            gpsdata.GPSHour = GPS.hour;
            gpsdata.GPSMinutes = GPS.minute;
            gpsdata.GPSSeconds = GPS.seconds;
            gpsdata.GPSDay = GPS.day;
            gpsdata.GPSMonth = GPS.month;
            gpsdata.GPSYear = GPS.year;
            latdata.GPSLatitudeDegrees = GPS.latitudeDegrees;
            londata.GPSLongitudeDegrees = GPS.longitudeDegrees;
            byte GPSDataCRC=i2ccrc8(&gpsdata.GPSHour,14);
            

            writeEEPROM(disk1,Weepromaddress+0,gpsdata.GPSHour);
            writeEEPROM(disk1,Weepromaddress+1,gpsdata.GPSMinutes);
            writeEEPROM(disk1,Weepromaddress+2,gpsdata.GPSSeconds);
            writeEEPROM(disk1,Weepromaddress+3,gpsdata.GPSDay);
            writeEEPROM(disk1,Weepromaddress+4,gpsdata.GPSMonth);
            writeEEPROM(disk1,Weepromaddress+5,gpsdata.GPSYear);
            writeEEPROM(disk1,Weepromaddress+6,latdata.LATArrayOfFourBytes[0]);
            writeEEPROM(disk1,Weepromaddress+7,latdata.LATArrayOfFourBytes[1]);
            writeEEPROM(disk1,Weepromaddress+8,latdata.LATArrayOfFourBytes[2]);
            writeEEPROM(disk1,Weepromaddress+9,latdata.LATArrayOfFourBytes[3]);
            writeEEPROM(disk1,Weepromaddress+10,londata.LONArrayOfFourBytes[0]);
            writeEEPROM(disk1,Weepromaddress+11,londata.LONArrayOfFourBytes[1]);
            writeEEPROM(disk1,Weepromaddress+12,londata.LONArrayOfFourBytes[2]);
            writeEEPROM(disk1,Weepromaddress+13,londata.LONArrayOfFourBytes[3]);
            writeEEPROM(disk1,Weepromaddress+14,GPSDataCRC);            
            
            eepromLastAddress = Weepromaddress;

            // this is incase of power failure or device failure  we know when the last eepro entry was done.
            sysdata.LastAddress[0] = eepromLastAddress;
            writeEEPROM(disk1,(eepromSize-eepromreserver)+0,sysdata.SysHour);
            writeEEPROM(disk1,(eepromSize-eepromreserver)+1,sysdata.SysMinutes);
            writeEEPROM(disk1,(eepromSize-eepromreserver)+2,sysdata.SysSeconds);
            writeEEPROM(disk1,(eepromSize-eepromreserver)+3,sysdata.SysDay);
            writeEEPROM(disk1,(eepromSize-eepromreserver)+4,sysdata.SysMonth);
            writeEEPROM(disk1,(eepromSize-eepromreserver)+5,sysdata.SysYear);
            writeEEPROM(disk1,(eepromSize-eepromreserver)+6,sysdata.SysMonth);
            writeEEPROM(disk1,(eepromSize-eepromreserver)+7,sysdata.LastAddress[0]);

            Weepromaddress +=GPSpageSize;

            digitalWrite(First_ledPin, LOW);

  } else if (stringComplete) {
    if  ( inputString == "Erase" )
    {
      Serial.println("Erase Command Recieved! This will ERASE the data log stored in the FLASH - Permanently!");
      Serial.println("\nERASING!");
      GPS.sendCommand(PMTK_LOCUS_ERASE_FLASH);
      Serial.println("Erased");
    } else if ( inputString == "Download" ) {
      
            while  ( Reepromaddress <= eepromLastAddress ) {
                gpsdata.GPSHour=readEEPROM(disk1,Reepromaddress+0);
                gpsdata.GPSMinutes=readEEPROM(disk1,Reepromaddress+1);
                gpsdata.GPSSeconds=readEEPROM(disk1,Reepromaddress+2);
                gpsdata.GPSDay=readEEPROM(disk1,Reepromaddress+3);
                gpsdata.GPSMonth=readEEPROM(disk1,Reepromaddress+4);
                gpsdata.GPSYear=readEEPROM(disk1,Reepromaddress+5);
                
                latdata.LATArrayOfFourBytes[0]=readEEPROM(disk1,Reepromaddress+6);
                latdata.LATArrayOfFourBytes[1]=readEEPROM(disk1,Reepromaddress+7);
                latdata.LATArrayOfFourBytes[2]=readEEPROM(disk1,Reepromaddress+8);
                latdata.LATArrayOfFourBytes[3]=readEEPROM(disk1,Reepromaddress+9);
                londata.LONArrayOfFourBytes[0]=readEEPROM(disk1,Reepromaddress+10);
                londata.LONArrayOfFourBytes[1]=readEEPROM(disk1,Reepromaddress+11);
                londata.LONArrayOfFourBytes[2]=readEEPROM(disk1,Reepromaddress+12);
                londata.LONArrayOfFourBytes[3]=readEEPROM(disk1,Reepromaddress+13);
                byte GPSDataCRC=readEEPROM(disk1,Reepromaddress+14);

    
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
                Serial.println(GPSDataCRC);  
                Serial.print("Calculated CrC is:  ");              
                Serial.println(i2ccrc8(&gpsdata.GPSHour,14),HEX);               

                Reepromaddress +=GPSpageSize;
            }
            
    } else if ( inputString == "Reset" ) {

        for (byte i=0; i < eepromSize ; i++) {
          Serial.print(readEEPROM(disk1,i), HEX);
          Serial.print(" ");
        }

        // the following will zero out the eeprom
        for (byte i=0; i < eepromSize ; i++) {
          writeEEPROM(disk1,i,0);  
        }

        for (byte i=0; i < eepromSize ; i++) {
          Serial.print(readEEPROM(disk1,i), HEX);
          Serial.print(" ");  
        }        

    } else if ( inputString == "Status" ) {
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
       
       digitalWrite(Second_ledPin, LOW);   
      }       
    } else if ( inputString == "StartLog" ) {
        if (GPS.LOCUS_ReadStatus()) {
              if (GPS.LOCUS_status)
              {
                   Serial.println("Already Running");  
              }
              else
              {
                  GPS.LOCUS_StartLogger();
                  // turn LED on:
                  digitalWrite(First_ledPin, HIGH);
                  Serial.println(" STARTED!");

              }
        }      
        else
       {

                 Serial.println(" no response :("); 
       }
     } else if ( inputString == "StopLog" ) {
       if (GPS.LOCUS_ReadStatus()) {
               if (GPS.LOCUS_status)
              {
                  GPS.LOCUS_StopLogger();
                  // turn LED off:
                  digitalWrite(First_ledPin, LOW);
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
      Serial.print("Unknown command!  ");
      Serial.println(inputString);
      Serial.println("command are! Download or Erase or Status");
    }
          // clear the string and the flag...
    inputString = "";
    stringComplete = false; 
  }  
}

/******************************************************************/
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO && c) {
#ifdef UDR0
    UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
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
}



/*
 From: http://www.ccontrolsys.com/w/How_to_Compute_the_Modbus_RTU_Message_CRC
 
https://www.lammertbies.nl/comm/info/crc-calculation.html
Using the 8 character ASCII input DEADBEEF (upper case)
the checksum is 0xDD18
The code below agrees with the online calculator here:
http://www.lammertbies.nl/comm/info/crc-calculation.html

*/

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

