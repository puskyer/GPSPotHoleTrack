// Alpha code for GPS modules for pothole tracking.
//
//  August 10th 2016
//


#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>    
 
#define disk1 0x50    //Address of 24C02 eeprom chip
// turn on GGA only
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"

// constants won't change. They're used here to
// set pin numbers:
const int First_buttonPin = 4;     // the number of the pushbutton pin
const int First_ledPin = 11;      // the number of the LED pin
const int Second_buttonPin = 5;     // the number of the pushbutton pin
const int Second_ledPin = 12;      // the number of the LED pin
const int Third_buttonPin = 6;     // the number of the pushbutton pin
const int Third_ledPin = 13;      // the number of the LED pin

// variables will change:
int First_buttonState = 0;          // variable for first pushbutton status
int Second_buttonState = 0;         // variable for reading the pushbutton status
int Third_buttonState = 0;         // variable for reading the pushbutton status

unsigned long StartButtonPress = 0;          // start of button press time
unsigned long StopButtonPress = 0;           // end of button press time
unsigned long First_buttonPressDelay = 0;
char AnswerIs ='?';

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

byte myHour;
byte myMinutes;
byte mySeconds;
byte myDay;
byte myMonth;
byte myYear;

union LATdata {
  float myLatitudeDegrees;
  byte LATArrayOfFourBytes[4];
};

union LONdata {
  float myLongitudeDegrees;
  byte LONArrayOfFourBytes[4];
};

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
  void writeEEPROM(byte, byte, byte);
  byte readEEPROM(int, int);
      
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:

  if (digitalRead(Third_buttonPin) == HIGH) {

    // turn LED on:
    digitalWrite(Third_ledPin, HIGH);
    delay(250);
    
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
 */
       // $GPRMC,171033.800,A,4503.0912,N,07523.7578,W,0.05,82.23,260716,,,A*4F


/*
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
*/
       
// $GPGGA,171033.800,4503.0912,N,07523.7578,W,1,06,1.40,52.2,M,-33.9,M,,*65
// $GPGGA,023900.000,4503.1057,N,07523.7231,W,1,07,1.65,80.6,M,-33.9,M,,*6C


/*
const int GPGGA_string_start=1;
const int GPGGA_string_end=6;
const int GPGGA_time_start=8;
const int GPGGA_time_end=17;
const int GPGGA_latitude_start=18;
const int GPGGA_latitude_end=19;
const int GPGGA_latitude_M_start=20;
const int GPGGA_latitude_M_end=25;
const int GPGGA_latitude_hemisphere=27;
const int GPGGA_longitude_start=29;
const int GPGGA_longitude_end=37;
const int GPGGA_longitude_hemisphere=39;
const int GPGGA_GPS_Quality=41;
*/

/*
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

       while (!GPS.parse(GPS.lastNMEA())) { Serial.println("Missed GPS Last NMEA, will try again"); }  // this also sets the newNMEAreceived() flag to false
            Serial.println("Writting to EEPROM!");

            union LATdata latdata;
            union LONdata londata;
            myHour = GPS.hour;
            myMinutes = GPS.minute;
            mySeconds = GPS.seconds;
            myDay = GPS.day;
            myMonth = GPS.month;
            myYear = GPS.year;
            latdata.myLatitudeDegrees = GPS.latitudeDegrees;
            londata.myLongitudeDegrees = GPS.longitudeDegrees;

            writeEEPROM(disk1,0,myHour);
            writeEEPROM(disk1,1,myMinutes);
            writeEEPROM(disk1,2,mySeconds);
            writeEEPROM(disk1,3,myDay);
            writeEEPROM(disk1,4,myMonth);
            writeEEPROM(disk1,5,myYear);
            writeEEPROM(disk1,6,latdata.LATArrayOfFourBytes[0]);
            writeEEPROM(disk1,7,latdata.LATArrayOfFourBytes[1]);
            writeEEPROM(disk1,8,latdata.LATArrayOfFourBytes[2]);
            writeEEPROM(disk1,9,latdata.LATArrayOfFourBytes[3]);
            writeEEPROM(disk1,10,londata.LONArrayOfFourBytes[0]);
            writeEEPROM(disk1,11,londata.LONArrayOfFourBytes[1]);
            writeEEPROM(disk1,12,londata.LONArrayOfFourBytes[2]);
            writeEEPROM(disk1,13,londata.LONArrayOfFourBytes[3]);
            digitalWrite(Third_ledPin, LOW);

  } else if (stringComplete) {
    if  ( inputString == "Erase" )
    {
      Serial.println("Erase Command Recieved! This will ERASE the data log stored in the FLASH - Permanently!");
      Serial.println("\nERASING!");
      GPS.sendCommand(PMTK_LOCUS_ERASE_FLASH);
      Serial.println("Erased");
    } else if ( inputString == "Download" ) {

            myHour=readEEPROM(disk1,0);
            myMinutes=readEEPROM(disk1,1);
            mySeconds=readEEPROM(disk1,2);
            myDay=readEEPROM(disk1,3);
            myMonth=readEEPROM(disk1,4);
            myYear=readEEPROM(disk1,5);
            latdata.LATArrayOfFourBytes[0]=readEEPROM(disk1,6);
            latdata.LATArrayOfFourBytes[1]=readEEPROM(disk1,7);
            latdata.LATArrayOfFourBytes[2]=readEEPROM(disk1,8);
            latdata.LATArrayOfFourBytes[3]=readEEPROM(disk1,9);
            londata.LONArrayOfFourBytes[0]=readEEPROM(disk1,10);
            londata.LONArrayOfFourBytes[1]=readEEPROM(disk1,11);
            londata.LONArrayOfFourBytes[2]=readEEPROM(disk1,12);
            londata.LONArrayOfFourBytes[3]=readEEPROM(disk1,13);

            Serial.println("Hours;Minutes;Seconds;Day;Moth;Year;LatitudeDegrees;myLongitudeDegrees");
            Serial.print(myHour);
            Serial.print(";");
            Serial.print(myMinutes);
            Serial.print(";");
            Serial.print(mySeconds);
            Serial.print(";");
            Serial.print(myDay);
            Serial.print(";");
            Serial.print(myMonth);
            Serial.print(";");
            Serial.print(myYear);
            Serial.print(";");
            Serial.print(latdata.myLatitudeDegrees);
            Serial.print(";");
            Serial.println(londata.myLongitudeDegrees);
            
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
