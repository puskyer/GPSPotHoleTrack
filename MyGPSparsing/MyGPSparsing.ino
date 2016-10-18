// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

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
  
  // initialize the pushbutton pin as an input:
  pinMode(First_buttonPin, INPUT);
  pinMode(Second_buttonPin, INPUT);
  pinMode(Third_buttonPin, INPUT);

  Wire.begin();  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Gather Location!");
  delay(200);
  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGA);
  
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

  delay(1000);
  // Ask for firmware version
  Serial.println(mySerial.println(PMTK_Q_RELEASE));

}

void loop()                     // run over and over again
{

// variables will change:
//int First_buttonState = 0;          // variable for first pushbutton status
//int Second_buttonState = 0;         // variable for reading the pushbutton status
//int Third_buttonState = 0;         // variable for reading the pushbutton status

unsigned long StartButtonPress = 0;          // start of button press time
unsigned long StopButtonPress = 0;           // end of button press time
char First_buttonAnswer = '?' ;
    
    // read the state of the pushbutton value:
//  delay(200);
//  First_buttonState = digitalRead(First_buttonPin);
//  Second_buttonState = digitalRead(Second_buttonPin);
//  Third_buttonState = digitalRead(Third_buttonPin);
  
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (digitalRead(First_buttonPin) == HIGH) {
    StartButtonPress = millis() ;
    while (digitalRead(First_buttonPin) == HIGH)
    { 
  /*    
      Serial.print("\nwaiting for Button 1....");
      Serial.print(StartButtonPress);
      Serial.print("....");
      Serial.print(StopButtonPress);
    */
      StopButtonPress = millis() ; 
    }

    if  (( StopButtonPress - StartButtonPress ) > 2000 && ( StopButtonPress - StartButtonPress ) < 5000 )
    {
    
      Serial.println("Button 1 pressed for more then 2 Sec but less then 5 Sec!");
      // turn LED off:
      digitalWrite(First_ledPin, LOW);
      // delay(500);
      Serial.print("\nStop LOGGING....");
        if (GPS.LOCUS_ReadStatus()) {
               if (GPS.LOCUS_StopLogger())
              {
                  Serial.println("Logger Stoped!");
              }
              else
              {
                  Serial.println(" no response :(");
              }         
        } 
        else
       {

                Serial.print("\nAlready Stoped...."); 
       }      
        // delay(1000); 
    } 
    else if ((StopButtonPress - StartButtonPress) > 5000 )
    {
      Serial.println("Button 1 pressed for more then 5 Sec!");
      Serial.println("This will ERASE the data log stored in the FLASH - Permanently!");
      Serial.print("Are you sure you want to do this? [Y/N]: ");
      while (( Serial.read() != 'Y' ) || ( Serial.read() != 'N' )) delay(10);
      if (First_buttonAnswer == 'Y') {
          Serial.println("\nERASING!");
          // delay(2000);
          GPS.sendCommand(PMTK_LOCUS_ERASE_FLASH);
          Serial.println("Erased");
      }
      else
      {
          Serial.println("Did not Erase");
      }
    }
    else
    {
      // turn LED on:
      digitalWrite(First_ledPin, HIGH);
      Serial.println("Button 1 pressed less then a sec!");
      // delay(500);
        Serial.print("\nCheck LOGGING....");
        if (GPS.LOCUS_ReadStatus()) {
                Serial.print("\nAlready LOGGING...."); 
        } 
        else
       {
              if (GPS.LOCUS_StartLogger())
              {
                  Serial.println(" STARTED!");
              }
              else
              {
                  Serial.println(" no response :(");
              }

       }
      }
  } else if (digitalRead(Second_buttonPin) == HIGH) {
    // turn LED on:
    digitalWrite(Second_ledPin, HIGH);
    Serial.println("Button 2 pressed");
    // delay(1000);
       
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
      }
  } else if (digitalRead(Third_buttonPin) == HIGH) {
    Serial.println("Button 3 pressed");
    if (GPS.LOCUS_status) 
    {
        // if a sentence is received, we can check the checksum, parse it...
        while (!GPS.newNMEAreceived()) { Serial.println("Missed GPS Last NMEA, will try again"); }
            String myLastNMEA;
            myLastNMEA = GPS.lastNMEA();   // this also sets the newNMEAreceived() flag to false
            Serial.println(myLastNMEA);
            String mytime;
            mytime = myLastNMEA.substring(8,18);
            Serial.println(mytime);
            return;                           // we can fail to parse a sentence in which case we should just wait for another
    }
    else 
    {
       // if a sentence is received, we can check the checksum, parse it...
       // $GPGGA,045343.800,4503.0817,N,07523.7473,W,1,06,1.23,94.8,M,-33.9,M,,*6C
        while (!GPS.newNMEAreceived()) { Serial.println("Missed GPS Last NMEA, will try again"); }
            String myLastNMEA;
            myLastNMEA = GPS.lastNMEA();   // this also sets the newNMEAreceived() flag to false
            Serial.println(myLastNMEA);
            String mytime;
            String myaltitude;
            String mylatitudeDegrees;
            String mylongitudeDegrees;
            String myFix;
            String myfixquality;
                        
            
            mytime = myLastNMEA.substring(8,18);
            Serial.println(mytime);
            return;                           // we can fail to parse a sentence in which case we should just wait for another
    }
  } else if (digitalRead(Second_buttonPin) == LOW) {
    // turn LED off:
    digitalWrite(Second_ledPin, LOW);   
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


void writeEEPROM(byte deviceaddress, byte eeaddress, byte data ) 
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
 
  return rdata;
}


