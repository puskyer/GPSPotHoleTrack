
#include <fmtDouble.h>
#include <crc8.h>


void setup() {
  // put your setup code here, to run once:



}

void loop() {
  // put your main code here, to run repeatedly:

  char GPSString[16];
  float myfloat = -74.123456;

  for (int i=0;i <= 16; i++) GPSString[i]="\0";

              Serial.println(myfloat);
              Serial.println(GPSString);
              fmtDouble(myfloat, 6, GPSString,16); 
              Serial.println();
              if (myfloat < 0) {
                Serial.print("-");
                Serial.println(GPSString);
              } else {
                Serial.println(GPSString); 
              }
             

}



