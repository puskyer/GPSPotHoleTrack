#include <stdbool.h>

#ifndef YUAA_GPS_IMU
#define YUAA_GPS_IMU

//Send this manually to the gps when you want more velocity data
//Don't send too many!
#define GPS_VELOCITY_REQUEST "$PTNLQTF*69\r\n"

//The structure for parsed NMEA 0183 data
//numDatums is the size of both datumIndices and datums,
//where datumIndices indicates which datums in the Nmea
//sentence are of interest and should be put into datums.
//Each datum should have space for 10 bytes.
//All values will be null-terminated and
//if for some reason the actual parsed data is longer,
//the extra characters beyond 9 will be discarded.
//This structure also stores the state of the parser,
//which state should not be modified by anything but the parser.
//However, readChecksum needs to be set to -1 by 
typedef struct
{
    //These need to be already initialized
    const char* tag;
    int numDatums;
    int* datumIndices;
    char** datums;
    //State, do not touch!
    char runningChecksum;
    bool hasBegunUtterance;
    int tagIndex;
    bool datumBegun;
    int datumOn;
    char datumData[10];
    int datumDataIndex;
    bool checksumBegun;
    int readChecksum;
} NmeaData;

//The structure for parsed GPS data
typedef struct
{
    //Coordinated Universal Time
    char utc[10];
    char latitude[10];
    char longitude[10];
    char altitude[10];
    //Horizontal Dilution of Precision (accuracy of measurement)
    char hdop[10];
    char satellites[10];
    char eastVelocity[10];
    char northVelocity[10];
    char upVelocity[10];
} GpsData;

//The structure for parsed IMU data
//(Inertial Measurement Unit)
//All values will be null-terminated and
//if for some reason the data is longer,
//the extra characters beyond 9 will be discarded.
typedef struct
{
    char yaw[10];
    char pitch[10];
    char roll[10];
    char accelX[10];
    char accelY[10];
    char accelZ[10];
} ImuData;

//Updates internal state with the new character
//Returns true if an entire sentence/utterance has
//just finished being read and checksummed correctly.
//nmea is only defined after true is returned.
//The checksum is done between the $ and * characters
bool parseNmea(char newChar, NmeaData* nmea);

//Parses $VNYMR sentences
//Updates internal state with the new character
//Returns true if an entire sentence/utterance has
//just finished being read and checksummed correctly,
//and only when true is returned will imuData be written to.
bool parseImu(char newChar, ImuData* imuData);


//Parses both normal and velocity gps tags...
//Updates internal state with the new character
//Returns true if an entire sentence/utterance has
//just finished being read and checksummed correctly,
//and only when true is returned will gpsData be written to.
bool parseGps(char newChar, GpsData* gpsData);

//Parses $GPGGA sentences
//Updates internal state with the new character
//Returns true if an entire sentence/utterance has
//just finished being read and checksummed correctly,
//and only when true is returned will gpsData be written to.
bool parseBaseGps(char newChar, GpsData* gpsData);

//Parses $PTNLRRF sentences
//Updates internal state with the new character
//Returns true if an entire sentence/utterance has
//just finished being read and checksummed correctly,
//and only when true is returned will gpsData be written to.
bool parseVelocityGps(char newChar, GpsData* gpsData);

#endif
