#include "gpsimu.h"
#include <string.h>

#define GPS_TAG "GPGGA,"
#define GPS_VELOCITY_TAG "PTNLRRF,"
#define IMU_TAG "VNYMR,"

//$VNYMR,+006.380,+000.023,-001.953,+1.0640,-0.2531,+3.0614,+00.005,+00.344,-09.758,-0.001222,-0.000450,-0.001218*67
//$GPGGA,121505,4807.038,N,01131.324,E,1,08,0.9,133.4,M,46.9,M,,*48
//$PTNLRRF,b,c,xxxxxx,xx,x,llll.lllll,d,yyyyy.yyyyy,e,xxxxx,1.1,2.2,3.3*2E
//$PTNLF,bc.lde*2E

//Updates internal state with the new character
//Returns true if an entire sentence/utterance has
//just finished being read and checksummed correctly.
//nmea is only defined after true is returned.
//The checksum is done between the $ and * characters
bool parseNmea(char newChar, NmeaData* nmea)
{
    //Do we need to find the $ marker of an utterance?
    if (!nmea->hasBegunUtterance)
    {
        if (newChar == '$')
        {
            nmea->hasBegunUtterance = true;
            //Clear old data
            for (int i = 0;i < nmea->numDatums; i++)
            {
                nmea->datums[i][0] = '\n';
            }
        }
        return false;
    }
    //Check all characters till we get a null
    else if (nmea->tag[nmea->tagIndex])
    {
        //We ought to have the next char of the tag now
        if (newChar == nmea->tag[nmea->tagIndex])
        {
            nmea->tagIndex++;
            //And we keep up with the checksum too, now
            nmea->runningChecksum ^= newChar;
            return false;
        }
    }
    //Read in central data until a * occurs, signifying a checksum
    else if (!nmea->checksumBegun)
    {
        //$ causes abort and restart....
        if (newChar != '$')
        {
            //Read in chars for a datum until terminated with ',' or '*'
            if (newChar == ',' || newChar == '*')
            {
                //Null-terminate the datum
                nmea->datumData[nmea->datumDataIndex] = '\0';
    
                //Is this datum requested?
                //Ignore unrequested ones...
                for (int i = 0;i < nmea->numDatums; i++)
                {
                    if (nmea->datumOn == nmea->datumIndices[i])
                    {
                        strncpy(nmea->datums[i], nmea->datumData, sizeof(nmea->datumData));
                    }
                }
                
                if (newChar == '*')
                {
                    nmea->checksumBegun = true;
                    //Also, make sure readChecksum is -1, as it may have initialized to 0
                    nmea->readChecksum = -1;
                }
                else
                {
                    nmea->datumOn++;
                    nmea->datumDataIndex = 0;
                    //Add to checksum
                    nmea->runningChecksum ^= newChar;
                }
                return false;
            }
            //We leave one spot for the null-terminator
            else if (nmea->datumDataIndex < sizeof(nmea->datumData) - 1)
            {
                nmea->datumData[nmea->datumDataIndex++] = newChar;
            }
            nmea->runningChecksum ^= newChar;
            return false;
        }
    }
    //Checksums here use uppercase hex
    else
    {
        int checkNum = -1;
        if (newChar >= 'A' && newChar <= 'F')
        {
            checkNum = (newChar - 'A') + 10;
        }
        else if (newChar >= '0' && newChar <= '9')
        {
            checkNum = (newChar - '0');
        }
        if (checkNum != -1)
        {
            //Are we on the first byte?
            if (nmea->readChecksum == -1)
            {
                nmea->readChecksum = checkNum * 16;
                return false;
            }
            else
            {
                nmea->readChecksum += checkNum;
                //Is it what we have come up with?
                if (nmea->readChecksum == nmea->runningChecksum)
                {
                    //Yay! We win!
                    //reset state before returning...
                    nmea->hasBegunUtterance = nmea->datumBegun = nmea->checksumBegun = false;
                    nmea->runningChecksum = nmea->tagIndex = nmea->datumOn = nmea->datumDataIndex = 0;
                    nmea->readChecksum = -1;
                    
                    return true;
                }
            }
        }
    }
    //If we fell through without returning, an error occurred and
    //we ought to reset the state.
    nmea->hasBegunUtterance = nmea->datumBegun = nmea->checksumBegun = false;
    nmea->runningChecksum = nmea->tagIndex = nmea->datumOn = nmea->datumDataIndex = 0;
    nmea->readChecksum = -1;

    //Give this a chance on failure to start a new sentence
    if (newChar == '$')
    {
        parseNmea(newChar, nmea);
    }
    
    return false;
}

//Fixes the latitude and longitude readings from the GPS
//into a nicer format.
//For example, "4124.8963","N" -> "+41.248963"
//Direction may be N,W,E,S
//Places the corrected version into output, which must be at least 11 bytes large.
//If latLon has more than 9 characters, the remaining ones will be thrown out.
//Does nothing and returns false if latLon and direction did not match the expected format.
//On success, returns true.
bool fixLatLon(char* output, const char* latLon, const char* direction)
{
    //Keep track of current locations in strings
    int outputIndex = 0;
    int latLonIndex = 0;

    //We need to move the decimal point two characters to the left
    //And add the + or -.
    char sign;
    switch (direction[0])
    {
        case 'N':
        case 'E':
            sign = '+';
            break;
        case 'S':
        case 'W':
            sign = '-';
            break;
        default:
            return false;
    }
    
    //Find the '.'!
    char* dotLocation = strchr(latLon, '.');
    if (!dotLocation)
    {
        return false;
    }
    int toDot = dotLocation - latLon;

    //Place sign
    output[outputIndex++] = sign;

    //Place numbers before decimal, less two of them
    //Only as space in output allows...
    int toCopyBeforeNewDecimal = toDot - 2;
    while (toCopyBeforeNewDecimal > 0 && outputIndex < 9)
    {
        output[outputIndex++] = latLon[latLonIndex++];
        toCopyBeforeNewDecimal--;
    }

    //If we have at least two more characters...
    if (outputIndex < 8)
    {
        //Add the dot
        output[outputIndex++] = '.';
        
        //Place any needed zeros because of a decimal
        //preceded by only 0 or 1 numbers
        //Only, of course, if we still have space
        int zerosNeeded = 2 - toDot;
        while (zerosNeeded > 0 && outputIndex < 9)
        {
            output[outputIndex++] = '0';
            zerosNeeded--;
        }

        //Place any numbers that had before preceded the old dot
        //That now follow it. At most two, maybe less.
        int movingNumbersToCopy = (toDot > 2) ? 2 : toDot;
        while (movingNumbersToCopy > 0 && outputIndex < 9)
        {
            output[outputIndex++] = latLon[latLonIndex++];
            movingNumbersToCopy--;
        }

        //Then skip the dot in latLon
        latLonIndex++;

        //Then copy what we can that remains in latLon, if anything
        while (latLon[latLonIndex] && outputIndex < 9)
        {
            output[outputIndex++] = latLon[latLonIndex++];
        }
    }

    //Add null-terminator to output
    output[outputIndex] = '\0';

    //Success!
    return true;
}

//Parses $VNYMR sentences
//Updates internal state with the new character
//Returns true if an entire sentence/utterance has
//just finished being read and checksummed correctly,
//and only when true is returned will imuData be written to.
bool parseImu(char newChar, ImuData* imuData)
{
    static ImuData imuD;
    static NmeaData nData;
    //One-time initialization
    if (!nData.tag)
    {
        nData.tag = IMU_TAG;
        nData.numDatums = 6;
        static int indices[] = {0, 1, 2, 6, 7, 8};
        nData.datumIndices = indices;
        static char* datums[] = {imuD.yaw, imuD.pitch, imuD.roll, imuD.accelX, imuD.accelY, imuD.accelZ};
        nData.datums = datums;
    }

    bool success = parseNmea(newChar, &nData);
    if (success)
    {
        //Copy over to output on success...
        memcpy(imuData, &imuD, sizeof(imuD));
    }
    
    return success;
}

//Parses both $GPGGA and $PTNLRRF sentences...
//Updates internal state with the new character
//Returns true if an entire sentence/utterance has
//just finished being read and checksummed correctly,
//and only when true is returned will gpsData be written to.
bool parseGps(char newChar, GpsData* gpsData)
{
    bool baseSuccess = parseBaseGps(newChar, gpsData);
    bool velocitySuccess = parseVelocityGps(newChar, gpsData);
    return baseSuccess || velocitySuccess;
}

//Parses $GPGGA sentences
//Updates internal state with the new character
//Returns true if an entire sentence/utterance has
//just finished being read and checksummed correctly,
//and only when true is returned will gpsData be written to.
bool parseBaseGps(char newChar, GpsData* gpsData)
{
    static GpsData gpsD;
    static NmeaData nData;
    //These are intermediates...
    static char latitude[10];
    static char latitudeDirection[10];
    static char longitude[10];
    static char longitudeDirection[10];
    //One-time initialization
    if (!nData.tag)
    {
        nData.tag = GPS_TAG;
        nData.numDatums = 8;
        static int indices[] = {0, 1, 2, 3, 4, 6, 7, 8};
        nData.datumIndices = indices;
        static char* datums[] = {gpsD.utc, latitude, latitudeDirection, longitude, longitudeDirection,
                                 gpsD.satellites, gpsD.hdop, gpsD.altitude};
        nData.datums = datums;
    }

    bool success = parseNmea(newChar, &nData);
    if (success)
    {
        //Correct the form of latitude and longitude
        fixLatLon(gpsD.latitude, latitude, latitudeDirection);
        fixLatLon(gpsD.longitude, longitude, longitudeDirection);
        //Copy over to output on success...
        memcpy(gpsData, &gpsD, sizeof(gpsD));
    }
    
    return success;
}

//Parses $PTNLRRF sentences
//Updates internal state with the new character
//Returns true if an entire sentence/utterance has
//just finished being read and checksummed correctly,
//and only when true is returned will gpsData be written to.
bool parseVelocityGps(char newChar, GpsData* gpsData)
{
    static GpsData gpsD;
    static NmeaData nData;
    //One-time initialization
    if (!nData.tag)
    {
        nData.tag = GPS_VELOCITY_TAG;
        nData.numDatums = 3;
        static int indices[] = {10, 11, 12};
        nData.datumIndices = indices;
        static char* datums[] = {gpsD.eastVelocity, gpsD.northVelocity, gpsD.upVelocity};
        nData.datums = datums;
    }

    bool success = parseNmea(newChar, &nData);
    if (success)
    {
        //Copy over to output on success...
        memcpy(gpsData, &gpsD, sizeof(gpsD));
    }
    
    return success;
}
