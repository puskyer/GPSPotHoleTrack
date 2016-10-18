#include "crc8.h"

unsigned char crc8Table[256];

//Creates the CRC-8 table
void initCrc8()
{
    //For each possible byte value...
    for (int i = 0;i < 256;i++)
    {
        //For "each bit" in that value, from high to low
        int valueBits = i;
        for (int j = 0;j < 8;j++)
        {
            //If that bit is set
            if (valueBits & 128)
            {
                valueBits <<= 1;
                //The remaining amount is xored with
                //A magical number that messes everything up! =]
                valueBits ^= 0xD5;
            }
            else
            {
                //Shift that bit out (also multiple remainder)
                valueBits <<= 1;
            }
        }
        crc8Table[i] = valueBits;
    }
}

//Calculates the CRC-8 checksum of the given data string
//Starting with a given initialChecksum so that multiple
//Calls may be strung together. Use 0 as a default.
unsigned char crc8(const char* data, unsigned char initialChecksum)
{
    //Check if we need to initialize
    //We check index one, which will need to not be 0
    //While index zero actually stays 0
    if (crc8Table[1] == 0)
    {
        initCrc8();
    }
    unsigned char checksum = initialChecksum;
    const char* charOn = data;
    while (*charOn)
    {
        checksum = crc8Table[checksum ^ *charOn];
        charOn++;
    }
    return checksum;
}
