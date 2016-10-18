#ifndef CRC8_H
#define CRC8_H

//Calculates the CRC-8 checksum of the given data string
//Starting with a given initialChecksum so that multiple
//Calls may be strung together. Use 0 as a default.
unsigned char crc8(const char* data, unsigned char initialChecksum);

#endif
