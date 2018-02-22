/*
	This is a library for the SN01 
	u-blox Neo-6 GPS

	The board uses I2C for communication.
	
	The board communicates with one I2C devices:
	- NEO-6
	
	Data Sheets:
	u-blox NEO-6 - https://www.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_(GPS.G6-HW-09005).pdf
*/

#include <xSN01.h>
#include "xSN01_Types.h"
#include <stdio.h>
#include <string.h>

/*-------------------Public Function---------------------*/

/********************************************************
 	Constructor
*********************************************************/
xSN01::xSN01(void)
{
	// Device I2C Address
	uint8_t GPS_I2C_ADDRESS	=	0x42;
	mySN01.tick = 0;
	mySN01.Type = 2;
	mySN01.GPS_checksum_calc = false;
	mySN01.bufferidx = 0;
	mySN01.NewData = 0;
	mySN01.Fix = 0;
	mySN01.Quality = 0;
	mySN01.PrintErrors = 0;
}

/********************************************************
 	Configure GPS 
*********************************************************/
void xSN01::begin(void)
{
	millis();
}


/********************************************************
 	Read Data from GPS
*********************************************************/
void xSN01::poll(void)
{
	readGPS();
}

/********************************************************
 	Request Time UTC from GPS
*********************************************************/
long xSN01::getTime(void)
{
	return mySN01.Time;
}

/********************************************************
 	Request altitude from GPS
*********************************************************/
long xSN01::getAltitude(void)
{
	return mySN01.Altitude;
}

/********************************************************
 	Request Latitude from GPS
*********************************************************/
long xSN01::getLatitude(void)
{
	return mySN01.Latitude;
}

/********************************************************
 	Request longitude from GPS
*********************************************************/
long xSN01::getLongitude(void)
{
	return mySN01.Longitude;
}

/********************************************************
 	Request date from GPS
*********************************************************/
long xSN01::getDate(void)
{
	return mySN01.Date;
}

/********************************************************
 	Request horizontal dilution of precision from GPS
*********************************************************/
float xSN01::getHDOP(void)
{
	return (((float)mySN01.HDOP)/10.0);
}

/********************************************************
 	Request speed over ground from GPS
*********************************************************/
float xSN01::getSOG(void)
{
	return (((float)mySN01.SoG)/100.0);
}					
		
/********************************************************
 	Request course over ground from GPS
*********************************************************/
float xSN01::getCOG(void)
{
	return (((float)mySN01.CoG)/100.0);
}	

/********************************************************
 	Request qaulity of results
*********************************************************/
uint8_t xSN01::getQuality(void)
{
	return mySN01.Quality;
}								
 
/********************************************************
 	Request number of satelittes connected from GPS
*********************************************************/
uint8_t xSN01::getSatelitesConnected(void)
{
	return (((float)mySN01.NumSats)/100.0);
}	

/********************************************************
 	Request if GPS has a satelitte fix
*********************************************************/
uint8_t xSN01::getFix(void)
{
	return mySN01.Fix;
}

/*-------------------Private Function---------------------*/

/********************************************************
 	Read GPS data over I2C
********************************************************/
void xSN01::readGPS(void)
{
	uint16_t numBytes = 0;
	// check if timeout on DDC
	if ( (millis() - mySN01.tick) > 100 ){
		// check how many bytes available to read
		numBytes = readBytes16(GPS_I2C_ADDRESS, GPS_BYTES_AVAILABLE);
		if ( numBytes > 0){
			readStream(numBytes);
		}
		// GPS must be read contineously
		mySN01.tick = millis();
	}
}

/********************************************************
 	Read GPS Stream over I2C
********************************************************/
void xSN01::readStream(int numBytes)
{
	char c = 0;
	
	xCore.write1(GPS_I2C_ADDRESS, GPS_DATA_STREAM);

	for ( int i = 0; i < numBytes; i++ ){
		c = xCore.readStream(GPS_I2C_ADDRESS);
		if( c == '$' ){
			mySN01.bufferidx = 0;
			mySN01.buffer[mySN01.bufferidx++] = c;
			mySN01.GPS_checksum = 0;
			mySN01.GPS_checksum_calc = true;
			continue;
		}
		if( c == '\r' ){
			mySN01.buffer[mySN01.bufferidx++] = 0;
			parseNemaGPS();
		}else{
			if(mySN01.bufferidx < (120 - 1)){
				if (c == '*'){
					mySN01.GPS_checksum_calc = false;    // Checksum calculation end
				}

				mySN01.buffer[mySN01.bufferidx++] = c;

				if (mySN01.GPS_checksum_calc){
					mySN01.GPS_checksum ^= c;   // XOR
				}

			}else{
				mySN01.bufferidx = 0; // Buffer overflow : restart
			}
		}
	}
}
 
/********************************************************
 	Parse Nema data
*********************************************************/
void xSN01::parseNemaGPS(void)
{
	byte NMEA_check;
	long aux_deg;
	long aux_min;
	char *parseptr;

	if (strncmp(mySN01.buffer, "$GPGGA", 6) == 0) {   // Check if sentence begins with $GPGGA
		if (mySN01.buffer[mySN01.bufferidx - 4] == '*') {      // Check for the "*" character
			NMEA_check = parseHex(mySN01.buffer[mySN01.bufferidx - 3]) * 16 + parseHex(mySN01.buffer[mySN01.bufferidx - 2]); // Read the checksums characters
			if (mySN01.GPS_checksum == NMEA_check) {     // Checksum validation
				mySN01.NewData = 1;  // New GPS Data
				parseptr = strchr(mySN01.buffer, ',') + 1;
				mySN01.Time = parseNumber(parseptr, 2);         // GPS UTC time hhmmss.ss
				parseptr = strchr(parseptr, ',') + 1;

				mySN01.Latitude = parseNumber(parseptr, 4) + 1;

				aux_deg = parseDecimal(parseptr, 2);     // degrees
				aux_min = parseNumber(parseptr + 2, 4);  // minutes (sexagesimal) => Convert to decimal
				mySN01.Latitude = aux_deg * 10000000 + (aux_min * 50) / 3; // degrees + minutes/0.6  (*10000000) (0.6 = 3/5)

				parseptr = strchr(parseptr, ',') + 1;

				if (*parseptr == 'S'){
					mySN01.Latitude = -1 * mySN01.Latitude;            // South Latitudes are negative
				}

				parseptr = strchr(parseptr, ',') + 1;

				// W Longitudes are Negative
				mySN01.Longitude = parseNumber(parseptr, 4) + 1;

				aux_deg = parseDecimal(parseptr, 3);     // degrees
				aux_min = parseNumber(parseptr + 3, 4);  // minutes (sexagesimal)
				mySN01.Longitude = aux_deg * 10000000 + (aux_min * 50) / 3; // degrees + minutes/0.6 (*10000000)

				//Longitude = -1*Longitude;                   // This Assumes that we are in W longitudes...
				parseptr = strchr(parseptr, ',') + 1;

				if (*parseptr == 'W'){
					mySN01.Longitude = -1 * mySN01.Longitude;            // West Longitudes are negative
				}

				parseptr = strchr(parseptr, ',') + 1;
				mySN01.Fix = parseDecimal(parseptr, 1);
				parseptr = strchr(parseptr, ',') + 1;
				mySN01.NumSats = parseDecimal(parseptr, 2);
				parseptr = strchr(parseptr, ',') + 1;
				mySN01.HDOP = parseNumber(parseptr, 1);         // HDOP * 10
				parseptr = strchr(parseptr, ',') + 1;
				mySN01.Altitude = parseNumber(parseptr, 1) / 10; // Altitude in decimeters*100 = milimeters
	
				if (mySN01.Fix < 1){
				 	mySN01.Quality = 0;      // No FIX
				}else if (mySN01.NumSats < 5){
					mySN01.Quality = 1;      // Bad (Num sats < 5)
				}else if (mySN01.HDOP > 30){
					mySN01.Quality = 2;      // Poor (HDOP > 30)
				}else if (mySN01.HDOP > 25){
					mySN01.Quality = 3;      // Medium (HDOP > 25)
				}else{
					mySN01.Quality = 4;      // Good (HDOP < 25)
				}
			}
		}			
	}else if (strncmp(mySN01.buffer, "$GPVTG", 6) == 0) {   // Check if sentence begins with $GPVTG
		if (mySN01.buffer[mySN01.bufferidx - 4] == '*') {      // Check for the "*" character
			NMEA_check = parseHex(mySN01.buffer[mySN01.bufferidx - 3]) * 16 + parseHex(mySN01.buffer[mySN01.bufferidx - 2]); // Read the checksums characters
			if (mySN01.GPS_checksum == NMEA_check) {     // Checksum validation
				parseptr = strchr(mySN01.buffer, ',') + 1;
				mySN01.CoG = (parseNumber(parseptr, 2));//*0xFF;     // Ground course in degrees * 100
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				mySN01.SoG = (parseNumber(parseptr, 2) * 10 / 36); // Convert Km/h to m/s
				}
			}
	}else if (strncmp(mySN01.buffer, "$GPRMC", 6) == 0) {   // Check if sentence begins with $GPVTG
		if (mySN01.buffer[mySN01.bufferidx - 4] == '*') {     // Check for the "*" character
			NMEA_check = parseHex(mySN01.buffer[mySN01.bufferidx - 3]) * 16 + parseHex(mySN01.buffer[mySN01.bufferidx - 2]); // Read the checksums characters
			if (mySN01.GPS_checksum == NMEA_check) {     // Checksum validation
				parseptr = strchr(mySN01.buffer, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				parseptr = strchr(parseptr, ',') + 1;
				mySN01.Date = parseNumber(parseptr, 1);
			}
		}
	}else{
		mySN01.bufferidx = 0;
	}
}

/********************************************************
 	Parse Hexidecimal
*********************************************************/
uint8_t xSN01::parseHex(char c)
{
	if (c < '0'){ 
		return (0); 
	}
	if (c <= '9'){ 
		return (c - '0'); 
	}
	if (c < 'A'){
	 	return (0); 
	}
	if (c <= 'F'){
	 	return ((c - 'A') + 10);	
	}
}

/********************************************************
 	Parse Decimal
*********************************************************/
long xSN01::parseDecimal(char *str, uint8_t num_car)
{
	long d = 0;
	byte i;
	
	i = num_car;
	while ((str[0] != 0) && (i > 0)) {
		if ((str[0] > '9') || (str[0] < '0')){
			return d;
		}
		d *= 10;
		d += str[0] - '0';
		str++;
		i--;
	}
  return d;	
}

/********************************************************
 	Parse Number
*********************************************************/
long xSN01::parseNumber (char *str, uint8_t numdec)
{
	long d = 0;
	byte ndec = 0;

	while (str[0] != 0) {
		if (str[0] == '.') {
		ndec = 1;
		}else{
			if ((str[0] > '9') || (str[0] < '0')){
        	return d;
			}
			d *= 10;
			d += str[0] - '0';
			if (ndec > 0){
				ndec++;
			}
			if (ndec > numdec){   // we reach the number of decimals...
				return d;
			}
		}
    str++;
	}
	return d;
}

/********************************************************
 	Read 16 Bytes From DDC
*********************************************************/
uint16_t xSN01::readBytes16(byte device, byte reg){
	uint16_t value;
	Wire.beginTransmission((uint8_t)device);
	Wire.write((uint8_t)reg);
	Wire.endTransmission(false);
	Wire.requestFrom((uint8_t)device, (uint8_t)2);
	value = Wire.read(); 
	value <<= 8;
	value |= Wire.read();
	return value;
}
