/*
	This is a library for the SN01 
	u-blox Neo-6 GPS

	The board uses I2C for communication.
	
	The board communicates with one I2C devices:
	- NEO-6
	
	Data Sheets:
	u-blox NEO-6 - https://www.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_(GPS.G6-HW-09005).pdf
*/

#ifndef Arduino_SN01_h
#define Arduino_SN01_h

// System Include
#include <inttypes.h>
#include "xCore.h"
#include "SN01_Types.h"
#include <stdio.h>
#include <string.h>

// GPS Registers
#define GPS_BYTES_AVAILABLE		0xFD
#define GPS_DATA_STREAM			0xFF

class xSN01: public xCoreClass
{
	public:
		/*
		* Create instance of SN01 data structure
		*/	
		structSN01 mySN01;

		/*
		* Constructor
		*/	
		xSN01(void);

		/*
		* Runs the setup of the sensor. 
		* Call this in setup(), before reading any sensor data.
		*/											
		void begin(void);							

		/*
		* Polls the sensor to check for new data
		* Call this in loop(), before reading any sensor data
		*/	
		void poll(void);						

		/*
		* GPS time
		* Call this in loop().
		*
		* @return Time 
		*/	
		long getTime(void);	

		/*
		* GPS Date
		* Call this in loop().
		*
		* @return Date
		*/	
		long getDate(void);									


		/*
		* GPS Latitude 
		* Call this in loop().
		*
		* @return true if setup was successful.
		*/
		long getLatitude(void);					

		/*
		* GPS Longitude
		* Call this in loop().
		*
		* @return Longitude
		*/	
		long getLongitude(void);					

		/*
		* GPS  Altitude
		* Call this in loop().
		*
		* @return Altitude
		*/	
		long getAltitude(void);					

		/*
		* GPS Horizontal Dilution of Precision
		* Call this in loop().
		*
		* @return HDOP
		*/	
		float getHDOP(void);			

		/*
		* GPS Speed over Ground
		* Call this in loop().
		*
		* @return SoG
		*/	
		float getSOG(void);				

		/*
		* GPS Course over Ground  
		* Call this in loop().
		*
		* @return CoG
		*/	
		float getCOG(void);				

		/*
		* GPS data qaulity. 
		* Call this in loop().
		*
		* @return Quality
		*/	
		uint8_t getQuality(void);									

		/*
		* GPS number of satelittes is connected to
		* Call this in loop().
		*
		* @return NumSats.
		*/	
		uint8_t getSatelitesConnected(void);	

		/*
		* GPS Fix to determine is connection is reliable
		* Call this in loop().
		*
		* @return Fix.
		*/	
		uint8_t getFix(void);
		
	private:
		/*
		* Reads GPS Register for amount of new data bytes available
		*/	
		void readGPS(void);				

		/*
		* Reads GPS stream and stores data in a buffer
		*
		* @param numBytes, number of bytes to read from stream
		*/	
		void readStream(int numBytes);		

		/*
		* Parse NEMA data to get usable data
		*
		*/	
		void parseNemaGPS(void);	

		/*
		* Parse HEX d
		*
		* @return
		*/	
		uint8_t parseHex(char c) ;		// parse the hex data

		/*
		* Parse Decimal
		*
		* @return Fix.
		*/	
		long parseDecimal(char *str, uint8_t num_car);	// parse decimal points

		/*
		* Parse Number
		*
		* @return Fix.
		*/	
		long parseNumber (char *str, uint8_t numdec);	// parse number

		unsigned long looptiming;		// used to calculate DDC timeout 2 seconds
		
		uint8_t GPS_I2C_ADDRESS;
};

#endif
