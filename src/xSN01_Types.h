/*
	This is a library for the SN01 data types
	u-blox Neo-6 GPS

	The board uses I2C for communication.
	
	The board communicates with one I2C devices:
	- NEO-6
	
	Data Sheets:
	u-blox NEO-6 - https://www.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_(GPS.G6-HW-09005).pdf
*/

#ifndef xSN01_Types_h
#define xSN01_Types_h

struct structSN01
{
  unsigned long tick;
  unsigned long looptick;
  uint8_t Type;       		// Type of GPS (library used)
  uint8_t NewData;   		  // 1:New GPS Data
  uint8_t PrintErrors; 		// 1: To Print GPS Errors (for debug)
  uint8_t Fix;        		// >=1:GPS FIX   0:No FIX (normal logic)
  uint8_t Quality;    		// GPS Signal quality
  uint8_t GPS_checksum;		// GPS checksum
  uint8_t GPS_checksum_calc;	
  int16_t bufferidx;			// buffer index 
  char buffer[120];

  long Time;			// variable to store time
  long Latitude;		// variable to store lattitute
  long Longitude;		// variable to store longitude
  long Altitude;		// variable to store altitude
  unsigned int HDOP;	// variable to store horizontal dilution of precision
  unsigned int SoG;		// variable to store speed over ground
  unsigned int CoG;		// variable to store course over ground
  long Date;			// variable to store date
  unsigned int NumSats;	// variable to store number of satelittes connected
};

#endif