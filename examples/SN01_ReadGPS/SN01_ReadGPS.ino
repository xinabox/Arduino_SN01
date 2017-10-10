/*************************************************************
  This is an examples for the SN01
  GPS Module
  
  This examples read data from a NEO 6M GPS
    
  The sensor communicates over the I2C Bus.
  
  ------------------------TIPS--------------------------
  ----->Wire.begin(2,14); Communication with CW01
  
*************************************************************/

#include "xCore.h"
#include "xSN01.h"

long tick_Print = 0;
long tick_Poll = 0;

void setup(){
	// Start the Serial comms 
	Serial.begin(115200);
  
	// Display what is being tested on serial monitor
	Serial.println("====================================");
	Serial.println("       XINABOX SN01 GPS Data        ");
	Serial.println("====================================");
  
	// Start the I2C Communication
	Wire.begin(); 
  
	// Start the Sensor
	SN01.begin();
}

void loop(){
	// Create a variable to store the data read from SN01
	long time = 0;
	long latitude = 0;
	long longitude = 0;
	long date = 0;
  
	// Poll the sensor to read all available data
	SN01.poll();
  
  	// Use a timer to print data once a second
  	if((millis() - loop_timing) > 1000){

  		// Get the date from GPS
		date = SN01.getDate();
		
		// Get the time from the GPS 
		time = SN01.getTime();

		// Get the latitude from GPS
		latitude = SN01.getLatitude();

		// Get the longitude from GPS
		longitude = SN01.getLongitude();
	  
		// Display the recorded data over the serial monitor
		Serial.print("GPS Time: ");
		Serial.println(time);
		Serial.print("GPS Date: ");
		Serial.println(date);
		Serial.print("GPS Latitude: ");
		Serial.println(latitude);
		Serial.print("GPS longitude: ");
		Serial.println(longitude);

		Serial.println();
		
		loop_timing = millis();
	}
}