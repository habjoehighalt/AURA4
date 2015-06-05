/*
 
 Select Mini pro 168 5v 16 Mhx
 HABJOE1 Arduino Y Flight script
 HABJOE - Andrew Myatt and Joseph Myatt Jan/2013

 This example connects the GPS via Software Serial.
 Initialise the GPS Module in Flight Mode and then echo's out the NMEA Data to the Arduinos onboard Serial port.
 
 This code is in the public domain.
 Incorporating code by:
 J Coxon (http://ukhas.org.uk/guides:falcom_fsa03)
 
 Incorprating modified elements of acceleroMMA7361 - Library for retrieving data from the MMA7361 accelerometer. 
 Copyright 2011-2012 Jef Neefs (neefs@gmail.com) and Jeroen Doggen (jeroendoggen@gmail.com)
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 NTX2 Radio. 2012 by M0UPU as part of a UKHAS Guide on linking NTX2 Modules to Arduino.
 RTTY code from Rob Harrison Icarus Project. http://ukhas.org.uk
 UBX code. http://ukhas.org.uk
 + Various others sources of code!!
 
 
 This software is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This software is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
*/ 
#include <Wire.h>
#include <I2Cdev.h>
#include <EasyTransferI2C_NL.h>
#include <String.h>
#include <util/crc16.h>

#define DEBUG_ON

/***************************************************
 * Main program
 **************************************************/

#define LED_ON 1
#define LED_OFF 0

//
// Arduino Pin assignment
//
static int PIN_NTX_TX = 9;        //Note: NTX Transmitter out
static int PIN_LED_GREEN = 13;    //Fixed: GREEN
	

 /***********************************************************************************************************************************
 * Data definitions
 * 
 *
 */

typedef struct {
		uint16_t t_count;  				//2
		byte 	year;					//1 		  
		byte    month;					//1	  
		byte    day;					//1	  
		byte    hour;					//1
		byte    minute;					//1
		byte    second;					//1
		long    i_lat;					//4
		long    i_long;					//4
		long    i_alt; 					//4	
		int16_t BMP085_T;				//2		Pressure Temp (*x0.1 DegC)
		int16_t DS18B20_T;				//2		DS18B20 Temp (*x0.1 DegC)		
		byte	sMobile;				//1
		byte	balloonCutDoFire;		//1
		byte	balloonCutFired;	  	//1
		byte 	b_sats;					//1
} IC2DATA_STRUCTURE;
     	 
EasyTransferI2C_NL ET; 			   //Easy Transfer
IC2DATA_STRUCTURE i2cVals;		             
 
/*********************************************************
 * User settings
 */

static char USR_PAYLOADNAME[]		= "$$AURA4";								// Include the $$ as well.
char 		telemetryString[80];

#define I2C_SLAVE_ADDRESS 10

/***********************************************************************************************************************************
 * user functions
 * 
 * 
 */
 
 /*****************************************
 * datadump
 */  

void setup() {
	pinMode(PIN_LED_GREEN, OUTPUT);
	pinMode(PIN_NTX_TX,OUTPUT);
	setPwmFrequency(PIN_NTX_TX, 1);

	LED_Status(LED_ON,0);
#ifdef DEBUG_ON
	Serial.begin(19200); 
#endif
	Wire.begin(I2C_SLAVE_ADDRESS);																// Uses the I2C interface to get the telemetry values.
	ET.begin(details(i2cVals), &Wire);
	//define handler function on receiving data ps. you don't want to put anything in this that takes any time.!!!
	Wire.onReceive(receive);	 

	for (int i = 0; i<10;i++){
		LED_Status(LED_ON,200);
		LED_Status(LED_OFF,100);
	}

}

void receive(int numBytes) {
// Don't put anything in this function
}

void LED_Status(int stat, int intDelay){
  if (stat == LED_ON){
    digitalWrite(PIN_LED_GREEN, HIGH);
  } 
  else {
    digitalWrite(PIN_LED_GREEN, LOW); 
  }
  if (intDelay > 0 ) delay(intDelay);
}

void LED_Status(int stat){
  LED_Status(stat, 1000);
}

/***************************************************
 * NTX2 Transmit functions
 * 
 **************************************************/

void rtty_txstring ( char * string) {

  //Simple function to sent a char at a time to rtty_txbyte function. 
  char c;
  c = *string++;    
  while (c != '\0'){
    rtty_txbyte (c);
    c = *string++;
  }
  rtty_txbyte (c);

}

void rtty_txbyte (char c){
  /* Simple function to sent each bit of a char to 
   ** rtty_txbit function. 
   ** NB The bits are sent Least Significant Bit first
   **
   ** All chars should be preceded with a 0 and 
   ** proceded with a 1. 0 = Start bit; 1 = Stop bit
   **/

  int i;
  rtty_txbit (0); // Start bit
  // Send bits for for char LSB first 

  for (i=0;i<7;i++) {// Change this here 7 or 8 for ASCII-7 / ASCII-8
    if (c & 1) rtty_txbit(1); 
    else rtty_txbit(0); 
    c = c >> 1;
  }
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit){
	if (bit) {
		// high
		analogWrite(PIN_NTX_TX,120);
	} else {
		// low
		analogWrite(PIN_NTX_TX,100);
	}

	delayMicroseconds(10000); // For 50 Baud uncomment this and the line below. 
	delayMicroseconds(10150); // For some reason you can't do 20150 it just doesn't work.
}

uint16_t gps_CRC16_checksum (char *string){
  size_t i;
  uint16_t crc;
  uint8_t c;
  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  } 
  return crc;
}

void setPwmFrequency(int pin, int divisor) {
 byte mode;
 if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
 switch(divisor) {
 case 1:
 mode = 0x01;
 break;
 case 8:
 mode = 0x02;
 break;
 case 64:
 mode = 0x03;
 break;
 case 256:
 mode = 0x04;
 break;
 case 1024:
 mode = 0x05;
 break;
 default:
 return;
 }
 if(pin == 5 || pin == 6) {
 TCCR0B = TCCR0B & 0b11111000 | mode;
 }
 else {
 TCCR1B = TCCR1B & 0b11111000 | mode;
 }
 }
 else if(pin == 3 || pin == 11) {
 switch(divisor) {
 case 1:
 mode = 0x01;
 break;
 case 8:
 mode = 0x02;
 break;
 case 32:
 mode = 0x03;
 break;
 case 64:
 mode = 0x04;
 break;
 case 128:
 mode = 0x05;
 break;
 case 256:
 mode = 0x06;
 break;
 case 1024:
 mode = 0x7;
 break;
 default:
 return;
 }
 TCCR2B = TCCR2B & 0b11111000 | mode;
 }
}

void loop() {
	LED_Status(LED_OFF,0);
	if(ET.receiveData()) {
		LED_Status(LED_ON,0);
		//Need to be made specific to the actual payload being used.

		sprintf(telemetryString, "%s,%d,%02d:%02d:%02d,%ld,%ld,%ld,%d,%d,%d,%d,%d", 
		USR_PAYLOADNAME,i2cVals.t_count,i2cVals.hour,i2cVals.minute,i2cVals.second,
		i2cVals.i_lat,i2cVals.i_long,i2cVals.i_alt,i2cVals.BMP085_T,i2cVals.DS18B20_T,i2cVals.b_sats,
		i2cVals.sMobile,i2cVals.balloonCutFired);

		unsigned int CHECKSUM = gps_CRC16_checksum(telemetryString);  // Calculates the checksum for this datastring
		char checksum_str[6];
		sprintf(checksum_str, "*%04X\n", CHECKSUM);
		strcat(telemetryString,checksum_str);	
#ifdef DEBUG_ON
		Serial.println(telemetryString);
#endif
		rtty_txstring("$$");
		rtty_txstring(telemetryString);
	}
}
