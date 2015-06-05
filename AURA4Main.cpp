/*
    
 HABJOE  Arduino Flight script
 HABJOE - Copyright Andrew Myatt and Joseph Myatt March/2013
 
 KEYWORDS: ARDUINO MEGA, MY_HIGH ALTITUDE BALLOON, BMP085, ADXL345, UBLOX, I2C, WIRING, SDCARD
 
 This code is in the public domain.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 This software is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This software is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 POLLS SENSORS GENRATES DATA STRUCTURE WHICH IS WRITTEN TO SD CARD AND SENT VIA 
 I2C INTERFACE TO MOBILE MODULE AND TRANSMITTER
 
 Xno,Yno,Date,Time,Lat,Long,altitude,angle,ascentRate,xAcc,yAcc,zAcc,Xgx,Xgy,Xgz,Xmx,Xmy,Xmz,tempin,pressure,tempex,compass,b_sats,age,ihdop,rOnOff,sMobile,sCIPSTATUS,sRssi,balloonCutDoFire,balloonCutFired,ascent
*/

/*
 * Include Files
 */
#include <SPI.h>
#include <SdFat.h>
#include <EasyTransferI2C_NL.h>
#include <TinyGPS_HJOE.h>
#include <Wire.h>
#include <L3G.h>
#include <ADXL345.h>
#include <Comp6DOF_n0m1.h>
#include <HMC5883L.h> // Reference the HMC5883L Compass Library
#include <BMP085.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>

/*
 * Definitions
 */
// build modifiers
//#define DEBUG_ON
#define MY_HIGH HIGH
#define MY_LOW LOW

// LED Definitions
#define SET_LED_WHITE 4
#define SET_LED_RED 3
#define SET_LED_BLUE 2
#define SET_LED_GREEN 1
#define SET_LED_OFF 0

// Millisecond wait before Sending to Transmit device
#define WAIT_NTXB  					9000
#define WAIT_SIM900  				10000

// Define filenames
#define SD_LOG_FILE        			"AURA4.CSV"
#define SD_C1_FILE        			"C1.txt"
#define SD_C2_FILE        			"C2.txt"
#define SD_C3_FILE        			"C3.txt"

// Arduino Pin assignment
#define PIN_GPS_RX 0        		//Fixed:Note: RX on Board is connected to RX on GPS Board
#define PIN_GPS_TX 1        		//Fixed:Note: TX on Board is connected to TX on GPS Board
#define PIN_LED_RED 6        		//Fixed: Red LED
#define PIN_LED_BLUE 5       		//Fixed: Blue LED
#define PIN_LED_GREEN 7      		//Fixed: Blue GREEN
#define PIN_IC2_SDA 20       		//Fixed: SDA
#define PIN_IC2_SLC 21       		//Fixed: SLC
#define PIN_SPI_CS 53        		//Fixed: Card Select for SD Card
#define ONE_WIRE_BUS 12				// Data wire is plugged into port 2 on the Arduino

#define I2C_SLV_SIM900_ADDRESS 9	//define slave i2c address
#define I2C_SLV_NTXB_ADDRESS 10		//define slave i2c address
#define ADXL345_ADDRESS 0x53		// Device address as specified in data sheet 
#define HMC5883_ADDRESS 0x1E 		//0011110b, I2C 7bit address of HMC5883
#define BMP085_ADDRESS 0x77  		// I2C address of BMP085
#define aref_voltage 3.3 

//#define SD_BUFF_SIZE 	512
#define SD_BUFF_SIZE 	1024
char SDBuffer[SD_BUFF_SIZE];

/* State of the flight */
#define PRESSURE_STEP 1000			//2000 pascals, about 300meters at see level
#define PRESSURE_INTEGRATE 5		//Number of pressure reading to average over
#define PRESSURE_CUT_ABOVE 80000	//pascals approx 2000 Metres

#define MIN_ALT_STEP 100			//cm/second min ascent rate i,e 1m/s
#define ALTITUDE_INTEGRATE 90		//Number of altitude reading to average over approx 30 seconds
#define ALTITUDE_CUTAWAY 3700000	//37,000 meters

int  press_idx = 0;
int  alt_idx = 0;
int  r_ascent = 1;

	

//Data structure used for sending data via the I2C and to the SD card
typedef union {
	typedef struct{
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
		//
		// End of mapping with I2C data.
		//
		byte 	hundredths;				//1
		long    i_angle;				//4	
		long    i_Hspeed;				//4		Horizontal speed
		long    i_Vspeed;				//4		Vertical speed
		unsigned long age; 				//4
		unsigned long ihdop;			//4
		int16_t Xax;					//2		accel x
		int16_t Xay;					//2		accel y
		int16_t Xaz;					//2		accel z
		int16_t Xgx;					//2		gyro x
		int16_t Xgy;					//2		gyro y
		int16_t Xgz; 					//2		gyro z 
		int16_t Xmx;					//2		mag x
		int16_t Xmy;					//2		mag y
		int16_t Xmz; 					//2		mag z 
		long    i_compass;				//4  	compass bearing using tilt compensation etc.
		long	BMP085_PFULL;			//4 
		long	BMP085_TFULL;			//4
		unsigned long usl_count;		//4	 
		byte	rOnOff;					//1 
		byte	sCIPSTATUS;				//1
		int		sRssi;					//2
		byte	ascent;					//1
		} LOG_DATA_STRUCTURE;	
    LOG_DATA_STRUCTURE vals;

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
	} I2C_STRUCTURE;
        I2C_STRUCTURE i2cOut;
} MYOUTPACKET;
		
typedef struct {
	byte	rOnOff;						//1 
	byte	sMobile;					//1
	byte	sCIPSTATUS;					//1
	int		sRssi;						//2
	byte	balloonCutFired;			//1
} IC2DATA_INSTRUCTURE;
IC2DATA_INSTRUCTURE i2cIn;
	
// Declare datatype variables
EasyTransferI2C_NL 	ETout;					//Easy Transfer
EasyTransferI2C_NL 	ETin;					//Easy Transfer
MYOUTPACKET			mD;						//packet of telemetry for other slave arduino's
SdFat 				SD;						//SD
SdFile 				dataFile;				//SDFile
SdFile				flagFile;				//SDflag
L3G 				LGgyro;					//L3G Gyro
TinyGPS_HJOE 		gps;					//GPS
ADXL345 			accel;					//accelerometer
HMC5883L 			compass;				//Magnetometer/Compass
Comp6DOF_n0m1 		sixDOF;					//Tilt compensation from Compass
BMP085 				dps;					//Pressure and Temp
OneWire 			oneWire(ONE_WIRE_BUS);	// Set up which Arduino pin will be used for the 1-wire interface to the sensor
DallasTemperature 	sensors(&oneWire);
DeviceAddress 		outsideThermometer = { 0x28, 0x44, 0xD8, 0x7D, 0x5, 0x0, 0x0, 0xD7 };


// Declare utility global variables variables
int 			error = 0;
bool			SENDWIRE = false;
bool			NEWGPSDATA;
unsigned long 	elapseSIM900;
unsigned long 	elapseNTXB;
unsigned long 	elapseAltCheck;
unsigned long	r_press;				//4
unsigned long	a_press;				//4
unsigned long	r_alt;					//4
unsigned long	a_alt;					//4		
/*
 * functions
 */
 
 /***************************************************
 * writes the data values to SD card as time 
 * efficiently as possible. 
 **************************************************/

void writeSDData() {
    char SDString[200] = "";
   	int tmp_year = mD.vals.year + 2000;	
	sprintf(SDString, "%ld,%d,%04d-%02d-%02d,%02d:%02d:%02d.%02d,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%ld,%d,%ld,%ld,%d,%d,%d,%d,%d,%d,%d\n", 
	mD.vals.usl_count,mD.vals.t_count,
	tmp_year, mD.vals.month, mD.vals.day,
	mD.vals.hour, mD.vals.minute, mD.vals.second, mD.vals.hundredths,
	mD.vals.i_lat,mD.vals.i_long,mD.vals.i_alt,mD.vals.i_angle,mD.vals.i_Hspeed,
	mD.vals.Xax,mD.vals.Xay,mD.vals.Xaz,
	mD.vals.Xgx,mD.vals.Xgy,mD.vals.Xgz,
	mD.vals.Xmx,mD.vals.Xmy,mD.vals.Xmz,
	mD.vals.BMP085_TFULL,mD.vals.BMP085_PFULL,mD.vals.DS18B20_T,
	mD.vals.i_compass,mD.vals.b_sats,mD.vals.age,mD.vals.ihdop,
	mD.vals.rOnOff, mD.vals.sMobile, mD.vals.sCIPSTATUS,
	mD.vals.sRssi, mD.vals.balloonCutDoFire,mD.vals.balloonCutFired,
	mD.vals.ascent);
	
   #ifdef DEBUG_ON	
		Serial.println(SDString);
   #endif 
		
   if (sizeof(SDBuffer)-strlen(SDBuffer) < strlen(SDString)) {
		dataFile.write(SDBuffer,strlen(SDBuffer));
		dataFile.sync();		
		memset(SDBuffer, 0, sizeof(SDBuffer));
   }
   strcat(SDBuffer,SDString);
}

/***************************************************
 * LED STATUS DISPLAY FUNTIONS
 * LED is a Common Anode, therefore the pin is the cathode
 * and must be set MY_LOW to be on, and MY_HIGH to be turned off! 
 **************************************************/
void SET_LED_Status(int stat, int intDelay){
  
  digitalWrite(PIN_LED_RED, MY_HIGH);
  digitalWrite(PIN_LED_GREEN, MY_HIGH);
  digitalWrite(PIN_LED_BLUE, MY_HIGH); 
  if (stat == SET_LED_RED) {
		digitalWrite(PIN_LED_RED, MY_LOW);
  } else if (stat == SET_LED_BLUE) {
		digitalWrite(PIN_LED_BLUE, MY_LOW);
  } else if (stat == SET_LED_GREEN) {
		digitalWrite(PIN_LED_GREEN, MY_LOW);
  } else if (stat == SET_LED_WHITE){
		digitalWrite(PIN_LED_RED, MY_LOW);
		digitalWrite(PIN_LED_GREEN, MY_LOW);
		digitalWrite(PIN_LED_BLUE, MY_LOW); 
  }
  if (intDelay > 0 ) delay(intDelay);
}

void SET_LED_Status(int stat){
  SET_LED_Status(stat, 1000);
}

/*
 * feedgps - reads the GPS serial input and passes input to GPS.endcode function 
 */
 static bool feedgps() {
  while (Serial1.available()) {
    if (gps.encode(Serial1.read()))
      return true;
	}
  return false;
}
/*
 * Setup 
 */
void setup() {
	wdt_enable(WDTO_8S);
	wdt_reset();
	//Setup Ports
	Serial.begin(115200);				//Start Debug Serial 0
	Serial1.begin(9600); 				//Start GPS Serial 1
	pinMode(PIN_LED_GREEN, OUTPUT);		//Blue GREEN
	pinMode(PIN_LED_RED, OUTPUT);		//Blue RED
	pinMode(PIN_LED_BLUE, OUTPUT);		//Blue LED
	pinMode(PIN_SPI_CS,OUTPUT);  		//Chip Select Pin for the SD Card
	pinMode(10, OUTPUT);				//SDcard library expect 10 to set set as output.
	
	// Initialise the GPS
	wdt_disable();
	gps.init();						
	gps.configureUbloxSettings();		// Configure Ublox for MY_HIGH altitude mode
	wdt_enable(WDTO_8S);
	// join I2C bus //start I2C transfer to the Module/Transmitter
	Wire.begin();
	ETout.begin(details(mD.i2cOut), &Wire);	//setup the data structure to transfer out
	ETin.begin(details(i2cIn), &Wire);		//setup the data structure to transfer in
	
	//Start up the LGgyro
    if (LGgyro.init()) {
		#ifdef DEBUG_ON	
			Serial.println("LGgyro OK");
		#endif
		LGgyro.enableDefault();
	} else {
		#ifdef DEBUG_ON	
			Serial.println("LGgyro not working");
		#endif
		SET_LED_Status(SET_LED_WHITE,500); 	//White LED
		SET_LED_Status(SET_LED_RED,1000); 	//Red LED 
	}

	//Start up the accelerometer
	accel = ADXL345(); 						// Create an instance of the accelerometer
	if(accel.EnsureConnected()) {			// Check that the accelerometer is connected.
		#ifdef DEBUG_ON	
			Serial.println("Connected to ADXL345.");
		#endif		
		accel.SetRange(2, true);				// Set the range of the accelerometer to a maximum of 2G.
		accel.EnableMeasurements();				// Tell the accelerometer to start taking measurements.		
	} else{
		#ifdef DEBUG_ON	
			Serial.println("Could not connect to ADXL345.");
		#endif
		SET_LED_Status(SET_LED_WHITE,500); 	//White LED
		SET_LED_Status(SET_LED_RED,2000); 	//Red LED 
	}

	//Start up the compass
	compass = HMC5883L(); 						// Construct a new HMC5883 compass.
	#ifdef DEBUG_ON	
		if(compass.EnsureConnected() == 1) {
			Serial.println("Connected to HMC5883L.");
		} else {
			Serial.println("Not Connected to HMC5883L.");
		}
	#endif
	error = compass.SetScale(1.3); 				// Set the scale of the compass.
	#ifdef DEBUG_ON	
		if(error != 0) {							// If there is an error, print it out.
			Serial.println("Compass Error 1");
			Serial.println(compass.GetErrorText(error));
		} else {
			Serial.println("Compass Ok 1");
		}
	#endif
	error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
	#ifdef DEBUG_ON	
		if(error != 0) {							// If there is an error, print it out.
			Serial.println("Compass error 2");
			Serial.println(compass.GetErrorText(error));
		} else {
			Serial.println("Compass Ok 2");
		}
	#endif	
	
	//Start up the Pressure Sensor
	dps = BMP085();
	dps.init(); 
	#ifdef DEBUG_ON
		Serial.print("BMP Mode ");
		Serial.println(dps.getMode());
	#endif	
	wdt_reset();
	// Start up the OneWire Sensors library and turn off blocking takes too long!
	sensors.begin();
	sensors.setWaitForConversion(false);
  	sensors.requestTemperaturesByAddress(outsideThermometer); // Send the command to get temperature
	
	//Initialise all of the record values
	mD.vals.t_count = 0;
	mD.vals.usl_count = 0;
	mD.vals.year = 0;
	mD.vals.month = 0;
	mD.vals.day = 0;
	mD.vals.hour = 0;
	mD.vals.minute = 0;
	mD.vals.second = 0;
	mD.vals.hundredths = 0;
	mD.vals.i_lat = 0;
	mD.vals.i_long = 0;
	mD.vals.i_alt = 0;
	mD.vals.b_sats = 0;
	mD.vals.i_angle = 0;
	mD.vals.i_Hspeed = 0;
	mD.vals.i_Vspeed = 0;
	mD.vals.age = 0;
	mD.vals.ihdop = 0;
	mD.vals.Xax = 0;
	mD.vals.Xay = 0;
	mD.vals.Xaz = 0;
	mD.vals.Xgx = 0;
	mD.vals.Xgy = 0;
	mD.vals.Xgz = 0;
	mD.vals.Xmx = 0;
	mD.vals.Xmy = 0;
	mD.vals.Xmz = 0;
	mD.vals.BMP085_T = 0;
	mD.vals.balloonCutFired =0;
	mD.vals.ascent=1;
	r_press = 999999; /* Reference pressure */
	a_press =0;
	r_alt = 0; /* Reference altitude */
	a_alt=0;	
	press_idx=0;
	alt_idx=0;
	
	//Connect to the SD Card	
	if(!SD.begin(PIN_SPI_CS, SPI_HALF_SPEED)) {
		#ifdef DEBUG_ON	
			Serial.println("SD not working!!");
		#endif 
		SET_LED_Status(SET_LED_WHITE,500); 	//White LED
		SET_LED_Status(SET_LED_RED,3000); 	//Red LED 
	} else {
		#ifdef DEBUG_ON	
			Serial.println("SD OK");
		#endif 	
		dataFile.open(SD_LOG_FILE, O_CREAT | O_WRITE | O_APPEND);	    //Open Logfile
		if (!dataFile.isOpen()) {
			#ifdef DEBUG_ON	
				Serial.println("SD Data File Not Opened");
			#endif 	
			SET_LED_Status(SET_LED_WHITE,500);
			SET_LED_Status(SET_LED_RED,3000);
		}
		if (flagFile.exists(SD_C1_FILE)) {
			mD.vals.balloonCutFired = 1;
			#ifdef DEBUG_ON	
				Serial.println("SD Flag 1 File Exists");
			#endif 	
		} else if (flagFile.exists(SD_C2_FILE)) {
			mD.vals.balloonCutFired = 2;
			#ifdef DEBUG_ON	
				Serial.println("SD Flag File Exists");
			#endif 				
		} else if (flagFile.exists(SD_C3_FILE)) {
			mD.vals.balloonCutFired = 3;
			#ifdef DEBUG_ON	
				Serial.println("SD Flag File Exists");
			#endif 
		}
	}

	//Cycle lights
	SET_LED_Status(SET_LED_OFF,0);  
	SET_LED_Status(SET_LED_RED,500);
	SET_LED_Status(SET_LED_GREEN,500);
	SET_LED_Status(SET_LED_BLUE,500);
	SET_LED_Status(SET_LED_OFF,0);  
	
	elapseSIM900 = millis();				//Elapse counter for data to SIM900
	elapseNTXB = millis();					//Elapse counter for data to NTXB
	elapseAltCheck = millis();				//Elapse counter for floater altitude check
	NEWGPSDATA = false;
	wdt_enable(WDTO_2S);
	wdt_reset();
}

/*
 * Main Loop 
 */
void loop() {
	wdt_reset();
	mD.vals.usl_count++;									//Increment main datarecord count
	AccelerometerScaled Ascaled = accel.ReadScaledAxis();	//Get Scaled Accelerometer
	AccelerometerRaw Araw = accel.ReadRawAxis();			//Get Raw Accelerometer
	MagnetometerScaled Mscaled = compass.ReadScaledAxis();	//Get Scaled Magnetometer
	MagnetometerRaw Mraw = compass.ReadRawAxis();			//Get Raw Magnetometer
	LGgyro.read();											//Get Gyro

	// offset compass by hard iron
	Mraw.XAxis += 40;
	Mraw.YAxis += 261;
	Mraw.ZAxis += 54;

	//write Acc, Mag, & Gyro values to record
	float AxisGs = Ascaled.XAxis;
	mD.vals.Xax = AxisGs * 100;
	AxisGs = Ascaled.YAxis;
	mD.vals.Xay = AxisGs * 100;
	AxisGs = Ascaled.ZAxis;
	mD.vals.Xaz = AxisGs * 100;
	mD.vals.Xmx = Mscaled.XAxis;
	mD.vals.Xmy = Mscaled.YAxis;
	mD.vals.Xmz = Mscaled.ZAxis;
	mD.vals.Xgx = LGgyro.g.x;
	mD.vals.Xgy = LGgyro.g.y;
	mD.vals.Xgz = LGgyro.g.z;

	//Perform tilt compensation calculation save to record
	sixDOF.compCompass(Mraw.XAxis, -Mraw.YAxis, -Mraw.ZAxis, Araw.XAxis, Araw.YAxis, Araw.ZAxis, true);
	float compHeading = sixDOF.atan2Int(sixDOF.xAxisComp(), sixDOF.yAxisComp());
	compHeading = compHeading /100;
	if (compHeading < 0 ) {
		compHeading = abs(compHeading);
	} else {
		compHeading = 180 - compHeading + 180;
	}
	mD.vals.i_compass = compHeading;
	
	//get BMP085 values save to record
	dps.getTemperature(&mD.vals.BMP085_TFULL); 
	dps.getPressure(&mD.vals.BMP085_PFULL);
	a_press  = a_press  + mD.vals.BMP085_PFULL;
	press_idx++;

 	mD.vals.BMP085_T = (int16_t)(mD.vals.BMP085_TFULL);
	
	mD.vals.DS18B20_T = (int16_t)(sensors.getTempC(outsideThermometer)* 10);
	sensors.requestTemperaturesByAddress(outsideThermometer); // Send the command to get temperatures
	
	if(ETin.receiveData(I2C_SLV_SIM900_ADDRESS)){            
		mD.vals.rOnOff	= 				i2cIn.rOnOff;	
		mD.vals.sMobile = 				i2cIn.sMobile;
		mD.vals.sCIPSTATUS = 			i2cIn.sCIPSTATUS;
		mD.vals.sRssi = 				i2cIn.sRssi;
		if (i2cIn.balloonCutFired > 0 && mD.vals.balloonCutFired == 0 ) {
			if (i2cIn.balloonCutFired == 1) {
				flagFile.open(SD_C1_FILE, O_CREAT | O_WRITE | O_APPEND);
			} else if (i2cIn.balloonCutFired == 2) {
				flagFile.open(SD_C2_FILE, O_CREAT | O_WRITE | O_APPEND);
			} else if (i2cIn.balloonCutFired == 3) {
				flagFile.open(SD_C3_FILE, O_CREAT | O_WRITE | O_APPEND);
			}
			char SDString[100] = "";
			int tmp_year = mD.vals.year + 2000;	
			sprintf(SDString, "%ld,%d,%04d-%02d-%02d,%02d:%02d:%02d.%02d,%ld,%ld,%ld\n", 
			mD.vals.usl_count,mD.vals.t_count,
			tmp_year, mD.vals.month, mD.vals.day,
			mD.vals.hour, mD.vals.minute, mD.vals.second, mD.vals.hundredths,
			mD.vals.i_lat,mD.vals.i_long,mD.vals.i_alt);
			flagFile.write(SDString,strlen(SDString));
			flagFile.sync();
			flagFile.close();
			mD.vals.balloonCutFired = i2cIn.balloonCutFired;
		}
	}
	
	//get GPS data
	byte lcount = 0;									//reset a loop counter
	while (!NEWGPSDATA && lcount++ < 255) {				//Exit the loop if we have new data or have been round it a number of times
		NEWGPSDATA = feedgps();							
	}
	if (NEWGPSDATA) {									//We have new GPS data, get all of the fields we need.
		int tmp_year = 0;
		gps.crack_datetime(&tmp_year, &mD.vals.month, &mD.vals.day,&mD.vals.hour, &mD.vals.minute, &mD.vals.second, &mD.vals.hundredths, &mD.vals.age);
		mD.vals.year = tmp_year - 2000;
		
		/* Update the ascent / descent status */
        if (gps.altitude() != TinyGPS_HJOE::GPS_INVALID_ALTITUDE && gps.altitude() >= 0) {
			gps.get_position(&mD.vals.i_lat, &mD.vals.i_long, &mD.vals.age);
			mD.vals.i_alt = gps.altitude(); 
			a_alt = a_alt + mD.vals.i_alt;
			alt_idx++;
			mD.vals.i_angle = gps.course();
			mD.vals.i_Hspeed = gps.speed(); 
			mD.vals.b_sats = gps.satellites();
			mD.vals.ihdop = gps.hdop();
		}
		SET_LED_Status(SET_LED_BLUE,0);					//Flash blue to show we are getting GPS data
	} else {
		SET_LED_Status(SET_LED_GREEN,0);				//Flash Green to show that we are looping but not getting GPS data
	}

	//Pressure used to cut away on burst.
	if (press_idx>PRESSURE_INTEGRATE) {
		a_press  = a_press /PRESSURE_INTEGRATE;
		r_ascent = mD.vals.ascent;	
		if(a_press  < r_press - PRESSURE_STEP) {
			/* Payload is rising */
			mD.vals.ascent = 1;
			r_press = a_press ;
		}
		else if(a_press  > r_press + PRESSURE_STEP) {
			/* Payload is falling */
			mD.vals.ascent = 0;
			r_press = a_press ;
		}
		/* If staring to fall above 2000 metres... */
		if(a_press  < PRESSURE_CUT_ABOVE && r_ascent == 1 && mD.vals.ascent == 0){
			mD.vals.balloonCutDoFire = 1;
		}
		a_press  = 0;
		press_idx = 0;
    }
	
	//altitude used to cut away as float protection or too high
	if (alt_idx>ALTITUDE_INTEGRATE) {
		a_alt = a_alt/ALTITUDE_INTEGRATE;
		if(r_press  < PRESSURE_CUT_ABOVE && mD.vals.ascent == 1) {		
			if (a_alt > ALTITUDE_CUTAWAY) {
						mD.vals.balloonCutDoFire = 2;
			} else if ((a_alt - r_alt) / ((millis()-elapseAltCheck)/1000) < MIN_ALT_STEP) {
						mD.vals.balloonCutDoFire = 3;
			}
		}
		r_alt = a_alt;
		a_alt=0;
		alt_idx=0;
		elapseAltCheck=millis();
	}
	
	//flip flop between I2C's to avoid both on one loop
	if (SENDWIRE && (millis() - elapseSIM900) > WAIT_SIM900) {
		mD.vals.t_count++;
		ETout.sendData(I2C_SLV_SIM900_ADDRESS);		
		elapseSIM900 = millis();
	}

	if (!SENDWIRE && (millis() - elapseNTXB) > WAIT_NTXB) {
		mD.vals.t_count++;
		ETout.sendData(I2C_SLV_NTXB_ADDRESS);
		elapseNTXB = millis();
		//get I2C_SLV_SIM900_ADDRESS data

	}

	writeSDData();										//Write the data record to the SD card
	SET_LED_Status(SET_LED_OFF,0);						//turn off the LED
	NEWGPSDATA = false;									//Reset the New GPS Data flag
	SENDWIRE = !SENDWIRE;								//Flipflop this 
}
