/*
 
 HABJOE Arduino Flight script
 HABJOE - Copyright Andrew Myatt and Joseph Myatt March/2013
 
 KEYWORDS: SEEDSTUDIO GPRS Shield, SIM900, ARDUINO, GPRS, HABHUB, HTTP PUT, SMS, HIGH ALTITUDE BALLOON
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
 
 RECEIVES A PACKET OF INFORMATION VIA A I2C INTERFACE AND SENDS IT TO SMS AND GRPS (HABHUB).

 NOTE: THE "MOBILE_STATUS" Pin (A0) must be wired to the 2V6 power output from the SIM900
 The enable positive feedback to the Arduino that the SIM900 is on or off.

*/ 

#include <Wire.h>
#include <I2Cdev.h>
#include <EasyTransferI2C_NL.h>
#include <String.h>
#include <util/crc16.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
/*********************************************************
 * uses Base64 and sha256 libraries.... (there is a bug in the sha256 librat on the net and it needs to be modified)
 */
#include <Base64.h>
#include <sha256.h>

#define DEBUG_ON
//#define NOTSEND_HAB

// NOTE WIRE BUFFER SIZE INCREASED TO 50 in wire.h

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
} IC2DATA_INSTRUCTURE;
IC2DATA_INSTRUCTURE i2cVals;

typedef struct {
	byte	rOnOff;						//1 
	byte	sMobile;					//1
	byte	sCIPSTATUS;					//1
	int		sRssi;						//2
	byte	balloonCutFired;			//1
} IC2DATA_OUTSTRUCTURE;
IC2DATA_OUTSTRUCTURE i2cOutVals;

EasyTransferI2C_NL ETin,ETout; 			   //Easy Transfer

/*********************************************************
 * User settings
 */
static long USR_OFFALTITUDE_CM		= 400000;								// In Centimeters Mtr*100 (i use 6000 ft)
static long USR_SMSDELAY			= 180000;								// every 3 minutes 
static long USR_CUTAWAYON			= 6000;								    // Time that cut away is on 6 seconds

static char USR_PAYRECEIVERNAME[] 	= "AURA4-GPRS";							// 
static char USR_PAYLOADNAME[]		= "$$AURA4";							// Include the $$ as well.
static char USR_APN[] 			    = "payandgo.o2.co.uk";					// Users APN 
//static char USR_APN[] 		    = "everywhere";							//\",\"eesecure\",\"secure";					// Users APN 
static char USR_SMSNO[] 			= "07971470757";						// Users mobile no for SMS messages
//static char USR_SMSNO[] 			= "07500607336";						// Users mobile no for SMS messages

int 		PUT_TEMPLATE_LEN;	// This is calculated in setup;
char 		telemetryString[80];  	

static char disSET_ON[] 	= "SET_ON";
static char disSET_OFF[] 	= "SET_OFF";
static char disRXCONF[] 	= "RXCONF";
static char disRXNK[] 		= "RXNK";

#define I2C_SLAVE_ADDRESS 9 												//define slave i2c address
#define ON_VOLTAGE		100													//actually around 450 but this value provides some flex

/*********************************************************
 * PROGMEM Variables
 */

#define R_PROGMEMTST 	0
#define R_ATCIPSTART 	1
#define R_ATCIPSEND	 	2
#define R_PUTHABHUB0 	3
#define R_PUTHABHUB1 	4
#define R_PUTHABHUB2 	5
#define R_PUTHABHUB3 	6
#define R_PUTHABHUB4 	7
#define R_PUTHABHUB5 	8
#define R_PUTHABHUB6 	9
#define R_PUTHABHUB7 	10
#define R_PUTHABHUB8 	11
#define R_PUTHABHUB9 	12
#define R_ATCMGS	 	13
#define R_ATCSTT 	 	14
#define R_ATCIPSTATUS 	15
#define R_ATCIPCLOSE 	16
#define R_ATS01 		17
#define R_TCMGF1 		18
#define R_ATCIPSPRT1 	19
#define R_ATCGATT1 		20
#define R_ATCIICR 		21
#define R_ATCIFSR 		22
#define R_ATCLVL 		23
#define R_ATCIPSHUT		24
#define R_ATCMIC015		25
#define R_ATCSQ  		26

prog_char PROGMEMTST[]  PROGMEM = "PROGMEM TEST OK";
prog_char ATCIPSTART[]  PROGMEM = "AT+CIPSTART=\"tcp\",\"habitat.habhub.org\",\"80\"";
prog_char ATCIPSEND[]   PROGMEM = "AT+CIPSEND";
prog_char PUTHABHUB0[]  PROGMEM = "PUT http://habitat.habhub.org/habitat/";
prog_char PUTHABHUB1[]  PROGMEM = " HTTP/1.0\r\n";
prog_char PUTHABHUB2[]  PROGMEM = "Host: habitat.habhub.org\r\nContent-Length: ";
prog_char PUTHABHUB3[]  PROGMEM = "\r\nContent-Type: application/json\r\n\r\n";
prog_char PUTHABHUB4[]  PROGMEM = "{\r";
prog_char PUTHABHUB5[]  PROGMEM = " \"type\": \"payload_telemetry\",\r \"data\": {\r \"_raw\": \"";
prog_char PUTHABHUB6[]  PROGMEM = "\"\r},\r \"receivers\": {\r \"";
prog_char PUTHABHUB7[]  PROGMEM = "\": {\r \"time_created\": \"";
prog_char PUTHABHUB8[]  PROGMEM = "\",\r \"time_uploaded\": \"";
prog_char PUTHABHUB9[]  PROGMEM = "\"\r}\r}\r}\r\r";
prog_char ATCMGS[]	    PROGMEM = "AT+CMGS = \"";
prog_char ATCSTT[] 	    PROGMEM = "AT+CSTT=\"";
prog_char ATCIPSTATUS[] PROGMEM = "AT+CIPSTATUS";
prog_char ATCIPCLOSE[]  PROGMEM = "AT+CIPCLOSE=1";
prog_char ATS01[]  		PROGMEM = "ATS0=1";
prog_char TCMGF1[]  	PROGMEM = "AT+CMGF=1";
prog_char ATCIPSPRT1[]  PROGMEM = "AT+CIPSPRT=1";
prog_char ATCGATT1[]    PROGMEM = "AT+CGATT=1";
prog_char ATCIICR[]     PROGMEM = "AT+CIICR";
prog_char ATCIFSR[]     PROGMEM = "AT+CIFSR";
prog_char ATCLVL[]   	PROGMEM = "AT+CLVL=99";
prog_char ATCIPSHUT[]   PROGMEM = "AT+CIPSHUT";
prog_char ATCMIC015[]   PROGMEM = "AT+CMIC=0,15";
prog_char ATCSQ[]   	PROGMEM = "AT+CSQ";

PROGMEM const char *prgm_table[] = 	 {   
  PROGMEMTST,ATCIPSTART,ATCIPSEND,
  PUTHABHUB0,PUTHABHUB1,PUTHABHUB2,
  PUTHABHUB3,PUTHABHUB4,PUTHABHUB5,
  PUTHABHUB6,PUTHABHUB7,PUTHABHUB8,
  PUTHABHUB9,ATCMGS,ATCSTT,
  ATCIPSTATUS,ATCIPCLOSE,ATS01,
  TCMGF1,ATCIPSPRT1,ATCGATT1,
  ATCIICR,ATCIFSR,ATCLVL,ATCIPSHUT,
  ATCMIC015,ATCSQ
  };
  
const int PROMEMBUFSIZE = 80;
char prgmembuffer[PROMEMBUFSIZE];

byte	GlBballoonCutFired;

/*********************************************************
 * Mobile commands
 */

static byte RXNK = 0;
static byte RXCONF = 1;
static byte SET_OFF = 0;
static byte SET_ON = 1;

/*********************************************************
 * Mobile User requests
 */

static int setMobileOn = 0;
static int setSendSMS = 1;
static int setSendPUTS = 2;
 
byte mobRequest[] = {SET_OFF,SET_OFF,SET_OFF};

/*********************************************************
 * Mobile Status
 */
static int state_Mobile 	= 0;	//is mobile on
static int state_CIPSTATUS 	= 1;    //gets current status
static int state_ATS0 		= 2;	//answer after 1 ring
static int state_CMGF1 		= 3;	//SMS in text mode
static int state_CIPSPRT 	= 4;	//Set prompt when data sends
static int state_ATCLVL 	= 5;	//Speak Level Control
static int state_ATCMIC015 	= 6;	//Mic Level Control
static int state_RSSI	 	= 7;	//Received Signal Indicator

byte mobState[] = {RXNK,99,RXNK,RXNK,RXNK,RXNK,RXNK,99};

/******************************************
 * Pin Definitions
 */

int MOBILE_ONOFF = 9;
int BALLOON_CUT = 50;
int MOBILE_STATUS = A0;

/*****************************************
 * Buffers
 */
const int BUFFSIZE = 100;
char inBuffer[BUFFSIZE];
char outBuffer[BUFFSIZE];

char LONGDATE[30] = "";

unsigned long ElaspeSMS;
unsigned long ElaspeHAB;

/*****************************************
 * SIM900 Status return strings
 */

static char S_INITIAL[] =		"IP INITIAL";
static char S_START[]	=		"IP START";
static char S_CONFIG[]	=		"IP CONFIG";
static char S_GPRSACT[]=		"IP GPRSACT";
static char S_STATUS[]	=		"IP STATUS";
static char S_CONNECTING[] =	"CONNECTING";
static char S_CONNECTOK[]=	 	"CONNECT OK";
static char S_CLOSING[]=		"P CLOSING";
static char S_CLOSED[]	=		"P CLOSED";
static char S_SDEACT[]	=		"P DEACT";
static char S_ERROR[]=	 		"ERROR";
static char S_OK[] = 			"OK";
static char Q_CSQ1[]=			"+CSQ:";


 /***********************************************************************************************************************************
 * Mobile module Management
 * 
 * sort of state machine to control the mobile module
 */
 
  /*****************************************
 * isMobileOn - reads analogRead pin MOBILE_STATUS - this is wired to 2.6volt SIM900 Power in the module.)
 */

void doReceive(int numBytes) {
// Don't put anything in this function as it stops the Mobile phone loop working correctly	
}

void doRequest() {
  ETout.flagSlaveSend();    //if the master requests it, set the flag so that ETout.sendData() works properly in the loop().
  ETout.sendData();          //An I2C SLAVE can only address the master, so no address is requested
}
 byte isMobileOn() {
 	byte ret_val;  
	if (analogRead(MOBILE_STATUS) > ON_VOLTAGE) {
		ret_val = RXCONF;
	} else {
		ret_val = RXNK;
	}
	return ret_val;
} 

 /*****************************************
 * getCIPSTATUS - gets the SIM900 GPRS status and sets mobState[state_CIPSTATUS], waits until valid state found or 2000 from start or last byte received.
 */
 
 void getCIPSTATUS()  {
    int i = 0;
	int iq = 0;
	byte ret_val = 99;
	char *ch;
	byte lastStatus = mobState[state_CIPSTATUS];
	Serial1.flush();
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCIPSTATUS])));	
	int la = strlen(prgmembuffer);
	mob_SerialPrintln(prgmembuffer);
    unsigned long start = millis();
	unsigned long lastbyte = millis();

	bool rd = true;

	while(rd)  {
		if (Serial1.available() >0)  {
			if (i < BUFFSIZE) {                      
				inBuffer[i] = Serial1.read();
				lastbyte = millis();
				if (iq++ > la) i++;
			} else {
				rd = false;		  
			}
		} 
		inBuffer[i]='\0'; 
	
		ch = strstr (inBuffer, S_INITIAL);
		if (ch != NULL) {
				ret_val = 0;
				rd = false;
		}
		if (rd != false) {
			ch = strstr (inBuffer, S_START);
			if (ch != NULL) {
				ret_val = 1;
				rd = false;
			}
		}
		if (rd != false) {		
			ch = strstr (inBuffer, S_CONFIG);
			if (ch != NULL) {
				ret_val = 2;
				rd = false;				
			}
		}
		if (rd != false) {			
			ch = strstr (inBuffer, S_GPRSACT);
			if (ch != NULL) {
				ret_val = 3;
				rd = false;				
			}
		}
		if (rd != false) {			
			ch = strstr (inBuffer, S_STATUS);
			if (ch != NULL) {
				ret_val = 4;
				rd = false;				
			}
		}
		if (rd != false) {			
			ch = strstr (inBuffer, S_CONNECTING);
			if (ch != NULL) {
				ret_val = 5;
				rd = false;				
			}
		}
		if (rd != false) {			
			ch = strstr (inBuffer, S_CONNECTOK);
			if (ch != NULL) {
				ret_val = 6;
				rd = false;				
			}
		}
		if (rd != false) {	
			ch = strstr (inBuffer, S_CLOSING);
			if (ch != NULL) {
				ret_val = 7;
				rd = false;				
			}
		}
		if (rd != false) {			
			ch = strstr (inBuffer, S_CLOSED);
			if (ch != NULL) {
				ret_val = 8;
				rd = false;
			}
		}
		if (rd != false) {				
			ch = strstr (inBuffer, S_SDEACT);
			if (ch != NULL) {
				ret_val = 9;
				rd = false;				
				}
		}
		if (rd != false) {			
			ch = strstr (inBuffer, S_ERROR);
			if (ch != NULL) {
				ret_val = lastStatus;
				rd = false;
			}
		}		
				
		if ((millis()-lastbyte)>2000) rd = false;	
	}
	mobState[state_CIPSTATUS] = ret_val;
	if (mobState[state_CIPSTATUS] == 99) {
		mobState[state_ATS0] = RXNK;
		mobState[state_CMGF1] = RXNK;
		mobState[state_CIPSPRT] = RXNK;
	}
	
#ifdef DEBUG_ON
    Serial.print("<--");
    Serial.println(inBuffer);	
    Serial.print("Status:\t");
    Serial.println(mobState[state_CIPSTATUS]);
#endif			
}

/*****************************************
 * getRSSI - gets the signal Strength
 */
  
 int getRSSI()  {
    int i = 0;
	int rssi = 0;
	int iq = 0;
	byte ret_val = 99;
	char *ch1;
	Serial1.flush();
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCSQ])));	
	int la = strlen(prgmembuffer);
	mob_SerialPrintln(prgmembuffer);
    unsigned long start = millis();
	unsigned long lastbyte = millis();

	bool rd = true;

	while(rd)  {
		if (Serial1.available() >0)  {
			if (i < BUFFSIZE) {                      
				inBuffer[i] = Serial1.read();
				lastbyte = millis();
				if (iq++ > la+1) i++;
			} else {
				rd = false;		  
			}
		} 
		if ((millis()-lastbyte)>500) rd = false;
	}
	inBuffer[i]='\0'; 
	ch1 = strstr (inBuffer, Q_CSQ1);
	if (ch1 != NULL) {
		ch1 = strtok(inBuffer,":");
		if (ch1 != NULL) {
			ch1 = strtok(NULL,":");
			int rssi = atoi(ch1);
			#ifdef DEBUG_ON	
				Serial.println("**RSSI**");
				Serial.println(rssi);	
				Serial.println("--");
			#endif
			if ((rssi >=0 && rssi <=31) || rssi ==99) {
				return rssi;
			} else {
				return -1;
			}
		}
	}
	return -1;
}


 /*****************************************
 * manageMobileModule - State machine to turn on/off module and when on set module settings, and achieve and maintain GPRS status 4
 */
 void modCheck(byte retStat) {
		if (retStat == RXNK) {
			getCIPSTATUS();			
			strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCIPSHUT])));					
			retStat = sendWaitChkResp(prgmembuffer, 100, S_OK,S_ERROR);		
		} else if (retStat == RXCONF){
		
		}
 }
 
 void manageMobileModule() {
	byte retStat;
 	mobState[state_Mobile] = isMobileOn();
	
	if (mobRequest[setMobileOn] != mobState[state_Mobile]) {
		digitalWrite(MOBILE_ONOFF,LOW);
		myDelay(1000);
		digitalWrite(MOBILE_ONOFF,HIGH);
		myDelay(2000);
		digitalWrite(MOBILE_ONOFF,LOW);
		myDelay(6000);
		return;
	}

	if ((mobState[state_Mobile] == RXNK) || (mobRequest[setMobileOn] != mobState[state_Mobile])){
		mobState[state_CIPSTATUS] = 99;
		mobState[state_RSSI] = -1;
		return;
	}
	
	getCIPSTATUS();
	
	if (mobState[state_ATS0] == RXNK) {
		strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATS01])));
		mobState[state_ATS0] = sendWaitChkResp(prgmembuffer, 2000, S_OK, S_ERROR);		    				//Auto Answer on 1 rings
	}
	if (mobState[state_CMGF1] == RXNK) {
		strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_TCMGF1])));
		mobState[state_CMGF1] = sendWaitChkResp(prgmembuffer, 2000, S_OK, S_ERROR);		    				//Set Text Messages to TXT mode
	}	
	if (mobState[state_CIPSPRT] == RXNK) {
		strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCIPSPRT1])));
		mobState[state_CIPSPRT] = sendWaitChkResp(prgmembuffer, 2000, S_OK, S_ERROR);		    			//set prompt when using CIP-SEND Prompt
	}	
	if (mobState[state_ATCLVL] == RXNK) {
		strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCLVL])));
		mobState[state_ATCLVL] = sendWaitChkResp(prgmembuffer, 2000, S_OK, S_ERROR);		    				//set speaker level to 14
	}	
	if (mobState[state_ATCMIC015] == RXNK) {
		strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCMIC015])));
		mobState[state_ATCMIC015] = sendWaitChkResp(prgmembuffer, 2000, S_OK, S_ERROR);		    				//set mic level to max
	}
		
	if (mobState[state_CIPSTATUS] <= 0) {
		strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCGATT1])));
		retStat = sendWaitChkResp(prgmembuffer, 2000, S_OK, S_ERROR);								//Attach to GPRS Service
		modCheck(retStat);
	}
	if (mobState[state_CIPSTATUS] <= 0) {
		strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCSTT])));
		strcat(prgmembuffer, USR_APN);																// Add users APN settings
		strcat(prgmembuffer, "\"");
		retStat = sendWaitChkResp(prgmembuffer, 2000, S_OK, S_ERROR);												//Start Task set APN
		modCheck(retStat);
	}
	
	if (mobState[state_CIPSTATUS] == 1)  {
		strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCIICR])));
		retStat = sendWaitChkResp(prgmembuffer, 2000, S_OK,S_ERROR);												//Bring up GRPS network.
		modCheck(retStat);		
	}
	
	if ((mobState[state_CIPSTATUS] == 2 || mobState[state_CIPSTATUS] == 3)) {
		strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCIFSR])));
		retStat = sendWaitChkResp("AT+CIFSR", 2000, ".",S_ERROR);													//get local IP Address
		modCheck(retStat);
	}
	if ((mobState[state_CIPSTATUS] == 8 || mobState[state_CIPSTATUS] == 9)) {							//Shut the TCP session
		strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCIPSHUT])));                                        
		retStat = sendWaitChkResp(prgmembuffer, 100, S_OK,S_ERROR);
		modCheck(retStat);       
    }
	mobState[state_RSSI] = getRSSI();
}

 /*****************************************
 * sendWaitChkResp - Sends a string to the module and looks for the expected return, keeps looking until found or timeDelay from start or last character recieved.
 */

 byte sendWaitChkResp(char* sent_string, int timeDelay,  char* expected_resp_string, char* alt_expected_resp_string,bool sendESC)  {
	Serial1.flush();
	int la;
	if (sendESC) {
		Serial1.println((char)26);
		la = 1;
	} else {
		mob_SerialPrintln(sent_string);	
		la = strlen(sent_string);
	}


    int i = 0;
	int iq = 0;
    char *ch;
	byte ret_val;
    unsigned long start = millis();
    unsigned long lastbyte = millis();
	bool rd = true;
	
	while(rd)  {
		wdt_reset();
		if (Serial1.available() >0)  {
			if (i < BUFFSIZE) {                      
				inBuffer[i] = Serial1.read();
				lastbyte = millis();
				if (iq++ >= la) i++;
			} else {
				rd = false;		  
			}
		} 
		inBuffer[i]='\0'; 
		ch = strstr (inBuffer, expected_resp_string);
		if (ch != NULL) {
				ret_val = RXCONF;
#ifdef DEBUG_ON	
				Serial.println("#**");
				Serial.println(la);
				Serial.print("Expected:\t");
				Serial.println(expected_resp_string);
				Serial.print("<--#**");
				Serial.println(inBuffer);	
				Serial.println("**#");
#endif
			return ret_val;
		}
		ch = strstr (inBuffer, alt_expected_resp_string);
		if (ch != NULL) {
				ret_val = RXNK;
#ifdef DEBUG_ON	
				Serial.println("#**");
				Serial.println(la);
				Serial.print("Expected Alt:\t");
				Serial.println(alt_expected_resp_string);
				Serial.print("<--#**");
				Serial.println(inBuffer);	
				Serial.println("**#");
#endif
			return ret_val;
		}
		if ((millis()-lastbyte)>timeDelay) rd = false;			
 	}
	ret_val = RXNK;

	return ret_val;
}

 byte sendWaitChkResp(char* sent_string, int timeDelay,  char* expected_resp_string, char* alt_expected_resp_string)  {
	return sendWaitChkResp(sent_string, timeDelay,expected_resp_string, alt_expected_resp_string, false);
}


 /***********************************************************************************************************************************
 * Message Hashing and Encryption
 * 
 * 
 */
 
 /*****************************************
 * mobilePrintHash - converts hashed data into a string
 */

void mobilePrintHash(uint8_t* hash) {
  int i;
  const char hashString[] = "0123456789abcdef";
  for (i=0; i<32; i++) {
    Serial1.print(hashString[hash[i]>>4]);
    Serial1.print(hashString[hash[i]&0xf]);
#ifdef DEBUG_ON		
    Serial.print(hashString[hash[i]>>4]);
    Serial.print(hashString[hash[i]&0xf]);	
#endif
	}
}

 /*****************************************
 * gps_CRC16_checksum - standard checksum routine
 */
 
uint16_t gps_CRC16_checksum (char *string) {
	size_t i;
	uint16_t crc;
	uint8_t c;
	crc = 0xFFFF;
	// Calculate checksum ignoring the first two $s
	for (i = 2; i < strlen(string); i++)
	{
		c = string[i];
		crc = _crc_xmodem_update (crc, c);
	}
	return crc;
}

 /*****************************************
 * mob_SerialPrintln/mob_SerialPrint - allows easy bebugging 
 */

void mob_SerialPrintln(char* send_char) {
	Serial1.println(send_char);
#ifdef DEBUG_ON	
	Serial.print("#-->#");
	Serial.println(send_char);
#endif
}

void mob_SerialPrint(char* send_char) {
	Serial1.print(send_char);
#ifdef DEBUG_ON	
	Serial.print("#>#");
	Serial.print(send_char);
#endif
}

void myDelay(int delaytime) {
        int t = delaytime;
	wdt_reset();
	while (t > 0) {
		if (t > 4000) {
			delay(4000);
		} else {
			delay(t);	
		}
		t = t - 4000;
        wdt_reset();
	}
}

 /***********************************************************************************************************************************
 * user functions
 * 
 * 
 */
 
 /*****************************************
 * datadump
 */  
void datadump() {
	//Need to be made specific to the actual payload being used.

	sprintf(telemetryString, "%s,%d,%02d:%02d:%02d,%ld,%ld,%ld,%d,%d,%d,%d,%d", 
	USR_PAYLOADNAME,i2cVals.t_count,i2cVals.hour,i2cVals.minute,i2cVals.second,
	i2cVals.i_lat,i2cVals.i_long,i2cVals.i_alt,i2cVals.BMP085_T,i2cVals.DS18B20_T,i2cVals.b_sats,
	i2cVals.sMobile,i2cVals.balloonCutFired);

	unsigned int CHECKSUM = gps_CRC16_checksum(telemetryString);  // Calculates the checksum for this datastring
	char checksum_str[6];
	sprintf(checksum_str, "*%04X\n", CHECKSUM);
	strcat(telemetryString,checksum_str);
	int tmp_year = i2cVals.year + 2000;
	sprintf(LONGDATE, "%04d-%02d-%02dT%02d:%02d:%02d.000Z",tmp_year, i2cVals.month, i2cVals.day,i2cVals.hour, i2cVals.minute, i2cVals.second); 
}
 
 
 /*****************************************
 * PUT2Habhub - formats and sends data to HABHUB via a constructed HTTP PUT
 */ 
void PUT2Habhub(char* telemetryString) {
	if (mobState[state_CIPSTATUS] < 4 && mobState[state_CIPSTATUS] > 7) return;

	int iLength, aLength;
	iLength = strlen(telemetryString);
	base64_encode(outBuffer, telemetryString, iLength);											//base64 encode the telemetry string
	aLength = strlen(outBuffer);
	
#ifdef DEBUG_ON	
	Serial.println(telemetryString);
	Serial.println(iLength);
	Serial.println(outBuffer);
	Serial.println(aLength);
#endif

#ifdef NOTSEND_HAB
	return;
#endif

	int l = PUT_TEMPLATE_LEN + aLength + 48;													// Get the total length of the string template + base64 encoded telemetry string
	Sha256.init();																				// Initialise Sha256 encoding

	for (int i=0; i<aLength; i++) {																// Send the Base64 encoded Telemetry string to the sha256 encyption function
		Sha256.write(outBuffer[i]);
	}
  
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCIPSTART])));					// attach to HABHUB
	sendWaitChkResp(prgmembuffer, 3000, S_CONNECTOK,S_ERROR);	

	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCIPSEND])));					// start a send session
	sendWaitChkResp(prgmembuffer, 3000, ">",S_ERROR);	
	
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_PUTHABHUB0])));					// first bit of put, the sha256 encypted base64 telemetry string is the ID
	mob_SerialPrint(prgmembuffer);
	mobilePrintHash(Sha256.result());
	myDelay(500);
	
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_PUTHABHUB1])));					// next bit
	mob_SerialPrint(prgmembuffer);
	myDelay(500);
	
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_PUTHABHUB2])));					// next bit
	mob_SerialPrint(prgmembuffer);			
	Serial1.print(l);																			// send the expected length of the json parts

#ifdef DEBUG_ON	
	Serial.print(l);
#endif
	myDelay(300);	
	
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_PUTHABHUB3])));					// next bit end of the PUT command.
	mob_SerialPrint(prgmembuffer);
	myDelay(200);
	
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_PUTHABHUB4])));					// JSON payload1
	mob_SerialPrint(prgmembuffer);			
	myDelay(200);
	
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_PUTHABHUB5])));					// JSON payload2
	mob_SerialPrint(prgmembuffer);			
	myDelay(200);
	
	mob_SerialPrint(outBuffer);																	// telemetry in Base64 send after _RAW
	myDelay(200);

	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_PUTHABHUB6])));					// JSON payload3
	mob_SerialPrint(prgmembuffer);			
	myDelay(200);
	
	mob_SerialPrint(USR_PAYRECEIVERNAME);															// The users payload name
	myDelay(200);
	
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_PUTHABHUB7])));					// JSON payload4
	mob_SerialPrint(prgmembuffer);				
	myDelay(100);
	
	mob_SerialPrint(LONGDATE);																		// date/time
							
	myDelay(100);
	
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_PUTHABHUB8])));					// JSON payload5
	mob_SerialPrint(prgmembuffer);				
	myDelay(100);

	mob_SerialPrint(LONGDATE);																	//(same) date/time											
	myDelay(100);
	
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_PUTHABHUB9])));					// JSON payload6
	mob_SerialPrint(prgmembuffer);			
	myDelay(100);
					
	int resp = sendWaitChkResp("",10000, S_OK, S_ERROR, true);														// Send Send character and wait.
	Serial.println("Send Response...");
	Serial.println(resp);
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCIPCLOSE])));					//close the send session
	sendWaitChkResp(prgmembuffer, 100, S_OK,S_ERROR);

}

 /*****************************************
 * SendTextMessage - does a SMS send message
 */ 

void SendTextMessage(char* telemetryString) {
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_ATCMGS])));
	strcat(prgmembuffer, USR_SMSNO);															// Add users number
	strcat(prgmembuffer, "\"");
	mob_SerialPrintln(prgmembuffer);
	myDelay(100);
	mob_SerialPrintln(telemetryString);															//the content of the message
	myDelay(100);
	sendWaitChkResp("",300, S_OK, S_ERROR, true);														// Send Send character and wait.
}
 
 	
void debugStates() {
	int involt;
	Serial.println("---");
	
	strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[R_PROGMEMTST ])));
	Serial.print("Progmem test:\t");
	Serial.println(prgmembuffer);

	Serial.print("Actual Volts:\t");
	Serial.println(analogRead(MOBILE_STATUS));

	Serial.print("IsMobileOn:\t");	
	if (isMobileOn() == SET_ON) Serial.println(disRXCONF);
	else Serial.println(disRXNK);
	
	Serial.print("setMobileOn:\t");
	if (mobRequest[setMobileOn] == SET_ON) 	Serial.println(disSET_ON);
	else Serial.println(disSET_OFF);

	Serial.print("setSendSMS:\t");
	if (mobRequest[setSendSMS] == SET_ON) 	Serial.println(disSET_ON);
	else Serial.println(disSET_OFF);	

	Serial.print("setSendPUTS:\t");
	if (mobRequest[setSendPUTS] == SET_ON) 	Serial.println(disSET_ON);
	else Serial.println(disSET_OFF);	
	
	Serial.print("state_Mobile:\t");	
	if (mobState[state_Mobile] == RXCONF) 	Serial.println(disRXCONF);
	else Serial.println(disRXNK);	

	Serial.print("state_ATS0:\t");	
	if (mobState[state_ATS0] == RXCONF) 	Serial.println(disRXCONF);
	else Serial.println(disRXNK);	

	Serial.print("state_CMGF1:\t");	
	if (mobState[state_CMGF1] == RXCONF) 	Serial.println(disRXCONF);
	else Serial.println(disRXNK);

	Serial.print("state_CIPSPRT:\t");	
	if (mobState[state_CIPSPRT] == RXCONF) 	Serial.println(disRXCONF);
	else Serial.println(disRXNK);

	Serial.print("state_ATCLVL:\t");	
	if (mobState[state_ATCLVL] == RXCONF) 	Serial.println(disRXCONF);
	else Serial.println(disRXNK);	
	
	Serial.print("state_ATCMIC015:\t");	
	if (mobState[state_ATCMIC015] == RXCONF) 	Serial.println(disRXCONF);
	else Serial.println(disRXNK);	
	
	Serial.print("state_CIPSTATUS:\t");
	Serial.println(mobState[state_CIPSTATUS]);

}

 /***********************************************************************************************************************************
 * Setup/loop
 * 
 * 
 */

void setup() {
	wdt_enable(WDTO_8S);
	wdt_reset();
	Serial.begin(115200);  
	Serial1.begin(19200);
  	Serial.println("Setup start...");
	
	pinMode(MOBILE_ONOFF, OUTPUT);
	pinMode(MOBILE_STATUS, INPUT); 
	pinMode(BALLOON_CUT,OUTPUT);		
	digitalWrite(BALLOON_CUT,LOW);
	
	ElaspeSMS =millis();
	ElaspeHAB =millis();

	Wire.begin(I2C_SLAVE_ADDRESS);																// Uses the I2C interface to get the telemetry values.
	Wire.onRequest(doRequest);
	Wire.onReceive(doReceive);
	ETin.begin(details(i2cVals), &Wire);
	ETout.begin(details(i2cOutVals),&Wire);
	//define handler function on receiving data ps. you don't want to put anything in this that takes any time.!!!
	 
	PUT_TEMPLATE_LEN = 0;																		//Calculate the size of the PUT template for future use
	for (int i = R_PUTHABHUB4; i<=R_PUTHABHUB9; i++) {											//be sure to change these if index table is changed
		strcpy_P(prgmembuffer, (char*)pgm_read_word(&(prgm_table[i])));					
		PUT_TEMPLATE_LEN += strlen(prgmembuffer);
	}
	PUT_TEMPLATE_LEN += strlen(USR_PAYRECEIVERNAME);											//Add the length of the payload name.
	mobRequest[setMobileOn] = SET_ON;	
	mobRequest[setSendPUTS] = SET_ON;
	mobRequest[setSendSMS]  = SET_ON;
	
}
	

void loop() {
	myDelay (2000);
	debugStates();

	i2cOutVals.rOnOff = mobRequest[setMobileOn];
	i2cOutVals.sMobile = mobState[state_Mobile];
	i2cOutVals.sCIPSTATUS = mobState[state_CIPSTATUS];
	i2cOutVals.sRssi = mobState[state_RSSI];
	i2cOutVals.balloonCutFired = GlBballoonCutFired;

#ifdef DEBUG_ON
	Serial.println("Returned Values via I2C");
	Serial.println(i2cOutVals.rOnOff);
	Serial.println(i2cOutVals.sMobile);
	Serial.println(i2cOutVals.sCIPSTATUS);
	Serial.println(i2cOutVals.sRssi);
	Serial.println(i2cOutVals.balloonCutFired);
#endif
	
#ifdef DEBUG_ON
/*
	if (Serial.available()) {
		switch(Serial.read()) {
		case '0':
			mobRequest[setMobileOn] = SET_ON;	   
		   break; 
		case '1': 
			mobRequest[setMobileOn] = SET_OFF;
		   break;
		case '2':
			mobRequest[setSendSMS] = SET_ON;
			break;
		case '3':
			mobRequest[setSendSMS] = SET_OFF;
			break;
		case '4':
			mobRequest[setSendPUTS] = SET_ON;
			break;
		case '5':
			mobRequest[setSendPUTS] = SET_OFF;
			break;
		}
    }
	*/
#endif

	manageMobileModule();
	
	if(ETin.receiveData()) {

		if (mobState[state_Mobile] == RXCONF) {
			datadump();		
			if (mobRequest[setSendSMS] == SET_ON) {
					if ((millis() - ElaspeSMS) > USR_SMSDELAY) {
						ElaspeSMS = millis();
						SendTextMessage(telemetryString);
					}
			}
			if (mobRequest[setSendPUTS] == SET_ON && i2cVals.b_sats > 0) {
					PUT2Habhub(telemetryString);
			}
		}
	}

	if (i2cVals.i_alt > USR_OFFALTITUDE_CM) {
		mobRequest[setMobileOn] = SET_OFF;
	} else {
		mobRequest[setMobileOn] = SET_ON;		
	}
	
	if (i2cVals.balloonCutDoFire > 0 && i2cVals.balloonCutFired == 0) {
		GlBballoonCutFired = i2cVals.balloonCutDoFire;
		i2cOutVals.balloonCutFired = i2cVals.balloonCutDoFire;
		digitalWrite(BALLOON_CUT,HIGH);
		myDelay(USR_CUTAWAYON);
		digitalWrite(BALLOON_CUT,LOW);		
	}
}
