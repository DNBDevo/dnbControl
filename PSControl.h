/*
 * PSControl.h
 *
 *  Created on: Apr 19, 2017
 *      Author: Evan Foley
 */

#ifndef PSCONTROL_H_
#define PSCONTROL_H_



#endif /* PSCONTROL_H_ */


#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include "dnbGPIO.h"
#include <time.h>
#include "math.h"



/*
Serial communications will use the following protocol:
<STX><CMD><,>ARG><,><CSUM><ETX>

Where:
<STX> = 1 ASCII 0x02 Start of Text character
<CMD> = 2 ASCII characters representing the command ID
<,> = 1 ASCII 0x2C character
<ARG> = Command Argument
<,> = 1 ASCII 0x2C character
<CSUM> = Checksum (see section 6.3 for details)
<ETX> = 1 ASCII 0x03 End of Text character8

///////////////////////////////////////////////////////////////////
COMMAND OVERVIEW

Command Name 								<CMD> 		<ARG> 			RANGE
Program User Configurations 				9 			See Desc. 		See Desc.
Program kV Setpoint 						10 			1-4 ASCII		0-4095
Program mA Setpoint 						11 			1-4 ASCII 		0-4095
Request kV Setpoint 						14 			None 			-
Request mA Setpoint 						15 			None 			-
Request Misc. Analog Monitor Readbacks 		20 			None 			-
Request Status 								22 			None 			-
Request Software Version 					23 			None 			-
Request Model Number 						26 			None 			-
Request User Configs 						27 			None 			-
Request Unit Scaling 						28 			None 			-
Request FPGA Version						43 			None 			-
Request kV monitor 							60 			None 			-
Request mA monitor 							61 			None 			-
Request Slave Faults 						68 			None 			-
Reset Faults 								74 			None 			-
Program Local/Remote Mode 					99 			1 ASCII 		0 or 1

///////////////////////////////////////////////////////////////////////
RESPONSE OVERVIEW

The command responses will follow the same format as outlined above in
section 6.1. This list is comprised of Commands with complex responses
only. Commands using a simple response will use the <$> character
(ASCII 0x24) as a “Success” response or a single character error code.
These responses will be eight ASCII characters in length.

Response Name 						<CMD> 			Response
Request kV Setpoint 				14 				11 ASCII
Request mA Setpoint 				15 				11 ASCII
Request Misc. Analog Readbacks 		20 				22 – 46 ASCII
Request Status 						22 				38 ASCII
Request Software Version 			23 				23 ASCII
Request Model Number 				26 				8 – 22 ASCII
Request User Configs 				27 				10 – 18 ASCII
Request Unit Scaling 				28 				10 – 15 ASCII
Request FPGA Version 				43 				23 ASCII
Request kV monitor					60 				8 – 11 ASCII
Request mA monitor 					61 				8 – 11 ASCII
Request Slave Faults 				68 				32 ASCII
Request System Voltages 			69 				46 ASCII


////////////////////////////////////////////////////////////////////////////
6.6.1 Program User Configurations
Description:
The host requests that the firmware set the user configurable items.

Direction:
Host to supply

Syntax:
<STX><09><,><ARG1><,><ARG2><,><CSUM><ETX>

Where:
<ARG1> = kV Rate Ramp
<ARG2> = mA Rate Ramp
<ARG3> = AOL Enable
<ARG4> = APT Enable

Range:
Valid values for ARG1 and ARG2 are from 0 through 10000 milliseconds.
In 10 milliseconds increments. For ARG3 and ARG4 they are 0 (disabled)
and 1 (enabled).

Response:
If an error is detected, the Power Supply responds with:
<STX>09,!,<ERROR>,<CSUM><ETX>
Please reference the Error Code section for a list of valid error codes.

If the message is accepted, the Power Supply responds with:
<STX>09,$,<CSUM><ETX>

Example:
Set kV and mA to a 10 ms ramp, with AOL and APT disabled.
<STX>09,10,10,0,0,<CSUM><ETX>
////////////////////////////////////////////////////////////////////////////

6.3 CHECKSUMS

The checksum is computed as follows:
- Add the <CMD>, <,>, and <ARG> bytes into a 16 bit (or larger) word.
The bytes are added as unsigned integers.
- Take the 2’s compliment (negate it).
- Truncate the result down to the eight least significant bits.
- Clear the most significant bit (bit 7) of the resultant byte, (bitwise AND with
0x7F).
- Set the next most significant bit (bit 6) of the resultant byte (bitwise OR
with 0x40).

Using this method, the checksum is always a number between 0x40 and 0x7F.
The checksum can never be confused with the <STX> or <ETX> control
characters, since these have non-overlapping ASCII values.
If the DSP detects a checksum error, the received message is ignored – no
acknowledge or data is sent back to the host. A timeout will act as an implied
NACK.

The following is sample code, written in Visual Basic, for the generation of
checksums:

7.0 ERROR CODES
Possible error codes are:
- 1 – Incorrectly formatted packet/message
- 2 – Invalid Command ID
- 3 – Parameter out of range
- 4 – Packet overrun FPGA Register read and write
- 5 – Flash programming error
- 7 – Bootloader failed
*/


//////////////////////////////////////////////////
//////////// Spellman ST commands ////////////////
//////////////////////////////////////////////////

//Command transmit functions for Spellman ST
								  //CMD		//<ARG>			//Range
#define ST_PROGRAM_USER_CONFIG       9
#define ST_PROGRAM_KV_SETPOINT       10		//1-4ASCII		//0-4095
#define ST_PROGRAM_MA_SETPOINT       11		//1-4ASCII		//0-4095
#define ST_REQUEST_KV_SETPOINT       14		//NONE			//-
#define ST_REQUEST_MA_SETPOINT       15		//NONE			//-
#define ST_REQUEST_ANALOG_READBACK   20		//NONE			//-
#define ST_REQUEST_STATUS            22		//NONE			//-
#define ST_REQUEST_SOFTWARE_VERSION  23		//NONE			//-
#define ST_REQUEST_MODEL_NUMBER      26		//NONE			//-
#define ST_REQUEST_USER_CONFIGS      27		//NONE			//-
#define ST_REQUEST_UNIT_SCALING      28		//NONE			//-
#define ST_REQUEST_FPGA_VERSION      43		//NONE			//-
#define ST_REQUEST_KV_MONITOR        60		//NONE			//-
#define ST_REQUEST_MA_MONITOR        61		//NONE			//-
#define ST_REQUEST_SLAVE_FAULTS      68		//NONE			//-
#define ST_REQUEST_SYSTEM_VOLTAGES   69		//NONE			//-
#define ST_RESET_FAULTS              74		//NONE			//-
#define ST_PROGRAM_LOCAL_REMOTE      99		//1 ASCII		//0 or 1

//////////////////////////////////////////////////
//////////// Spellman SLM commands ///////////////
//////////////////////////////////////////////////

//Command transmit functions for Spellman SLM
								  //CMD		//<ARG>			//Range
#define SLM_PROGRAM_RS232_BAUD		  7			//1 ASCII		//1-5 (1 - 9.6k, 2 - 19.2k, 3 - 38.4k, 4 - 57.6k, 5 - 115.2k)
#define SLM_PROGRAM_USER_CONFIG       9			//9 ASCII		//See Manual
#define SLM_PROGRAM_KV_SETPOINT       10		//1-4ASCII		//0-4095
#define SLM_PROGRAM_MA_SETPOINT       11		//1-4ASCII		//0-4095
#define SLM_REQUEST_KV_SETPOINT       14		//NONE			//-
#define SLM_REQUEST_MA_SETPOINT       15		//NONE			//-
#define SLM_REQUEST_ANALOG_MONITOR    19		//NONE			//-
#define SLM_REQUEST_HVON_COUNTER      21		//NONE			//-
#define SLM_REQUEST_STATUS			  22		//NONE			//-
#define SLM_REQUEST_SOFTWARE_VERSION  23		//NONE			//-
#define SLM_REQUEST_HARDWARE_VERSION  24		//NONE			//-
#define SLM_REQUEST_WEBSERVER_VERSION 25		//NONE			//-
#define SLM_REQUEST_MODEL_NUMBER      26		//NONE			//-
#define SLM_RESET_HVON_COUNTER        30		//NONE			//-
#define SLM_RESET_FAULTS          	  31		//NONE			//-
#define SLM_REQUEST_INTERLOCK_STATUS  55		//NONE			//-
#define SLM_REQUEST_KV_MONITOR        60		//NONE			//-
#define SLM_REQUEST_MA_MONITOR        61		//NONE			//-
#define SLM_REQUEST_15V_LVPS		  65		//NONE			//-
#define SLM_REQUEST_FAULTS			  68
#define SLM_WATHDOG_TICKLE		      88		//NONE			//-
#define SLM_WATHDOG_ENABLE			  89		//1 ASCII		//0 or 1
#define SLM_HV_ON_OFF			      98		//1 ASCII		//0 or 1
#define SLM_PROGRAM_LOCAL_REMOTE      99		//1 ASCII		//0 or 1

//////////////////////////////////////////////////
//////////////// DNB PS commands /////////////////
//////////////////////////////////////////////////
#define DNB_RESET_FAULTS			  101		//Reset Faults on either SLM or SL power supply.


#define DEFAULT_CURRENT				  1000     ////Default current value from 0-4095, This should match the same define in 32u4PSControl.ino

#define SEND_DELAY					  1000

typedef struct
{
		int port;
        int cmd;
        char data[20][20];
        char *strData;
        char rxData[50];
}RXData_PS;

typedef struct
{
	//Held these as values from 0-4095
	uint32_t currLevelVoltage;
	uint32_t tarLevelVoltage;
	uint32_t setLevelVoltage;
	uint32_t setLevelCurr;
	uint32_t maxVoltageVal;
	uint32_t minVoltageVal;
	uint32_t maxCurrVal;
	uint32_t minCurrVal;

	//Held as normal voltage values
	uint32_t maxVoltage;
	uint32_t minVoltage;
	uint32_t maxCurr;
	uint32_t minCurr;
	uint32_t maxVoltageLim;
	uint32_t minVoltageLim;
	uint32_t maxCurrLim;
	uint32_t minCurrLim;

	//Type of Power supply
	char type[4];
}PSData;


//send command with data
void sendCommand(int, int , char *);

//send command with no data
void requestCommand(int, int);

//read command
int readCommand(RXData_PS*, int);

//SPI transfer commands
void sendSPI(unsigned char*, int);
int rcvSPI(char*, int);
void initSPI(void);

//Parsing into power supply RX data structure
void strToRXStruct(RXData_PS* , char*);

//External functions
void initHVPS(int);
void zeroHVPS(int);
void zeroPSVoltage(int);
