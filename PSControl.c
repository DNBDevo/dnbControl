/*
 * PSControl.c
 *
 *  Created on: Apr 19, 2017
 *      Author: Evan Foley
 */
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
Request System Voltages						69			None			-
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


//Includes
#include "PSControl.h"

#define TXMAXLEN 100
#define RXMAXLEN 100

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

uint8_t checksumCmd(char*, char*, int);


char STX[] = {0x02,0};
char ETX[] = {0x03,0};


//For commands with arguments.
void sendCommand(int port, int cmd, char *data)
{
	int len;
	int cmdlen;
	int sendLen;
	uint8_t chxSum;
	char cmdStr[] = {0};
	char portStr[1];
	char txSend[TXMAXLEN];
	char chxSumChar[] = {0,0};

	//get len of input data and add 2 for the cmd
	len = (int) strlen(data);
	if(cmd < 10)
	{
		sprintf(cmdStr, "0%d", cmd);
	}
	else
	{
		sprintf(cmdStr, "%d", cmd);
	}
	sprintf(portStr, "%d", port);
	cmdlen = (int) strlen(cmdStr);
	sendLen = len + cmdlen + 7;



	//calc checksum
	chxSum = checksumCmd(cmdStr, data, len);
	chxSumChar[0] = (char) chxSum;

	txSend[0] = '\0';
	strcpy(txSend, STX);
	strcat(txSend, portStr);
	strcat(txSend, ",");
	strcat(txSend, cmdStr);	 	//local/remote cmd
	strcat(txSend, ",");
	strcat(txSend, data);
	strcat(txSend, ",");
	strcat(txSend, chxSumChar);		// chxsum value
	strcat(txSend, ETX);		//End text value

	sendSPI(txSend, sendLen);
	printf("txSend: %s\n", txSend);
	fflush(stdout);
	usleep(SEND_DELAY);
}

//For commands with no arguments.
void requestCommand(int port, int cmd)
{
	int len;
	int sendLen;
	uint8_t chxSum;
	char cmdStr[4];
	char txSend[TXMAXLEN];
	char chxSumChar[] = {0,0};
	char portStr[1];
	int cmdlen;



	if(cmd < 10)
	{
		sprintf(cmdStr, "0%d", cmd);
	}
	else
	{
		sprintf(cmdStr, "%d", cmd);
	}

	cmdStr[(int) log(cmd)] = '\0';

	sprintf(portStr, "%d", port);

	//calc checksum
	chxSum = checksumCmd(cmdStr, "1,", 0);
	chxSumChar[0] = chxSum;

	strcpy(txSend, STX);
	strcat(txSend, portStr);
	strcat(txSend, ",");
	strcat(txSend, cmdStr);	 	//local/remote cmd
	strcat(txSend, ",");
	strcat(txSend, chxSumChar);		// chxsum value
	strcat(txSend, ETX);		//End text value

	//get len of input data and add 2 for the cmd
	cmdlen = (int) strlen(cmdStr);
	sendLen =  6 + cmdlen;

	printf("cmdLen: %d\n", cmdlen);

	//send command
	sendSPI(txSend, sendLen);
	printf("cmd: %d\n", cmd);
	fflush(stdout);
	usleep(SEND_DELAY);
}

//Wrapper function for rcvSPI in case we need to do something to the data or pointer.
int readCommand(RXData_PS *psData, int maxLen)
{
	//read FIFO Data until end char
	char *data = (char*)malloc(sizeof(char) * maxLen);

	rcvSPI(data, maxLen);

	strToRXStruct(psData, data);

	return 0;
}


uint8_t checksumCmd(char* cmd, char* arg, int len)
{
	uint16_t chxSum = 0;
	uint8_t returnSum;
	int i;

	//Add in the cmd chars (2)
	chxSum = chxSum + cmd[0];
	chxSum = chxSum + cmd[1];
	chxSum = chxSum + ',';

	//Add in the argument chars (len)
	if(len != 0)
	{
		for(i = 0; i < len; i++)
		{
			chxSum = chxSum + arg[i];
		}
	}


	chxSum = chxSum * -1;

	returnSum = (uint8_t) chxSum;

	returnSum = returnSum & 0x7F;

	returnSum = returnSum | 0x40;

	return returnSum;
}

void strToRXStruct(RXData_PS *rxPSData, char *rxData)
{
	char * token;
	char * rxStr = NULL;

	strcpy(rxPSData->rxData, rxData);

	//Remove everything to 0x02... there may be just data in this string.
	token = strsep(&rxData, STX);
	if(!(rxData))
	{
		//something went wrong we should exit this function.
		return;
	}
	//Grab port data
	token = strsep(&rxData, ",");

	if(token != NULL)
	{
		rxPSData->port = atoi(token);
	}

	//Grab cmd data
	token = strsep(&rxData, ",");

	if(token != NULL)
	{
		rxPSData->cmd = atoi(token);
	}

	//Check for the token to be null and exit if so.
	if(token == NULL)
	{
		return;
	}

	//Check how many commas are left in the rxData string
	int i, count = 0;
	for(i = 0, count = 0; rxData[i]; i++)
	{
		count += (rxData[i] == ',');
	}

	//based on the number of data points get that amount of data and place it into the struct.
	int j;
	for(j = 0; j < count; j++)
	{
		token = strsep(&rxData, ",");
		strcpy(rxPSData->data[j], token);
	}
}


void sendSPI(unsigned char* sendTX, int txLen)
{
	unsigned int len = 1;
	uint8_t pinFlags = 0x00;
	unsigned speed = 512;

	//Set flags for SPI send/rcv (4-wire)
	//pinFlags = pinFlags | SPI_PIN_USE_CE0(1); //(3-wire?)

	unsigned char *rxbuf1 = malloc(sizeof(char) * txLen);

	//This should return junk data.
	spiTransfer(speed, pinFlags, sendTX, rxbuf1, txLen, txLen);
}

int rcvSPI(char * rxData, int lenRCV)
{
	uint8_t txBuf[lenRCV + 1];
	uint8_t pinFlags = 0x00;
	unsigned speed = 512;
	int ret;

	//Make rxBuf to recieve the data
	uint8_t *rxbuf = malloc(sizeof(char) * lenRCV);

	//Make data to rcv. 0x05 will make the 32u4 fill the buffer for output and send you the data.
	int i;
	for(i = 0; i < lenRCV + 1; i++)
	{
		txBuf[i] = 0x05;
	}

	spiTransfer(speed, pinFlags, txBuf, rxbuf, lenRCV, lenRCV);

	//The first thing we rcv is 0x00. This breaks string functions so we are going to replace it with 0x05.
	rxbuf[0] = 0x05;
	//For debug. Output the return data to the screen
	for(ret = 0; ret < lenRCV; ret++)
	{
		if(rxbuf[ret] > 0)
		{
			printf("%.2X \n", rxbuf[ret]);
			fflush(stdout);
		}
	}
	strcpy(rxData, rxbuf);
	return 0;
}

void initSPI(void)
{
	uint8_t initFlags = 0x00;
	uint8_t pinFlags = 0x00;

	//set use CE0 and CE1 on the pin flags.
	pinFlags = pinFlags | SPI_PIN_USE_CE0(1) | SPI_PIN_USE_CE1(1);

	//initFlags are set to 0 for default
	initFlags = initFlags | SPI_INIT_SCLK_PHASE(0);

	spiInit(initFlags, pinFlags);
}

/////////////////////////////////////////////////////////////////////////////////////
///////////////////// External PS Control Functions /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
void initHVPS(int port)
{
	char data[5];
	//Put the PS in a state of HV output.
	//Faults reset
	requestCommand(port, DNB_RESET_FAULTS);

	//Current > 0 (DEFAULT_CURRENT).
	sprintf(data, "%d", DEFAULT_CURRENT);
	data[(int) log(DEFAULT_CURRENT)] = '\0';
	sendCommand(port, SLM_PROGRAM_MA_SETPOINT, data);
	sendCommand(port, SLM_PROGRAM_KV_SETPOINT, "0");
	//HV ON
	sendCommand(port, SLM_HV_ON_OFF, "1");

}

void zeroHVPS(int port)
{
	//Put PS in standby state.
	//HV OFF
	sendCommand(port, SLM_HV_ON_OFF, "0");
	//Voltage = 0;
	sendCommand(port, SLM_PROGRAM_KV_SETPOINT, "0");
	//Current= 0;
	sendCommand(port, SLM_PROGRAM_MA_SETPOINT, "0");
}

void zeroPSVoltage(int port)
{
	//Voltage = 0;
	sendCommand(port, SLM_PROGRAM_KV_SETPOINT, "0");
}
