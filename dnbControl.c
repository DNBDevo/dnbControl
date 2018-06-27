/*
 * dnbControl.c
 *
 *  Created on: Mar 6, 2017
 *      Author: Evan Foley
 */
/*Includes*/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>
#include "dnbGPIO.h"
#include "geniePi.h"
#include <pthread.h>
#include <string.h>
#include "dnbWaves.h"
#include <math.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include "PSControl.h"
#include <time.h>
#include <signal.h>

//UART Control
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

/*Define global variables*/

/*4D Genie Variables*/
/*Object Variables*/
#define IKNOB 		1
#define	ISLIDER	 	4
#define	ITRACKBAR	5
#define IWINBUTTON 	6
#define IFORM 		10
#define IKEYBOARD	13
#define ILEDDIGITS 	15
#define ISTRING 	17
#define IUSRBUTTON 	33
#define I4DBUTTON	30
#define IUSRIMAGES  27

#define FORMITEMNUM 50
#define FORMNUMMAX  20
#define FUNCCALLMAX 10
#define FUNCVARMAX  10
#define SYSNUMMAX   20
#define PINNUMMAX   50
#define VARNAMEMAX  20

/*Serial Command Variables*/
#define WRITEOBJ 				1
#define WRITESTRASCII 			2
#define WRITESTRUNI				3
#define WRITECONTRAST			4
#define REPORTOBJ				5
#define REPORTEVENT				7
#define WRITEMAGICBYTES			8
#define WIRTEMAGICDOUBLEBYTES 	9

/*Function Variables*/
#define INCVAR        1
#define DECVAR        2
#define CHANGEFORM    3
#define BEGINWAVE     4
#define CANCELWAVE    5
#define SETWFSYSTEM   6
#define CURRVAR		  7
#define RELEASEGPIO   8
#define INCCURRVAR    9
#define	DECCURRVAR    10
#define CHANGEMODVAL  11
#define PREPWF		  12
#define LOADWFVARS    13
#define SINGLEWF	  14
#define SETPOTVAL	  15
#define SETPOTZERO	  16
#define READPSVOLTAGE 17
#define SETPSVOLTAGE  18
#define SETPSTARVAL   19
#define SETPSZERO	  20
#define INITWFSYSTEM  21
#define SETCURRLOOP   22
#define PSMOD		  23
#define DEAULTPINSYSTEM 24
#define FORMCHANGED		25
#define DELAY   26
#define UPDATEITEM		27
#define ZEROPSVOLTAGE	28


//////////////////////////////////////////////
///////// Define Variables ///////////////////
//////////////////////////////////////////////
/*Base waveform Variables for Burst and Multiple Stroke*/
#define PULSERNDMAX    1
#define PULSERNDMIN    2
#define PULSECOUNT     3
#define PULSEWIDTH     4
#define BURSTRNDMAX    5
#define BURSTRNDMIN    6
#define BURSTCOUNT     7
#define CYCLEWIDTH     8
#define INHIBITTIME    9
#define INHIBITDELAY   10
#define CHARGETIME     11
#define CHARGEDELAY    12
#define PS1CURRVOLT	   15
#define PS1TARVOLT	   16
#define PS2CURRVOLT	   17
#define PS2TARVOLT	   18
#define WFINFO		   21
#define CHARGECOUNT    22
#define CHARGEINTERVAL 23

//PowerSupply variables
#define CURRPSVOLT	   13
#define TARPSVOLT	   14

//Internal variables
#define CURRMOD		   13
#define CURRLOOP	   14
#define CURRVARNAME	   19
#define CURRWFNAME	   20
#define RXMAXLEN 	   50
#define CURRMODMAX	   10000
#define INITFORM	   5
#define RUNFORM		   2
#define INITPINS	   1

/*Waveform type to create from burst library*/
#define GEN_MODE_NULL 		 -1
#define GEN_MODE_DISABLED 	  0
#define GEN_MODE_WAVE_ONLY 	  1
#define GEN_MODE_INHIBIT 	  2
#define GEN_MODE_CHARGE 	  3
#define GEN_MODE_DUALINHIBIT  4
#define GEN_MODE_MULTI_CHARGE 5

//Generate waveform defines
#define WF_REUSE 			0
#define WF_NEW 				1

//bounce values
#define SWITCHDEBOUNCE  1000000		//Output button debounce in uS.
#define MODLOOPTIME		1

//SPI defines
#define SPI0			1
#define SPI1			2
#define SPIRCVLEN	   30
#define ADCH0			1
#define ADCH1			2
#define ADCH2			3
#define ADCH3			4
#define ADCH4			5
#define ADCH5			6
#define ADCH6			7
#define ADCH7			8

//Power Supply defines
#define PS1				 1
#define PS2				 2
#define PSNUM			 2
#define PS_SLIDER_MAX	 100
#define PSMODVAL		 10

//Loop defines
#define MAINLOOP		1
#define MSLOOP			2
#define MODVALLOOP		3

//Define variables for loop dropouts/Interrupts
#define SLIDER_DROP_TIME .3
#define STOPBUTTONINDEX 6

//Define functions
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// Input File Format ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
/*
 *_______Pin File_______
 * name,*systemName*,*generatorType*
 * *GPIO#*,*pinMode*,*pinDelay(mS)*,*pinFunction*
 * ....
 * Example:
 * name,default,2
 * 12,0,10,TEST
 * 18,1,100,LOW
 * 19,1,1000,HIGH
 *
 * Pins will be activated in order that they appear with the pinDelay value in mS between each pin.
 *
 ***** Generator Modes (defined in dnbWaves.h) *****
 * #define GEN_MODE_NULL -1
 * #define GEN_MODE_DISABLED 0
 * #define GEN_MODE_WAVE_ONLY 1
 * #define GEN_MODE_INHIBIT 2
 * #define GEN_MODE_CHARGE	3
 * #define GEN_MODE_DUALINHIBIT 4
 *
 ***** Pin Modes *****
 * #define PIN_MODE_INPUT  0
 * #define PIN_MODE_OUTPUT 1
 * #define PIN_MODE_ALT0   4
 * #define PIN_MODE_ALT1   5
 * #define PIN_MODE_ALT2   6
 * #define PIN_MODE_ALT3   7
 * #define PIN_MODE_ALT4   3
 * #define PIN_MODE_ALT5   2
 *
 ***** Pin Functions *****
 * CHARGE 			Charge pin
 * PULSE			Pulse pin
 * INHIBIT			Single pulse inhibit
 * WAVEINHIBIT		Full train inhibit
 * HIGH				Pin is high (1)
 * LOW				Pin is low (0)
 * TEST				- Nothing -
 *
 *
 *
 *_______Form File_______
 *
 * Form,#
 * Item1Id,Item1Type,Form#,{FunctionID1, FunctionID2,...},{{FuncVar1,FuncVar2,...},{FuncVar1,FuncVar2,...},...}
 * Item2Id,Item2Type,Form#,{FunctionID1, FunctionID2,...},{{FuncVar1,FuncVar2,...},{FuncVar1,FuncVar2,...},...}
 * ...
 * Form,#
 * ...
 * ...
 *
 * You can link multiple internal variables to an ISTRING on screen
 * ISTRINGS: Function = 0, {thisArrayIndexToUseAsDefault,VARIABLE1,VARIABLE2,...}
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *_______WaveForm File_______
 *
 *
 * */
















static struct genieReplyStruct replyData;

/*Data structures*/
typedef struct
{
	char string[20];
}string;

//A single thing on a form
typedef struct
{
	int id;
	int type;
	int formNum;
	int functionCall[FUNCCALLMAX];
	double funcVars[FUNCCALLMAX][FUNCVARMAX];
}formItem;

//All things on a single form
typedef struct
{
	formItem FormItems[FORMITEMNUM];
}formStruct;

//All form data
typedef struct
{
	formStruct allFormData[FORMNUMMAX];
}allForms;

//Data being sent to serial connection
typedef struct
{

}TxSerialData;

typedef struct fileData
{
	/// ADD ALL NECESSARY DATA THAT IS IN FILE HERE
}fileData;

typedef struct
{
	uint32_t value;
	uint32_t min;
	uint32_t max;
	char name[VARNAMEMAX];
}waveformItem_uint32;

typedef struct
{
	double value;
	double min;
	double max;
	char name[VARNAMEMAX];
}waveformItem_double;

typedef struct waveformVars
{
	waveformItem_uint32 PulseRndMax;
	waveformItem_uint32 PulseRndMin;
	waveformItem_uint32 PulseCount;
	waveformItem_uint32 PulseWidth;
	waveformItem_double BurstRndMax;
	waveformItem_double BurstRndMin;
	waveformItem_uint32 BurstCount;
	waveformItem_double CycleWidth;
	waveformItem_uint32 InhibitTime;
	waveformItem_uint32 InhibitDelay;
	waveformItem_uint32 ChargeTime;
	waveformItem_uint32 ChargeDelay;
	waveformItem_uint32 ChargeCount;
	waveformItem_uint32 ChargeInterval;
}waveformVars;

typedef struct
{
	uint64_t wavePinMask;
	uint64_t inhibitPinMask;
	uint64_t waveInhibitPinMask;
	uint64_t chargePinMask;
	uint64_t SCRPinMask;

}wavePins;

typedef union
{
	uint64_t U64;
	uint32_t U32[2];
	uint8_t U8[8];
}maskUnion;

typedef struct
{
	maskUnion pinMask;
	int pinDelay;
	unsigned pinNum;
	int pinMode;
	char * pinType;
}pin;

typedef struct
{
	pin pins[PINNUMMAX];
	char * name;
	int genMode;
}pinSystem;

typedef struct
{
	int profileNum;
	int genMode;
	char * profileName;
}currentWaveProfile;

typedef struct
{
	int var;
	double mod;
	char name[VARNAMEMAX];
}currVar_t;


typedef struct
{
	char file[50];
}currFile_t;

//SPI Globals.....
typedef struct
{
	const char *device;
	uint8_t mode;
	uint8_t bits;
	uint32_t speed;
	uint16_t delay;
	int ntransfers;
	int toggle_cs;
}SPIdev;

/*End Globals*/

/*Prototypes functions*/
int getCurrentForm(void);
void initSerial(void);
void releaseSerial(void);
void readFormFile(char[]);
void setPSLevels(void);
void intHandler(void);
char* strDisplayFormat(int, int);
char* specificDisplayFormat0(int, int);
//uint8_t * SPISend(uint8_t*);
/*End Prototypes functions*/

/*Make form data*/
allForms Forms;

//Output loop variables
int contRun = 0;
int run = 0;

//If we are in a output loop this will be 1. Otherwise if we want to return to main loop then it will be 0
int currLoop;

/*Initialize waveform variables*/
waveformVars waveform = {0,0,0,0,0,0,0,0,0,0,0,0};

//Initialize pin set
pinSystem systems[SYSNUMMAX];
wavePins waveformPins = {0,0,0,0,0};

//setup currentWaveProfile
currentWaveProfile currSystem = {-1, -1, NULL};
currVar_t currVar = {-1,1,""};

//create mask for all pins I want to use.
maskUnion allPins = {0};

//Power Supply Data
//First Strike PS = allPSData[0], FIRSTSTRIKEPS = 1
//Second Strike PS = allPSData[1], SUBSTRIKEPS = 2
RXData_PS rxPSData[PSNUM];
PSData allPSData[PSNUM];


//Set text file structure
char * formFile = "/opt/blade/dnbGPIO/ProtoRev2/WF1245Forms.txt";
char * pinFile = "/opt/blade/dnbGPIO/ProtoRev2/WF1245PinData.txt";
char * waveFile = "/opt/blade/dnbGPIO/ProtoRev2/WF1245Data.txt";
currFile_t currFiles[3];


int main(void)
{
	//Setup a signal catcher for ctrl-C (SIGINT)
	signal(SIGINT, intHandler);


	//Sleep for a while. Slow it down. Get ready for the THUNDURRR
	sleep(1);

	//Loop variable; 1 = Pulse loop; 0 = Main loop
	currLoop = MAINLOOP;

	//Set default waveform files to pull data from
	strcpy(currFiles[0].file, "/opt/blade/dnbGPIO/ProtoRev2/defaultMultiStroke.txt");
	strcpy(currFiles[1].file, "/opt/blade/dnbGPIO/ProtoRev2/defaultBurst.txt");
	strcpy(currFiles[2].file, "/opt/blade/dnbGPIO/ProtoRev2/WF1245Data.txt");



	gpioInitialise();

	/*Initialize Serial Connection*/
	//releaseSerial();
	//initSerial();

	//Init system
	initSystem();

	//Hardcode PS data for now.
	setPSLevels();

	//initialize default system. since changeForm from here does not send a change form report message
	//changeForm(0);


	//TEST SPI System
	gpioMode((0x00 & SPI_PIN_CE0_P36(1) ? 36 : 8), PI_ALT0);
	//testDNBSPI();

	//Leave Splash screen form
	//changeForm(1);
	//highlight WF1 selection and set curr system to WF1 system
	//genieWriteObj(IUSRBUTTON,0,1);
	double sys[] = {0};
	setCurrSystem(sys);
	initCurrSystem();

	//after initializing the default system we highlight the WF1 selection and NEED to change the selected
	//pin system or we do not properly move into the run screen if the only thing we hit is run.
	sys[0] = 1;
	setCurrSystem(sys);

	//prepWaveform();
	///OUTPUT TESTING


	/////////DEBUG TIMER////////////
	struct timespec startTime1;
	struct timespec currTime1;
	time_t diff1;
	long int nDiff1;
	double elapsedTime1;
	clock_gettime(CLOCK_MONOTONIC, &startTime1);
	////////////////////////////////////

	////////////////////////////////////////
	//Init variables on PS control board when starting the program.
	initHVPS(1);
	usleep(50000);
	initHVPS(2);
	/////////////////////////////////////////

	while(1)
	{
		//////////////////////////
		////////DEBUG TIMER///////
		//////////////////////////

		clock_gettime(CLOCK_MONOTONIC, &currTime1);
		diff1 = currTime1.tv_sec - startTime1.tv_sec;
		elapsedTime1 = diff1;
		if(elapsedTime1 > 1)
		{
			//readPSVoltage(1);
			//updateItem(0,1,ISTRING);
			//readPSVoltage(2);
			//updateItem(0,3,ISTRING);

			clock_gettime(CLOCK_MONOTONIC, &startTime1);
		}
		/////////////////////////////////////////////////

		/// WF 3 TESTING ///
		//////////////////////////////////////////////////////////////////////////////////
		setWavesVars();
		getchar();
		printf("BEGINWAVE, CurrGenMode: %d\n", currSystem.genMode);
		fflush(stdout);
		usleep(100000);
		GenerateWaveForm(3);
		beginWave(1);
		///////////////////////////////////////////////////////////////////////////////////


		//////////////////////////
		//Serial Loop for screen//
		//////////////////////////
		usleep(200000);
		if(genieReplyAvail() == 1)
		{
			getSerialData();
			outputData();
			serHandler();
			clock_gettime(CLOCK_MONOTONIC, &startTime1);
		}
	}

	return 0;
}
////////////////////////////////////////////////////////
//////////// Interrupt Handler /////////////////////////
////////////////////////////////////////////////////////

void intHandler(void)
{
	printf("Terminating Program.\nCleaning up and exiting.\n");

	//Turn PS voltage to zero
	zeroHVPS(2);
	zeroHVPS(1);

	//termindate(lol) program. Release GPIOs and exit.
	terminateProgram();
	exit(EXIT_FAILURE);
}


///////////////////////////////////
////// UART0 Serial Functions//////
///////////////////////////////////
void initSerial(void)
{
	if(genieSetup("/dev/ttyAMA0",115200) == 0)
	{
		printf("Serial Connection Established.");
	}

	//Check serial connection to the screen. Lets ask for the Form #. If it returns < 0 then the screen is not connected.
	int currForm = -1;
	if((currForm = getCurrentForm()) < 0)
	{
		printf("Screen not found!\n\n EXITING PROGRAM\n");
		//Maybe reboot screen here and attempt to connect again if screen is not found
		//Run a screen TX buffer filler possibly to clear the screen out of a locked state?

		//Exit program if no screen
		//exit(EXIT_FAILURE);

	}
}

void releaseSerial(void)
{
	genieClose();
	printf("releaseSerial called\n");
}

void serHandler()
{
	int currForm = -1;
	int formItem = -1;
	int currID   = -1;
	int currFunc = -1;
	int port;
	double * currVars;
	uint8_t funcMod = 0;

	//Check current form so we dont have to iterate too many times.
	currForm = getCurrentForm();

	//function modifier sent from screen
	funcMod = replyData.cmd & 0xF0;
	funcMod = funcMod >> 4;
	//Mask out the function modifier
	replyData.cmd = replyData.cmd & 0x0F;

	if(replyData.cmd == REPORTEVENT)
	{
		//Grab the item
		for(formItem = 0; formItem< FORMITEMNUM; formItem++)
		{
			if((replyData.object == Forms.allFormData[currForm].FormItems[formItem].type) && (replyData.index ==  Forms.allFormData[currForm].FormItems[formItem].id))
			{
				//currID = Forms.allFormData[currForm].FormItems[formItem].id;
				break;
			}
		}
	}


	int i;

	uint level;
	int stringItem1 = 0;
	char string1[20];
	int loop = -1;
	double buff;
	//Item was grabbed lets do something with it!
	//Run functions attached to the button

	i = 0;
	while(Forms.allFormData[currForm].FormItems[formItem].functionCall[i] != 0)
	{
		currFunc = Forms.allFormData[currForm].FormItems[formItem].functionCall[i];
		currVars = Forms.allFormData[currForm].FormItems[formItem].funcVars[i];
		if(currFunc > 0)
		{
			switch(currFunc)
			{
			case INCVAR:
				incVar(currVars);
				//updateForm();
				break;
			case DECVAR:
				decVar(currVars);
				//updateForm();
				break;

			//CHANGE FORM FUNCTION, Will change the form, loop, and pin structure.
			//Variable format: ...,CHANGEFORM,{targetForm,targetLoop,initPinsBool}
			// Target form:double value, targetLoop:MAINLOOP,MSLOOP..., intiPinsBool:INITPINS,NULL
			case CHANGEFORM:
				loop = currLoop;
				formChanged(currVars);
				//changeForm((int) Forms.allFormData[currForm].FormItems[formItem].funcVars[0]);

				if((currLoop == 2) && (currLoop != loop))
				{
					//updateForm();
					waveOutputLoop();
				}

				else if((currLoop == 3) && (currLoop != loop))
				{
					//updateForm();
					modValLoop();
				}

				break;
			case BEGINWAVE:
				beginWave(1);
				break;
			case CANCELWAVE:
				//setPSZero();
				cancelWave();
				break;
			case SETWFSYSTEM:
				setCurrSystem(currVars);
				genieWriteObj(IUSRIMAGES, 1, currVars[1]);
				break;
			case INITWFSYSTEM:
				initCurrSystem();
				break;
			case CURRVAR:
				setCurrVar(currVars);
				genieWriteObj(IUSRIMAGES, 2, currVars[1]);
				sprintf(string1, "%*s", 20, currVar.name);
				genieWriteStr(8, string1);
				fflush(stdout);
				for(stringItem1 = 0; stringItem1< FORMITEMNUM; stringItem1++)
				{
					if((ISTRING == Forms.allFormData[currForm].FormItems[stringItem1].type) && (9 ==  Forms.allFormData[currForm].FormItems[stringItem1].id))
					{
						break;
					}
				}

				Forms.allFormData[currForm].FormItems[stringItem1].funcVars[i][0] = Forms.allFormData[currForm].FormItems[formItem].funcVars[i][1] + 1.;
				updateSpecItem0(currForm,9,ISTRING);

				break;
			case RELEASEGPIO:
				releasePins();
				break;
			case INCCURRVAR:
				incCurrVar();
				updateCurrVarString(currForm);
				updateSpecItem0(currForm,9,ISTRING);
				break;
			case DECCURRVAR:
				decCurrVar();
				updateCurrVarString(currForm);
				updateSpecItem0(currForm,9,ISTRING);
				break;
			case CHANGEMODVAL:
				changeModVal(currVars);
				updateItem(currForm,6,ISTRING);
				break;
			case PREPWF:
				prepWaveform();
				break;
			case LOADWFVARS:
				loadWFVars(currVars);
				prepWaveform();
				updateForm();
				updateSpecItem0(currForm,9,ISTRING);
				break;
			case SINGLEWF:
				singleWFOut();
				break;
			case SETPOTVAL:
				break;
			case SETPOTZERO:
				setPSZero();
				terminateProgram();
				break;
			case READPSVOLTAGE:
				if(replyData.data == 0)
				{
					readPSVoltage((int) currVars[0]);
				}
				break;
			case SETPSVOLTAGE:
				if(replyData.data == 0)
				{
					setPSVoltage((int) currVars[0]);
				}
				//updateForm();
				break;
			case SETPSTARVAL:
				//Set the target value of the power supply. We do this after the user changes the voltage on the screen.
				//Before this was a slider but in practice it does not work well if you attempt to move it too quickly.
				break;
			case SETPSZERO:
				setPSZero();
				printf("SetPSZero");
				break;
			case SETCURRLOOP:
				setCurrLoop(currVars[0]);

				if(currLoop == 2)
				{
					updateForm();
					waveOutputLoop();
				}

				if(currLoop == 3)
				{
					updateForm();
					modValLoop();
				}
				break;
			case PSMOD:
				psMod(currVars);
				break;
			case DEAULTPINSYSTEM:
				defaultPinSystem(currVars);
				break;
			case FORMCHANGED:
				formChanged(currVars);

				printf("CurrLoop %d",currLoop);
				fflush(stdout);
				if(currLoop == 2)
				{
					//updateForm();
					waveOutputLoop();
				}

				if(currLoop == 3)
				{
					//updateForm();
					modValLoop();
				}
				break;
			case DELAY:
				usleep(currVars[0]);
				break;
			case UPDATEITEM:
				updateItem((int) currVars[0],(int) currVars[1],(int) currVars[2]);
				break;
			case ZEROPSVOLTAGE:
				zeroPSVoltage(currVars[0]);
				allPSData[(int) currVars[0] - 1].currLevelVoltage = 0;
				allPSData[(int) currVars[0] - 1].setLevelVoltage = 0;
				allPSData[(int) currVars[0] - 1].tarLevelVoltage = 0;
				break;
			default:
				break;
			}
		}
	//updateForm();
	i++;
	usleep(5000);
	}
}

void outputData()
{
	printf("CMD: %d\n", replyData.cmd);
	printf("OBJ: %d\n", replyData.object);
	printf("index: %d\n", replyData.index);
	printf("data: %d\n", replyData.data);
	fflush(stdout);
}

void getSerialData()
{
	genieGetReply(&replyData);
}


int getCurrentForm()
{
	//return current form
	return genieReadObj(IFORM,3);
}

/////////////////////////////////////////////
///////////// Loop functions ////////////////
/////////////////////////////////////////////

int waveOutputLoop(void)
{
	//Make sure the current loop is correct.
	currLoop = MSLOOP;

	//Timing variables
	struct timespec startTime;
	struct timespec currTime;
	time_t diff;
	long int nDiff;
	double elapsedTime;

	//set maskUnion variables for burst button
	maskUnion pulsePinMask;
	maskUnion pin;
	maskUnion val;

	//set GPIO 12 to the pin mask
	pin.U64 = 12;

	pulsePinMask.U64 = 1;
	pulsePinMask.U64 = pulsePinMask.U64 << pin.U64;

	//Find current data on form and if there are power supply strings to update.
	int currForm = -1;
	int formItem = -1;


	//setup start time for clock
	clock_gettime(CLOCK_MONOTONIC, &startTime);

	//Write to status
	genieWriteStr(2,"Ready");

	while(1)
	{
		//////////////////////////
		//Serial Loop for screen//
		//////////////////////////
		usleep(1000);
		if(genieReplyAvail() == 1)
		{
			getSerialData();
			outputData();
			serHandler();
			usleep(1000);
		}


		//////////////////////////////////
		/////Check output button press////
		//////////////////////////////////
		//Pin 12
		val.U64 = dnbGPIOLevels();

		val.U32[0] = val.U32[0] & pulsePinMask.U32[0];
		val.U32[1] = val.U32[1] & pulsePinMask.U32[1];


		///////////////////////////////////
		////Clock Timing for re-strike ////
		///////////////////////////////////

		clock_gettime(CLOCK_MONOTONIC, &currTime);
		diff = currTime.tv_sec - startTime.tv_sec;
		nDiff = currTime.tv_nsec - startTime.tv_nsec;
		elapsedTime = diff + (nDiff * 1e-9);

		if((val.U32[1] || val.U32[0]))
		{
			//Lock all buttons on the screen
			genieWriteStr(2,"*Running*");
			lockRunScreenButtons();

			printf("BEGINWAVE, CurrGenMode: %d\n", currSystem.genMode);
			fflush(stdout);
			usleep(100000);
			GenerateWaveForm(currSystem.genMode);

			beginWave(1);

			//Wait until waveform is not running to continue.
			while(WaveRunning())
			{
				if(genieReplyAvail() == 1)
				{
					getSerialData();
					if((replyData.object == IUSRBUTTON) &&(replyData.index == STOPBUTTONINDEX))
					{
						cancelWave();
					}
				}
			};

			//Hold loop until button is released.
			while(1)
			{
				val.U64 = dnbGPIOLevels();

				val.U32[0] = val.U32[0] & pulsePinMask.U32[0];
				val.U32[1] = val.U32[1] & pulsePinMask.U32[1];
				if(!(val.U32[1] || val.U32[0]))
				{
					break;
				}
			}

			//release locked buttons on screen
			genieWriteStr(2,"Ready");
			unlockRunScreenButtons();
		}

		//Break to main loop if we want to get to main loop.
		if(currLoop == 1)
		{
			break;
		}
	}
	return 1;
}

//Continuous loop to mod variables? Ex: Hold down to change. This is not working yet
void modValLoop(void)
{
	while(1)
	{

		//////////////////////////
		//Serial Loop for screen//
		//////////////////////////
		usleep(100);
		if(genieReplyAvail() == 1)
		{
			getSerialData();
			outputData();
			serHandler();
			usleep(100);
		}

		if(currLoop == 1)
		{
			break;
		}
	}
}

void modValue(void)
{
	int i;
	int formItem = 0;
	char string[40];
	clock_t begin;
	double time_spent;
	begin = clock();
	int currVoltPS1;
	int tarVoltPS1;
	int currVoltPS2;
	int tarVoltPS2;

	while(1)
	{
	time_spent = (double)(clock() - begin) / CLOCKS_PER_SEC;

	if (time_spent>=.3)
	{

		if(currVar.mod < 3)
		{
			decCurrVar();
		}
		if(currVar.mod > 3)
		{
			incCurrVar();
		}

		for(formItem = 0; formItem< FORMITEMNUM; formItem++)
		{
			//if((Forms.allFormData[2].FormItems[formItem].type == ISTRING) && ((int)Forms.allFormData[2].FormItems[formItem].funcVars[0] == currVar.var))
				//break;
		}

		switch(currVar.var)
		{
		case PULSERNDMAX:
			sprintf(string, "Pulse Rnd Max.\n%d uS", waveform.PulseRndMax.value);
			break;
		case PULSERNDMIN:
			sprintf(string, "Pulse Rnd Min.\n%d uS", waveform.PulseRndMin.value);
			break;
		case PULSECOUNT:
			sprintf(string, "Pulse Count\n%d", waveform.PulseCount.value);
			break;
		case PULSEWIDTH:
			sprintf(string, "Pulse Width\n%d uS", waveform.PulseWidth.value);
			break;
		case BURSTRNDMAX:
			sprintf(string, "Burst Rnd. Max.\n%f mS", waveform.BurstRndMax.value);
			break;
		case BURSTRNDMIN:
			sprintf(string, "Burst Rnd. Min.\n%f mS", waveform.BurstRndMin.value);
			break;
		case BURSTCOUNT:
			sprintf(string, "Burst Count\n%d", waveform.BurstCount.value);
			break;
		case CYCLEWIDTH:
			sprintf(string, "Cycle Time\n%f S", waveform.CycleWidth.value);
			break;
		case INHIBITTIME:
			sprintf(string, "Inhibit Time\n%d uS", waveform.InhibitTime.value);
			break;
		case INHIBITDELAY:
			sprintf(string, "Inhibit Delay\n%d uS", waveform.InhibitDelay.value);
			break;
		case CHARGETIME:
			sprintf(string, "%d", waveform.ChargeTime.value);
			break;
		case CHARGEDELAY:
			sprintf(string, "%d", waveform.ChargeDelay.value);
			break;
		case CHARGECOUNT:
			sprintf(string, "%d", waveform.ChargeCount.value);
			break;
		case CHARGEINTERVAL:
			sprintf(string, "%d", waveform.ChargeInterval.value);
			break;
		case CURRMOD:
			sprintf(string, "%*d", 8, (int) currVar.mod);
			break;
		case PS1CURRVOLT:
			currVoltPS1 = ((double) allPSData[0].currLevelVoltage/(4095.0)) * (int) allPSData[0].maxVoltage;
			sprintf(string, "Curr Voltage\n%d V", currVoltPS1);
			break;
		case PS1TARVOLT:
			tarVoltPS1 = ((double) allPSData[0].tarLevelVoltage/(4095.0)) * (int) allPSData[0].maxVoltage;
			sprintf(string, "Set Voltage\n%d V", tarVoltPS1);
			break;
		case PS2CURRVOLT:
			currVoltPS2 = ((double) allPSData[1].currLevelVoltage/(4095.0)) * (int) allPSData[1].maxVoltage;
			sprintf(string, "Curr Voltage\n%d V", currVoltPS2);
			break;
		case PS2TARVOLT:
			tarVoltPS2 = ((double) allPSData[1].tarLevelVoltage/(4095.0)) * (int) allPSData[1].maxVoltage;
			sprintf(string, "Set Voltage\n%d V", tarVoltPS2);
			break;
		default:
			break;
		}

		genieWriteStr(Forms.allFormData[2].FormItems[formItem].id, string);
		begin = clock();
	}
	}
}

/////////////////////////////////////////////
//Callable functions by the Form structure.//
/////////////////////////////////////////////
void incVar(double *vars)
{
	int func = (int) vars[0];
	switch(func)
	{
		case PULSERNDMAX:
			waveform.PulseRndMax.value += (uint32_t) vars[1];
			if((waveform.PulseRndMax.value) < waveform.PulseRndMax.min){waveform.PulseRndMax.value = waveform.PulseRndMax.max;};
			if((waveform.PulseRndMax.value) > waveform.PulseRndMax.max){waveform.PulseRndMax.value = waveform.PulseRndMax.max;};
			break;
		case PULSERNDMIN:
			waveform.PulseRndMin.value += (uint32_t) vars[1];
			if((waveform.PulseRndMin.value) < waveform.PulseRndMin.min){waveform.PulseRndMin.value = waveform.PulseRndMin.max;};
			if((waveform.PulseRndMin.value) > waveform.PulseRndMin.max){waveform.PulseRndMin.value = waveform.PulseRndMin.max;};
			break;
		case PULSECOUNT:
			waveform.PulseCount.value += (uint32_t) vars[1];
			if((waveform.PulseCount.value) < waveform.PulseCount.min){waveform.PulseCount.value = waveform.PulseCount.max;};
			if((waveform.PulseCount.value) > waveform.PulseCount.max){waveform.PulseCount.value = waveform.PulseCount.max;};
			break;
		case PULSEWIDTH:
			waveform.PulseWidth.value += (uint32_t) vars[1];
			if((waveform.PulseWidth.value) < waveform.PulseWidth.min){waveform.PulseWidth.value = waveform.PulseWidth.max;};
			if((waveform.PulseWidth.value) > waveform.PulseWidth.max){waveform.PulseWidth.value = waveform.PulseWidth.max;};
			break;
		case BURSTRNDMAX:
			waveform.BurstRndMax.value += vars[1];
			if((waveform.BurstRndMax.value) < waveform.BurstRndMax.min){waveform.BurstRndMax.value = waveform.BurstRndMax.max;};
			if((waveform.BurstRndMax.value) > waveform.BurstRndMax.max){waveform.BurstRndMax.value = waveform.BurstRndMax.max;};
			break;
		case BURSTRNDMIN:
			waveform.BurstRndMin.value += vars[1];
			if((waveform.BurstRndMin.value) < waveform.BurstRndMin.min){waveform.BurstRndMin.value = waveform.BurstRndMin.max;};
			if((waveform.BurstRndMin.value) > waveform.BurstRndMin.max){waveform.BurstRndMin.value = waveform.BurstRndMin.max;};
			break;
		case BURSTCOUNT:
			waveform.BurstCount.value += (uint32_t) vars[1];
			if((waveform.BurstCount.value) < waveform.BurstCount.min){waveform.BurstCount.value = waveform.BurstCount.max;};
			if((waveform.BurstCount.value) > waveform.BurstCount.max){waveform.BurstCount.value = waveform.BurstCount.max;};
			break;
		case CYCLEWIDTH:
			waveform.CycleWidth.value += vars[1];
			if((waveform.CycleWidth.value) < waveform.CycleWidth.min){waveform.CycleWidth.value = waveform.CycleWidth.max;};
			if((waveform.CycleWidth.value) > waveform.CycleWidth.max){waveform.CycleWidth.value = waveform.CycleWidth.max;};
			break;
		case INHIBITTIME:
			waveform.InhibitTime.value += (uint32_t) vars[1];
			if((waveform.InhibitTime.value) < waveform.InhibitTime.min){waveform.InhibitTime.value = waveform.InhibitTime.max;};
			if((waveform.InhibitTime.value) > waveform.InhibitTime.max){waveform.InhibitTime.value = waveform.InhibitTime.max;};
			break;
		case INHIBITDELAY:
			waveform.InhibitDelay.value += (uint32_t) vars[1];
			if((waveform.InhibitDelay.value) < waveform.InhibitDelay.min){waveform.InhibitDelay.value = waveform.InhibitDelay.max;};
			if((waveform.InhibitDelay.value) > waveform.InhibitDelay.max){waveform.InhibitDelay.value = waveform.InhibitDelay.max;};
			break;
		case CHARGETIME:
			waveform.ChargeTime.value += (uint32_t) vars[1];
			if((waveform.ChargeTime.value) < waveform.ChargeTime.min){waveform.ChargeTime.value = waveform.ChargeTime.max;};
			if((waveform.ChargeTime.value) > waveform.ChargeTime.max){waveform.ChargeTime.value = waveform.ChargeTime.max;};
			break;
		case CHARGEDELAY:
			waveform.ChargeDelay.value += (uint32_t) vars[1];
			if((waveform.ChargeDelay.value) < waveform.ChargeDelay.min){waveform.ChargeDelay.value = waveform.ChargeDelay.max;};
			if((waveform.ChargeDelay.value) > waveform.ChargeDelay.max){waveform.ChargeDelay.value = waveform.ChargeDelay.max;};
			break;
		case CHARGECOUNT:
			waveform.ChargeCount.value += (uint32_t) vars[1];
			if((waveform.ChargeCount.value) < waveform.ChargeCount.min){waveform.ChargeCount.value = waveform.ChargeCount.max;};
			if((waveform.ChargeCount.value) > waveform.ChargeCount.max){waveform.ChargeCount.value = waveform.ChargeCount.max;};
			break;
		case CHARGEINTERVAL:
			waveform.ChargeInterval.value += (uint32_t) vars[1];
			if((waveform.ChargeInterval.value) < waveform.ChargeInterval.min){waveform.ChargeInterval.value = waveform.ChargeInterval.max;};
			if((waveform.ChargeInterval.value) > waveform.ChargeInterval.max){waveform.ChargeInterval.value = waveform.ChargeInterval.max;};
			break;
		case PS1TARVOLT:
			allPSData[0].tarLevelVoltage += (uint32_t) vars[1];
			if(allPSData[0].tarLevelVoltage > (4095.0 * ((double) allPSData[0].maxVoltageLim/(double) allPSData[0].maxVoltage))){allPSData[0].tarLevelVoltage = 4095.0 * ((double) allPSData[0].maxVoltageLim/(double) allPSData[0].maxVoltage);};
			//if(allPSData[0].tarLevelVoltage < (4095 * (allPSData[0].minVoltageLim/allPSData[0].minVoltage))){allPSData[0].tarLevelVoltage = 4095 * (allPSData[0].minVoltageLim/allPSData[0].minVoltage);};
			break;
		case PS2TARVOLT:
			allPSData[1].tarLevelVoltage += (uint32_t) vars[1];
			if(allPSData[1].tarLevelVoltage > (4095.0 * ((double) allPSData[1].maxVoltageLim/(double) allPSData[1].maxVoltage))){allPSData[1].tarLevelVoltage = 4095.0 * ((double) allPSData[1].maxVoltageLim/(double) allPSData[1].maxVoltage);};
			//if(allPSData[1].tarLevelVoltage < (4095 * (allPSData[1].minVoltageLim/allPSData[1].minVoltage))){allPSData[1].tarLevelVoltage = 4095 * (allPSData[1].minVoltageLim/allPSData[1].minVoltage);};
			break;
		default:
			break;
	}
}

void decVar(double *vars)
{

	printf("This is decVar %f, %f\n", vars[0], vars[1]);
	fflush(stdout);
	switch((int) vars[0])
	{
		case PULSERNDMAX:
			waveform.PulseRndMax.value -= (uint32_t) vars[1];
			if((waveform.PulseRndMax.value) < waveform.PulseRndMax.min){waveform.PulseRndMax.value = waveform.PulseRndMax.min;};
			if((waveform.PulseRndMax.value) > waveform.PulseRndMax.max){waveform.PulseRndMax.value = waveform.PulseRndMax.min;};
			break;
		case PULSERNDMIN:
			waveform.PulseRndMin.value -= (uint32_t) vars[1];
			if((waveform.PulseRndMin.value) < waveform.PulseRndMin.min){waveform.PulseRndMin.value = waveform.PulseRndMin.min;};
			if((waveform.PulseRndMin.value) > waveform.PulseRndMin.max){waveform.PulseRndMin.value = waveform.PulseRndMin.min;};
			break;
		case PULSECOUNT:
			waveform.PulseCount.value -= (uint32_t) vars[1];
			if((waveform.PulseCount.value) < waveform.PulseCount.min){waveform.PulseCount.value = waveform.PulseCount.min;};
			if((waveform.PulseCount.value) > waveform.PulseCount.max){waveform.PulseCount.value = waveform.PulseCount.min;};
			break;
		case PULSEWIDTH:
			waveform.PulseWidth.value -= (uint32_t) vars[1];
			if((waveform.PulseWidth.value) < waveform.PulseWidth.min){waveform.PulseWidth.value = waveform.PulseWidth.min;};
			if((waveform.PulseWidth.value) > waveform.PulseWidth.max){waveform.PulseWidth.value = waveform.PulseWidth.min;};
			break;
		case BURSTRNDMAX:
			waveform.BurstRndMax.value -= vars[1];
			if((waveform.BurstRndMax.value) < waveform.BurstRndMax.min){waveform.BurstRndMax.value = waveform.BurstRndMax.min;};
			if((waveform.BurstRndMax.value) > waveform.BurstRndMax.max){waveform.BurstRndMax.value = waveform.BurstRndMax.min;};
			break;
		case BURSTRNDMIN:
			waveform.BurstRndMin.value -= vars[1];
			if((waveform.BurstRndMin.value) < waveform.BurstRndMin.min){waveform.BurstRndMin.value = waveform.BurstRndMin.min;};
			if((waveform.BurstRndMin.value) > waveform.BurstRndMin.max){waveform.BurstRndMin.value = waveform.BurstRndMin.min;};
			break;
		case BURSTCOUNT:
			waveform.BurstCount.value -= (uint32_t) vars[1];
			if((waveform.BurstCount.value) < waveform.BurstCount.min){waveform.BurstCount.value = waveform.BurstCount.min;};
			if((waveform.BurstCount.value) > waveform.BurstCount.max){waveform.BurstCount.value = waveform.BurstCount.min;};
			break;
		case CYCLEWIDTH:
			waveform.CycleWidth.value -= vars[1];
			if((waveform.CycleWidth.value) < waveform.CycleWidth.min){waveform.CycleWidth.value = waveform.CycleWidth.min;};
			if((waveform.CycleWidth.value) > waveform.CycleWidth.max){waveform.CycleWidth.value = waveform.CycleWidth.min;};
			break;
		case INHIBITTIME:
			waveform.InhibitTime.value -= (uint32_t) vars[1];
			if((waveform.InhibitTime.value) < waveform.InhibitTime.min){waveform.InhibitTime.value = waveform.InhibitTime.min;};
			if((waveform.InhibitTime.value) > waveform.InhibitTime.max){waveform.InhibitTime.value = waveform.InhibitTime.min;};
			break;
		case INHIBITDELAY:
			waveform.InhibitDelay.value -= (uint32_t) vars[1];
			if((waveform.InhibitDelay.value) < waveform.InhibitDelay.min){waveform.InhibitDelay.value = waveform.InhibitDelay.min;};
			if((waveform.InhibitDelay.value) > waveform.InhibitDelay.max){waveform.InhibitDelay.value = waveform.InhibitDelay.min;};
			break;
		case CHARGETIME:
			waveform.ChargeTime.value -= (uint32_t) vars[1];
			if((waveform.ChargeTime.value) < waveform.ChargeTime.min){waveform.ChargeTime.value = waveform.ChargeTime.min;};
			if((waveform.ChargeTime.value) > waveform.ChargeTime.max){waveform.ChargeTime.value = waveform.ChargeTime.min;};
			break;
		case CHARGEDELAY:
			waveform.ChargeDelay.value -= (uint32_t) vars[1];
			if((waveform.ChargeDelay.value) < waveform.ChargeDelay.min){waveform.ChargeDelay.value = waveform.ChargeDelay.min;};
			if((waveform.ChargeDelay.value) > waveform.ChargeDelay.max){waveform.ChargeDelay.value = waveform.ChargeDelay.min;};
			break;
		case CHARGECOUNT:
			waveform.ChargeCount.value -= (uint32_t) vars[1];
			if((waveform.ChargeCount.value) < waveform.ChargeCount.min){waveform.ChargeCount.value = waveform.ChargeCount.min;};
			if((waveform.ChargeCount.value) > waveform.ChargeCount.max){waveform.ChargeCount.value = waveform.ChargeCount.min;};
			break;
		case CHARGEINTERVAL:
			waveform.ChargeInterval.value -= (uint32_t) vars[1];
			if((waveform.ChargeInterval.value) < waveform.ChargeInterval.min){waveform.ChargeInterval.value = waveform.ChargeInterval.min;};
			if((waveform.ChargeInterval.value) > waveform.ChargeInterval.max){waveform.ChargeInterval.value = waveform.ChargeInterval.min;};
			break;
		case PS1TARVOLT:
			if(allPSData[0].tarLevelVoltage < (uint32_t) vars[1])
			{
				allPSData[0].tarLevelVoltage = 0;
				break;
			};
			allPSData[0].tarLevelVoltage -= (uint32_t) vars[1];
			if(allPSData[0].tarLevelVoltage > (4095 * ((double) allPSData[0].maxVoltageLim/(double) allPSData[0].maxVoltage))){allPSData[0].tarLevelVoltage = 4095 * ((double) allPSData[0].maxVoltageLim/(double) allPSData[0].maxVoltage);};
			break;
		case PS2TARVOLT:
			if(allPSData[1].tarLevelVoltage < ((uint32_t) vars[1]))
			{
				allPSData[1].tarLevelVoltage = 0;
				break;
			};
			allPSData[1].tarLevelVoltage -= (uint32_t) vars[1];
			if(allPSData[1].tarLevelVoltage > (4095 * ((double) allPSData[1].maxVoltageLim/(double) allPSData[1].maxVoltage))){allPSData[1].tarLevelVoltage = 4095 * ((double) allPSData[1].maxVoltageLim/(double) allPSData[1].maxVoltage);};
			break;
		default:
			break;
	}

}

void formChanged(double *Var)
{
	printf("CalledFormChanged");
	double *ps;
	int formItem;
	char string1[50];
	char string2[20];

	//set the loop
	setCurrLoop(Var[1]);


	if((int) Var[0] == 2)
	{
		//Select the Multiple Stroke button
		genieWriteObj(IUSRBUTTON, 10, 1);

	}

	if((int) Var[2] == INITPINS)
	{
		//Go to select waveform screen form 1
		if((int) Var[0] == 1)
		{
			//Go to Init Generator Screen
			changeForm(INITFORM);
			//Set default pin system
			defaultPinSystem();
		}

		//Go to run screen screen 2
		if((int) Var[0] == 2)
		{


			//Go to Init Generator Screen
			changeForm(INITFORM);

			usleep(1000);
			//Initialize the pin profile.
			initWaveform();

		}
	}

	changeForm((int) Var[0]);

	if((int) Var[0] == 4)
	{
		sprintf(string1, "     Select A Variable");
		genieWriteStr(8,string1);

		sprintf(string2, "             -");
		genieWriteStr(9,string2);


		genieWriteObj(IUSRBUTTON, 11, 0);
		genieWriteObj(IUSRBUTTON, 12, 0);
		genieWriteObj(IUSRBUTTON, 13, 0);

	}

}

void changeForm(int var)
{
	//Change the form
	genieWriteObj(IFORM, var, 0);
	//Update form here
	updateForm();
}

void updateForm()
{
	int sliderItem = -1;
	int stringItem = -1;
	int currForm = 0;
	char string[100];
	char tempString0[20];
	char tempString1[20];
	int currVoltPS1;
	int tarVoltPS1;
	int currVoltPS2;
	int tarVoltPS2;
	int itemID;

	//grab current form
	currForm = getCurrentForm();

	printf("CurrForm: %d\n", currForm);
	fflush(stdout);

	//loop through objects and update the screen with them.
	for(stringItem = 0; stringItem < FORMITEMNUM; stringItem++)
	{

		if((Forms.allFormData[currForm].FormItems[stringItem].type == ISTRING) && (Forms.allFormData[currForm].FormItems[stringItem].funcVars[0][0] > 0))
		{
			int strIndex = (int) Forms.allFormData[currForm].FormItems[stringItem].funcVars[0][0];


			//ADD ITEMS HERE TO NOT UPDATE//
			itemID = Forms.allFormData[currForm].FormItems[stringItem].id;
			if(((itemID == 9) || (itemID == 8)) && (currForm == 4)){continue;};
			//END NO UPDATE//

			int varItem = (int) Forms.allFormData[currForm].FormItems[stringItem].funcVars[0][strIndex];

			switch(varItem)
				{

					case PULSERNDMAX:
						//sprintf(string, "Pulse Rnd Max.\n%d uS", waveform.PulseRndMax.value);
						sprintf(string, "%d mS", waveform.PulseRndMax.value/1000);
						break;
					case PULSERNDMIN:
						//sprintf(string, "Pulse Rnd Min.\n%d uS", waveform.PulseRndMin.value);
						sprintf(string, "%d mS", waveform.PulseRndMin.value/1000);
						break;
					case PULSECOUNT:
						//sprintf(string, "Pulse Count\n%d", waveform.PulseCount.value);
						sprintf(string, "%d", waveform.PulseCount.value);
						break;
					case PULSEWIDTH:
						sprintf(string, "Pulse Width\n%d uS", waveform.PulseWidth.value);
						break;
					case BURSTRNDMAX:
						sprintf(string, "Burst Rnd. Max.\n%f mS", waveform.BurstRndMax.value);
						break;
					case BURSTRNDMIN:
						sprintf(string, "Burst Rnd. Min.\n%f mS", waveform.BurstRndMin.value);
						break;
					case BURSTCOUNT:
						sprintf(string, "Burst Count\n %d", waveform.BurstCount.value);
						break;
					case CYCLEWIDTH:
						sprintf(string, "Cycle Time\n%f S", waveform.CycleWidth.value);
						break;
					case INHIBITTIME:
						sprintf(string, "Inhibit Time\n%d uS", waveform.InhibitTime.value);
						break;
					case INHIBITDELAY:
						sprintf(string, "Inhibit Delay\n%d uS", waveform.InhibitDelay.value);
						break;
					case CHARGETIME:
						sprintf(string, "%d", waveform.ChargeTime.value);
						break;
					case CHARGEDELAY:
						sprintf(string, "%d", waveform.ChargeDelay.value);
						break;
					case CHARGECOUNT:
						sprintf(string, "%d", waveform.ChargeCount.value);
						break;
					case CHARGEINTERVAL:
						sprintf(string, "%d", waveform.ChargeInterval.value);
						break;
					case CURRMOD:
						sprintf(string, "%*d", 8, (int) currVar.mod);
						break;
					case CURRVARNAME:
						sprintf(string, "%s", currVar.name);
						break;
					case CURRWFNAME:
						sprintf(string, "%s", currSystem.profileName);
						break;
					case PS1CURRVOLT:
						currVoltPS1 = ((double) allPSData[0].currLevelVoltage/(4095.0)) * (int) allPSData[0].maxVoltage;
						sprintf(string, "Curr Voltage\n%d V", currVoltPS1);
						break;
					case PS1TARVOLT:
						tarVoltPS1 = ((double) allPSData[0].tarLevelVoltage/(4095.0)) * (int) allPSData[0].maxVoltage;
						sprintf(string, "%d V", tarVoltPS1);
						break;
					case PS2CURRVOLT:
						currVoltPS2 = ((double) allPSData[1].currLevelVoltage/(4095.0)) * (int) allPSData[1].maxVoltage;
						sprintf(string, "Curr Voltage\n%d V", currVoltPS2);
						break;
					case PS2TARVOLT:
						tarVoltPS2 = ((double) allPSData[1].tarLevelVoltage/(4095.0)) * (int) allPSData[1].maxVoltage;
						sprintf(string, "%d V", tarVoltPS2);
						break;
					case WFINFO:
						sprintf(tempString0, "%dmS - %dmS\n", waveform.PulseRndMin.value/1000, waveform.PulseRndMax.value/1000);
						sprintf(string, "\n        Pulse Random Range:\n%*s\n                Pulse Count:\n%*d", 27, tempString0, 25, waveform.PulseCount);
						break;
					default:
						break;
				}


			//If we got a string from the Form structure output it to the form.
			genieWriteStr(Forms.allFormData[currForm].FormItems[stringItem].id, string);
		}
		usleep(1000);
	}
}

//INPROGRESS UPDATE A SINGLE ITEM
//Currently only works for strings
void updateItem(int currForm, int itemID, int type)
{
	char string[50];
	int stringItem;

	for(stringItem = 0; stringItem < FORMITEMNUM; stringItem++)
	{
		if((Forms.allFormData[currForm].FormItems[stringItem].type == type) && (Forms.allFormData[currForm].FormItems[stringItem].id == itemID))
		{
			break;
		}
	}
	strcpy(string, strDisplayFormat(currForm, stringItem));
	genieWriteStr(itemID, string);

}

void updateSpecItem0(int currForm, int itemID, int type)
{
	char string[50];
	char stringDisplay[5];
	int stringItem;

	for(stringItem = 0; stringItem < FORMITEMNUM; stringItem++)
	{
		if((Forms.allFormData[currForm].FormItems[stringItem].type == type) && (Forms.allFormData[currForm].FormItems[stringItem].id == itemID))
		{
			break;
		}
	}

	strcpy(string, specificDisplayFormat0(currForm, stringItem));
	genieWriteStr(itemID, string);
}

void updateCurrVarString(int currForm)
{
	char string[50];
	sprintf(string, "TESTING");
	int stringItem;

	for(stringItem = 0; stringItem < FORMITEMNUM; stringItem++)
	{
		if((Forms.allFormData[currForm].FormItems[stringItem].type == ISTRING) && (Forms.allFormData[currForm].FormItems[stringItem].funcVars[0][1] == currVar.var))
		{
			break;
		}
	}

	strcpy(string, strDisplayFormat(currForm, stringItem));
	genieWriteStr(Forms.allFormData[currForm].FormItems[stringItem].id, string);
}


void prepWaveform(void)
{
	//Set variables in dnbWaves.h
	setWavesVars();

	//Generate a new waveform
	GenerateWaveForm(-1);

	//Create the waveform and put into DMA for run

}

void beginWave(int repeat)
{
	//Run dnbWaves.h wrapper wave start.
	ReadyWaveForm(repeat);

	printf("OUTPUTING A WAVE\n");

	StartWaveform();
}

void cancelWave(void)
{
	StopWaveform();
	//WaveTerminate();
}

void setCurrSystem(double * i)
{
	currSystem.profileNum = i[0];
	currSystem.profileName = systems[(int) i[0]].name;
	currSystem.genMode = systems[(int) i[0]].genMode;

	//releasePins();
}

void defaultPinSystem(double *Var)
{
	double system[] = {0};
	double sys[1];
	//Turn off PS
	//setPSZero();
	sys[0] = currSystem.profileNum;
	//Set waveform default system
	setCurrSystem(system);
	initCurrSystem(system);

	setCurrSystem(sys);

	//wait 10mS for voltage to equalize
	usleep(10000);
}

void pulseSCR(void)
{
	//Set SCR pin data
	SCRWFData();

	//Ready and output waveform for SCR pulse
	setGenMode(1);
	GenerateWaveForm(1);
	ReadyWaveForm(1);
	StartWaveform();

	//Set output pin to wavePin
	setWavePinMask(waveformPins.wavePinMask);
	//Reset Gen mode to normal
	setGenMode(currSystem.genMode);
	//Set WF Vars to normal
	setWavesVars();
	//Generate waveform ready to output
	GenerateWaveForm(1);
}

void SCRWFData(void)
{
	setPulseRndMax(1000);
	setPulseRndMin(1000);
	setPulseCount(1);
	setPulseWidth(10);
	setBurstCount(1);
	setCycleWidth(1);
	setWavePinMask(waveformPins.SCRPinMask);
}

void initCurrSystem(double * Var)
{
	initWaveform();
}

void setCurrVar(double * Var)
{
	currVar.var = (int) Var[0];

	switch(currVar.var)
	{
	case PULSERNDMAX:
		sprintf(currVar.name,"Pulse Random Max");
		break;
	case PULSERNDMIN:
		sprintf(currVar.name,"Pulse Random Min");
		break;
	case PULSECOUNT:
		sprintf(currVar.name,"Pulse Count");
		break;
	case PULSEWIDTH:
		waveform.PulseWidth.value += (uint32_t) currVar.mod;
		break;
	case BURSTRNDMAX:
		waveform.BurstRndMax.value += currVar.mod;
		break;
	case BURSTRNDMIN:
		waveform.BurstRndMin.value += currVar.mod;
		break;
	case BURSTCOUNT:
		waveform.BurstCount.value += (uint32_t) currVar.mod;
		break;
	case CYCLEWIDTH:
		waveform.CycleWidth.value += currVar.mod;
		break;
	case INHIBITTIME:
		waveform.InhibitTime.value += (uint32_t) currVar.mod;
		break;
	case INHIBITDELAY:
		waveform.InhibitDelay.value += (uint32_t) currVar.mod;
		break;
	case CHARGETIME:
		waveform.ChargeTime.value += (uint32_t) currVar.mod;
		break;
	case CHARGEDELAY:
		waveform.ChargeDelay.value += (uint32_t) currVar.mod;
		break;
	case CHARGECOUNT:
		waveform.ChargeCount.value += (uint32_t) currVar.mod;
		break;
	case CHARGEINTERVAL:
		waveform.ChargeInterval.value += (uint32_t) currVar.mod;
		break;
	default:
		break;
	}
}

void decCurrVar(void)
{
	double varInfo[2];
	varInfo[0] = (double) currVar.var;
	varInfo[1] = currVar.mod;

	switch(currVar.var)
	{
	case PULSERNDMAX:
		varInfo[1] = currVar.mod * 1000;
		break;
	case PULSERNDMIN:
		varInfo[1] = currVar.mod * 1000;
		break;
	case PULSECOUNT:
		break;
	case PULSEWIDTH:
		break;
	case BURSTRNDMAX:
		break;
	case BURSTRNDMIN:
		break;
	case BURSTCOUNT:
		break;
	case CYCLEWIDTH:
		break;
	case INHIBITTIME:
		break;
	case INHIBITDELAY:
		break;
	case CHARGETIME:
		break;
	case CHARGEDELAY:
		break;
	default:
		break;
	}
	decVar(varInfo);
}

void incCurrVar(void)
{
	double varInfo[2];
	varInfo[0] = (double) currVar.var;
	varInfo[1] = currVar.mod;

	switch(currVar.var)
	{
	case PULSERNDMAX:
		varInfo[1] = currVar.mod * 1000;
		break;
	case PULSERNDMIN:
		varInfo[1] = currVar.mod * 1000;
		break;
	case PULSECOUNT:
		break;
	case PULSEWIDTH:
		break;
	case BURSTRNDMAX:
		break;
	case BURSTRNDMIN:
		break;
	case BURSTCOUNT:
		break;
	case CYCLEWIDTH:
		break;
	case INHIBITTIME:
		break;
	case INHIBITDELAY:
		break;
	case CHARGETIME:
		break;
	case CHARGEDELAY:
		break;
	default:
		break;
	}
	incVar(varInfo);
}

//This is a mod value for the slider. Sliding up will add values at each point. Sliding down will decrease values.
void changeModVal(double *Val)
{
    if((int) Val[0] == 1)
    {
    	if(currVar.mod < CURRMODMAX)
    	{
            currVar.mod = currVar.mod * 10;
    	}
    }
    else if((int) Val[0] == 0)
    {
        if(currVar.mod > 1)
        {
           currVar.mod = currVar.mod / 10;
        }
    }

}


void loadWFVars(double * Val)
{
	printf("%s", currFiles[(int) Val[0]].file);
	fflush(stdout);
	readWaveformFile(currFiles[(int) Val[0]].file);
}

void checkGPIOModes(void)
{
	int mode = 0;
	for (mode = 0; mode < 53; mode++)
		printf("%2d ", mode);
	printf("\n");

	for (mode = 0; mode < 53; mode++)
		printf("%2d ", dnbGPIOMode(mode, -1));
	printf("\n");

	fflush(stdout);
}

void singleWFOut(void)
{
	setPulseRndMax(1000);
	setPulseRndMin(1000);
	setPulseCount(1);
	setBurstCount(1);

	//GenerateWaveForm(-1);
}

void setCurrLoop(double loop)
{
	currLoop = (int) loop;
}

//Set target voltage on slider until slider is released.
void PSSetTarLoop(double *ps, int level)
{
	struct timespec startTime;
	struct timespec currTime;
	double diff;
	uint setLevel;
	int currForm = -1;
	int formItem = -1;


	setPSTarVal(ps, level);

	clock_gettime(CLOCK_MONOTONIC, &startTime);

	while(1)
	{
		printf("In Slider Loop\n");
		fflush(stdout);
		usleep(1000);
		if(genieReplyAvail() == 1)
		{
			getSerialData();
			outputData();
			usleep(1000);


			currForm = getCurrentForm();
			//get the item
			replyData.cmd = replyData.cmd & 0x0F;

			if(replyData.cmd == REPORTEVENT)
			{
				//Grab the item
				for(formItem = 0; formItem< FORMITEMNUM; formItem++)
				{
					if((replyData.object == Forms.allFormData[currForm].FormItems[formItem].type) && (replyData.index ==  Forms.allFormData[currForm].FormItems[formItem].id))
					{
						printf("IGOT THE ITEM\n");
						fflush(stdout);

						break;
					}
				}
			}

			//set the value then update the form.
			setPSTarVal(Forms.allFormData[currForm].FormItems[formItem].funcVars[0], replyData.data);
			updateForm();
			//restart the timer for dropout
			clock_gettime(CLOCK_MONOTONIC, &startTime);
		}

		clock_gettime(CLOCK_MONOTONIC, &currTime);
		diff = (currTime.tv_sec - startTime.tv_sec) + ((currTime.tv_nsec - startTime.tv_nsec) * 1e-9);
		printf("DiffTime: %f", diff);
		if(diff > SLIDER_DROP_TIME)
		{
			setLevel = genieReadObj(Forms.allFormData[currForm].FormItems[formItem].type, Forms.allFormData[currForm].FormItems[formItem].id);
			//set the PSTarVal to this value
			setPSTarVal(ps, setLevel);

			break;
		}
	}
}

//Set target voltage on slider until slider is released.
void PSSetTarLoop2(double *ps, int level)
{
	struct timespec startTime;
	struct timespec currTime;
	double diff;
	uint setLevel;
	int currForm = -1;
	int formItem = -1;


	setPSTarVal(ps, level);

	clock_gettime(CLOCK_MONOTONIC, &startTime);

	while(1)
	{
		printf("In Slider Loop\n");
		fflush(stdout);
		usleep(1000);
		if(genieReplyAvail() == 1)
		{
			getSerialData();
			outputData();
			usleep(1000);


			currForm = getCurrentForm();
			//get the item
			replyData.cmd = replyData.cmd & 0x0F;

			if(replyData.cmd == REPORTEVENT)
			{
				//Grab the item
				for(formItem = 0; formItem< FORMITEMNUM; formItem++)
				{
					if((replyData.object == Forms.allFormData[currForm].FormItems[formItem].type) && (replyData.index ==  Forms.allFormData[currForm].FormItems[formItem].id))
					{
						printf("IGOT THE ITEM\n");
						fflush(stdout);

						break;
					}
				}
			}

			//set the value then update the form.
			setPSTarVal(Forms.allFormData[currForm].FormItems[formItem].funcVars[0], replyData.data);
			updateForm();
			//restart the timer for dropout
			clock_gettime(CLOCK_MONOTONIC, &startTime);
		}

		clock_gettime(CLOCK_MONOTONIC, &currTime);
		diff = (currTime.tv_sec - startTime.tv_sec) + ((currTime.tv_nsec - startTime.tv_nsec) * 1e-9);
		printf("DiffTime: %f", diff);
		if(diff > SLIDER_DROP_TIME)
		{
			setLevel = genieReadObj(Forms.allFormData[currForm].FormItems[formItem].type, Forms.allFormData[currForm].FormItems[formItem].id);
			//set the PSTarVal to this value
			setPSTarVal(ps, setLevel);

			break;
		}
	}
}

//Set the target value of the PS. This will set values from 0 - PS_SLIDER_MAX which is the highest slider value on the screen slider.
void setPSTarVal(double *ps, int level)
{
	//PS takes in 0-4095. Slider is from 0 to PS_SLIDER_MAX
	int voltageLimit;

	//Set upper bracketing.
	voltageLimit = (4095 * ((double) allPSData[(int) ps[0] - 1].maxVoltageLim / (double) allPSData[(int) ps[0] - 1].maxVoltage));

	printf("VoltLimit %d", voltageLimit);

	level = ((double) level/(double) PS_SLIDER_MAX) * voltageLimit;

	allPSData[(int) ps[0] - 1].tarLevelVoltage = (uint32_t) level;
}

//Set target voltages of all PS to zero and apply those voltages.
void setPSZero(void)
{
	int i;
	for(i = 0; i<PSNUM; i++)
	{
		allPSData[i].currLevelVoltage = 0;
		allPSData[i].tarLevelVoltage = 0;
		allPSData[i].setLevelVoltage = 0;
	}
	setPSVoltage(1);
	setPSVoltage(2);
}

//modify the current target and set voltage for the PS and send that to the PS
void psMod(double *ps)
{
	double voltageLimit;

	//Set upper bracketing.
	voltageLimit = (4095.0 * ((double) allPSData[(int) ps[0] - 1].maxVoltageLim / (double) allPSData[(int) ps[0] - 1].maxVoltage)) - PSMOD;

	if(((int) ps[1] == 1) && (allPSData[(int) ps[0] - 1].tarLevelVoltage <= voltageLimit))
	{
		allPSData[(int) ps[0] - 1].tarLevelVoltage += PSMODVAL;
		setPSVoltage(ps);
		genieWriteObj(ISLIDER, (int) ps[0] - 1, (((double) allPSData[(int) ps[0] - 1].tarLevelVoltage)/voltageLimit) * PS_SLIDER_MAX);
		printf("Slider Position: %d", (((double) allPSData[(int) ps[0] - 1].tarLevelVoltage)/voltageLimit) * PS_SLIDER_MAX);

	}
	if(((int) ps[1] == 2) && (allPSData[(int) ps[0] - 1].tarLevelVoltage >= (PSMOD)))
	{
		allPSData[(int) ps[0] - 1].tarLevelVoltage -= PSMODVAL;
		setPSVoltage(ps);
		genieWriteObj(ISLIDER, (int) ps[0] - 1, (((double) allPSData[(int) ps[0] - 1].tarLevelVoltage)/voltageLimit) * PS_SLIDER_MAX);
	}
}


void setPSVoltage(int port)
{
	char data[5];

	//set the set level to the tar level so we know what it is supposed to be at.
	allPSData[port - 1].setLevelVoltage = allPSData[port - 1].tarLevelVoltage;

	sprintf(data, "%d", allPSData[port - 1].setLevelVoltage);

	sendCommand(port, SLM_PROGRAM_KV_SETPOINT, data);
}

void readPSVoltage(int port)
{
	requestCommand(port, 60);
	usleep(100000);
	readCommand(&rxPSData[port - 1], RXMAXLEN);
	allPSData[port - 1].currLevelVoltage = atoi(rxPSData[port - 1].data[0]);
}

void terminateProgram(void)
{
	WaveTerminate();
	//exit(EXIT_FAILURE);

}

void setPSLocalRemote(int port, int i)
{
	char txSend[] = "1,";


	sendCommand(port, SLM_PROGRAM_LOCAL_REMOTE, txSend);


}

void setNominalCurrent(int port)
{
	char txSend[] = "100,";

	sendCommand(port, SLM_PROGRAM_MA_SETPOINT, txSend);
}

void PSStatusRequest(void)
{
	char txSend[] = {};

	requestCommand(1, SLM_REQUEST_STATUS);
}
////////////////////////////////////////////////////
////////////Initialization functions////////////////
////////////////////////////////////////////////////
void initSystem(void)
{
	//Grab formData and pinData for current generator setup.

	readPinFile(pinFile);

	readFormFile(formFile);

	//Read in waveform file if available
	readWaveformFile(waveFile);

	//initialize waveform lib
	WaveInit();

	//init UART1
	uart1Init(115200);

	//Update the current form.
	//updateForm();
}



void initWaveform(void)
{
	//Turn off/inhibit power supply power.
	setWFLogMode(1);
	//output all pin modes before inits
	//checkGPIOModes();

	//Turn generator outputs on for current waveform and initialize all Pins
	initPins(currSystem.profileNum);

	//output all pin modes after inits
	//checkGPIOModes();

	//set generator mode
	setGenMode(currSystem.genMode);
}

void initPins(int waveNum)
{
	int i = 0;

	//set all waveformPinMasks to zero
	waveformPins.chargePinMask = 0;
	waveformPins.inhibitPinMask = 0;
	waveformPins.wavePinMask = 0;
	waveformPins.waveInhibitPinMask = 0;

	//Set all pin modes
	initPinModes(waveNum);


	//Setup variables for delay
	double delayUS;
	struct timespec startTime;
	struct timespec currTime;
	time_t diff;
	long int nDiff;
	double elapsedTime;

	//Set all states of pins or set charge/pulse/inhibit variables for waveformPins.
	for(i = 0; i < PINNUMMAX; i++)
	{
		//set current time
		clock_gettime(CLOCK_MONOTONIC, &startTime);
		elapsedTime = 0;


		printf("SetPin: %d\n",systems[waveNum].pins[i].pinNum);
		fflush(stdout);
		//check if the pin does not exist.
		if(systems[waveNum].pins[i].pinNum == 0){break;}

		//DelayMs(500);

		if((strcmp(systems[waveNum].pins[i].pinType, "CHARGE")) == 0)
		{
			waveformPins.chargePinMask = waveformPins.chargePinMask | systems[waveNum].pins[i].pinMask.U64;
		}

		else if((strcmp(systems[waveNum].pins[i].pinType, "PULSE")) == 0)
		{
			waveformPins.wavePinMask = waveformPins.wavePinMask | systems[waveNum].pins[i].pinMask.U64;
		}

		else if((strcmp(systems[waveNum].pins[i].pinType, "INHIBIT")) == 0)
		{
			waveformPins.inhibitPinMask = waveformPins.inhibitPinMask | systems[waveNum].pins[i].pinMask.U64;
			printf("Set inhibit pin low");
			fflush(stdout);
			dnbGPIOSetLow(systems[waveNum].pins[i].pinMask.U64);
		}

		else if((strcmp(systems[waveNum].pins[i].pinType, "WAVEINHIBIT")) == 0)
		{
			waveformPins.waveInhibitPinMask = waveformPins.waveInhibitPinMask | systems[waveNum].pins[i].pinMask.U64;
			printf("Set waveinhibit pin low");
			fflush(stdout);
			dnbGPIOSetLow(systems[waveNum].pins[i].pinMask.U64);
		}

		else if((strcmp(systems[waveNum].pins[i].pinType, "HIGH")) == 0)
		{
			dnbGPIOSetHigh(systems[waveNum].pins[i].pinMask.U64);
		}

		else if((strcmp(systems[waveNum].pins[i].pinType, "LOW")) == 0)
		{
			dnbGPIOSetLow(systems[waveNum].pins[i].pinMask.U64);
		}

		else if ((strcmp(systems[waveNum].pins[i].pinType, "DISCHARGE")) == 0)
		{
			//Do things here to discharge the circuit.
		}

		else if((strcmp(systems[waveNum].pins[i].pinType, "SCR")) == 0)
		{
			waveformPins.SCRPinMask = waveformPins.SCRPinMask | systems[waveNum].pins[i].pinMask.U64;
			pulseSCR();
		}
		else
		{
			//continue;
		}




		//set delay value
		delayUS = systems[waveNum].pins[i].pinDelay / 1000.;

		//wait until we are finished!
		while(elapsedTime < delayUS)
		{

		//Get current time
		clock_gettime(CLOCK_MONOTONIC, &currTime);

		//calculate time variables
		diff = currTime.tv_sec - startTime.tv_sec;
		nDiff = currTime.tv_nsec - startTime.tv_nsec;
		elapsedTime = diff + (nDiff * 1e-9);


		//Set resolution of the loop.
		usleep(10000);

		}
	}

	//Set charge/inhibit/pulse pins in dnbWaves
	setChargeMask(waveformPins.chargePinMask);
	setInhibitMask(waveformPins.inhibitPinMask);
	setWavePinMask(waveformPins.wavePinMask);
	setWaveInhibitMask(waveformPins.waveInhibitPinMask);
}

void initPinModes(int waveNum)
{
	int i;

	//Loop through pins and set pin modes
	for(i = 0; i < PINNUMMAX; i++)
	{
		if(systems[waveNum].pins[i].pinNum > 0)
		{
			dnbGPIOMode(systems[waveNum].pins[i].pinNum, systems[waveNum].pins[i].pinMode);
		}
	}
}



void releasePins(void)
{

	int i = 0;
	long long profile = 1;

	//Set all output pin masks to zero
	waveformPins.chargePinMask = 0;
	waveformPins.inhibitPinMask = 0;
	waveformPins.wavePinMask = 0;

	//Release all pins/turn off all pins
	dnbGPIOSetLow(allPins.U64);

	for(i = 0; i < 53; i++)
	{
		if(allPins.U64 & profile)
		{
			dnbGPIOMode(i,0);
		}

		profile = profile << 1;
	}

}

///////////////////////////
//File Parsing functions.//
///////////////////////////
void readWaveformFile(char file[])
{
	FILE *fd;
	char *mode = "r";
	size_t len=0;
	ssize_t read;
	char * line = NULL;
	char * token;
	char * dummy;

	fd = fopen(file, mode);
	if (fd == NULL)
	{
			printf("HALP NO WAVEFORM FILE.\nExiting program.\nPlease include a waveform file with all relevant variables.\n");
			fflush(stdout);
	        exit(EXIT_FAILURE);
	}

	//Grab a line from the text file
	while ((read = getline(&line, &len, fd)) !=-1)
	{


		//Separate the line at ','
		while((token = strsep(&line, ",")) != NULL)
		{
			//save token to be compared to for this iteration.
			char * i = token;

			if(strcmp(i, "PulseRndMax") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.PulseRndMax.value = (uint32_t) atoi(token);

					//Grab Item Min Value
					token = strsep(&line, ",");
					waveform.PulseRndMax.min = (uint32_t) atoi(token);

					//Grab Item Max Value
					token = strsep(&line, ",");
					waveform.PulseRndMax.max = (uint32_t) atoi(token);

					printf("PulseRndMax: %d\n", waveform.PulseRndMax.value);
					break;
			}
			else if(strcmp(i, "PulseRndMin") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.PulseRndMin.value = (uint32_t) atoi(token);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.PulseRndMin.min = (uint32_t) atoi(token);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.PulseRndMin.max = (uint32_t) atoi(token);

					printf("PulseRndMin: %d\n", waveform.PulseRndMin.value);
					break;
			}
			else if(strcmp(i, "PulseCount") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.PulseCount.value = (uint32_t) atoi(token);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.PulseCount.min = (uint32_t) atoi(token);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.PulseCount.max = (uint32_t) atoi(token);

					printf("PulseCount: %d\n", waveform.PulseCount.value);
					break;
			}
			else if(strcmp(i, "PulseWidth") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.PulseWidth.value = (uint32_t) atoi(token);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.PulseWidth.min = (uint32_t) atoi(token);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.PulseWidth.max = (uint32_t) atoi(token);

					printf("PulseWidth: %d\n", waveform.PulseWidth.value);
					break;
			}
			else if(strcmp(i, "BurstCount") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.BurstCount.value = (uint32_t) atoi(token);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.BurstCount.min = (uint32_t) atoi(token);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.BurstCount.max = (uint32_t) atoi(token);

					printf("BurstCount: %d\n", waveform.BurstCount.value);
					break;
			}
			else if(strcmp(i, "InhibitTime") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.InhibitTime.value = (uint32_t) atoi(token);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.InhibitTime.min = (uint32_t) atoi(token);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.InhibitTime.max = (uint32_t) atoi(token);

					printf("InhibitTime: %d\n", waveform.InhibitTime.value);
					break;
			}
			else if(strcmp(i, "InhibitDelay") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.InhibitDelay.value = (uint32_t) atoi(token);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.InhibitDelay.min = (uint32_t) atoi(token);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.InhibitDelay.max = (uint32_t) atoi(token);

					printf("InhibitDelay: %d\n", waveform.InhibitDelay.value);
					break;
			}
			else if(strcmp(i, "ChargeTime") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.ChargeTime.value = (uint32_t) atoi(token);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.ChargeTime.min = (uint32_t) atoi(token);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.ChargeTime.max = (uint32_t) atoi(token);

					printf("ChargeTime: %d\n", waveform.ChargeTime.value);
					break;
			}
			else if(strcmp(i, "ChargeDelay") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.ChargeDelay.value = (uint32_t) atoi(token);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.ChargeDelay.min = (uint32_t) atoi(token);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.ChargeDelay.max = (uint32_t) atoi(token);

					printf("ChargeDelay: %d\n", waveform.ChargeDelay.value);
					break;
			}
			else if(strcmp(i, "ChargeCount") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.ChargeCount.value = (uint32_t) atoi(token);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.ChargeCount.min = (uint32_t) atoi(token);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.ChargeCount.max = (uint32_t) atoi(token);

					printf("ChargeCount: %d\n", waveform.ChargeCount.value);
					break;
			}
			else if(strcmp(i, "ChargeInterval") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.ChargeInterval.value = (uint32_t) atoi(token);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.ChargeInterval.min = (uint32_t) atoi(token);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.ChargeInterval.max = (uint32_t) atoi(token);

					printf("ChargeInterval: %d\n", waveform.ChargeInterval.value);
					break;
			}
			else if(strcmp(i, "BurstRndMax") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.BurstRndMax.value = strtod(token, &dummy);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.BurstRndMax.min = strtod(token, &dummy);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.BurstRndMax.max = strtod(token, &dummy);

					printf("BurstRndMax: %f\n", waveform.BurstRndMax.value);
					break;
			}
			else if(strcmp(i, "BurstRndMin") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.BurstRndMin.value = strtod(token, &dummy);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.BurstRndMin.min = strtod(token, &dummy);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.BurstRndMin.max = strtod(token, &dummy);


					printf("BurstRndMin: %f\n", waveform.BurstRndMin.value);
					break;
			}
			else if(strcmp(i, "CycleWidth") == 0){
					//Grab Item Value
					token = strsep(&line, ",");
					waveform.CycleWidth.value = strtod(token, &dummy);

					//Grab Item Min
					token = strsep(&line, ",");
					waveform.CycleWidth.min = strtod(token, &dummy);

					//Grab Item Max
					token = strsep(&line, ",");
					waveform.CycleWidth.max = strtod(token, &dummy);

					printf("CycleWidth: %f\n", waveform.CycleWidth.value);
					break;
			}

		}
	}
	fclose(fd);

}

void readFormFile(char file[])
{
	FILE *fd;
	char *mode = "r";
	size_t len=0;
	ssize_t read;
	char * line = NULL;
	char * token;
	char * token1;
	char * tokencp;
	char * dummy;
	char * linecp;

	//volatile variables for form data
	int thisForm;
	int thisItem = 0;
	int thisVar;
	int count1;
	int j;


	fd = fopen(file, mode);
	if (fd == NULL){
			printf("No Form File\nExiting program.");
			fflush(stdout);
	        exit(EXIT_FAILURE);
	}
	//Grab a line from the text file
	while ((read = getline(&line, &len, fd)) !=-1)
		{
			//Break line at comma
			//check if the length of the line is lower than 2 so new lines wont make strcmp crash
			if((len <= 2)){continue;}

			token = strsep(&line, ",");

			//Check if its the form information
			if(strcmp(token,"Form") == 0)
			{
				token = strsep(&line, ",");
				thisForm = atoi(token);

				continue;
			}

			//Grab the data for each of the items within the form.
			//The items have the form: {id, type, formnum, funcCall, {funcVars1, funcVars2,...}}

			//Grab the item id
			Forms.allFormData[thisForm].FormItems[thisItem].id = atoi(token);

			//Separate the Item type
			token = strsep(&line, ",");

			//Grab item type
			Forms.allFormData[thisForm].FormItems[thisItem].type = defToInt(token);

			//Separate the Item form number
			token = strsep(&line, ",");

			//Grab form num
			Forms.allFormData[thisForm].FormItems[thisItem].formNum = atoi(token);

			//Separate the function calls
			//token = strsep(&line, ",");
			token = strsep(&line, "}");

			tokencp = strsep(&token, "{");

			//Count number of commas left in the string
			int i, count = 0;
			for(i = 0, count = 0; token[i]; i++)
			{
				count += (token[i] == ',');
			}

			//Grab the function calls in string
			for(i = 0; i <= count; i++)
			{
				if((tokencp = strsep(&token, ",")) == NULL)
				{
					if(defToInt(token) != -1)
					{
						Forms.allFormData[thisForm].FormItems[thisItem].functionCall[i] = defToInt(token);
						continue;
					}
				}
				else
				{
					if(defToInt(tokencp) != -1)
					{
						Forms.allFormData[thisForm].FormItems[thisItem].functionCall[i] = defToInt(tokencp);
						continue;
					}
				}

			}


			//Forms.allFormData[thisForm].FormItems[thisItem].functionCall = defToInt(token);

			//Grab the function input data
			//remove the brackets from the data


			tokencp = strsep(&line, "{");

			//Count number of } left in the string
			for(j = 0, count1 = 0; line[j]; j++)
			{
				count1 += (line[j] == '}');
			}
			//for loop here

			for(j = 0; j < (count1 - 1); j++)
			{
				token = strsep(&line, "}");
				tokencp = strsep(&token, "{");

				tokencp = token;
				//Count number of commas left in the string
				for(i = 0, count = 0; tokencp[i]; i++)
				{
					count += (tokencp[i] == ',');
				}

				//Loop through each item
				for(i = 0; i <= count; i++)
				{
					linecp = token;

					if((tokencp = strsep(&token, ",")) == NULL)
					{
						if(defToInt(linecp) != -1)
						{
							Forms.allFormData[thisForm].FormItems[thisItem].funcVars[j][i] = (double) defToInt(linecp);
							continue;
						}
						else
						{
							Forms.allFormData[thisForm].FormItems[thisItem].funcVars[j][i] = strtod(linecp, &dummy);
							continue;
						}
					}

					else
					{

						if(defToInt(tokencp) != -1)
						{
							Forms.allFormData[thisForm].FormItems[thisItem].funcVars[j][i] = (double) defToInt(tokencp);
							continue;
						}
						else
						{
							Forms.allFormData[thisForm].FormItems[thisItem].funcVars[j][i] = strtod(tokencp, &dummy);
							continue;
						}
					}

				}
			}
			//Go to next item
			thisItem++;
			line = NULL;
		}

	 fclose(fd);

}

void readPinFile(char file[])
{
	FILE *fd;
	char *mode = "r";
	size_t len=0;
	ssize_t read;
	char * line = NULL;
	char * token;

	//volatile variables for form data
	int currSys = -1;
	int thisPin = 0;

	fd = fopen(file, mode);
	if (fd == NULL)
	{
		printf("No Pin File\nExiting program.");
		fflush(stdout);
		exit(EXIT_FAILURE);
	}

	//Grab a line from the text file
	while ((read = getline(&line, &len, fd)) !=-1)
	{
		token = strsep(&line, ",");
		if(strcmp(token, "name") == 0)
		{
			//I have a new pin system. Grab name and move to next iteration.
			currSys += 1;
			thisPin = 0;


			//get system name
			token = strsep(&line, ",");
			systems[currSys].name = token;

			//get generator mode
			token = strsep(&line, "\n");
			systems[currSys].genMode = atoi(token);
			continue;
		}

		//Grab pin number and save to mask and pin num.

		systems[currSys].pins[thisPin].pinMask.U64 = 1;
		systems[currSys].pins[thisPin].pinMask.U64 = systems[currSys].pins[thisPin].pinMask.U64 << atoi(token);
		systems[currSys].pins[thisPin].pinNum = atoi(token);

		allPins.U64 = allPins.U64 | systems[currSys].pins[thisPin].pinMask.U64;

		//Get the pin mode.
		token = strsep(&line, ",");
		systems[currSys].pins[thisPin].pinMode = atoi(token);

		//get pinDelay
		token = strsep(&line, ",");
		systems[currSys].pins[thisPin].pinDelay = atoi(token);

		//Get pin type
		char * stringcp = NULL;
		stringcp = line;

		if((token = strsep(&line, "\n")) == NULL)
		{
			systems[currSys].pins[thisPin].pinType = stringcp;
		}
		systems[currSys].pins[thisPin].pinType = token;

		thisPin++;
		len=0;
		line = NULL;
	}
	fclose(fd);
}
///////////////////////////////////////////
///////////Define parser Functions/////////
///////////////////////////////////////////
int defToInt(char * thing)
{

	//VARIABLES
	if(strcmp(thing, "PULSERNDMAX") == 0){return 1;}

	else if(strcmp(thing, "PULSERNDMIN") == 0){return 2;}

	else if(strcmp(thing, "PULSECOUNT") == 0){return 3;}

	else if(strcmp(thing, "PULSEWIDTH") == 0){return 4;}

	else if(strcmp(thing, "BURSTRNDMAX") == 0){return 5;}

	else if(strcmp(thing, "BURSTRNDMIN") == 0){return 6;}

	else if(strcmp(thing, "BURSTCOUNT") == 0){return 7;}

	else if(strcmp(thing, "CYCLEWIDTH") == 0){return 8;}

	else if(strcmp(thing, "INHIBITTIME") == 0){return 9;}

	else if(strcmp(thing, "INHIBITDELAY") == 0){return 10;}

	else if(strcmp(thing, "CHARGETIME") == 0){return 11;}

	else if(strcmp(thing, "CHARGEDELAY") == 0){return 12;}

	else if(strcmp(thing, "CHARGECOUNT") == 0){return 22;}

	else if(strcmp(thing, "CHARGEINTERVAL") == 0){return 23;}

	else if(strcmp(thing, "WFINFO") == 0){return 21;}


	//VISI SCREEN ITEMS
	else if(strcmp(thing, "IKNOB") == 0){return 1;}

	else if(strcmp(thing, "ISLIDER") == 0){return 4;}

	else if(strcmp(thing, "ITRACKBAR") == 0){return 5;}

	else if(strcmp(thing, "IWINBUTTON") == 0){return 6;}

	else if(strcmp(thing, "IFORM") == 0){return 10;}

	else if(strcmp(thing, "IKEYBOARD") == 0){return 13;}

	else if(strcmp(thing, "ILEDDIGITS") == 0){return 15;}

	else if(strcmp(thing, "ISTRING") == 0){return 17;}

	else if(strcmp(thing, "IUSRBUTTON") == 0){return 33;}

	else if(strcmp(thing, "I4DBUTTON") == 0){return 30;}

	else if(strcmp(thing, "IUSRIMAGES") == 0){return 27;}


	//FUNCTIONS
	else if(strcmp(thing, "INCVAR") == 0){return 1;}

	else if(strcmp(thing, "DECVAR") == 0){return 2;}

	else if(strcmp(thing, "CHANGEFORM") == 0){return 3;}

	else if(strcmp(thing, "BEGINWAVE") == 0){return 4;}

	else if(strcmp(thing, "CANCELWAVE") == 0){return 5;}

	else if(strcmp(thing, "SETWFSYSTEM") == 0){return 6;}

	else if(strcmp(thing, "CURRVAR") == 0){return 7;}

	else if(strcmp(thing, "CURRMOD") == 0){return 13;}

	else if(strcmp(thing, "CURRVARNAME") == 0){return 19;}

	else if(strcmp(thing, "RELEASEGPIO") == 0){return 8;}

	else if(strcmp(thing, "INCCURRVAR") == 0){return 9;}

	else if(strcmp(thing, "DECCURRVAR") == 0){return 10;}

	else if(strcmp(thing, "CHANGEMODVAL") == 0){return 11;}

	else if(strcmp(thing, "PREPWF") == 0){return 12;}

	else if(strcmp(thing, "LOADWFVARS") == 0){return 13;}

	else if(strcmp(thing, "SINGLEWF") == 0){return 14;}

	else if(strcmp(thing, "SETPOTVAL") == 0){return 15;}

	else if(strcmp(thing, "SETPOTZERO") == 0){return 16;}

	else if(strcmp(thing, "READPSVOLTAGE") == 0){return 17;}

	else if(strcmp(thing, "SETPSVOLTAGE") == 0){return 18;}

	else if(strcmp(thing, "SETPSTARVAL") == 0){return 19;}

	else if(strcmp(thing, "SETPSZERO") == 0){return 20;}

	else if(strcmp(thing, "INITWFSYSTEM") == 0){return 21;}

	else if(strcmp(thing, "SETCURRLOOP") == 0){return 22;}

	else if(strcmp(thing, "PSMOD") == 0){return 23;}

	else if(strcmp(thing, "DEAULTPINSYSTEM") == 0){return 24;}

	else if(strcmp(thing, "FORMCHANGED") == 0){return 25;}

	else if(strcmp(thing, "DELAY") == 0){return 26;}

	else if(strcmp(thing, "UPDATEITEM") == 0){return 27;}

	else if(strcmp(thing, "ZEROPSVOLTAGE") == 0){return 28;}

	//INTERNAL VARS
	else if(strcmp(thing, "INITPINS") == 0){return 1;}

	//POWERSUPPLY SYSTEM VARS
	else if(strcmp(thing, "PS1") == 0){return 1;}

	else if(strcmp(thing, "PS2") == 0){return 2;}

	else if(strcmp(thing, "PS1CURRVOLT") == 0){return 15;}

	else if(strcmp(thing, "PS1TARVOLT") == 0){return 16;}

	else if(strcmp(thing, "PS2CURRVOLT") == 0){return 17;}

	else if(strcmp(thing, "PS2TARVOLT") == 0){return 18;}

	else if(strcmp(thing, "CURRWFNAME") == 0){return 20;}


	//LOOP VARS
	else if(strcmp(thing, "MAINLOOP") == 0){return 1;}

	else if(strcmp(thing, "MSLOOP") == 0){return 2;}

	else if(strcmp(thing, "MODVALLOOP") == 0){return 3;}

	else {return -1;}
}
//////////////////////////////////////////
/////// String Functions /////////////////
//////////////////////////////////////////
char* strDisplayFormat(int currForm, int item)
{
	static char string1[50];
	static char string3[50];
	char string2[10];
	int doublebuf;
	int strBuffer;
	int i = 0;

	int strIndex = (int) Forms.allFormData[currForm].FormItems[item].funcVars[0][0];
	int varItem = (int) Forms.allFormData[currForm].FormItems[item].funcVars[0][strIndex];

	printf("varItem: %d", varItem);

	switch(varItem)
	{

		case PULSERNDMAX:
			//sprintf(string1, "Pulse Rnd Max.\n%d uS", waveform.PulseRndMax.value);
			sprintf(string1, "%d mS", waveform.PulseRndMax.value/1000);
			break;
		case PULSERNDMIN:
			//sprintf(string1, "Pulse Rnd Min.\n%d uS", waveform.PulseRndMin.value);
			sprintf(string1, "%d mS", waveform.PulseRndMin.value/1000);
			break;
		case PULSECOUNT:
			//sprintf(string1, "Pulse Count\n%d", waveform.PulseCount.value);
			sprintf(string1, "%d", waveform.PulseCount.value);
			break;
		case PULSEWIDTH:
			sprintf(string1, "Pulse Width\n%d uS", waveform.PulseWidth.value);
			break;
		case BURSTRNDMAX:
			sprintf(string1, "Burst Rnd. Max.\n%f mS", waveform.BurstRndMax.value);
			break;
		case BURSTRNDMIN:
			sprintf(string1, "Burst Rnd. Min.\n%f mS", waveform.BurstRndMin.value);
			break;
		case BURSTCOUNT:
			sprintf(string1, "Burst Count\n %d", waveform.BurstCount.value);
			break;
		case CYCLEWIDTH:
			sprintf(string1, "Cycle Time\n%f S", waveform.CycleWidth.value);
			break;
		case INHIBITTIME:
			sprintf(string1, "Inhibit Time\n%d uS", waveform.InhibitTime.value);
			break;
		case INHIBITDELAY:
			sprintf(string1, "Inhibit Delay\n%d uS", waveform.InhibitDelay.value);
			break;
		case CHARGETIME:
			sprintf(string1, "%d", waveform.ChargeTime.value);
			break;
		case CHARGEDELAY:
			sprintf(string1, "%d", waveform.ChargeDelay.value);
			break;
		case CHARGECOUNT:
			sprintf(string1, "%d", waveform.ChargeCount.value);
			break;
		case CHARGEINTERVAL:
			sprintf(string1, "%d", waveform.ChargeInterval.value);
			break;
		case CURRMOD:
			sprintf(string1, "%*d", 8, (int) currVar.mod);
			break;
		case CURRVARNAME:
			sprintf(string1, "%s", currVar.name);
			break;
		case PS1CURRVOLT:
			doublebuf = ((double) allPSData[0].currLevelVoltage/(4095.0)) * (int) allPSData[0].maxVoltage;
			sprintf(string1, "Curr Voltage\n%d V", doublebuf);
			break;
		case PS1TARVOLT:
			doublebuf = ((double) allPSData[0].tarLevelVoltage/(4095.0)) * (int) allPSData[0].maxVoltage;
			strBuffer = (int) log10(doublebuf);
			for(i = 0; i < (5-doublebuf);i++)
			{
				string2[i] = ' ';
			}
			string2[i] = '\0';
			sprintf(string3, "%d V", doublebuf);
			sprintf(string1, strcat(string3, string2));
			break;
		case PS2CURRVOLT:
			doublebuf = ((double) allPSData[1].currLevelVoltage/(4095.0)) * (int) allPSData[1].maxVoltage;
			sprintf(string1, "Curr Voltage\n%d V", doublebuf);
			break;
		case PS2TARVOLT:
			doublebuf = ((double) allPSData[1].tarLevelVoltage/(4095.0)) * (int) allPSData[1].maxVoltage;
			sprintf(string1, "%d V", doublebuf);
			break;
		case WFINFO:
			sprintf(string1, "Pulse Random Max -\n%dmS - %dmS\n\nPulse Count - %d", waveform.PulseRndMin.value/1000, waveform.PulseRndMax.value/1000, waveform.PulseCount);
			break;
		default:
			break;
	}
	return string1;
}


char* specificDisplayFormat0(int currForm, int item)
{
	static char string1[50];
	int doublebuf;
	int doublebuf1;

	int strIndex = (int) Forms.allFormData[currForm].FormItems[item].funcVars[0][0];
	int varItem = (int) Forms.allFormData[currForm].FormItems[item].funcVars[0][strIndex];

	switch(varItem)
	{

		case PULSERNDMAX:
			//sprintf(string1, "Pulse Rnd Max.\n%d uS", waveform.PulseRndMax.value);
			sprintf(string1, "%*d mS",11, waveform.PulseRndMax.value/1000);
			break;
		case PULSERNDMIN:
			//sprintf(string1, "Pulse Rnd Min.\n%d uS", waveform.PulseRndMin.value);
			sprintf(string1, "%*d mS",11, waveform.PulseRndMin.value/1000);
			break;
		case PULSECOUNT:
			//sprintf(string1, "Pulse Count\n%d", waveform.PulseCount.value);
			sprintf(string1, "%*d",12, waveform.PulseCount.value);
			break;
		case PULSEWIDTH:
			sprintf(string1, "Pulse Width\n%d uS", waveform.PulseWidth.value);
			break;
		case BURSTRNDMAX:
			sprintf(string1, "Burst Rnd. Max.\n%f mS", waveform.BurstRndMax.value);
			break;
		case BURSTRNDMIN:
			sprintf(string1, "Burst Rnd. Min.\n%f mS", waveform.BurstRndMin.value);
			break;
		case BURSTCOUNT:
			sprintf(string1, "Burst Count\n %d", waveform.BurstCount.value);
			break;
		case CYCLEWIDTH:
			sprintf(string1, "Cycle Time\n%f S", waveform.CycleWidth.value);
			break;
		case INHIBITTIME:
			sprintf(string1, "Inhibit Time\n%d uS", waveform.InhibitTime.value);
			break;
		case INHIBITDELAY:
			sprintf(string1, "Inhibit Delay\n%d uS", waveform.InhibitDelay.value);
			break;
		case CHARGETIME:
			sprintf(string1, "%d", waveform.ChargeTime.value);
			break;
		case CHARGEDELAY:
			sprintf(string1, "%d", waveform.ChargeDelay.value);
			break;
		case CHARGECOUNT:
			sprintf(string1, "%d", waveform.ChargeCount.value);
			break;
		case CHARGEINTERVAL:
			sprintf(string1, "%d", waveform.ChargeInterval.value);
			break;
		case CURRMOD:
			sprintf(string1, "%*d", 8, (int) currVar.mod);
			break;
		case CURRVARNAME:
			sprintf(string1, "%s", currVar.name);
			break;
		case PS1CURRVOLT:
			doublebuf = ((double) allPSData[0].currLevelVoltage/(4095.0)) * (int) allPSData[0].maxVoltage;
			sprintf(string1, "Curr Voltage\n%d V", doublebuf);
			break;
		case PS1TARVOLT:
			doublebuf = ((double) allPSData[0].tarLevelVoltage/(4095.0)) * (int) allPSData[0].maxVoltage;
			sprintf(string1, "%d V", doublebuf);
			break;
		case PS2CURRVOLT:
			doublebuf = ((double) allPSData[1].currLevelVoltage/(4095.0)) * (int) allPSData[1].maxVoltage;
			sprintf(string1, "Curr Voltage\n%d V", doublebuf);
			break;
		case PS2TARVOLT:
			doublebuf = ((double) allPSData[1].tarLevelVoltage/(4095.0)) * (int) allPSData[1].maxVoltage;
			sprintf(string1, "%f V", doublebuf);
			break;
		default:
			break;
	}
	return string1;
}

//////////////////////////////////////////
///////// Waves library functions/////////
//////////////////////////////////////////
int setWavesVars()
{
	setPulseRndMax(waveform.PulseRndMax.value);
	setPulseRndMin(waveform.PulseRndMin.value);
	setPulseCount(waveform.PulseCount.value);
	setPulseWidth(waveform.PulseWidth.value);
	setBurstRndMax(waveform.BurstRndMax.value);
	setBurstRndMin(waveform.BurstRndMin.value);
	setBurstCount(waveform.BurstCount.value);
	setCycleWidth(waveform.CycleWidth.value);
	setInhibitTime(waveform.InhibitTime.value);
	setInhibitDelay(waveform.InhibitDelay.value);
	setChargeTime(waveform.ChargeTime.value);
	setChargeDelay(waveform.ChargeDelay.value);
	setChargeCount(waveform.ChargeCount.value);
	setChargeInterval(waveform.ChargeInterval.value);

	return 1;
}


////////////////////////////////////////////
//////////// DEBUG Functions ///////////////
////////////////////////////////////////////
void printFormData(void)
{
	int form = -1;
	int thing = -1;
	for(form=0;form<1;form++)
	{
		for(thing=0;thing<50;thing++)
		{
			printf("ID: %d\n", Forms.allFormData[form].FormItems[thing].id);
			printf("FunctionCall: %d\n", Forms.allFormData[form].FormItems[thing].functionCall);
		}
	}
}


/////////////////////////////////////////////
/////////// SPI Functions ///////////////////
/////////////////////////////////////////////


/////////////////////////////////////////////
/////////// UART Functions //////////////////
/////////////////////////////////////////////

void testPSControl(void)
{
	int uart0_filestream = -1;

	uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}


	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);



	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;

	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = 0x02;
	*p_tx_buffer++ = '9';
	*p_tx_buffer++ = '9';
	*p_tx_buffer++ = ',';
	*p_tx_buffer++ = '1';
	*p_tx_buffer++ = ',';
	*p_tx_buffer++ = 'E';
	*p_tx_buffer++ = 0x03;

	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
	}



	if (uart0_filestream != -1)
	{
		// Read up to 255 characters from the port if they are there
		unsigned char rx_buffer[256];
		int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
		if (rx_length < 0)
		{
			//An error occured (will occur if there are no bytes)
		}
		else if (rx_length == 0)
		{
			//No data waiting
		}
		else
		{
			//Bytes received
			rx_buffer[rx_length] = '\0';
			printf("%i bytes read : %s\n", rx_length, rx_buffer);
		}
	}

	printf("CloseUart");
	fflush(stdout);

	close(uart0_filestream);
}

void setPSLevels(void)
{
	//PS1, 6kV, 5mA  First Strike supply
	allPSData[0].maxVoltage = 6000;
	allPSData[0].maxCurr = 5;
	allPSData[0].minVoltage = 0;
	allPSData[0].minCurr = 0;
	allPSData[0].maxVoltageLim = 4900;
	allPSData[0].minVoltageLim = 0;
	allPSData[0].maxCurrLim = 5;
	allPSData[0].minCurrLim = 0;
	allPSData[0].setLevelCurr = 0;
	allPSData[0].maxVoltageVal = 4095 * (((double) allPSData[0].maxVoltageLim) / ((double) allPSData[0].maxVoltage));
	allPSData[0].minVoltageVal = 0;
	allPSData[0].maxCurrVal =  4095 * (((double) allPSData[0].maxCurrLim) / ((double) allPSData[0].maxCurr));
	allPSData[0].minCurrVal = 0;

	//PS2, 2kV, 12A  Restrike Supply
	allPSData[1].maxVoltage = 3000;
	allPSData[1].maxCurr = 8;
	allPSData[1].minVoltage = 0;
	allPSData[1].minCurr = 0;
	allPSData[1].maxVoltageLim = 3000;
	allPSData[1].minVoltageLim = 0;
	allPSData[1].maxCurrLim = 8;
	allPSData[1].minCurrLim = 0;
	allPSData[1].setLevelCurr = 0;
	allPSData[1].maxVoltageVal = 4095 * (((double) allPSData[1].maxVoltageLim) / ((double) allPSData[1].maxVoltage));
	allPSData[1].minVoltageVal = 0;
	allPSData[1].maxCurrVal =  4095 * (((double) allPSData[1].maxCurrLim) / ((double) allPSData[1].maxCurr));
	allPSData[1].minCurrVal = 0;
}

////////////////////////////
/////////Test Functions/////
////////////////////////////
void testUART(void)
{
	//UART 0
	printf("UART0\n");
	genieWriteObj(IFORM, (int) 1, 0);

	//UART 1
	printf("init UART1 115200 baud\n");
	fflush(stdout);
	uint32_t baud = 9600;
	//printf("ReturnInit: %d\n", uartInit(9600));
	fflush(stdout);

	char txData[] = {0x01, 0x0A, 0x01, 0x00, 0x01, 0x0B};
	int len;
	len = ARRAY_SIZE(&txData);

	printf("send UART1\n");
	fflush(stdout);
}




void testDNBSPI(void)
{
	unsigned int len = 1;
	int ret;
	unsigned speed = 64;
	int port = 2;

	unsigned char txData0[] = {0x39, 0x38};
	unsigned char data[] = {0x31};



	unsigned char *txbuf = malloc(sizeof(char) * len);
	unsigned char *rxbuf = malloc(sizeof(char) * len);

	uint8_t txData1[] = {0x39, 0x39, 0x00};

	len = ARRAY_SIZE(txData1);
	unsigned char *txbuf1 = malloc(sizeof(char) * len);
	memcpy(txbuf1, txData1, sizeof(char) * len);

	printf("dataCommand txData: %s\n",txbuf1);
	fflush(stdout);

	unsigned char *rxbuf1 = malloc(sizeof(char) * len);

	sendCommand(port, 98, txbuf1);

	sleep(1);

	//printf("%s", rcvSPI(SPIRCVLEN));

}

////////////////////////////////////////////////////////
//////////////// SPECIAL FUNCTIONS /////////////////////
////////////////////////////////////////////////////////
void lockRunScreenButtons(void)
{
	genieWriteObj(34,0,0);
}

void unlockRunScreenButtons(void)
{
	genieWriteObj(34,1,0);
}
