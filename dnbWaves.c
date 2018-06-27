
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
//for file access
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include "dnbWaves.h"
#include "dnbGPIO.h"

//================ pin masks for outputs===============
uint64_t PmWave; //waveform transmit line(s)
uint64_t PmInhibit; //power supply inhibit control line(s)
uint64_t PmCharge; //charge relay control line(s)
uint64_t PmWaveInhibit; //Total inhibit power supply control line(s)

//================ pulse gen variables ================
uint32_t PulseRndMax;
uint32_t PulseRndMin;
uint32_t PulseCount;
uint32_t PulseWidth;
double BurstRndMax;
double BurstRndMin;
uint32_t BurstCount;
double CycleWidth;
uint32_t InhibitTime;
uint32_t InhibitDelay;
uint32_t ChargeTime;
uint32_t ChargeDelay;
uint32_t ChargeCount;
uint32_t ChargeInterval;

//================== wave generation data ===============
uint32_t downs[1000]; 
int lastDown;
int WFID;

uint8_t loging; //wheather to log to file or not
int GenMode; //Stored Generator Mode
char inited; //whether dnbGPIO is initilized
FILE *fp;

//================= internal function prototype =========
void waveCreate();
int PrintWaveFormOnly();
int PrintWaveFormInhibit();
int PrintWaveFormCharge();
int PrintWaveFormDowns(int genMode);

//=================================================================
//                       external sets for globals
//=================================================================
uint32_t setPulseRndMax(uint32_t Value)
{
	if(Value != UINT32_NULL)
		PulseRndMax = Value;
	return PulseRndMax;
}

uint32_t setPulseRndMin(uint32_t Value)
{
	if (Value != UINT32_NULL)
		PulseRndMin = Value;
	return PulseRndMin;
}

uint32_t setPulseCount(uint32_t Value)
{
	if (Value != UINT32_NULL)
		PulseCount = Value;
	return PulseCount;
}

uint32_t setPulseWidth(uint32_t Value)
{
	if (Value != UINT32_NULL)
		PulseWidth = Value;
	return PulseWidth;
}

double setBurstRndMax(double Value)
{
	if (Value >= 0)
		BurstRndMax = Value;
	return BurstRndMax;
}

double setBurstRndMin(double Value)
{
	if (Value >= 0)
		BurstRndMin = Value;
	return BurstRndMin;
}

uint32_t setBurstCount(uint32_t Value)
{
	if (Value != UINT32_NULL)
		BurstCount = Value;
	return BurstCount;
}

double setCycleWidth(double Value)
{
	if (Value >= 0)
		CycleWidth = Value;
	return CycleWidth;
}

uint32_t setInhibitTime(uint32_t Value)
{
	if (Value != UINT32_NULL)
		InhibitTime = Value;
	return InhibitTime;
}

uint32_t setInhibitDelay(uint32_t Value)
{
	if (Value != UINT32_NULL)
		InhibitDelay = Value;
	return InhibitDelay;
}

uint32_t setChargeTime(uint32_t Value)
{
	if (Value != UINT32_NULL)
		ChargeTime = Value;
	return ChargeTime;
}

uint32_t setChargeDelay(uint32_t Value)
{
	if (Value != UINT32_NULL)
		ChargeDelay = Value;
	return ChargeDelay;
}

uint32_t setChargeCount(uint32_t Value)
{
	if (Value != UINT32_NULL)
		ChargeCount = Value;
	return ChargeCount;
}

uint32_t setChargeInterval(uint32_t Value)
{
	if (Value != UINT32_NULL)
		ChargeInterval = Value;
	return ChargeInterval;
}

int setGenMode(int Value)
{
	if (Value != GEN_MODE_NULL)
		GenMode = Value;
	return GenMode;
}

void setWavePinMask(uint64_t mask)
{
	int i;
	uint64_t hld = 1;
	PmWave = mask;
	if (inited > 0)
	{
		for (i = 0; i < PI_MAX_GPIO; i++)
		{
			if (mask & hld)
				gpioMode(i, PI_OUTPUT);
			hld = hld << 1;
		}
	}
	else
		inited = -1;
}

void setInhibitMask(uint64_t mask)
{
	int i;
	uint64_t hld = 1;
	PmInhibit = mask;
	if (inited > 0)
	{
		for (i = 0; i < PI_MAX_GPIO; i++)
		{
			if (mask & hld)
				gpioMode(i, PI_OUTPUT);
			hld = hld << 1;
		}
	}
	else
		inited = -1;
}

void setWaveInhibitMask(uint64_t mask)
{
	int i;
	uint64_t hld = 1;
	PmWaveInhibit = mask;
	if (inited > 0)
	{
		for (i = 0; i < PI_MAX_GPIO; i++)
		{
			if (mask & hld)
				gpioMode(i, PI_OUTPUT);
			hld = hld << 1;
		}
	}
	else
		inited = -1;
}

void setChargeMask(uint64_t mask)
{
	int i;
	uint64_t hld = 1;
	PmCharge = mask;
	if (inited > 0)
	{
		for (i = 0; i < PI_MAX_GPIO; i++)
		{
			if (mask & hld)
				gpioMode(i, PI_OUTPUT);
			hld = hld << 1;
		}
	}
	else
		inited = -1;

}

int setWFLogMode(int Value)
{
	if (Value >= 0)
		loging = Value > 0;
	return loging;
}

//=================================================================
//         waveform generation    
//=================================================================

int GenerateWaveForm(int genMode)
{
	if (genMode <= GEN_MODE_NULL)
		genMode = GenMode;

	if (genMode <= GEN_MODE_DISABLED)
		return -1;
	else if (genMode == GEN_MODE_WAVE_ONLY)
		return GenerateWaveFormOnly();
	else if (genMode == GEN_MODE_INHIBIT)
		return GenerateWaveFormInhibit();
	else if (genMode == GEN_MODE_CHARGE)
		return GenerateWaveFormCharge();
	else if (genMode == GEN_MODE_DUALINHIBIT)
		return GenerateWaveFormDualInhibit();
	else if (genMode == GEN_MODE_MULTI_CHARGE)
			return GenerateWaveFormMultiCharge();
	else
		return -1;
}

int GenerateWaveFormOnly()
{
	int i;
	printf("::inited = %d\n", inited);
	if (inited <= 0)
		return -2;

	if (WFID >= 0)
		WaveDelete(WFID);

	WFID = -1;
	waveCreate();

	for (i = 0; i <= lastDown ; i++) //< (BurstCount * PulseCount)
	{
		WFID = WaveAddEventRaw(PmWave,0,PulseWidth, WFID);
		if (WFID < 0)
			break;
		WFID = WaveAddEventRaw(0,PmWave,downs[i], WFID);
		if (WFID < 0)
			break;
	}
	GenMode = GEN_MODE_WAVE_ONLY;
	return WFID;// WavePrepRun(wid, 1);
}

void debugPrintWF()
{
	fp = stdout;
}

int PrintWaveFormOnly()
{
	int i;
	if (fp == NULL)
		return -1;

	for (i = 0; i <= lastDown; i++)//< (BurstCount * PulseCount)
	{
		fprintf(fp,"[%d]%d_", PulseWidth, downs[i]);
	}
	fprintf(fp,"\n");
	fflush(fp);
	return -1;
}

int GenerateWaveFormInhibit()
{
	int i;
	uint32_t post = InhibitTime - PulseWidth - InhibitDelay;

	if (WFID >= 0)
		WaveDelete(WFID);

	WFID = -1;

	waveCreate();

	if (inited <= 0)
		return -1;

	for (i = 0; i <= lastDown; i++)//< (BurstCount * PulseCount)
	{
		WFID = WaveAddEventRaw(PmInhibit, 0, InhibitDelay, WFID);
		if (WFID < 0)
			break;
		WFID = WaveAddEventRaw(PmWave, 0, PulseWidth, WFID);
		if (WFID < 0)
			break;
		WFID = WaveAddEventRaw(0,PmWave, post, WFID);
		if (WFID < 0)
			break;
		WFID = WaveAddEventRaw(0, PmInhibit, downs[i] - post - InhibitDelay, WFID);
		if (WFID < 0)
			break;
	}
	GenMode = GEN_MODE_INHIBIT;
	return WFID;// WavePrepRun(wid, 1);
}

int PrintWaveFormInhibit()
{
	int i;
	uint32_t post = InhibitTime - PulseWidth - InhibitDelay;

	if (fp == NULL)
		return -1;

	for (i = 0; i <= lastDown; i++)//< (BurstCount * PulseCount)
	{
		fprintf(fp, "{%d", InhibitDelay);
		fprintf(fp, "[%d", PulseWidth);
		fprintf(fp, "]%d_", post);
		fprintf(fp, "}%d", downs[i] - post - InhibitDelay);
	}
	fprintf(fp, "\n");
	fflush(fp);
	return -1;
}

int GenerateWaveFormDualInhibit()
{
	int i;
	uint64_t PmDualInhibit = PmWaveInhibit | PmInhibit;
	uint32_t post = InhibitTime - PulseWidth - InhibitDelay;

	if (WFID >= 0)
		WaveDelete(WFID);

	WFID = -1;

	waveCreate();

	if (inited <= 0)
		return -1;

	for (i = 0; i <= lastDown; i++)//< (BurstCount * PulseCount)
	{
		if(i == 0)
		{
			WFID = WaveAddEventRaw(PmDualInhibit, 0, InhibitDelay, WFID);
		}
		else
		{
			WFID = WaveAddEventRaw(PmInhibit, 0, InhibitDelay, WFID);
		}
		if (WFID < 0)
			break;
		WFID = WaveAddEventRaw(PmWave, 0, PulseWidth, WFID);
		if (WFID < 0)
			break;
		WFID = WaveAddEventRaw(0,PmWave, post, WFID);
		if (WFID < 0)
			break;
		if(i == lastDown)
		{
			WFID = WaveAddEventRaw(0, PmDualInhibit, downs[i] - post - InhibitDelay, WFID);
		}
		else
		{
			WFID = WaveAddEventRaw(0, PmInhibit, downs[i] - post - InhibitDelay, WFID);
		}

		if (WFID < 0)
			break;
	}
	GenMode = GEN_MODE_INHIBIT;
	return WFID;// WavePrepRun(wid, 1);
}

int PrintWaveFormDowns(int genMode)
{
	if (fp == NULL)
		return -1;

	if (genMode < 0)
		genMode = GenMode;

	switch (genMode)
	{
	case GEN_MODE_CHARGE:
		fprintf(fp, "(%d)%d_[%d]%dP%dB", ChargeTime, ChargeDelay, PulseWidth,PulseCount,BurstCount);
		break;
	case GEN_MODE_INHIBIT:
		fprintf(fp, "{%d}%d_[%d]%dP%dB", InhibitTime, InhibitDelay, PulseWidth,PulseCount,BurstCount);
		break;
	case GEN_MODE_WAVE_ONLY:
		fprintf(fp, "[%d]%dP%dB", PulseWidth,PulseCount,BurstCount);
		break;
	default:
		return -1;
	}
	int i;
	for (i = 0; i <= lastDown; i++)
		fprintf(fp, "%d,", downs[i]);
	fprintf(fp, "\n");
	fflush(fp);
	return 1;
}

int GenerateWaveFormCharge()
{
	int i;
	uint32_t pre = ChargeTime + ChargeDelay;
	
	if (WFID >= 0)
		WaveDelete(WFID);

	WFID = -1;

	waveCreate();

	if (inited <= 0)
		return -1;

	for (i = 0; i <= lastDown; i++)//< (BurstCount * PulseCount)
	{
		WFID = WaveAddEventRaw(PmCharge, 0, ChargeTime, WFID);
		if (WFID < 0)
			break;
		WFID = WaveAddEventRaw(0, PmCharge, ChargeDelay, WFID);
		if (WFID < 0)
			break;
		WFID = WaveAddEventRaw(PmWave, 0, PulseWidth, WFID);
		if (WFID < 0)
			break;
		WFID = WaveAddEventRaw(0, PmWave, downs[i] - pre, WFID);
		if (WFID < 0)
			break;
		
	}
	GenMode = GEN_MODE_CHARGE;
	return WFID;// WavePrepRun(wid, 1);
}

int GenerateWaveFormMultiCharge()
{
	int i;
	int j;
	uint32_t pre = (ChargeTime + ChargeInterval) * ChargeCount + ChargeDelay;

	if (WFID >= 0)
		WaveDelete(WFID);

	WFID = -1;

	waveCreate();

	if (inited <= 0)
		return -1;

	for (i = 0; i <= lastDown; i++)//< (BurstCount * PulseCount)
	{
		for(j = 0; j < ChargeCount; j++)
		{
			WFID = WaveAddEventRaw(PmCharge, 0, ChargeTime, WFID);
			if (WFID < 0)
				break;
			WFID = WaveAddEventRaw(0, PmCharge, ChargeInterval, WFID);
			if (WFID < 0)
				break;
		}
		WFID = WaveAddEventRaw(0, PmCharge, ChargeDelay, WFID);
		if (WFID < 0)
			break;
		WFID = WaveAddEventRaw(PmWave, 0, PulseWidth, WFID);
		if (WFID < 0)
			break;
		WFID = WaveAddEventRaw(0, PmWave, downs[i] - pre, WFID);
		if (WFID < 0)
			break;

	}
	GenMode = GEN_MODE_CHARGE;
	return WFID;// WavePrepRun(wid, 1);
}

int PrintWaveFormCharge()
{
	int i;
	uint32_t pre = ChargeTime + ChargeDelay;

	if (fp == NULL)
		return -1;

	for (i = 0; i <= lastDown; i++)//< (BurstCount * PulseCount)
	{
		fprintf(fp, "(%d", ChargeTime);
		fprintf(fp, ")%d", ChargeDelay);
		fprintf(fp, "[%d", PulseWidth);
		fprintf(fp, "]%d_", downs[i] - pre);
	}
	fprintf(fp, "\n");
	fflush(fp);
	return -1;
}

void waveCreate()
{
	int bc, pc;
	uint32_t bslush;
	uint32_t holder;
	int rtrim, rnum;

	//zero out holder array
	for (bc = 0; bc < 1000; bc++)
		downs[bc] = 0;
	lastDown = -1;

	//set down[] for last pulse of each burst to total burst width
	if(BurstCount == 1) //1 burst = use cycle width
		downs[PulseCount - 1] = (uint32_t)(CycleWidth * 1000000);
	else
	{
		
		bslush = (uint32_t)((CycleWidth - (BurstCount * BurstRndMin)) * 1000000);
		rtrim = (int)((BurstRndMax - BurstRndMin) * 1000000) + 1;
		for (bc = 1; bc <= BurstCount; bc++)
		{
			if (rtrim > bslush)
				rtrim = bslush+1;
			rnum = rand() % rtrim;
			//printf(":burst %d + %d\n", rnum, (uint32_t)(BurstRndMin * 1000000));
			downs[bc*PulseCount - 1] = (uint32_t)((BurstRndMin * 1000000) + rnum);
			bslush -= rnum;
		}
	}

	//randomize bursts order
	for (bc = 1; bc <= BurstCount; bc++)
	{
		rnum = (rand() % BurstCount) + 1;
		if (rnum != bc)
		{
			holder = downs[bc*PulseCount - 1];
			downs[bc*PulseCount - 1] = downs[rnum * PulseCount - 1];
			downs[rnum * PulseCount - 1] = holder;
		}
	}

	//set pulse width per burst
	for (bc = 0; bc < BurstCount; bc++)
	{
		bslush = downs[(bc+1)*PulseCount - 1];
		bslush -= (PulseCount * (PulseWidth + PulseRndMin));
		rtrim = PulseRndMax - PulseRndMin + 1;
		//sudo randomize pulse delay widths
		for (pc = 0; pc < PulseCount; pc++)
		{
			if (rtrim > bslush)
				rtrim = bslush + 1;
			rnum = rand() % rtrim;
			downs[(bc*PulseCount) + pc] = PulseRndMin + rnum;
			bslush -= rnum;
			lastDown++;
		}
		//randomize pulse order
		for (pc = 0; pc < PulseCount; pc++)
		{
			rnum = rand() % PulseCount;
			if (rnum != pc)
			{
				holder = downs[(bc * PulseCount) + pc];
				downs[(bc * PulseCount) + pc] = downs[(bc * PulseCount) + rnum];
				downs[(bc * PulseCount) + rnum] = holder;
			}
		}
		//add remaining slush to last pulse delay
		downs[(bc + 1)*PulseCount - 1] += bslush;
	}
	
}

//=================================================================
//             dnbGPIO init / terminate
//=================================================================
int WaveInit()
{
	//any error codes returned are defined in dngGPIO.h
	int ret = gpioInitialise();
	if (ret <= 0)
		return ret;
	if (inited < 0)
	{
		int i;
		uint64_t hld = 1;
		for (i = 0; i < PI_MAX_GPIO; i++)
		{
			if (PmWave & hld || PmInhibit & hld || PmCharge & hld)
				gpioMode(i, PI_OUTPUT);
			hld = hld << 1;
		}
	}
	inited = 1;
	return ret;
}

void WaveTerminate()
{
	if (fp != NULL)
		fclose(fp);
	gpioTerminate();
}

//=================================================================
//             wafeform prep and run
//=================================================================
int ReadyWaveForm(int repeat)
{
	if (inited <= 0)
		return -1;
	if (WaveRunning())
		return -1;
	int ret = WavePrepRun(WFID, repeat);
	if (ret < 0)
		return ret;
	if (loging)
	{
		if (fp == NULL)
		{
			char wd[500];
			char wd2[500];
			time_t rt;
			if (getcwd(wd, 500) == NULL)
				return ret;
			//printf(":wdir %s\n", wd);
			snprintf(wd2, 500, "%s%s",wd, "/LOGS");
			//printf(":ndir %s\n", wd2);
			mkdir(wd2, S_IRWXU | S_IRWXG | S_IRWXO);
			time(&rt);
			snprintf(wd, 500, "%s/%s", wd2, asctime(gmtime(&rt)));
			int i;
			for (i = 0; i < 500 && wd[i] != '\0'; i++)
			{
				if (wd[i] == ' ')//32
					wd[i] = '_'; //95;
				else if (wd[i] == '\n')
					wd[i] = '\0';
			}
			snprintf(wd2, 500, "%s.txt", wd);
			//printf(":file %s\n", wd2);
			fp = fopen(wd2, "w+");
		}

		PrintWaveFormDowns(GenMode);
		
		//if (GenMode == GEN_MODE_WAVE_ONLY)
		//	PrintWaveFormOnly();
		//else if (GenMode == GEN_MODE_INHIBIT)
		//	PrintWaveFormInhibit();
		//else if (GenMode == GEN_MODE_CHARGE)
		//	PrintWaveFormCharge();
	}
	return ret;
}

void StartWaveform()
{
	if (inited <= 0)
		return;
	WaveStartRun();
	if (loging && fp != NULL)
	{
		time_t rt;
		time(&rt);
		fprintf(fp, "START: %s", asctime(gmtime(&rt)));
		fflush(fp);
	}
		
}

void StopWaveform()
{
	if (inited <= 0)
		return;
	WaveStopRun();
	dnbGPIOSetLow(PmWave);
	dnbGPIOSetLow(PmCharge);
	dnbGPIOSetLow(PmInhibit);
}

//=================================================================
//             dnbGPIO pin access wappers
//=================================================================
unsigned dnbGPIOMode(unsigned gpio, int mode)
{
	return gpioMode(gpio, mode);
}
uint64_t dnbGPIOLevels()
{
	return gpioLevels();
}
void dnbGPIOSetHigh(uint64_t pinMap)
{
	return gpioSetHigh(pinMap);
}
void dnbGPIOSetLow(uint64_t pinMap)
{
	return gpioSetLow(pinMap);
}



