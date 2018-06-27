#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "dnbVerify.h"
#include "dnbGPIO.h"
#include "dnbWaves.h"

#include <unistd.h> //for usleep


int main(void)
{
	char inpt[100];
	char rept[100];
	char rsafe;
	char *res;
	char *res2;
	int pin;
	int mode;
	int i;
	char tosend[25];
	double dbl;
	

	uint64_u setm, clrm;
	uint64_u rwav, rinh, rchg;
	int gm = 0;
	uint32_t dla;

	int curWf = -1;
	int init = -1;
	rwav.U64 = 0;
	rinh.U64 = 0;
	rchg.U64 = 0;


	while (1)
	{
		printf("CMD: ");
		fgets(inpt, 100, stdin);
		rsafe = 1;
		if (inpt[0] == 0x0A)
		{
			for (i = 0; i < 100; i++)
				inpt[i] = rept[i];
			rsafe = 0;
		}

		if (res = strfnd(inpt, "read\0"))
		{
			res += 4;
			setm.U64 = strtoull(res, &res2, 16);
			printf("Val: 0x%.8x%.8x\n", setm.U32[1], setm.U32[0]);
		}
		else if (res = strfnd(inpt,"burst\0"))
		{
			setPulseRndMax(1000);
			setPulseRndMin(50);
			setPulseCount(20);
			setPulseWidth(20);
			setBurstRndMax(0.300);
			setBurstRndMin(0.030);
			setBurstCount(3);
			setCycleWidth(1.50);
			setInhibitTime(24);
			setInhibitDelay(2);
			setChargeTime(2);
			setChargeDelay(2);
			curWf = GenerateWaveForm(gm);
			printf(":ReadyWaveForm: mode %d %d %d\n",gm ,curWf,ReadyWaveForm(1));
		}
		else if (res = strfnd(inpt, "stroke\0"))
		{
			setPulseRndMax(200000);
			setPulseRndMin(10000);
			setPulseCount(14);
			setPulseWidth(8000);
			setBurstRndMax(0.0);
			setBurstRndMin(0.0);
			setBurstCount(1);
			setCycleWidth(1.50);
			setInhibitTime(9000);
			setInhibitDelay(500);
			setChargeTime(500);
			setChargeDelay(500);
			curWf = GenerateWaveForm(gm);
			printf(":ReadyWaveForm: mode %d %d %d\n",gm , curWf, ReadyWaveForm(1));
		}
		else if (res = strfnd(inpt, "pinw\0"))
		{
			res += 4;
			rwav.U64 = strtoll(res, NULL, 16);

			setWavePinMask(rwav.U64);
			if (rchg.U64 == 0 && rinh.U64 != 0)
				gm = GEN_MODE_INHIBIT;
			else if (rchg.U64 != 0 && rinh.U64 == 0)
				gm = GEN_MODE_CHARGE;
			else if (rwav.U64 != 0)
				gm = GEN_MODE_WAVE_ONLY;
			else
				gm = GEN_MODE_DISABLED;


			printf(":Wave 0x%.8X%.8X Charge 0x%.8X%.8X Inhibit 0x%.8X%.8X\n", rwav.U32[1], rwav.U32[0], rchg.U32[1], rchg.U32[0], rinh.U32[1], rinh.U32[0]);
		}
		else if (res = strfnd(inpt, "pinc\0"))
		{
			res += 4;
			rchg.U64 = strtoll(res, NULL, 16);

			if (rchg.U64)
			{
				setChargeMask(rchg.U64);
				rinh.U64 = 0;
				gm = GEN_MODE_CHARGE;
			}
			else if (rwav.U64)
				gm = GEN_MODE_WAVE_ONLY;
			else
				gm = GEN_MODE_DISABLED;
			printf(":Wave 0x%.8X%.8X Charge 0x%.8X%.8X Inhibit 0x%.8X%.8X\n", rwav.U32[1], rwav.U32[0], rchg.U32[1], rchg.U32[0], rinh.U32[1], rinh.U32[0]);
		}
		else if (res = strfnd(inpt, "pini\0"))
		{
			res += 4;
			rinh.U64 = strtoll(res, NULL, 16);

			if (rinh.U64)
			{
				setInhibitMask(rinh.U64);
				rchg.U64 = 0;
				gm = GEN_MODE_INHIBIT;
			}
			else if (rwav.U64)
				gm = GEN_MODE_WAVE_ONLY;
			else
				gm = GEN_MODE_DISABLED;
			printf(":Wave 0x%.8X%.8X Charge 0x%.8X%.8X Inhibit 0x%.8X%.8X\n", rwav.U32[1], rwav.U32[0], rchg.U32[1], rchg.U32[0], rinh.U32[1], rinh.U32[0]);
		}
		else if (res = strfnd(inpt, "log\0"))
		{
			res += 3;
			pin = strtol(res, NULL, 10);
			printf(":loging %d\n", setWFLogMode(pin));
		}
		else if (res = strfnd(inpt, "add\0"))
		{
			rsafe = 0;
			printf("Pinmask Set: 0x");
			fgets(inpt, 100, stdin);
			setm.U64 = strtoull(inpt, NULL, 16);
			printf("Pinmask Clr: 0x");
			fgets(inpt, 100, stdin);
			clrm.U64 = strtoull(inpt, NULL, 16);
			printf("Delay uS: ");
			fgets(inpt, 100, stdin);
			dla = strtoul(inpt, NULL, 10);

			curWf = WaveAddEventRaw(setm.U64, clrm.U64, dla, curWf);
			printf("WF id: %d\n", curWf);
			if (curWf >= 0)
			{
				printf("Set: 0x%.8x%.8x\n", setm.U32[1], setm.U32[0]);
				printf("Clr: 0x%.8x%.8x\n", clrm.U32[1], clrm.U32[0]);
				printf("Delay: %duS\n", dla);
			}
		}
		else if (res = strfnd(inpt, "spiinit\0"))
		{
			printf("SPI init: ");
			spiInit(0, 0);
			printf("OK\n");
		}
		else if (res = strfnd(inpt, "init\0"))
		{
			//init = gpioInitialise();
			init = WaveInit();
			printf("::gpio init: %d\n",init);
		}
		else if (res = strfnd(inpt, "dma time\0"))
		{
			res += 8;
			pin = strtol(res, &res2, 10);
			mode = strtol(res2, NULL, 10);
			if (pin <= 0)
				pin = 1;
			if (mode <= 0)
				mode = 1;
			printf("dma: %d, %d\n",pin, debugDMA(pin,mode));
		}
		else if (res = strfnd(inpt, "dma clock\0"))
		{
			printf("clock: %d\n", debugClock());
		}
		else if (res = strfnd(inpt, "mode\0"))
		{
			res += 4;
			pin = strtol(res, &res2, 10);
			if (pin >= 0)
			{
				mode = strtol(res2, NULL, 10);
				if (mode < 0)
				{
					printf(":Pin %d is in mode %d\n", pin, gpioMode(pin, -1));
				}
				else if (mode >= 0)
				{
					printf(":Pin %d set to mode %d\n", pin, gpioMode(pin, mode));
				}
			}
			else
			{
				for (mode = 0; mode < PI_MAX_GPIO; mode++)
					printf("%2d ", mode);
				printf("\n");
				for (mode = 0; mode < PI_MAX_GPIO; mode++)
					printf("%2d ", gpioMode(mode, -1));
				printf("\n");
				for (mode = 0; mode < PI_MAX_GPIO; mode++)
					printf("%2d ", dnbGPIOMode(mode, -1));
				printf("\n");
			}
		}
		else if (res = strfnd(inpt, "prep\0"))
		{
			res += 4;
			mode = strtol(res, NULL, 10);
			printf(":Prep WF %d mode %d: %d\n", curWf, mode, WavePrepRun(curWf, mode));
		}
		else if (res = strfnd(inpt, "delete\0"))
		{
			printf(":Delete WF %d: %d\n",curWf,WaveDelete(curWf));
			curWf = -1;
		}
		else if (res = strfnd(inpt, "set\0"))
		{
			res += 3;
			pin = strtol(res, &res2, 10);
			if (pin >= 0)
			{
				mode = strtol(res2, NULL, 10);

				setm.U64 = 0;
				if (pin >= 32)
					setm.U32[1] = 1 << (pin - 32);
				else
					setm.U32[0] = 1 << pin;

				if (mode > 0)
				{
					printf(":Set Pin %d (0x%.8x%.8x) HIGH\n", pin, setm.U32[1], setm.U32[0]);
					gpioSetHigh(setm.U64);
				}
				else
				{
					printf(":Set Pin %d (0x%.8x%.8x) LOW\n", pin, setm.U32[1], setm.U32[0]);
					gpioSetLow(setm.U64);
				}
			}
		}
		else if (res = strfnd(inpt, "timeing\0"))
		{
			res += 7;
			pin = strtol(res, &res2, 10);

			debugTiming(pin);
		}
		else if (res = strfnd(inpt, "baud\0"))
		{
			res += 4;
			pin = strtol(res, &res2, 10);

			printf(":uart Init %d\n", uart1Init(pin));
		}
		else if (res = strfnd(inpt, "uart in\0"))
		{
			if (uart1RxAvail() > 0)
			{
				pin = uart1RxCurr(tosend, 25);
				printf(":in %d ||", pin);
				for (i = 0; i <= pin; i++)
					printf("%.2X ", tosend[i]);
				printf("\n");
				fflush(stdin);
			}
		}
		else if (res = strfnd(inpt, "uart out\0"))
		{
			res += 8;
			pin = strtol(res, &res2, 10);

			tosend[0] = 0x1;
			tosend[1] = 0x0a;
			tosend[2] = pin;
			tosend[3] = 0x0;
			tosend[4] = 0x0;
			tosend[5] = 0x1 ^ 0xa ^ tosend[2];
			/*printf(":toSend: ");
			for (i = 0; i < 6; i++)
				printf("%.2X ", tosend[i]);*/
			printf(":uart Send %d\n", uart1TxDma(tosend, 6));
		}
		else if (res = strfnd(inpt, "uart cnt\0"))
		{
			res += 8;
			pin = strtol(res, &res2, 10);
			if (pin > 25)
				pin = 25;
			for (i = 0; i < pin; i++)
				tosend[i] = 0x42;
			printf(":uart Send %d\n", uart1TxDma(tosend, pin));
		}
		else if (res = strfnd(inpt, "uart clr\0"))
		{
			printf(":start\n");
			tosend[0] = 0;
			for (i = 0; i < 30 && (uart1RxAvail() == 0); i++)
			{
				printf("%d: %d\n",i,uart1TxDma(tosend,1));
				usleep(1000);
			}
			printf(":done\n");
		}
		else if (res = strfnd(inpt, "get\0"))
		{
			setm.U64 = gpioLevels();
			printf(":Levels: 0x%.8x%.8x\n", setm.U32[0], setm.U32[1]);
		}
		else if (res = strfnd(inpt, "help\0"))
		{
			rsafe = 0;
			printf("\"init\" to initilize GPIO memory and setings\n");
			printf("\"mode -1\" for list of modes of all pins\n");
			printf("\"mode x y\" to set pin x to mode y\n");
			printf("\"set x y\" to set output level of pin x to low(0), or high(1)\n");
			printf("\"add\" will prompt for info to add a waveform event\n");
			printf("\"prep\" will send curent waveform to dma memory\n");
			printf("\"run\" runs a preped waveform\n");
			printf("\"stop\" will end any running waveform\n");
			printf("\"delete\" deletes current waveform from working (non dma) memory\n");
			printf("\"read h\" will read in h as hex and echo it\n");
			printf("\"wave x\" to setup randomised wavefom into dma\n");
			printf("\"exit\" will free GPIO memory if needed and close proram\n");
		}
		
		else if (res = strfnd(inpt, "run\0"))
		{
			//WaveStartRun();
			StartWaveform();
			printf(":Started\n");
		}
		else if (res = strfnd(inpt, "stop\0"))
		{
			WaveStopRun();
			printf(":Stoped\n");
		}
		else if (res = strfnd(inpt, "exit\0"))
		{
			printf("::exit\n");
			break;
		}
		else if (res = strfnd(inpt, "vdma\0"))
		{
			WaveVerifyDMA();
		}
		else if (res = strfnd(inpt, "vwf\0"))
		{
			res += 3;
			pin = strtol(res, NULL, 10);
			WaveVerifyWave(pin);
		}
		else if (res = strfnd(inpt, "quick\0"))
		{
			res += 5;
			setm.U64 = strtoull(res, &res2, 16);
			mode = strtol(res2, NULL, 10);
			if (mode <= 0)
				mode = 3;
			for (i = 1; i <= mode; i++)
			{
				curWf = WaveAddEventRaw(setm.U64, 0, 1, curWf);
				printf("%d, ", curWf);
				curWf = WaveAddEventRaw(0, setm.U64, i==mode?20:1, curWf);
				printf("%d, ", curWf);
			}
			printf("-\n");
		}
		else if (res = strfnd(inpt, "wfvar\0"))
		{

			if (res2 = strfnd(inpt, "px\0"))
			{
				res2 += 2;
				pin = strtol(res2, NULL, 10);
				printf("Pulse Rnd Max: %d\n", setPulseRndMax(pin));
			}
			else if (res2 = strfnd(inpt, "pn\0"))
			{
				res2 += 2;
				pin = strtol(res2, NULL, 10);
				printf("Pulse Rnd Min: %d\n", setPulseRndMin(pin));
			}
			else if (res2 = strfnd(inpt, "pc\0"))
			{
				res2 += 2;
				pin = strtol(res2, NULL, 10);
				printf("Pulse Count: %d\n", setPulseCount(pin));
			}
			else if (res2 = strfnd(inpt, "pw\0"))
			{
				res2 += 2;
				pin = strtol(res2, NULL, 10);
				printf("Pulse Width: %d\n", setPulseWidth(pin));
			}
			else if (res2 = strfnd(inpt, "bx\0"))
			{
				res2 += 2;
				dbl = strtod(res2, NULL);
				printf("Burst Rnd Max: %f\n", setBurstRndMax(dbl));
			}
			else if (res2 = strfnd(inpt, "bn\0"))
			{
				res2 += 2;
				dbl = strtod(res2, NULL);
				printf("Burst Rnd Min: %f\n", setBurstRndMin(dbl));
			}
			else if (res2 = strfnd(inpt, "bc\0"))
			{
				res2 += 2;
				pin = strtol(res2, NULL, 10);
				printf("Burst Count: %d\n", setBurstCount(pin));
			}
			else if (res2 = strfnd(inpt, "cw\0"))
			{
				res2 += 2;
				dbl = strtod(res2, NULL);
				printf("Cycle Width: %f\n", setCycleWidth(dbl));
			}
			else if (res2 = strfnd(inpt, "it\0"))
			{
				res2 += 2;
				pin = strtol(res2, NULL, 10);
				printf("Inhibit Time: %d\n", setInhibitTime(pin));
			}
			else if (res2 = strfnd(inpt, "id\0"))
			{
				res2 += 2;
				pin = strtol(res2, NULL, 10);
				printf("Inhibit Delay: %d\n", setInhibitDelay(pin));
			}
			else if (res2 = strfnd(inpt, "ct\0"))
			{
				res2 += 2;
				pin = strtol(res2, NULL, 10);
				printf("Charge Time: %d\n", setChargeTime);
			}
			else if (res2 = strfnd(inpt, "cd\0"))
			{
				res2 += 2;
				pin = strtol(res2, NULL, 10);
				printf("Charge Delay: %d\n", setChargeDelay(pin));
			}
			else if (res2 = strfnd(inpt, "go\0"))
			{
				setGenMode(GEN_MODE_WAVE_ONLY);
				debugPrintWF();
				printf("WF: %d\n",GenerateWaveFormOnly());
				ReadyWaveForm(1);
			}
			else
			{
				printf("not recognised\n");
				rsafe = 0;
			}

		}
		else
		{
			printf("::CMD NOT RECOGNISED (\"help\" for commands)::\n");
			rsafe = 0;
		}
		if (rsafe)
		{
			for (i = 0; i < 100; i++)
				rept[i] = inpt[i];
		}
	}
	if (init >= 0)
	{
		printf(":gpio Terminate\n");
		//gpioTerminate();
		WaveTerminate();
	}
	printf("<End of Line>\n");
}


//Searches 'in' for first instance of 'what' and
//returns pointer to first matching char of 'in', or NULL
//NOTE: both 'in' & 'what' must be null terminated ('\0')
char * strfnd(const char *in, const char *what)
{
	int sub;
	const char *i;
	const char *w;

	while (*in != '\0')
	{
		if (*in == *what)
		{
			sub = 1;
			i = in + sub;
			w = what + sub;
			while (*i != '\0' && *w != '\0' && *i == *w)
			{
				sub++;
				i = in + sub;
				w = what + sub;
			}
			if (*w == '\0')
				return (char *)in;
		}
		in++;
	}
	return NULL;
}