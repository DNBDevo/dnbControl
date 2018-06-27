#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <ctype.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
//#include <math.h>

#include "dnbGPIO.h"

#define THOUSAND 1000
#define MILLION  1000000
#define BILLION  1000000000


//mem loc's of periphial buss
#define PI_PERI_BUS 0x7E000000

#define AUX_BASE   (pi_peri_phys + 0x00215000)
#define CLK_BASE   (pi_peri_phys + 0x00101000)
#define DMA_BASE   (pi_peri_phys + 0x00007000)
#define DMA15_BASE (pi_peri_phys + 0x00E05000)
#define GPIO_BASE  (pi_peri_phys + 0x00200000)
#define PCM_BASE   (pi_peri_phys + 0x00203000)
#define PWM_BASE   (pi_peri_phys + 0x0020C000)
#define SPI_BASE   (pi_peri_phys + 0x00204000)
#define SYST_BASE  (pi_peri_phys + 0x00003000)
#define UART_BASE  (pi_peri_phys + 0x00201000)

//len of each buss
#define AUX_LEN   0xD8
#define CLK_LEN   0xA8
#define DMA_LEN   0x1000 /* allow access to all channels */
#define GPIO_LEN  0xB4
#define PCM_LEN   0x24
#define PWM_LEN   0x28
#define SPI_LEN   0x18
#define SYST_LEN  0x1C
#define UART_LEN  0x8C

#define BYTS_P_REG 4

//regester mem loc's (32b)
#define GPFSEL0    0

#define GPSET0     7
#define GPSET1     8

#define GPCLR0    10
#define GPCLR1    11

#define GPLEV0    13
#define GPLEV1    14

#define GPEDS0    16
#define GPEDS1    17

#define GPREN0    19
#define GPREN1    20
#define GPFEN0    22
#define GPFEN1    23
#define GPHEN0    25
#define GPHEN1    26
#define GPLEN0    28
#define GPLEN1    29
#define GPAREN0   31
#define GPAREN1   32
#define GPAFEN0   34
#define GPAFEN1   35

#define GPPUD     37
#define GPPUDCLK0 38
#define GPPUDCLK1 39

//dma regester mem offsets (32b)
#define DMA_CS        0
#define DMA_CONBLK_AD 1
#define DMA_DEBUG     8

/* DMA CS Control and Status bits */
#define DMA_CHANNEL_RESET       (1<<31)
#define DMA_WAIT_ON_WRITES      (1<<28)
#define DMA_PANIC_PRIORITY(x) ((x)<<20)
#define DMA_PRIORITY(x)       ((x)<<16)
#define DMA_INTERRUPT_STATUS    (1<< 2)
#define DMA_END_FLAG            (1<< 1)
#define DMA_ACTIVATE            (1<< 0)

/* DMA control block "info" field bits */
#define DMA_NO_WIDE_BURSTS          (1<<26)
#define DMA_PERIPHERAL_MAPPING(x) ((x)<<16)
#define DMA_BURST_LENGTH(x)       ((x)<<12)
#define DMA_SRC_IGNORE              (1<<11)
#define DMA_SRC_DREQ                (1<<10)
#define DMA_SRC_WIDTH               (1<< 9)
#define DMA_SRC_INC                 (1<< 8)
#define DMA_DEST_IGNORE             (1<< 7)
#define DMA_DEST_DREQ               (1<< 6)
#define DMA_DEST_WIDTH              (1<< 5)
#define DMA_DEST_INC                (1<< 4)
#define DMA_WAIT_RESP               (1<< 3)

//common info settings
#define NORMAL_DMA (DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP)
#define INCR_DMA (DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_SRC_INC | DMA_DEST_INC)
#define TIMED_DMA(x) (DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(x))

//dma peripheral IDs
#define DMA_PERIPH_ID_ALWAYS		0
#define DMA_PERIPH_ID_DSI			1
#define DMA_PERIPH_ID_PCM_TX		2
#define DMA_PERIPH_ID_PCM_RX		3
#define DMA_PERIPH_ID_SMI			4
#define DMA_PERIPH_ID_PWM			5
#define DMA_PERIPH_ID_SPI_TX		6
#define DMA_PERIPH_ID_SPI_RX		7
#define DMA_PERIPH_ID_BSC_TX		8
#define DMA_PERIPH_ID_BSC_RX		9
//      UNUSED						10
#define DMA_PERIPH_ID_E_MMC			11
#define DMA_PERIPH_ID_UART_TX		12
#define DMA_PERIPH_ID_SD_HOST		13
#define DMA_PERIPH_ID_UART_RX		14
#define DMA_PERIPH_ID_DSI2			15
#define DMA_PERIPH_ID_SLIMBUS_MCTX	16
#define DMA_PERIPH_ID_HDMI			17
#define DMA_PERIPH_ID_SLIMBUS_MCRX	18
#define DMA_PERIPH_ID_SLIMBUS_DC0	19
#define DMA_PERIPH_ID_SLIMBUS_DC1	20
#define DMA_PERIPH_ID_SLIMBUS_DC2	21
#define DMA_PERIPH_ID_SLIMBUS_DC3	22
#define DMA_PERIPH_ID_SLIMBUS_DC4	23
#define DMA_PERIPH_ID_FIFO0_SMI		24
#define DMA_PERIPH_ID_FIFO1_SMI		25
#define DMA_PERIPH_ID_FIFO2_SMI		26
#define DMA_PERIPH_ID_SLIMBUS_DC5	27
#define DMA_PERIPH_ID_SLIMBUS_DC6	28
#define DMA_PERIPH_ID_SLIMBUS_DC7	29
#define DMA_PERIPH_ID_SLIMBUS_DC8	30
#define DMA_PERIPH_ID_SLIMBUS_DC9	31

// DMA debug regester bit masks
#define DMA_DEBUG_READ_ERR           (1<<2)
#define DMA_DEBUG_FIFO_ERR           (1<<1)
#define DMA_DEBUG_RD_LST_NOT_SET_ERR (1<<0)

//lite dma regesters info
#define DMA_LITE_FIRST 7
#define DMA_LITE_MAX 0xfffc

//pwm regesters mem loc (32b)
#define PWM_CTL      0
#define PWM_STA      1
#define PWM_DMAC     2
#define PWM_RNG1     4
#define PWM_DAT1     5
#define PWM_FIFO     6
#define PWM_RNG2     8
#define PWM_DAT2     9

//pwm CTL flags
#define PWM_CTL_MSEN2 (1<<15)
#define PWM_CTL_PWEN2 (1<<8)
#define PWM_CTL_MSEN1 (1<<7)
#define PWM_CTL_CLRF1 (1<<6)
#define PWM_CTL_USEF1 (1<<5)
#define PWM_CTL_MODE1 (1<<1)
#define PWM_CTL_PWEN1 (1<<0)

//pwm DMAC flags
#define PWM_DMAC_ENAB      (1 <<31)
#define PWM_DMAC_PANIC(x) ((x)<< 8)
#define PWM_DMAC_DREQ(x)   (x)

// pcm regesters
#define PCM_CS     0
#define PCM_FIFO   1
#define PCM_MODE   2
#define PCM_RXC    3
#define PCM_TXC    4
#define PCM_DREQ   5
#define PCM_INTEN  6
#define PCM_INTSTC 7
#define PCM_GRAY   8

//pcm cs regester bits
#define PCM_CS_STBY     (1 <<25)
#define PCM_CS_SYNC     (1 <<24)
#define PCM_CS_RXSEX    (1 <<23)
#define PCM_CS_RXERR    (1 <<16)
#define PCM_CS_TXERR    (1 <<15)
#define PCM_CS_DMAEN    (1  <<9)
#define PCM_CS_RXTHR(x) ((x)<<7)
#define PCM_CS_TXTHR(x) ((x)<<5)
#define PCM_CS_RXCLR    (1  <<4)
#define PCM_CS_TXCLR    (1  <<3)
#define PCM_CS_TXON     (1  <<2)
#define PCM_CS_RXON     (1  <<1)
#define PCM_CS_EN       (1  <<0)

//pcm mode regester bits
#define PCM_MODE_CLK_DIS  (1  <<28)
#define PCM_MODE_PDMN     (1  <<27)
#define PCM_MODE_PDME     (1  <<26)
#define PCM_MODE_FRXP     (1  <<25)
#define PCM_MODE_FTXP     (1  <<24)
#define PCM_MODE_CLKM     (1  <<23)
#define PCM_MODE_CLKI     (1  <<22)
#define PCM_MODE_FSM      (1  <<21)
#define PCM_MODE_FSI      (1  <<20)
#define PCM_MODE_FLEN(x)  ((x)<<10)
#define PCM_MODE_FSLEN(x) ((x)<< 0)

//pcm rx regester bits
#define PCM_RXC_CH1WEX    (1  <<31)
#define PCM_RXC_CH1EN     (1  <<30)
#define PCM_RXC_CH1POS(x) ((x)<<20)
#define PCM_RXC_CH1WID(x) ((x)<<16)
#define PCM_RXC_CH2WEX    (1  <<15)
#define PCM_RXC_CH2EN     (1  <<14)
#define PCM_RXC_CH2POS(x) ((x)<< 4)
#define PCM_RXC_CH2WID(x) ((x)<< 0)

//pcm tx regester bits
#define PCM_TXC_CH1WEX    (1  <<31)
#define PCM_TXC_CH1EN     (1  <<30)
#define PCM_TXC_CH1POS(x) ((x)<<20)
#define PCM_TXC_CH1WID(x) ((x)<<16)
#define PCM_TXC_CH2WEX    (1  <<15)
#define PCM_TXC_CH2EN     (1  <<14)
#define PCM_TXC_CH2POS(x) ((x)<< 4)
#define PCM_TXC_CH2WID(x) ((x)<< 0)

//pcm dreq regester bits
#define PCM_DREQ_TX_PANIC(x) ((x)<<24)
#define PCM_DREQ_RX_PANIC(x) ((x)<<16)
#define PCM_DREQ_TX_REQ_L(x) ((x)<< 8)
#define PCM_DREQ_RX_REQ_L(x) ((x)<< 0)

//pcm interupt regester bits
#define PCM_INTEN_RXERR (1<<3)
#define PCM_INTEN_TXERR (1<<2)
#define PCM_INTEN_RXR   (1<<1)
#define PCM_INTEN_TXW   (1<<0)

//pcm intstc regester bits
#define PCM_INTSTC_RXERR (1<<3)
#define PCM_INTSTC_TXERR (1<<2)
#define PCM_INTSTC_RXR   (1<<1)
#define PCM_INTSTC_TXW   (1<<0)

//pcm gray regester bits
#define PCM_GRAY_FLUSH (1<<2)
#define PCM_GRAY_CLR   (1<<1)
#define PCM_GRAY_EN    (1<<0)

//==// may need to change
#define FLUSH_PAGES 1024

#define MB_DEV_MAJOR 100
#define MB_DEV1 "/dev/vcio"
#define MB_DEV2 "/dev/pigpio-mb"

#define MB_IOCTL _IOWR(MB_DEV_MAJOR, 0, char *)

#define SPI_CS   0
#define SPI_FIFO 1
#define SPI_CLK  2
#define SPI_DLEN 3
#define SPI_LTOH 4
#define SPI_DC   5

//--------------------- clock info -------------------------
#define CLK_PASSWD  (0x5A<<24)

#define CLK_CTL_MASH(x)((x)<<9)
#define CLK_CTL_BUSY    (1 <<7)
#define CLK_CTL_KILL    (1 <<5)
#define CLK_CTL_ENAB    (1 <<4)
#define CLK_CTL_SRC(x) ((x)<<0)

#define CLK_SRCS 2

#define CLK_CTL_SRC_OSC  1
#define CLK_CTL_SRC_PLLD 6

#define CLK_OSC_FREQ   19200000
#define CLK_PLLD_FREQ 500000000

#define CLK_DIV_DIVI(x) ((x)<<12)
#define CLK_DIV_DIVF(x) ((x)<< 0)

#define CLK_GP0_CTL 28
#define CLK_GP0_DIV 29
#define CLK_GP1_CTL 30
#define CLK_GP1_DIV 31
#define CLK_GP2_CTL 32
#define CLK_GP2_DIV 33

#define CLK_PCMCTL 38
#define CLK_PCMDIV 39

#define CLK_PWMCTL 40
#define CLK_PWMDIV 41

#define SYST_CS      0
#define SYST_CLO     1
#define SYST_CHI     2

#define PI_CLOCK_PWM 0
#define PI_CLOCK_PCM 1

#define PWM_TIMER (((PWM_BASE + PWM_FIFO*BYTS_P_REG) & 0x00ffffff) | PI_PERI_BUS)
#define PCM_TIMER (((PCM_BASE + PCM_FIFO*BYTS_P_REG) & 0x00ffffff) | PI_PERI_BUS)
//----------------------------------------------------------

//-------------------MEM SIZE INFO--------------------------
#define STACK_SIZE (256*1024)

#define PAGE_SIZE 4096

//cb is 32 bytes (8 reg * 4byte)
//ool is 4 byte
//1 wf pulse is 3 cb (1 set, 1 clear, 1 delay)
//1 wf pulse can be 4 ool (set0, set 1, clr0, clr1)
//
#define CBS_PER_OPAGE 109
#define OOL_PER_OPAGE  151

#define CBS_PER_DPAGE 128
#define OOL_PER_DPAGE 1024

#define PWM_FREQS 18

#define PAGES_PER_BLOCK 53
#define CB_PG_PER_DMA_BLOCK 46
#define OOL_PG_PER_DMA_BLOCK 7


#define MB_END_TAG 0
#define MB_PROCESS_REQUEST 0

#define MB_ALLOCATE_MEMORY_TAG 0x3000C
#define MB_LOCK_MEMORY_TAG     0x3000D
#define MB_UNLOCK_MEMORY_TAG   0x3000E
#define MB_RELEASE_MEMORY_TAG  0x3000F

#define WF_WORKING_PAGES		3000
#define WF_EVENTS_PER_PAGE		204

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)
//----------------------------------------------------------

#define PI_STARTING 0
#define PI_RUNNING  1
#define PI_ENDING   2


#define DBG_MIN_LEVEL 0
#define DBG_ALWAYS    0
#define DBG_STARTUP   1
#define DBG_DMACBS    2
#define DBG_SCRIPT    3
#define DBG_USER      4
#define DBG_INTERNAL  5
#define DBG_SLOW_TICK 6
#define DBG_FAST_TICK 7
#define DBG_MAX_LEVEL 8

#ifndef EMBEDDED_IN_VM
#define DBG(level, format, arg...) DO_DBG(level, format, ## arg)
#else
#define DBG(level, format, arg...)
#endif

#define DO_DBG(level, format, arg...)                              \
   {                                                               \
      if (gpioCfg.dbgLevel >= level)                               \
         fprintf(stderr, "%s %s: " format "\n" ,                   \
            myTimeStamp(), __FUNCTION__ , ## arg);                 \
   }

#define SOFT_ERROR(x, format, arg...)                              \
   do                                                              \
   {                                                               \
      DBG(DBG_ALWAYS, format, ## arg);                             \
      return x;                                                    \
   }                                                               \
   while (0)

#define CHECK_INITED                                               \
   do                                                              \
   {                                                               \
      if (!libInitialised)                                         \
      {                                                            \
         DBG(DBG_ALWAYS,                                           \
           "pigpio uninitialised, call gpioInitialise()");         \
         return PI_NOT_INITIALISED;                                \
      }                                                            \
   }                                                               \
   while (0)

//----------------STRUCTS ----------------------------------

//typedef struct
//{
//	rawCbs_t cb[CBS_PER_OPAGE]; //128
//	uint32_t OOL[OOL_PER_OPAGE];
//	uint32_t periphData;
//} dmaPage_t;

typedef struct  /*4096 bytes of DMA CB data */
{
	rawCbs_t slot[CBS_PER_DPAGE];
} dmaCBPage_t;

typedef struct  /*4096 bytes of DMA OOL data */
{
	uint32_t slots[OOL_PER_DPAGE];
} dmaOOLPage_t;

typedef struct
{
	unsigned  handle;        /* mbAllocateMemory() */
	uintptr_t bus_addr;      /* mbLockMemory() */
	uintptr_t *virtual_addr; /* mbMapMem() */
	unsigned  size;          /* in bytes */
} DMAMem_t;



typedef struct              /* holds 1 page of wave sets*/
{
	int32_t		WFId;		/* WaveFom ID stored in page (-1 = none)*/
	waveMake_t	slot[WF_EVENTS_PER_PAGE];	/* event settings*/
	int32_t		lastSlot;	/* last slot used in page */
	int32_t		nextPage;	/* wfPage index of next page used for this WFId*/
	int32_t		prevPage;	/* wfPage index of last page used for this WFID*/
} wfPage_t;

typedef struct
{
	unsigned bufferMilliseconds;
	unsigned clockMicros;
	unsigned clockPeriph;
	unsigned DMAprimaryChannel;
	unsigned DMAsecondaryChannel;
	unsigned socketPort;
	unsigned ifFlags;
	unsigned memAllocMode;
	unsigned dbgLevel;
	unsigned alertFreq;
	uint32_t internals;
	/*
	0-3: dbgLevel
	4-7: alertFreq
	*/
} gpioCfg_t;

static void waveMakeCopy(waveMake_t *to, waveMake_t *from)
{
	to->clrMask = from->clrMask;
	to->setMask = from->setMask;
	to->delayUs = from->delayUs;
}

//----------------------------------------------------------



/* global -------------------------------------------------------- */

/* initialise once then preserve */
static volatile uint32_t piCores = 0;
static volatile uint32_t pi_peri_phys = 0x20000000;
static volatile uint32_t pi_dram_bus = 0x40000000;
static volatile uint32_t pi_mem_flag = 0x0C;

static int libInitialised = 0;

/* initialise if not libInitialised */

static wfPage_t WFPages[WF_WORKING_PAGES];
static int WFCurId = -1;	/*current WFId in use*/

/* resources which must be released on gpioTerminate */
static int fdLock = -1;
static int fdMem = -1;
static int fdPmap = -1;
static int fdMbox = -1;

static DMAMem_t *dmaMboxBlk = MAP_FAILED;
static uintptr_t * * dmaPMapBlk = MAP_FAILED;
//>static dmaPage_t * * dmaVirt = MAP_FAILED;
//>static dmaPage_t * * dmaBus = MAP_FAILED;
static dmaCBPage_t * * dmaCBVirt = MAP_FAILED;
static dmaOOLPage_t * * dmaOOLVirt = MAP_FAILED;
static dmaCBPage_t * * dmaCBBus = MAP_FAILED;
static dmaOOLPage_t * * dmaOOLBus = MAP_FAILED;

static volatile uint32_t * auxReg = MAP_FAILED;
static volatile uint32_t * clkReg = MAP_FAILED;
static volatile uint32_t * dmaReg = MAP_FAILED;
static volatile uint32_t * gpioReg = MAP_FAILED;
static volatile uint32_t * pcmReg = MAP_FAILED;
static volatile uint32_t * pwmReg = MAP_FAILED;
static volatile uint32_t * spiReg = MAP_FAILED;
static volatile uint32_t * systReg = MAP_FAILED;
static volatile uint32_t * uartReg = MAP_FAILED;

static volatile uint32_t * dmaIn = MAP_FAILED;
static volatile uint32_t * dmaOut = MAP_FAILED;

static volatile int runState = PI_STARTING;
static struct timespec libStarted;
static int waveClockInited = 0;
static uint64_t gpioMask;

static int gpioMaskSet = 0;

static volatile gpioCfg_t gpioCfg =
{
	PI_DEFAULT_BUFFER_MILLIS,
	PI_DEFAULT_CLK_MICROS,
	PI_DEFAULT_CLK_PERIPHERAL,
	PI_DEFAULT_DMA_PRIMARY_CHANNEL,
	PI_DEFAULT_DMA_SECONDARY_CHANNEL,
	PI_DEFAULT_SOCKET_PORT,
	PI_DEFAULT_IF_FLAGS,
	PI_DEFAULT_MEM_ALLOC_MODE,
	0, /* dbgLevel */
	0, /* alertFreq */
	0, /* internals */
};

static int initInitialise(void);
static unsigned gpioHardwareRevision(void);
static void initClearGlobals(void);
static int initCheckPermitted(void);
static void initCheckLockFile(void);
static int initGrabLockFile(void);
static int initPeripherals(void);
static uint32_t * initMapMem(int fd, uint32_t addr, uint32_t len);
static int initAllocDMAMem(void);
static int initPagemapBlock(int block);
static int initZaps(int pmapFd, void *virtualBase, int pageNum, uint32_t * dmaBlockAdr);
static uint32_t myGpioDelay(uint32_t micros);
static int initMboxBlock(int block);
static int mbDMAAlloc(DMAMem_t *DMAMemP, unsigned size, uint32_t pi_mem_flag);
static int mbProperty(int fd, void *buf);
static void *mbMapMem(unsigned base, unsigned size);
static unsigned mbAllocateMemory(int fd, unsigned size, unsigned align, unsigned flags);
static unsigned mbLockMemory(int fd, unsigned handle);
static unsigned mbUnlockMemory(int fd, unsigned handle);
static unsigned mbReleaseMemory(int fd, unsigned handle);
static int mbCreate(char *dev);
static int mbOpen(void);
static void mbClose(int fd);
static void initClock(int mainClock);
static void initHWClk(int clkCtl, int clkDiv, int clkSrc, int divI, int divF, int MASH);
static void initPWM(unsigned bits);
static void initPCM(unsigned bits);
static void flushMemory(void);
static void myGpioSleep(int seconds, int micros);
static int mbUnmapMem(void *addr, unsigned size);
static void mbDMAFree(DMAMem_t *DMAMemP);
static char * myTimeStamp();

static void initReleaseResources(void);

static int wfGetNextNew(int count);
static int wfVerifyWFId(int WFId);
static int wfLastPage(int WFId);


//========================================================================
//                              start init
//========================================================================

int gpioInitialise(void)
{
	int status;

	if (libInitialised) return DNBGPIO_VERSION;

	DBG(DBG_STARTUP, "not initialised, initialising");

	runState = PI_STARTING;

	status = initInitialise();

	if (status < 0)
	{
		runState = PI_ENDING;
		initReleaseResources();
	}
	else
	{
		libInitialised = 1;
		runState = PI_RUNNING;
	}

	return status;
}

//-----------------------------------------------------------------------------
static int initInitialise(void)
{
	int rev, i, model;
	//>struct sockaddr_in server;
	//>char * portStr;
	//>unsigned port;
	//>struct sched_param param;
	//>pthread_attr_t pthAttr;

	DBG(DBG_STARTUP, "");

	waveClockInited = 0;

	clock_gettime(CLOCK_REALTIME, &libStarted);

	rev = gpioHardwareRevision();

	initClearGlobals();

	if (initCheckPermitted() < 0) return PI_INIT_FAILED;

	fdLock = initGrabLockFile();

	if (fdLock < 0)
		SOFT_ERROR(PI_INIT_FAILED, "Can't lock %s", PI_LOCKFILE);

	if (!gpioMaskSet)
	{
		if (rev == 0) gpioMask = PI_DEFAULT_UPDATE_MASK_UNKNOWN;
		else if (rev <   4) gpioMask = PI_DEFAULT_UPDATE_MASK_B1;
		else if (rev <  16) gpioMask = PI_DEFAULT_UPDATE_MASK_A_B2;
		else if (rev == 17) gpioMask = PI_DEFAULT_UPDATE_MASK_COMPUTE;
		else if (rev  < 20) gpioMask = PI_DEFAULT_UPDATE_MASK_APLUS_BPLUS;
		else
		{
			model = (rev >> 4) & 0xFF;

			/* model
			0=A 1=B
			2=A+ 3=B+
			4=Pi2B
			5=Alpha
			6=Compute Module
			7=Unknown
			8=Pi3B
			9=Zero
			*/
			if (model <  2) gpioMask = PI_DEFAULT_UPDATE_MASK_A_B2;
			else if (model <  4) gpioMask = PI_DEFAULT_UPDATE_MASK_APLUS_BPLUS;
			else if (model == 4) gpioMask = PI_DEFAULT_UPDATE_MASK_PI2B;
			else if (model == 6) gpioMask = PI_DEFAULT_UPDATE_MASK_COMPUTE;
			else if (model == 8) gpioMask = PI_DEFAULT_UPDATE_MASK_PI3B;
			else if (model == 9) gpioMask = PI_DEFAULT_UPDATE_MASK_ZERO;
			else                 gpioMask = PI_DEFAULT_UPDATE_MASK_UNKNOWN;
		}

		//>gpioMaskSet = 1;
	}

	//>#ifndef EMBEDDED_IN_VM
	//>sigSetHandler();
	//>#endif

	if (initPeripherals() < 0) return PI_INIT_FAILED;

	if (initAllocDMAMem() < 0) return PI_INIT_FAILED;

	/* done with /dev/mem */

	if (fdMem != -1)
	{
		close(fdMem);
		fdMem = -1;
	}

	//>param.sched_priority = sched_get_priority_max(SCHED_FIFO);
	//>
	//>if (gpioCfg.internals & PI_CFG_RT_PRIORITY)
	//>	sched_setscheduler(0, SCHED_FIFO, &param);

	//initClock(1); /* initialise main clock */

	atexit(gpioTerminate);

	//>if (pthread_attr_init(&pthAttr))
	//>	SOFT_ERROR(PI_INIT_FAILED, "pthread_attr_init failed (%m)");
	//>
	//>if (pthread_attr_setstacksize(&pthAttr, STACK_SIZE))
	//>	SOFT_ERROR(PI_INIT_FAILED, "pthread_attr_setstacksize failed (%m)");
	//>
	//>if (pthread_create(&pthAlert, &pthAttr, pthAlertThread, &i))
	//>	SOFT_ERROR(PI_INIT_FAILED, "pthread_create alert failed (%m)");
	//>
	//>pthAlertRunning = 1;

	//>if (!(gpioCfg.ifFlags & PI_DISABLE_FIFO_IF))
	//>{
	//>	if (pthread_create(&pthFifo, &pthAttr, pthFifoThread, &i))
	//>		SOFT_ERROR(PI_INIT_FAILED, "pthread_create fifo failed (%m)");
	//>
	//>	pthFifoRunning = 1;
	//>}

	//>if (!(gpioCfg.ifFlags & PI_DISABLE_SOCK_IF))
	//>{
	//>	fdSock = socket(AF_INET, SOCK_STREAM, 0);
	//>
	//>	if (fdSock == -1)
	//>		SOFT_ERROR(PI_INIT_FAILED, "socket failed (%m)");
	//>
	//>	portStr = getenv(PI_ENVPORT);
	//>
	//>	if (portStr) port = atoi(portStr); else port = gpioCfg.socketPort;
	//>
	//>	server.sin_family = AF_INET;
	//>	if (gpioCfg.ifFlags & PI_LOCALHOST_SOCK_IF)
	//>	{
	//>		server.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	//>	}
	//>	else
	//>	{
	//>		server.sin_addr.s_addr = htonl(INADDR_ANY);
	//>	}
	//>	server.sin_port = htons(port);
	//>
	//>	if (bind(fdSock, (struct sockaddr *)&server, sizeof(server)) < 0)
	//>		SOFT_ERROR(PI_INIT_FAILED, "bind to port %d failed (%m)", port);
	//>
	//>	if (pthread_create(&pthSocket, &pthAttr, pthSocketThread, &i))
	//>		SOFT_ERROR(PI_INIT_FAILED, "pthread_create socket failed (%m)");
	//>
	//>	pthSocketRunning = 1;
	//>}

	myGpioDelay(20000);

	//>flushMemory();

	return DNBGPIO_VERSION;
}


/* ----------------------------------------------------------------------- */

/*
2 2  2  2 2 2  1 1 1 1  1 1 1 1  1 1 0 0 0 0 0 0  0 0 0 0
5 4  3  2 1 0  9 8 7 6  5 4 3 2  1 0 9 8 7 6 5 4  3 2 1 0

W W  S  M M M  B B B B  P P P P  T T T T T T T T  R R R R

W  warranty void if either bit is set

S  0=old (bits 0-22 are revision number) 1=new (following fields apply)

M  0=256 1=512 2=1024

B  0=Sony 1=Egoman 2=Embest 3=Unknown 4=Embest

P  0=2835, 1=2836, 2=2837

T  0=A 1=B 2=A+ 3=B+ 4=Pi2B 5=Alpha 6=Compute Module 7=Unknown 8=Pi3B 9=Zero

R  PCB board revision

*/

static unsigned gpioHardwareRevision(void)
{
	static unsigned rev = 0;

	FILE * filp;
	char buf[512];
	char term;

	DBG(DBG_USER, "");

	if (rev) return rev;

	piCores = 0;

	filp = fopen("/proc/cpuinfo", "r");

	if (filp != NULL)
	{
		while (fgets(buf, sizeof(buf), filp) != NULL)
		{
			if (piCores == 0)
			{
				if (!strncasecmp("model name", buf, 10))
				{
					if (strstr(buf, "ARMv6") != NULL)
					{
						piCores = 1;
						pi_peri_phys = 0x20000000;
						pi_dram_bus = 0x40000000;
						pi_mem_flag = 0x0C;
					}
					else if (strstr(buf, "ARMv7") != NULL)
					{
						piCores = 4;
						pi_peri_phys = 0x3F000000;
						pi_dram_bus = 0xC0000000;
						pi_mem_flag = 0x04;
					}
					else if (strstr(buf, "ARMv8") != NULL)
					{
						piCores = 4;
						pi_peri_phys = 0x3F000000;
						pi_dram_bus = 0xC0000000;
						pi_mem_flag = 0x04;
					}
				}
			}

			if (!strncasecmp("revision\t:", buf, 10))
			{
				if (sscanf(buf + 10, "%x%c", &rev, &term) == 2)
				{
					if (term != '\n') rev = 0;
				}
			}
		}

		fclose(filp);
	}
	return rev;
}

//-----------------------------------------------------------------------

static void initClearGlobals(void)
{
	int i;

	DBG(DBG_STARTUP, "");

	//>alertBits = 0;
	//>monitorBits = 0;
	//>notifyBits = 0;
	//>scriptBits = 0;
	//>gFilterBits = 0;
	//>nFilterBits = 0;
	//>wdogBits = 0;

	//>pthAlertRunning = 0;
	//>pthFifoRunning = 0;
	//>pthSocketRunning = 0;

	//>wfc[0] = 0;
	//>wfc[1] = 0;
	//>wfc[2] = 0;

	//>wfcur = 0;

	//>wfStats.micros = 0;
	//>wfStats.highMicros = 0;
	//>wfStats.maxMicros = PI_WAVE_MAX_MICROS;

	//>wfStats.pulses = 0;
	//>wfStats.highPulses = 0;
	//>wfStats.maxPulses = PI_WAVE_MAX_PULSES;

	//>wfStats.cbs = 0;
	//>wfStats.highCbs = 0;
	//>wfStats.maxCbs = (PI_WAVE_BLOCKS * PAGES_PER_BLOCK * CBS_PER_OPAGE);

	//>gpioGetSamples.func = NULL;
	//>gpioGetSamples.ex = 0;
	//>gpioGetSamples.userdata = NULL;
	//>gpioGetSamples.bits = 0;

	//>for (i = 0; i <= PI_MAX_USER_GPIO; i++)
	//>{
	//>	wfRx[i].mode = PI_WFRX_NONE;
	//>
	//>	gpioAlert[i].func = NULL;
	//>}

	//>for (i = 0; i <= PI_MAX_GPIO; i++)
	//>{
	//>	gpioInfo[i].is = GPIO_UNDEFINED;
	//>	gpioInfo[i].width = 0;
	//>	gpioInfo[i].range = PI_DEFAULT_DUTYCYCLE_RANGE;
	//>	gpioInfo[i].freqIdx = DEFAULT_PWM_IDX;
	//>}

	//>for (i = 0; i<PI_NOTIFY_SLOTS; i++)
	//>{
	//>gpioNotify[i].seqno = 0;
	//>	gpioNotify[i].state = PI_NOTIFY_CLOSED;
	//>}

	//>for (i = 0; i <= PI_MAX_SIGNUM; i++)
	//>{
	//>	gpioSignal[i].func = NULL;
	//>	gpioSignal[i].ex = 0;
	//>	gpioSignal[i].userdata = NULL;
	//>}

	//>for (i = 0; i <= PI_MAX_TIMER; i++)
	//>{
	//>	gpioTimer[i].running = 0;
	//>	gpioTimer[i].func = NULL;
	//>}

	/* calculate the usable PWM frequencies */

	//>for (i = 0; i<PWM_FREQS; i++)
	//>{
	//>	pwmFreq[i] =
	//>		(1000000.0 /
	//>		((float)PULSE_PER_CYCLE*gpioCfg.clockMicros*pwmCycles[i])) + 0.5;
	//>
	//>	DBG(DBG_STARTUP, "f%d is %d", i, pwmFreq[i]);
	//>}

	//>inpFifo = NULL;
	//>outFifo = NULL;

	for (i = 0; i < WF_WORKING_PAGES; i++)
	{
		WFPages[i].WFId = -1;
		WFPages[i].lastSlot = -1;
		WFPages[i].nextPage = -1;
		WFPages[i].prevPage = -1;
	}

	fdLock = -1;
	fdMem = -1;
	//>fdSock = -1;

	dmaMboxBlk = MAP_FAILED;
	dmaPMapBlk = MAP_FAILED;
	dmaCBVirt = MAP_FAILED;
	dmaCBBus = MAP_FAILED;
	dmaOOLVirt = MAP_FAILED;
	dmaOOLBus = MAP_FAILED;

	auxReg = MAP_FAILED;
	clkReg = MAP_FAILED;
	dmaReg = MAP_FAILED;
	gpioReg = MAP_FAILED;
	pcmReg = MAP_FAILED;
	pwmReg = MAP_FAILED;
	systReg = MAP_FAILED;
	spiReg = MAP_FAILED;
}

//-----------------------------------------------------------------------

static int initCheckPermitted(void)
{
	if ((fdMem = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
	{
		DBG(DBG_ALWAYS,
			"\n" \
			"+---------------------------------------------------------+\n" \
			"|Sorry, you don't have permission to run this program.    |\n" \
			"|Try running as root, e.g. precede the command with sudo. |\n" \
			"+---------------------------------------------------------+\n\n");
		return -1;
	}
	return 0;
}

//-----------------------------------------------------------------------

static void initCheckLockFile(void)
{
	int fd;
	int count;
	int pid;
	int err;
	int delete;
	char str[20];

	fd = open(PI_LOCKFILE, O_RDONLY);

	if (fd != -1)
	{
		DBG(DBG_STARTUP, "lock file exists");
		delete = 1;

		count = read(fd, str, sizeof(str) - 1);

		if (count)
		{
			pid = atoi(str);
			err = kill(pid, 0);
			if (!err) delete = 0; /* process still exists */
			DBG(DBG_STARTUP, "lock file pid=%d err=%d", pid, err);
		}

		close(fd);
		DBG(DBG_STARTUP, "lock file delete=%d", delete);

		if (delete) unlink(PI_LOCKFILE);
	}
}

//-----------------------------------------------------------------------

static int initGrabLockFile(void)
{
	int fd;
	int lockResult;
	char pidStr[20];

	initCheckLockFile();

	/* try to grab the lock file */

	fd = open(PI_LOCKFILE, O_WRONLY | O_CREAT | O_EXCL | O_TRUNC, 0644);

	if (fd != -1)
	{
		lockResult = flock(fd, LOCK_EX | LOCK_NB);

		if (lockResult == 0)
		{
			sprintf(pidStr, "%d\n", (int)getpid());

			write(fd, pidStr, strlen(pidStr));
		}
		else
		{
			close(fd);
			return -1;
		}
	}

	return fd;
}

//-----------------------------------------------------------------------

static int initPeripherals(void)
{
	uint32_t dmaBase;

	DBG(DBG_STARTUP, "");

	gpioReg = initMapMem(fdMem, GPIO_BASE, GPIO_LEN);

	if (gpioReg == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap gpio failed (%m)");

	/* dma channels 0-14 share one page, 15 has another */

	dmaBase = DMA_BASE;


	dmaReg = initMapMem(fdMem, dmaBase, DMA_LEN);
	
	if (dmaReg == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap dma failed (%m)");

	if (gpioCfg.DMAprimaryChannel < 15)
	{
		dmaIn = dmaReg + (gpioCfg.DMAprimaryChannel * 0x40);//0x40
		dmaOut = dmaReg + (gpioCfg.DMAsecondaryChannel * 0x40 ); //0x40
	}	

	DBG(DBG_STARTUP, "DMA #%d @ %08X @ %08X",
		gpioCfg.DMAprimaryChannel, dmaBase, (uint32_t)dmaIn);

	//>DBG(DBG_STARTUP, "debug reg is %08X", dmaIn[DMA_DEBUG]);

	clkReg = initMapMem(fdMem, CLK_BASE, CLK_LEN);

	if (clkReg == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap clk failed (%m)");

	systReg = initMapMem(fdMem, SYST_BASE, SYST_LEN);

	if (systReg == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap syst failed (%m)");

	spiReg = initMapMem(fdMem, SPI_BASE, SPI_LEN);
	
	if (spiReg == MAP_FAILED)
	SOFT_ERROR(PI_INIT_FAILED, "mmap spi failed (%m)");

	pwmReg = initMapMem(fdMem, PWM_BASE, PWM_LEN);

	if (pwmReg == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap pwm failed (%m)");

	pcmReg = initMapMem(fdMem, PCM_BASE, PCM_LEN);

	if (pcmReg == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap pcm failed (%m)");

	auxReg = initMapMem(fdMem, AUX_BASE, AUX_LEN);
	
	if (auxReg == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap aux failed (%m)");

	uartReg = initMapMem(fdMem, UART_BASE, UART_LEN);

	if (uartReg == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap uart0 failed (%m)");

	return 0;
}

//------------------------------------------------------------------------

static uint32_t * initMapMem(int fd, uint32_t addr, uint32_t len)
{
	return (uint32_t *)mmap(0, len,
		PROT_READ | PROT_WRITE | PROT_EXEC,
		MAP_SHARED | MAP_LOCKED,
		fd, addr);
}

//-------------------------------------------------------------------------

static int initAllocDMAMem(void)
{
	int i;//>, servoCycles, superCycles;
	int status;

	DBG(DBG_STARTUP, "");

	/* allocate memory for pointers to virtual and bus memory pages */
	/* allocate 46 CB and 7 OOL pages per BLOCK*/

	//virtual CB pointers
	dmaCBVirt = mmap(0,CB_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS * sizeof(dmaCBPage_t *),
		PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_LOCKED,
		-1, 0);
	if(dmaCBVirt == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap dma CB virtual failed (%m)");
	//virtual OOL pointers
	dmaOOLVirt = mmap(0, OOL_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS * sizeof(dmaOOLPage_t *),
		PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_LOCKED,
		-1, 0);
	if (dmaOOLVirt == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap dma OOL virtual failed (%m)");
	//buss CB pointers
	dmaCBBus = mmap(0, CB_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS * sizeof(dmaCBPage_t *),
		PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_LOCKED,
		-1, 0);
	if (dmaCBBus == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap dma CB Buss failed (%m)");
	//buss OOL pointers
	dmaOOLBus = mmap(0, OOL_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS * sizeof(dmaOOLPage_t *),
		PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_LOCKED,
		-1, 0);
	if (dmaOOLBus == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap dma OOL Buss failed (%m)");

	//>dmaVirt = mmap(
	//>	0, PAGES_PER_BLOCK * PI_WAVE_BLOCKS * sizeof(dmaPage_t *),
	//>	PROT_READ | PROT_WRITE,
	//>	MAP_PRIVATE | MAP_ANONYMOUS | MAP_LOCKED,
	//>	-1, 0);
	//>
	//>if (dmaVirt == MAP_FAILED)
	//>	SOFT_ERROR(PI_INIT_FAILED, "mmap dma virtual failed (%m)");
	//>
	//>dmaBus = mmap(
	//>	0, PAGES_PER_BLOCK * PI_WAVE_BLOCKS * sizeof(dmaPage_t *),
	//>	PROT_READ | PROT_WRITE,
	//>	MAP_PRIVATE | MAP_ANONYMOUS | MAP_LOCKED,
	//>	-1, 0);
	//>
	//>if (dmaBus == MAP_FAILED)
	//>	SOFT_ERROR(PI_INIT_FAILED, "mmap dma bus failed (%m)");


	if ((gpioCfg.memAllocMode == PI_MEM_ALLOC_PAGEMAP) ||
		((gpioCfg.memAllocMode == PI_MEM_ALLOC_AUTO) &&
		(gpioCfg.bufferMilliseconds > PI_DEFAULT_BUFFER_MILLIS)))
	{
		/* pagemap allocation of DMA memory */

		dmaPMapBlk = mmap(
			0, PI_WAVE_BLOCKS * sizeof(dmaCBPage_t *),
			PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS | MAP_LOCKED,
			-1, 0);

		if (dmaPMapBlk == MAP_FAILED)
			SOFT_ERROR(PI_INIT_FAILED, "pagemap mmap block failed (%m)");

		fdPmap = open("/proc/self/pagemap", O_RDONLY);

		if (fdPmap < 0)
			SOFT_ERROR(PI_INIT_FAILED, "pagemap open failed(%m)");

		for (i = 0; i<PI_WAVE_BLOCKS; i++)
		{
			status = initPagemapBlock(i);
			if (status < 0)
			{
				close(fdPmap);
				return status;
			}
		}

		close(fdPmap);

		DBG(DBG_STARTUP, "dmaPMapBlk=%08X dmaIn=%08X",
			(uint32_t)dmaPMapBlk, (uint32_t)dmaIn);
	}
	else
	{
		/* mailbox allocation of DMA memory */

		dmaMboxBlk = mmap(
			0, PI_WAVE_BLOCKS * sizeof(DMAMem_t),
			PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS | MAP_LOCKED,
			-1, 0);

		if (dmaMboxBlk == MAP_FAILED)
			SOFT_ERROR(PI_INIT_FAILED, "mmap mbox block failed (%m)");

		fdMbox = mbOpen();

		if (fdMbox < 0)
			SOFT_ERROR(PI_INIT_FAILED, "mbox open failed(%m)");

		for (i = 0; i < PI_WAVE_BLOCKS; i++)
		{
			//>//>
			status = initMboxBlock(i);
			if (status < 0)
			{
				mbClose(fdMbox);
				return status;
			}
		}

		mbClose(fdMbox);

		DBG(DBG_STARTUP, "dmaMboxBlk=%08X dmaIn=%08X",
			(uint32_t)dmaMboxBlk, (uint32_t)dmaIn);
	}

	DBG(DBG_STARTUP,
		"gpioReg=%08X pwmReg=%08X pcmReg=%08X clkReg=%08X auxReg=%08X",
		(uint32_t)gpioReg, (uint32_t)pwmReg,
		(uint32_t)pcmReg, (uint32_t)clkReg, (uint32_t)auxReg);

	//>for (i = 0; i<DMAI_PAGES; i++)
	//>	DBG(DBG_STARTUP, "dmaIBus[%d]=%08X", i, (uint32_t)dmaIBus[i]);

	//>if (gpioCfg.dbgLevel >= DBG_DMACBS)
	//>{
	//>	fprintf(stderr, "*** INPUT DMA CONTROL BLOCKS ***\n");
	//>	for (i = 0; i<NUM_CBS; i++) dmaCbPrint(i);
	//>}

	return 0;
}

//---------------------------------------------------------------------------

static int initPagemapBlock(int block)
{
	int trys, ok;
	unsigned pageNum;
	uint32_t * blockAdr;

	DBG(DBG_STARTUP, "block=%d", block);

	dmaPMapBlk[block] = mmap(
		0, (PAGES_PER_BLOCK*PAGE_SIZE),
		PROT_READ | PROT_WRITE,
		MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE | MAP_LOCKED,
		-1, 0);

	if (dmaPMapBlk[block] == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap dma block %d failed (%m)", block);

	/* force allocation of physical memory */

	memset((void *)dmaPMapBlk[block], 0xAA, (PAGES_PER_BLOCK*PAGE_SIZE));

	memset((void *)dmaPMapBlk[block], 0xFF, (PAGES_PER_BLOCK*PAGE_SIZE));

	memset((void *)dmaPMapBlk[block], 0, (PAGES_PER_BLOCK*PAGE_SIZE));

	pageNum = block * PAGES_PER_BLOCK;

	blockAdr = mmap(
		0, (PAGES_PER_BLOCK*PAGE_SIZE),
		PROT_READ | PROT_WRITE,
		MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE | MAP_LOCKED,
		-1, 0);
	if(blockAdr == MAP_FAILED)
		SOFT_ERROR(PI_INIT_FAILED, "mmap dma CB block %d failed (%m)", block);
	munmap((void *)blockAdr, PAGES_PER_BLOCK*PAGE_SIZE);
	

	//>dmaVirt[pageNum] = mmap(
	//>	0, (PAGES_PER_BLOCK*PAGE_SIZE),
	//>	PROT_READ | PROT_WRITE,
	//>	MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE | MAP_LOCKED,
	//>	-1, 0);
	//>
	//>if (dmaVirt[pageNum] == MAP_FAILED)
	//>	SOFT_ERROR(PI_INIT_FAILED, "mmap dma block %d failed (%m)", block);
	//>
	//>munmap(dmaVirt[pageNum], PAGES_PER_BLOCK*PAGE_SIZE);

	trys = 0;
	ok = 0;

	while ((trys < 10) && !ok)
	{
		if (initZaps(fdPmap,
			dmaPMapBlk[block],
			pageNum,
			blockAdr) == 0) ok = 1;
		else myGpioDelay(50000);

		++trys;
	}

	if (!ok) SOFT_ERROR(PI_INIT_FAILED, "initZaps failed");

	return 0;
}

//===========================================================================

static int initZaps
//(int  pmapFd, void *virtualBase, int  basePage, int  pages)
(int pmapFd, void *virtualBase, int pageNum, uint32_t * dmaBlockAdr )
{
	int n;
	long index;
	off_t offset;
	ssize_t t;
	uint32_t physical;
	int status;
	uint32_t pageAdr;
	unsigned long long pa;
	int CBSet;

	DBG(DBG_STARTUP, "");

	status = 0;

	pageAdr = (uint32_t)dmaBlockAdr;
	
	index = ((uint32_t)virtualBase / PAGE_SIZE) * 8;

	offset = lseek(pmapFd, index, SEEK_SET);

	if (offset != index)
		SOFT_ERROR(PI_INIT_FAILED, "lseek pagemap failed (%m)");

	for (n = 0; n<PAGES_PER_BLOCK; n++)
	{
		t = read(pmapFd, &pa, sizeof(pa));

		if (t != sizeof(pa))
			SOFT_ERROR(PI_INIT_FAILED, "read pagemap failed (%m)");

		DBG(DBG_STARTUP, "pf%d=%016llX", n, pa);

		physical = 0x3FFFFFFF & (PAGE_SIZE * (pa & 0xFFFFFFFF));

		if (physical)
		{
			if (pageNum + n < PI_WAVE_BLOCKS * CB_PG_PER_DMA_BLOCK)
			{
				dmaCBBus[pageNum + n] = (dmaCBPage_t *)(physical | pi_dram_bus);

				dmaCBVirt[pageNum + n] = mmap
				(
					(void *)pageAdr,
					PAGE_SIZE,
					PROT_READ | PROT_WRITE,
					MAP_SHARED | MAP_FIXED | MAP_LOCKED | MAP_NORESERVE,
					fdMem,
					physical
				);
			}
			else
			{
				dmaOOLBus[(pageNum - (PI_WAVE_BLOCKS * CB_PG_PER_DMA_BLOCK)) + n] = (dmaOOLPage_t *)(physical | pi_dram_bus);

				dmaOOLVirt[(pageNum - (PI_WAVE_BLOCKS * CB_PG_PER_DMA_BLOCK)) + n] = mmap
				(
					(void *)pageAdr,
					PAGE_SIZE,
					PROT_READ | PROT_WRITE,
					MAP_SHARED | MAP_FIXED | MAP_LOCKED | MAP_NORESERVE,
					fdMem,
					physical
				);
			}
		}
		else status = 1;

		pageAdr += PAGE_SIZE;
	}

	return status;
}

//=============================================================

static int mbUnmapMem(void *addr, unsigned size)
{
	/* 0 okay, -1 fail */
	return munmap(addr, size);
}

//=============================================================

static void mbDMAFree(DMAMem_t *DMAMemP)
{
	if (DMAMemP->handle)
	{
		mbUnmapMem(DMAMemP->virtual_addr, DMAMemP->size);
		mbUnlockMemory(fdMbox, DMAMemP->handle);
		mbReleaseMemory(fdMbox, DMAMemP->handle);
		DMAMemP->handle = 0;
	}
}

//=======================================================================

static uint32_t myGpioDelay(uint32_t micros)
{
	uint32_t start;

	start = systReg[SYST_CLO];

	if (micros <= PI_MAX_BUSY_DELAY)
	{
		while ((systReg[SYST_CLO] - start) <= micros);
	}
	else
	{
		myGpioSleep(micros / MILLION, micros%MILLION);
	}

	return (systReg[SYST_CLO] - start);
}

//============================================================

static void myGpioSleep(int seconds, int micros)
{
	struct timespec ts, rem;

	ts.tv_sec = seconds;
	ts.tv_nsec = micros * 1000;

	while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem))
	{
		/* copy remaining time to ts */
		ts = rem;
	}
}

//============================================================

static int initMboxBlock(int block)
{
	int n, ok;
	unsigned page;
	uintptr_t virtualAdr;
	uintptr_t busAdr;

	DBG(DBG_STARTUP, "block=%d", block);

	ok = mbDMAAlloc
	(&dmaMboxBlk[block], PAGES_PER_BLOCK * PAGE_SIZE, pi_mem_flag);

	if (!ok) SOFT_ERROR(PI_INIT_FAILED, "init mbox zaps failed");

	page = block * PAGES_PER_BLOCK;

	virtualAdr = (uintptr_t)dmaMboxBlk[block].virtual_addr;
	busAdr = dmaMboxBlk[block].bus_addr;

	for (n = 0; n<PAGES_PER_BLOCK; n++)
	{
		if (page + n < PI_WAVE_BLOCKS * CB_PG_PER_DMA_BLOCK)
		{
			dmaCBVirt[page + n] = (dmaCBPage_t *)virtualAdr;
			dmaCBBus[page + n] = (dmaCBPage_t *)busAdr;
		}
		else
		{
			dmaOOLVirt[page + n - (PI_WAVE_BLOCKS * CB_PG_PER_DMA_BLOCK)] = (dmaOOLPage_t *)virtualAdr;
			dmaOOLBus[page + n - (PI_WAVE_BLOCKS * CB_PG_PER_DMA_BLOCK)] = (dmaOOLPage_t *) busAdr;
		}
		virtualAdr += PAGE_SIZE;
		busAdr += PAGE_SIZE;
	}

	return 0;
}

//===================================================

static int mbDMAAlloc(DMAMem_t *DMAMemP, unsigned size, uint32_t pi_mem_flag)
{
	DMAMemP->size = size;

	DMAMemP->handle =
		mbAllocateMemory(fdMbox, size, PAGE_SIZE, pi_mem_flag);

	if (DMAMemP->handle)
	{
		DMAMemP->bus_addr = mbLockMemory(fdMbox, DMAMemP->handle);
		DMAMemP->virtual_addr =
			mbMapMem(BUS_TO_PHYS(DMAMemP->bus_addr), size);

		return 1;
	}
	return 0;
}

//===================================================

static int mbProperty(int fd, void *buf)
{
	return ioctl(fd, MB_IOCTL, buf);
}

//===================================================

static void *mbMapMem(unsigned base, unsigned size)
{
	void *mem = MAP_FAILED;

	mem = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fdMem, base);

	return mem;
}

//===================================================

static unsigned mbAllocateMemory(
	int fd, unsigned size, unsigned align, unsigned flags)
{
	int i = 1;
	unsigned p[32];

	p[i++] = MB_PROCESS_REQUEST;
	p[i++] = MB_ALLOCATE_MEMORY_TAG;
	p[i++] = 12;
	p[i++] = 12;
	p[i++] = size;
	p[i++] = align;
	p[i++] = flags;
	p[i++] = MB_END_TAG;
	p[0] = i * sizeof(*p);

	mbProperty(fd, p);

	return p[5];
}

//===================================================

static unsigned mbLockMemory(int fd, unsigned handle)
{
	int i = 1;
	unsigned p[32];

	p[i++] = MB_PROCESS_REQUEST;
	p[i++] = MB_LOCK_MEMORY_TAG;
	p[i++] = 4;
	p[i++] = 4;
	p[i++] = handle;
	p[i++] = MB_END_TAG;
	p[0] = i * sizeof(*p);

	mbProperty(fd, p);

	return p[5];
}

//===================================================

static unsigned mbUnlockMemory(int fd, unsigned handle)
{
	int i = 1;
	unsigned p[32];

	p[i++] = MB_PROCESS_REQUEST;
	p[i++] = MB_UNLOCK_MEMORY_TAG;
	p[i++] = 4;
	p[i++] = 4;
	p[i++] = handle;
	p[i++] = MB_END_TAG;
	p[0] = i * sizeof(*p);

	mbProperty(fd, p);

	return p[5];
}

//===================================================

static unsigned mbReleaseMemory(int fd, unsigned handle)
{
	int i = 1;
	unsigned p[32];

	p[i++] = MB_PROCESS_REQUEST;
	p[i++] = MB_RELEASE_MEMORY_TAG;
	p[i++] = 4;
	p[i++] = 4;
	p[i++] = handle;
	p[i++] = MB_END_TAG;
	p[0] = i * sizeof(*p);

	mbProperty(fd, p);

	return p[5];
}

//===================================================

static int mbCreate(char *dev)
{
	/* <0 error */

	unlink(dev);

	return mknod(dev, S_IFCHR | 0600, makedev(MB_DEV_MAJOR, 0));
}

//===================================================

static int mbOpen(void)
{
	/* <0 error */

	int fd;

	fd = open(MB_DEV1, 0);

	if (fd < 0)
	{
		mbCreate(MB_DEV2);
		fd = open(MB_DEV2, 0);
	}
	return fd;
}

//===================================================

static void mbClose(int fd)
{
	close(fd);
}

//===================================================

static void initClock(int mainClock)
{
	const unsigned BITS = 10;
	int clockPWM;
	unsigned clkCtl, clkDiv, clkSrc, clkDivI, clkDivF, clkMash, clkBits;
	char *per;
	unsigned micros;

	DBG(DBG_STARTUP, "mainClock=%d", mainClock);

	if (mainClock) micros = gpioCfg.clockMicros;
	else           micros = 1;//PI_WF_MICROS;

	clockPWM = mainClock ^ (gpioCfg.clockPeriph == PI_CLOCK_PCM);

	if (clockPWM)
	{
		clkCtl = CLK_PWMCTL;
		clkDiv = CLK_PWMDIV;
		per = "PWM";
	}
	else
	{
		clkCtl = CLK_PCMCTL;
		clkDiv = CLK_PCMDIV;
		per = "PCM";
	}

	clkSrc = CLK_CTL_SRC_PLLD;
	clkDivI = 50 * micros; /* 10      MHz - 1      MHz */
	clkBits = BITS;        /* 10/BITS MHz - 1/BITS MHz */
	clkDivF = 0;
	clkMash = 0;

	DBG(DBG_STARTUP, "%s PLLD divi=%d divf=%d mash=%d bits=%d",
		per, clkDivI, clkDivF, clkMash, clkBits);

	initHWClk(clkCtl, clkDiv, clkSrc, clkDivI, clkDivF, clkMash);

	if (clockPWM) initPWM(BITS);
	else          initPCM(BITS);

	myGpioDelay(2000);
}

//=======================================================

static void initHWClk
(int clkCtl, int clkDiv, int clkSrc, int divI, int divF, int MASH)
{
	DBG(DBG_INTERNAL, "ctl=%d div=%d src=%d /I=%d /f=%d M=%d",
		clkCtl, clkDiv, clkSrc, divI, divF, MASH);

	/* kill the clock if busy, anything else isn't reliable */

	if (clkReg[clkCtl] & CLK_CTL_BUSY)
	{
		do
		{
			clkReg[clkCtl] = CLK_PASSWD | CLK_CTL_KILL;
		} while (clkReg[clkCtl] & CLK_CTL_BUSY);
	}

	clkReg[clkDiv] = (CLK_PASSWD | CLK_DIV_DIVI(divI) | CLK_DIV_DIVF(divF));

	usleep(10);

	clkReg[clkCtl] = (CLK_PASSWD | CLK_CTL_MASH(MASH) | CLK_CTL_SRC(clkSrc));

	usleep(10);

	clkReg[clkCtl] |= (CLK_PASSWD | CLK_CTL_ENAB);
}

//======================================================

static void initPWM(unsigned bits)
{
	DBG(DBG_STARTUP, "bits=%d", bits);

	/* reset PWM */

	pwmReg[PWM_CTL] = 0;

	myGpioDelay(10);

	pwmReg[PWM_STA] = -1;

	myGpioDelay(10);

	/* set number of bits to transmit */

	pwmReg[PWM_RNG1] = bits;

	myGpioDelay(10);

	//>dmaVirt[0]->periphData = 1;

	/* enable PWM DMA, raise panic and dreq thresholds to 15 */

	pwmReg[PWM_DMAC] = PWM_DMAC_ENAB |
		PWM_DMAC_PANIC(15) |
		PWM_DMAC_DREQ(15);

	myGpioDelay(10);

	/* clear PWM fifo */

	pwmReg[PWM_CTL] = PWM_CTL_CLRF1;

	myGpioDelay(10);

	/* enable PWM channel 1 and use fifo */

	pwmReg[PWM_CTL] = PWM_CTL_USEF1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1;
}

/* ----------------------------------------------------------------------- */

static void initPCM(unsigned bits)
{
	DBG(DBG_STARTUP, "bits=%d", bits);

	/* disable PCM so we can modify the regs */

	pcmReg[PCM_CS] = 0;

	myGpioDelay(1000);

	pcmReg[PCM_FIFO] = 0;
	pcmReg[PCM_MODE] = 0;
	pcmReg[PCM_RXC] = 0;
	pcmReg[PCM_TXC] = 0;
	pcmReg[PCM_DREQ] = 0;
	pcmReg[PCM_INTEN] = 0;
	pcmReg[PCM_INTSTC] = 0;
	pcmReg[PCM_GRAY] = 0;

	myGpioDelay(1000);
//>//>
	pcmReg[PCM_MODE] = PCM_MODE_FLEN(bits - 1); /* # bits in frame */

												/* enable channel 1 with # bits width */

	pcmReg[PCM_TXC] = PCM_TXC_CH1EN | PCM_TXC_CH1WID(bits - 8);

	pcmReg[PCM_CS] |= PCM_CS_STBY; /* clear standby */

	myGpioDelay(1000);

	pcmReg[PCM_CS] |= PCM_CS_TXCLR; /* clear TX FIFO */

	pcmReg[PCM_CS] |= PCM_CS_DMAEN; /* enable DREQ */

	pcmReg[PCM_DREQ] = PCM_DREQ_TX_PANIC(16) | PCM_DREQ_TX_REQ_L(30);

	pcmReg[PCM_INTSTC] = 0b1111; /* clear status bits */

								 /* enable PCM */

	pcmReg[PCM_CS] |= PCM_CS_EN;

	/* enable tx */

	pcmReg[PCM_CS] |= PCM_CS_TXON;

	//>dmaIVirt[0]->periphData = 0x0F;
}

//================================================

static void flushMemory(void)
{
	static int val = 0;

	void *dummy;

	dummy = mmap(
		0, (FLUSH_PAGES*PAGE_SIZE),
		PROT_READ | PROT_WRITE | PROT_EXEC,
		MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE | MAP_LOCKED,
		-1, 0);

	if (dummy == MAP_FAILED)
	{
		DBG(DBG_STARTUP, "mmap dummy failed (%m)");
	}
	else
	{
		memset(dummy, val++, (FLUSH_PAGES*PAGE_SIZE));
		memset(dummy, val++, (FLUSH_PAGES*PAGE_SIZE));
		munmap(dummy, FLUSH_PAGES*PAGE_SIZE);
	}
}

//=================================================

static char * myTimeStamp()
{
	static struct timeval last;
	static char buf[32];
	struct timeval now;

	struct tm tmp;

	gettimeofday(&now, NULL);

	if (now.tv_sec != last.tv_sec)
	{
		localtime_r(&now.tv_sec, &tmp);
		strftime(buf, sizeof(buf), "%F %T", &tmp);
		last.tv_sec = now.tv_sec;
	}

	return buf;
}


//========================================================================
//                              end init
//========================================================================

//========================================================================
//                              start release
//========================================================================

void gpioTerminate(void)
{
	int i;

	DBG(DBG_USER, "");

	if (!libInitialised) return;

	DBG(DBG_STARTUP, "initialised, terminating");

	runState = PI_ENDING;

	gpioMaskSet = 0;

	/* reset DMA */

	if (dmaReg != MAP_FAILED) dmaIn[DMA_CS] = DMA_CHANNEL_RESET;
	if (dmaReg != MAP_FAILED) dmaOut[DMA_CS] = DMA_CHANNEL_RESET;

#ifndef EMBEDDED_IN_VM
	if (gpioCfg.internals & PI_CFG_STATS)
	{
		fprintf(stderr,
			"\n#####################################################\n");
		fprintf(stderr, "pigpio version=%d internals=%X\n",
			DNBGPIO_VERSION, gpioCfg.internals);

		//>fprintf(stderr,
		//>	"micros=%d allocMode=%d dmaInitCbs=%d DMARestarts=%d\n",
		//>	gpioCfg.clockMicros, gpioCfg.memAllocMode,
		//>	gpioStats.dmaInitCbsCount, gpioStats.DMARestarts);

		//>fprintf(stderr,
		//>	"samples %u maxSamples %u maxEmit %u emitFrags %u\n",
		//>	gpioStats.numSamples, gpioStats.maxSamples,
		//>	gpioStats.maxEmit, gpioStats.emitFrags);

		//>fprintf(stderr, "cbTicks %d, cbCalls %u\n",
		//>	gpioStats.cbTicks, gpioStats.cbCalls);

		//>fprintf(stderr, "pipe: good %u, short %u, would block %u\n",
		//>	gpioStats.goodPipeWrite, gpioStats.shortPipeWrite,
		//>	gpioStats.wouldBlockPipeWrite);

		//>fprintf(stderr, "alertTicks %u, lateTicks %u, moreToDo %u\n",
		//>	gpioStats.alertTicks, gpioStats.lateTicks, gpioStats.moreToDo);

		//>for (i = 0; i< TICKSLOTS; i++)
		//>	fprintf(stderr, "%9u ", gpioStats.diffTick[i]);

		fprintf(stderr,
			"\n#####################################################\n\n\n");
	}
#endif

	initReleaseResources();

	fflush(NULL);

	libInitialised = 0;
}

//======================================================

static void initReleaseResources(void)
{
	int i;

	DBG(DBG_STARTUP, "");

	/* shut down running threads */

	//>for (i = 0; i <= PI_MAX_USER_GPIO; i++)
	//>{
	//>	if (gpioISR[i].pth)
	//>	{
	//>		/* destroy thread, unexport GPIO */
	//>
	//>		gpioSetISRFunc(i, 0, 0, NULL);
	//>	}
	//>}

	//>for (i = 0; i <= PI_MAX_TIMER; i++)
	//>{
	//>	if (gpioTimer[i].running)
	//>	{
	//>		/* destroy thread */
	//>
	//>		pthread_cancel(gpioTimer[i].pthId);
	//>		pthread_join(gpioTimer[i].pthId, NULL);
	//>		gpioTimer[i].running = 0;
	//>	}
	//>}

	//>if (pthAlertRunning)
	//>{
	//>	pthread_cancel(pthAlert);
	//>	pthread_join(pthAlert, NULL);
	//>	pthAlertRunning = 0;
	//>}

	//>if (pthFifoRunning)
	//>{
	//>	pthread_cancel(pthFifo);
	//>	pthread_join(pthFifo, NULL);
	//>	pthFifoRunning = 0;
	//>}

	//>if (pthSocketRunning)
	//>{
	//>	pthread_cancel(pthSocket);
	//>	pthread_join(pthSocket, NULL);
	//>	pthSocketRunning = 0;
	//>}

	/* release mmap'd memory */

	if (auxReg != MAP_FAILED) munmap((void *)auxReg, AUX_LEN);
	if (clkReg != MAP_FAILED) munmap((void *)clkReg, CLK_LEN);
	if (dmaReg != MAP_FAILED) munmap((void *)dmaReg, DMA_LEN);
	if (gpioReg != MAP_FAILED) munmap((void *)gpioReg, GPIO_LEN);
	if (pcmReg != MAP_FAILED) munmap((void *)pcmReg, PCM_LEN);
	if (pwmReg != MAP_FAILED) munmap((void *)pwmReg, PWM_LEN);
	if (systReg != MAP_FAILED) munmap((void *)systReg, SYST_LEN);
	if (spiReg != MAP_FAILED) munmap((void *)spiReg, SPI_LEN);

	auxReg = MAP_FAILED;
	clkReg = MAP_FAILED;
	dmaReg = MAP_FAILED;
	gpioReg = MAP_FAILED;
	pcmReg = MAP_FAILED;
	pwmReg = MAP_FAILED;
	systReg = MAP_FAILED;
	spiReg = MAP_FAILED;

	//release DMA CB BUS memory
	if (dmaCBBus != MAP_FAILED)
	{
		munmap(dmaCBBus,
			CB_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS * sizeof(dmaCBPage_t *));
	}
	dmaCBBus = MAP_FAILED;
	//release DMA CB Virt meory
	if (dmaCBVirt != MAP_FAILED)
	{
		for (i = 0; i<CB_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS; i++)
		{
			munmap(dmaCBVirt[i], PAGE_SIZE);
		}

		munmap(dmaCBVirt,
			CB_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS * sizeof(dmaCBPage_t *));
	}
	dmaCBVirt = MAP_FAILED;

	//release DMA OOL BUS memory
	if (dmaOOLBus != MAP_FAILED)
	{
		munmap(dmaOOLBus,
			OOL_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS * sizeof(dmaOOLPage_t *));
	}
	dmaOOLBus = MAP_FAILED;
	//release DMA OOL Virt meory
	if (dmaOOLVirt != MAP_FAILED)
	{
		for (i = 0; i<OOL_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS; i++)
		{
			munmap(dmaOOLVirt[i], PAGE_SIZE);
		}

		munmap(dmaOOLVirt,
			OOL_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS * sizeof(dmaOOLPage_t *));
	}
	dmaOOLVirt = MAP_FAILED;

	//release pagemap memory
	if (dmaPMapBlk != MAP_FAILED)
	{
		for (i = 0; i<PI_WAVE_BLOCKS; i++)
		{
			munmap(dmaPMapBlk[i], PAGES_PER_BLOCK*PAGE_SIZE);
		}

		munmap(dmaPMapBlk, PI_WAVE_BLOCKS * sizeof(dmaCBPage_t *));
	}
	dmaPMapBlk = MAP_FAILED;

	//release mailbox memory
	if (dmaMboxBlk != MAP_FAILED)
	{
		fdMbox = mbOpen();

		for (i = 0; i<PI_WAVE_BLOCKS; i++)
		{
			mbDMAFree(&dmaMboxBlk[PI_WAVE_BLOCKS - i - 1]);
		}

		mbClose(fdMbox);

		munmap(dmaMboxBlk, PI_WAVE_BLOCKS * sizeof(DMAMem_t));
	}
	dmaMboxBlk = MAP_FAILED;

	//>if (inpFifo != NULL)
	//>{
	//>	fclose(inpFifo);
	//>	unlink(PI_INPFIFO);
	//>	inpFifo = NULL;
	//>}

	//>if (outFifo != NULL)
	//>{
	//>	fclose(outFifo);
	//>	unlink(PI_OUTFIFO);
	//>	outFifo = NULL;
	//>}

	if (fdMem != -1)
	{
		close(fdMem);
		fdMem = -1;
	}

	if (fdLock != -1)
	{
		close(fdLock);
		unlink(PI_LOCKFILE);
		fdLock = -1;
	}

	//>if (fdSock != -1)
	//>{
	//>	close(fdSock);
	//>	fdSock = -1;
	//>}

	if (fdPmap != -1)
	{
		close(fdPmap);
		fdPmap = -1;
	}

	if (fdMbox != -1)
	{
		close(fdMbox);
		fdMbox = -1;
	}

	//>gpioStats.DMARestarts = 0;
	//>gpioStats.dmaInitCbsCount = 0;
}

//========================================================================
//                              end release
//========================================================================

//========================================================================
//                              start wf gen
//========================================================================
//NOTES:wfPage[x].WFId = wfPage[] ID of waveForm starting block (-1 for empty)
//		wfPage[x].slots[] = event data storage
//		wfPage[x].lastSlot = WFPages[x].slots[] ID of last set element (-1 for empty)
//		wfPage[x].nextPage = wfPage[] ID of next page for this WFId (-1 for none)
//		wfPage[x].prevPage = WFPage[] ID of previous page for this WFId (-1 for first)
//IF: wfPage[x].WFId == x THEN wfPage[x] is start of WFID x
//IF: wfpage[x].WFId != x THEN wfPage[x] is sub page of wfPage[x].WFId

int WaveAddEvent(waveMake_t EventData, int WFId)
{
	int id = wfVerifyWFId(WFId);
	if (id < 0)
		return PI_NO_WAVEFORM_ID;

	int lpg = wfLastPage(id);
	if (WFPages[lpg].lastSlot >= WF_EVENTS_PER_PAGE -1) //LAST EVENT ON PAGE IS FILLED
	{
		int npg = wfGetNextNew(1);
		if (npg < 0) //no new pages available
			return PI_WAVE_PAGE_OVER;
		WFPages[lpg].nextPage = npg;
		WFPages[npg].WFId = id;
		WFPages[npg].nextPage = -1;
		WFPages[npg].lastSlot = -1;
		WFPages[npg].prevPage = lpg;
		lpg = npg;
	}
	
	WFPages[lpg].lastSlot++;
	waveMakeCopy(&WFPages[lpg].slot[WFPages[lpg].lastSlot], &EventData);
	return id;
}

int WaveAddEventRaw(uint64_t setMask, uint64_t clrMask, uint32_t usDwell, int WFID)
{
	waveMake_t inpt;
	inpt.clrMask = clrMask;
	inpt.setMask = setMask;
	inpt.delayUs = usDwell;
	return WaveAddEvent(inpt, WFID);
}

//============================================================

int WaveAddEvents(waveMake_t * EventDatas, int count, int WFId)
{
	int id = wfVerifyWFId(WFId);
	if (id < 0)
		return PI_NO_WAVEFORM_ID;

	int lpg = wfLastPage(id);
	int i;
	for (i = 0; i < count; i++)
	{
		if (WFPages[lpg].lastSlot >= WF_EVENTS_PER_PAGE -1) //LAST EVENT ON PAGE IS FILLED
		{
			int npg = wfGetNextNew(1);
			if (npg < 0) //no new pages available
				return PI_WAVE_PAGE_OVER;
			WFPages[lpg].nextPage = npg;
			WFPages[npg].WFId = id;
			WFPages[npg].nextPage = -1;
			WFPages[npg].lastSlot = -1;
			WFPages[npg].prevPage = lpg;
			lpg = npg;
		}

		WFPages[lpg].lastSlot++;
		//printf("%d set: %x  clr: %x", i, EventDatas[i].setMask, EventDatas[i].clrMask);
		waveMakeCopy(&WFPages[lpg].slot[WFPages[lpg].lastSlot], &EventDatas[i]);
		//printf("%d out set: %x  clr: %x", i, WFPages[lpg].slot[WFPages[lpg].lastSlot].setMask, WFPages[lpg].slot[WFPages[lpg].lastSlot].clrMask);
	}
	return id;
}

//============================================================

int WaveDelete(int WFId)
{
	//validate wfid
	if (WFId < 0)
		return PI_NO_WAVEFORM_ID;
	int id = wfVerifyWFId(WFId);
	if (id < 0)
		return PI_NO_WAVEFORM_ID;

	int npg = id;
	do
	{
		WFPages[npg].WFId = -1;
		WFPages[npg].lastSlot = -1;
		int hld = WFPages[npg].nextPage;
		WFPages[npg].nextPage = -1;
		WFPages[npg].prevPage = -1;
		npg = hld;
	} while (npg >= 0 && npg < WF_WORKING_PAGES);
	return id;
}

//============================================================

//int WaveInsert(waveMake_t EventData, int WFId, int Location)
//{
//	//validate wfid
//	if (WFId < 0)
//		return PI_NO_WAVEFORM_ID;
//	int id = wfVerifyWFId(WFId);
//	if (id < 0)
//		return PI_NO_WAVEFORM_ID;
//
//	//add page if needed to fit 1 more
//	int lpg = wfLastPage(WFId);
//	if (WFPages[lpg].lastSlot >= WF_EVENTS_PER_PAGE -1)
//	{
//		int npg = wfGetNextNew();
//		if (npg < 0) //no new pages available
//			return PI_WAVE_PAGE_OVER;
//		WFPages[lpg].nextPage = npg;
//		WFPages[npg].WFId = id;
//		WFPages[npg].nextPage = -1;
//		WFPages[npg].lastSlot = 0;
//		WFPages[npg].prevPage = lpg;
//		lpg = npg;
//	}
//
//	int pgs = Location / WF_EVENTS_PER_PAGE;
//	int ipg;
//	int slot = Location % WF_EVENTS_PER_PAGE;
//	int islt;
//	int cpg = lpg;
//	int cslt = WFPages[lpg].lastSlot;
//
//	while (true)
//	{
//
//	}
//
//
//	////advance pages to change loc page
//	//for (int ipg = 0; ipg < pgs; ipg++)
//	//{
//	//	if (WFPages[cpg].nextPage < 0)
//	//		return PI_WAVE_LOC_OVER;
//	//	cpg = WFPages[cpg].nextPage;
//	//}
//	//if (WFPages[cpg].lastSlot < slot)
//	//	return PI_WAVE_LOC_OVER;
//	////store displaced element
//	//waveMake_t hold,tmp;
//	//waveMakeCopy(hold,WFPages[cpg].slot[islt]);
//	////insert new element
//	//waveMakeCopy(WFPages[dpg].slot[islt],EventData);
//	//islt++;
//	////shift remaining elements down 1
//	//while (WFPages[cpg].nextPage >= 0 || islt <= WFPages[cpg].lastSlot)
//	//{
//	//	//next page?
//	//	if (islt >= WF_EVENTS_PER_PAGE)
//	//	{
//	//		cpg = WFPages[cpg].nextPage;
//	//		islt = 0;
//	//	}
//	//	//store displaced element
//	//	waveMakeCopy(tmp,WFPages[cpg].slot[islt]);
//	//	//insert last element
//	//	waveMakeCopy(WFPages[cpg].slot[islt],hold);
//	//	waveMakeCopy(hold,tmp);
//	//	islt++;
//	//}
//	////last insert
//	//waveMakeCopy(WFPages[cpg].slot[islt],hold);
//	//WFPages[cpg].lastSlot = islt;
//	return WFId;
//}

//============================================================

int WaveInsert(waveMake_t * EventData, int Len, int WFId, int Location)
{
	if (Len < 1)
		return -1;
	//validate wfid
	if (WFId < 0)
		return PI_NO_WAVEFORM_ID;
	int id = wfVerifyWFId(WFId);
	if (id < 0)
		return PI_NO_WAVEFORM_ID;

	int lpg = wfLastPage(WFId);
	int adslt = WFPages[lpg].lastSlot + Len;
	int adpg = adslt / WF_EVENTS_PER_PAGE;
	adslt = adslt % WF_EVENTS_PER_PAGE;

	//need more pages?
	if (adpg > 0)
	{
		//enought empties?
		int npg = wfGetNextNew(adpg);
		if (npg < 0)
			return PI_WAVE_PAGE_OVER;
		//add new pages
		int i;
		for (i = 0; i < adpg && npg >= 0; i++)
		{
			WFPages[lpg].nextPage = npg;
			WFPages[npg].WFId = WFId;
			WFPages[npg].lastSlot = -1;
			WFPages[npg].nextPage = -1;
			lpg = npg;
		}
	}

	int pgs = Location / WF_EVENTS_PER_PAGE;
	int ipg = 0;
	int slot = Location % WF_EVENTS_PER_PAGE;
	int islt = slot;
	int cpg = WFId;
	//advance pages to change loc page
	for (ipg = 0; ipg < pgs; ipg++)
	{
		if (WFPages[cpg].nextPage < 0)
			return PI_WAVE_LOC_OVER;
		cpg = WFPages[cpg].nextPage;
	}
	if (WFPages[cpg].lastSlot < slot)
		return PI_WAVE_LOC_OVER;
}

//============================================================

static int wfGetNextNew(int count)
{
	int first = -1;
	int cnt = 0;
	int i;
	for (i = 0; i < WF_WORKING_PAGES && cnt < count; i++)
	{
		if (WFPages[i].WFId < 0)
		{
			cnt++;
			if (first < 0)
				first = i;
		}
	}
	if (cnt >= count)
		return first;
	return -1;
}

//============================================================

static int wfVerifyWFId(int WFId)
{
	if (WFId < 0)
	{
		int id = wfGetNextNew(1);
		WFPages[id].WFId = id;
		WFPages[id].lastSlot = -1;
		WFPages[id].nextPage = -1;
		WFPages[id].prevPage = -1;
		return id;
	}
		
	if (WFPages[WFId].WFId == WFId)
		return WFId;
	return -1;
}

//============================================================

static int wfLastPage(int WFId)
{
	if (WFId < 0)
		return -1;
	int pg = WFId;
	while (WFPages[pg].nextPage >= 0 && WFPages[pg].nextPage != pg)
		pg = WFPages[pg].nextPage;
	return pg;
}

//===========================================================

int WaveVerifyWave(int WFID)
{
	if (WFID < 0)
		return -1;
	
	int pg = WFID;
	uint64_u s,c;

	if (WFPages[pg].WFId != WFID)
		return -1;

	while (pg >= 0 && WFPages[pg].WFId == WFID)
	{
		int i;
		for (i = 0; i <= WFPages[pg].lastSlot; i++)
		{
			s.U64 = WFPages[pg].slot[i].setMask;
			c.U64 = WFPages[pg].slot[i].clrMask;
			printf(":PG %d Slot %d Set 0x%.8X%.8X Clr 0x%.8X%.8X Dla %dus\n", pg, i, s.U32[1], s.U32[0], c.U32[1], c.U32[0], WFPages[pg].slot[i].delayUs);
		}
		pg = WFPages[pg].nextPage;
	}
	return 0;
}

//========================================================================
//                              end wf gen
//========================================================================

//========================================================================
//                              start wf run
//========================================================================
//NOTE: dmaVirt to write data to dma data storage
//NOTE: DMA system uses dmaBus mem locations for data transfers

#define cbP(x) ((x)/CBS_PER_DPAGE)
#define cbS(x) ((x)%CBS_PER_DPAGE)
#define oolP(x) ((x)/OOL_PER_DPAGE)
#define oolS(x) ((x)%OOL_PER_DPAGE)

//todo: make this wfid[] in
int WavePrepRun(int WFId, int Repeat)
{
	//validate wfid
	if (WFId < 0)
		return PI_NO_WAVEFORM_ID;
	int id = wfVerifyWFId(WFId);
	if (id < 0)
		return PI_NO_WAVEFORM_ID;

	int wfCur = WFId; //holds curent wfPage[] id
	int wfSlt = 0;	  //holds curent wfPage[wfCur].slot[] id

	//stop  current run

	const int dmaMaxCB = CB_PG_PER_DMA_BLOCK * CBS_PER_DPAGE * PI_WAVE_BLOCKS;
	int dmaCB = 0;
	rawCbs_t *dmaCur = &dmaCBVirt[cbP(dmaCB)]->slot[cbS(dmaCB)];
	int dmaOOL = 0;
	
	//start with a 20us delay
	if (gpioCfg.clockPeriph != PI_CLOCK_PCM)
	{
		dmaCur->info = NORMAL_DMA | TIMED_DMA(2);
		dmaCur->dst = PCM_TIMER;
		//data sent to pcm FIFO for delay timing
		dmaOOLVirt[oolP(dmaOOL)]->slots[oolS(dmaOOL)] = 0x0F;
	}
	else
	{
		dmaCur->info = NORMAL_DMA | TIMED_DMA(5);
		dmaCur->dst = PWM_TIMER;
		//data sent to pwm FIFO for delay timing
		dmaOOLVirt[oolP(dmaOOL)]->slots[oolS(dmaOOL)] = 1;

	}
	dmaOOL++;
	dmaCur->src = (uint32_t)(&dmaOOLBus[0]->slots[0]);
	dmaCur->length = BYTS_P_REG * 20 ; /* 20 micros delay */
	dmaCB++;
	dmaCur->next = (uint32_t)(&dmaCBBus[cbP(dmaCB)]->slot[cbS(dmaCB)]);

	int more = WFPages[wfCur].lastSlot >= wfSlt;
	uint64_t mask;
	while (dmaCB < dmaMaxCB && more)
	{
		//Set mask used
		mask = WFPages[wfCur].slot[wfSlt].setMask;
		if (mask)
		{
			dmaCur = &dmaCBVirt[cbP(dmaCB)]->slot[cbS(dmaCB)];
			if (mask & 0xFFFFFFFF00000000 && mask & 0x00000000FFFFFFFF) //both port 0 & 1 used
			{
				dmaOOLVirt[oolP(dmaOOL)]->slots[oolS(dmaOOL)] = (uint32_t)(mask & 0xFFFFFFFF);
				dmaOOLVirt[oolP(dmaOOL + 1)]->slots[oolS(dmaOOL + 1)] = (uint32_t)(mask >> 32 & 0xFFFFFFFF);
				dmaCur->src = (uint32_t)(&dmaOOLBus[oolP(dmaOOL)]->slots[oolS(dmaOOL)]);
				dmaCur->dst = ((GPIO_BASE + (GPSET0 * 4)) & 0x00ffffff) | PI_PERI_BUS;
				dmaCur->info = INCR_DMA;
				dmaCur->length = 8;
				dmaOOL += 2;
				
			}
			else if (mask & 0x00000000FFFFFFFF) //only port 0 used
			{
				dmaOOLVirt[oolP(dmaOOL)]->slots[oolS(dmaOOL)] = (uint32_t)(mask & 0xFFFFFFFF);
				dmaCur->src = (uint32_t)(&dmaOOLBus[oolP(dmaOOL)]->slots[oolS(dmaOOL)]);
				dmaCur->dst = ((GPIO_BASE + (GPSET0 * 4)) & 0x00ffffff) | PI_PERI_BUS;
				dmaCur->length = 4;
				dmaCur->info = NORMAL_DMA;
				dmaOOL++;
			}
			else //only port 1 used
			{
				dmaOOLVirt[oolP(dmaOOL)]->slots[oolS(dmaOOL)] = (uint32_t)(mask >> 32 & 0xFFFFFFFF);
				dmaCur->src = (uint32_t)(&dmaOOLBus[oolP(dmaOOL)]->slots[oolS(dmaOOL)]);
				dmaCur->dst = ((GPIO_BASE + (GPSET1 * 4)) & 0x00ffffff) | PI_PERI_BUS;
				dmaCur->length = 4;
				dmaCur->info = NORMAL_DMA;
				dmaOOL++;
			}
			dmaCB++;
			dmaCur->next = (uint32_t)(&dmaCBBus[cbP(dmaCB)]->slot[cbS(dmaCB)]);
		}

		//clear mask used
		mask = WFPages[wfCur].slot[wfSlt].clrMask;
		if (mask)
		{
			dmaCur = &dmaCBVirt[cbP(dmaCB)]->slot[cbS(dmaCB)];
			if (mask & 0xFFFFFFFF00000000 && mask & 0x00000000FFFFFFFF) //both port 0 & 1 used
			{
				dmaOOLVirt[oolP(dmaOOL)]->slots[oolS(dmaOOL)] = (uint32_t)(mask & 0xFFFFFFFF);
				dmaOOLVirt[oolP(dmaOOL + 1)]->slots[oolS(dmaOOL + 1)] = (uint32_t)(mask >> 32 & 0xFFFFFFFF);
				dmaCur->src = (uint32_t)(&dmaOOLBus[oolP(dmaOOL)]->slots[oolS(dmaOOL)]);
				dmaCur->dst = ((GPIO_BASE + (GPCLR0 * 4)) & 0x00ffffff) | PI_PERI_BUS;
				dmaCur->info = INCR_DMA;
				dmaCur->length = 8;
				dmaOOL += 2;

			}
			else if (mask & 0x00000000FFFFFFFF) //only port 0 used
			{
				dmaOOLVirt[oolP(dmaOOL)]->slots[oolS(dmaOOL)] = (uint32_t)(mask & 0xFFFFFFFF);
				dmaCur->src = (uint32_t)(&dmaOOLBus[oolP(dmaOOL)]->slots[oolS(dmaOOL)]);
				dmaCur->dst = ((GPIO_BASE + (GPCLR0 * 4)) & 0x00ffffff) | PI_PERI_BUS;
				dmaCur->length = 4;
				dmaCur->info = NORMAL_DMA;
				dmaOOL++;
			}
			else //only port 1 used
			{
				dmaOOLVirt[oolP(dmaOOL)]->slots[oolS(dmaOOL)] = (uint32_t)(mask >> 32 & 0xFFFFFFFF);
				dmaCur->src = (uint32_t)(&dmaOOLBus[oolP(dmaOOL)]->slots[oolS(dmaOOL)]);
				dmaCur->dst = ((GPIO_BASE + (GPCLR1 * 4)) & 0x00ffffff) | PI_PERI_BUS;
				dmaCur->length = 4;
				dmaCur->info = NORMAL_DMA;
				dmaOOL++;
			}
			dmaCB++;
			dmaCur->next = (uint32_t)(&dmaCBBus[cbP(dmaCB)]->slot[cbS(dmaCB)]);
		}

		//delay used
		if (WFPages[wfCur].slot[wfSlt].delayUs)
		{
			uint32_t delayLeft = WFPages[wfCur].slot[wfSlt].delayUs;
			while (delayLeft > 0)
			{
				dmaCur = &dmaCBVirt[cbP(dmaCB)]->slot[cbS(dmaCB)];

				if (gpioCfg.clockPeriph != PI_CLOCK_PCM)
				{
					dmaCur->info = NORMAL_DMA | TIMED_DMA(2);
					dmaCur->dst = PCM_TIMER;
				}
				else
				{
					dmaCur->info = NORMAL_DMA | TIMED_DMA(5);
					dmaCur->dst = PWM_TIMER;
				}

				dmaCur->length = BYTS_P_REG * delayLeft;
				if ((gpioCfg.DMAsecondaryChannel >= DMA_LITE_FIRST) &&
					(delayLeft > DMA_LITE_MAX))
				{
					dmaCur->length = DMA_LITE_MAX;
				}
				delayLeft -= (dmaCur->length / BYTS_P_REG);

				dmaCur->src = (uint32_t)(&dmaOOLBus[0]->slots[0]);
				dmaCB++;
				dmaCur->next = (uint32_t)(&dmaCBBus[cbP(dmaCB)]->slot[cbS(dmaCB)]);
			}
		}

		wfSlt++;
		if (wfSlt >= WF_EVENTS_PER_PAGE && WFPages[wfCur].nextPage >= 0)
		{
			wfSlt = 0;
			wfCur = WFPages[wfCur].nextPage;
		}
		more = WFPages[wfCur].lastSlot >= wfSlt;
	}

	//repitition
	if (Repeat < 0) //repeat inf
		dmaCur->next = (uint32_t)(&dmaCBBus[0]->slot[1]);
	else if (Repeat > 1) //repeat x
	{
		//add 1 dma per repeat
		//repeat by changing last base wf CB.next to consecutive 'repeat' CBs
//		rawCbs_t *dmaLast = dmaCur;

		int refresh = dmaCB;
		int i;
		int dmaCBL = dmaCB - 1;
		for (i = 0; i < Repeat && dmaCB < dmaMaxCB; i++)
		{
			
			dmaCur = &dmaCBVirt[cbP(dmaCB)]->slot[cbS(dmaCB)];
			dmaOOLVirt[oolP(dmaOOL)]->slots[oolS(dmaOOL)] = (uint32_t)&dmaCBBus[cbP(dmaCB + 1)]->slot[cbS(dmaCB + 1)];
			dmaCur->src = (uint32_t)(&dmaOOLBus[oolP(dmaOOL)]->slots[oolS(dmaOOL)]);
			dmaCur->dst = (uint32_t)(&dmaCBBus[cbP(dmaCBL)]->slot[cbS(dmaCBL)].next);    //(&dmaLast->next);
			dmaCur->info = NORMAL_DMA;
			dmaCur->length = 4;
			dmaCur->next = (uint32_t)(&dmaCBBus[0]->slot[1]);
			dmaCB++;
			dmaOOL++;
		}
		//reset last base WF CB.next to orig
		dmaOOL--;
		dmaOOLVirt[oolP(dmaOOL)]->slots[oolS(dmaOOL)] = (uint32_t)&dmaCBBus[cbP(refresh)]->slot[cbS(refresh)];
		dmaOOL++;
		dmaCur->next = 0;
	}
	else //run once
		dmaCur->next = 0;
	return dmaCB;
}

void WaveVerifyDMA()
{
	rawCbs_t * dmaCur = &dmaCBVirt[0]->slot[0];
	uint32_t hld;
	int cont = 1;
	int i = 0;
	while (cont)
	{
		printf("ADR: @0x%.8X ", &dmaCBBus[cbP(i)]->slot[cbS(i)]);
		printf("src: ");
		hld = dmaCur->src;
		printf("%.8X", hld);
		printf(" dst: ");
		hld = dmaCur->dst;
		printf("%.8X", hld);
		printf(" nxt: ");
		hld = dmaCur->next;
		printf("%.8X", hld);
		printf(" len: ");
		hld = dmaCur->length;
		printf("%d\n", hld);
		
		cont = dmaCur->next > 0;
		if (cont)
		{
			i++;
			dmaCur = &dmaCBVirt[cbP(i)]->slot[cbS(i)];
		}
	} 
}

void WaveStartRun()
{
	DBG(DBG_STARTUP, "");
	
	initClock(0);

	dmaOut[DMA_CS] = DMA_CHANNEL_RESET;

	dmaOut[DMA_CS] = DMA_INTERRUPT_STATUS | DMA_END_FLAG;

	dmaOut[DMA_CONBLK_AD] = (uint32_t)(&dmaCBBus[0]->slot[0]);

	/* clear READ/FIFO/READ_LAST_NOT_SET error bits */

	dmaOut[DMA_DEBUG] = DMA_DEBUG_READ_ERR |
		DMA_DEBUG_FIFO_ERR |
		DMA_DEBUG_RD_LST_NOT_SET_ERR;


	dmaOut[DMA_CS] = DMA_WAIT_ON_WRITES |
		DMA_PANIC_PRIORITY(8) |
		DMA_PRIORITY(8) |
		DMA_ACTIVATE;
}

void WaveStopRun()
{
	DBG(DBG_USER, "");

	CHECK_INITED;

	dmaOut[DMA_CS] = DMA_CHANNEL_RESET;
	dmaOut[DMA_CONBLK_AD] = 0;
}

int WaveRunning()
{
	return dmaOut[DMA_CONBLK_AD] != 0;
}

//========================================================================
//                              end wf run
//========================================================================

//========================================================================
//                              start gpio
//========================================================================

unsigned gpioMode(unsigned gpio, int mode)
{
	int reg, shift, old_mode;

	DBG(DBG_USER, "gpio=%d mode=%d", gpio, mode);

	CHECK_INITED;

	if (gpio > PI_MAX_GPIO)
		SOFT_ERROR(PI_BAD_GPIO, "bad gpio (%d)", gpio);

	if (mode > 7)
		SOFT_ERROR(PI_BAD_MODE, "gpio %d, bad mode (%d)", gpio, mode);

	reg = gpio / 10;
	shift = (gpio % 10) * 3;

	old_mode = (gpioReg[reg] >> shift) & 7;
	if(mode >= 0 && mode != old_mode)
		gpioReg[reg] = (gpioReg[reg] & ~(7 << shift)) | (mode << shift);
	return (gpioReg[reg] >> shift) & 7;
}

uint64_t gpioLevels()
{
	uint64_t otpt = gpioReg[GPLEV1];
	otpt = otpt << 32;
	otpt |= gpioReg[GPLEV0];
	return otpt;
}

void gpioSetHigh(uint64_t pinMap)
{
	gpioReg[GPSET0] = (uint32_t)(pinMap & 0xFFFFFFFF);
	gpioReg[GPSET1] = (uint32_t)(pinMap >> 32 & 0xFFFFFFFF);
}

void gpioSetLow(uint64_t pinMap)
{
	gpioReg[GPCLR0] = (uint32_t)(pinMap & 0xFFFFFFFF);
	gpioReg[GPCLR1] = (uint32_t)(pinMap >> 32 & 0xFFFFFFFF);
}

//========================================================================
//                              end gpio
//========================================================================

//========================================================================
//                              start spi
//========================================================================

//#define SPI_INIT_SCKL_PHASE(X) ((X)&1)
//#define SPI_INIT_SCLK_REST(X) (((X)&1)<<1)
//#define SPI_INIT_DNE_INTRPT(X) (((X)&1)<<2)
//#define SPI_INIT_RX_FULL_INTRPT(X) (((X)&1)<<3)
//#define SPI_INIT_CE0_LVL(X) (((X)&1)<<4)
//#define	SPI_INIT_CE1_LVL(X) (((X)&1)<<5)
//
//#define SPI_PIN_USE_CE0(X) ((X)&1)
//#define SPI_PIN_CE0_P36(X) (((X)&1)<<2)
//#define SPI_PIN_USE_CE1(X) (((X)&1)<<3)
//#define SPI_PIN_CE1_P35(X) (((X)&1)<<4)
//#define SPI_PIN_MOSI_P37(X) (((X)&1)<<5)
//#define SPI_PIN_MISO_P38(X) (((X)&1)<<6)
//#define SPI_PIN_SCLK_P39(X) (((X)&1)<<7)

void spiInit(uint8_t initFlags, uint8_t pinFlags)
{

	//INIT
	//0x01 sclk phase
	//0x02 sclk rest state
	//0x04 use done interupt
	//0x08 use rx full interupt
	//0x10 ce0 active level
	//0x20 ce1 active level
	uint32_t cs = (initFlags & 3) << 2;
	if (initFlags & SPI_INIT_DNE_INTRPT(1))
		cs |= (1 << 9);
	fflush(stdout);
	if (initFlags & SPI_INIT_RX_FULL_INTRPT(1))
		cs |= (1 << 10);


	//if (initFlags & SPI_INIT_CE0_LVL(1))
		//cs |= (1 << 21);
	//if (initFlags & SPI_INIT_CE1_LVL(1))
		//cs |= (1 << 22);
	spiReg[SPI_CS] = cs;
	if (pinFlags & SPI_PIN_USE_CE0(1))
		gpioMode((pinFlags & SPI_PIN_CE0_P36(1) ? 36 : 8), PI_ALT0);

	if (pinFlags & SPI_PIN_USE_CE1(1))
		gpioMode((pinFlags & SPI_PIN_CE1_P35(1) ? 35 : 7), PI_ALT0);




	gpioMode((pinFlags & SPI_PIN_MOSI_P37(1) ? 37 : 9), PI_ALT0);
	gpioMode((pinFlags & SPI_PIN_MISO_P38(1) ? 38 : 10), PI_ALT0);
	gpioMode((pinFlags & SPI_PIN_SCLK_P39(1) ? 39 : 11), PI_ALT0);

}

//#define SPI_SEND_USE_CE1(X) ((X)&1)
//#define SPI_SEND_CE_POLARITY(X) (((X)&1)<<1)
//#define SPI_SEND_LOSSI_MODE(X) (((X)&1)<<2)
//#define SPI_SEND_BIDIR_MODE(X) (((X)&1)<<3)

void spiTransfer(unsigned speed, uint8_t sendFlags, uint8_t * txBuff, uint8_t * rxBuff, unsigned txLen, unsigned rxLen)
{
	uint32_t cs = spiReg[SPI_CS] & 0x300cf0c;
	int i,r;
	uint32_t theVoid;

	//printf("T&: 0x%.8X T#: %d  R&: 0x%.8X R#: %d\n", txBuff, txLen, rxBuff, rxLen);

	if (sendFlags & SPI_SEND_USE_CE1(1))
	{
		cs |= 1;
		if (sendFlags & SPI_SEND_CE_POLARITY(1))
			cs |= 0x400040;
	}
	else if (sendFlags & SPI_SEND_CE_POLARITY(1))
		cs |= 0x0200040;
	if (sendFlags & SPI_SEND_LOSSI_MODE(1))
		cs |= (1 << 13);

	//printf("CS: 0x%.8X\n",cs);

	spiReg[SPI_CS] = cs | (3<<4); //setup to standby (clear tx/rx fifos)
	spiReg[SPI_CLK] = 250000000 / speed;

	spiReg[SPI_CS] |= (1 << 7); //transaction in progress

	if (sendFlags & SPI_SEND_BIDIR_MODE(1)) // 3 wire mode
	{
		for (i = 0; i < txLen; i++)
		{
			while (!(spiReg[SPI_CS] & 0x40000)); //tx fifo full
			spiReg[SPI_FIFO] = txBuff[i];
			while (spiReg[SPI_CS] & 0x20000) //rx fifo has 'data'
				theVoid = spiReg[SPI_FIFO];
		}

		spiReg[SPI_CS] |= (1 << 12); //set REN for recieve mode

		for (r = 0; r < rxLen;)
		{
			while (!(spiReg[SPI_CS] & 0x40000)); //tx fifo full
			spiReg[SPI_FIFO] = 0; // dummy write
			while ((spiReg[SPI_CS] & 0x20000) && (r < rxLen)) //rx fifo has data
			{
				rxBuff[r] = spiReg[SPI_FIFO];
				r++;
			}
		}
	}
	else // 4 wire mode
	{
		r = 0;
		for (i = 0; i < txLen || r < rxLen; i++)
		{
			while (!(spiReg[SPI_CS] & 0x40000)); //tx fifo full

			if (txBuff && i < txLen)
			{
				//printf("TX: 0x%.2X\n", txBuff[i]);
				spiReg[SPI_FIFO] = txBuff[i];
			}
			else
			{
				//printf("TX: 0x%.2X (trash)\n", 0);
				spiReg[SPI_FIFO] = 0;
			}

			while (!(spiReg[SPI_CS] & 0x20000)); //rx fifo has data
			//{
				
				if (r < rxLen)
				{
					if (rxBuff)
					{
						rxBuff[r] = spiReg[SPI_FIFO];
						//printf("RX: 0x%.2X\n", rxBuff[r]);
					}
					r++;
				}
				else
				{
					theVoid = spiReg[SPI_FIFO];
					//printf("RX: 0x%.2X (void)\n", theVoid);
				}
			//}
		}
	}

	while (!(spiReg[SPI_CS] & (1 << 16))); // wait for done

	spiReg[SPI_CS] = cs;
}

//========================================================================
//                              end spi
//========================================================================

void spiSoft(int cs,unsigned clkHz, uint8_t * txBuff, uint8_t * rxBuff, int txLen, int rxLen)
{
	gpioMode(7, PI_OUTPUT); //ce1
#define S_PIN_CE1 0x80
	gpioMode(8, PI_OUTPUT); //ce0
#define S_PIN_CE0 0x100
	gpioMode(9, PI_INPUT); //input
#define S_PIN_IN 0x200
	gpioMode(10, PI_OUTPUT); //output
#define S_PIN_OUT 0x400
	gpioMode(11, PI_OUTPUT); //clk
#define S_PIN_CLK 0x800

	unsigned dly = 250000 / clkHz; //quater cycle int us
	

	int t = 0;
	int r = 0;
	uint32_t tme;

	gpioSetLow(S_PIN_IN | S_PIN_CLK);
	gpioSetHigh(S_PIN_CE1 | S_PIN_CE0);
	tme = dly * 8;
	usleep(tme); //2 cycle delay after pin setup

	if (cs == 0)
		gpioSetLow(S_PIN_CE0);
	else
		gpioSetLow(S_PIN_CE1);

	usleep(tme); // 2 cycle delay after cs activeate

	tme = dly * 2;
	
	for (; t < txLen || r < rxLen;)
	{
		int b = 0;

		if (r < rxLen)
			rxBuff[r] = 0;

		for (b = 0; b < 8; b++)
		{
			//delay quater cycle
			usleep(dly); 

			//set next bit to out
			if (t < txLen)
			{
				if (txBuff && (txBuff[t] & (0x80 >> b)))
					gpioSetHigh(S_PIN_OUT);
				else
					gpioSetLow(S_PIN_OUT);
			}
			else
				gpioSetLow(S_PIN_OUT);

			//delay quarter cycle
			usleep(dly);

			//clk high
			gpioSetHigh(S_PIN_CLK);

			//read if needed
			if (r < rxLen)
			{
				if (rxBuff && (gpioLevels() & S_PIN_IN))
					rxBuff[r] |= (0x80 >> b);
			}

			//delay half cycle
			usleep(tme);
			gpioSetLow(S_PIN_CLK);
		}
		if (t < txLen)
			t++;
		if (r < rxLen)
			r++;
	}

	//delay 2 cycles
	tme = dly * 8;
	usleep(tme);

	if (cs == 0)
		gpioSetHigh(S_PIN_CE0);
	else
		gpioSetHigh(S_PIN_CE1);

}

//========================================================================
//                              end spi soft
//========================================================================

//========================================================================
//                            start uart 1
//========================================================================
 
#define UART1_AUX_ENAB 1
#define UART1_AUX_ENAB_MASK 0x01

#define UART1_IO 16

#define UART1_INTRPT_SET 17
#define UART1_INTRPT_GET 18
#define UART1_CNTRL 19
#define UART1_RTS 20
#define UART1_STATUS 21
#define UART1_STATUS_RX_DATA 0x1
#define UART1_STATUS_RX_OVR 0x2
#define UART1_CTS 22
#define UART1_SCRATCH 23
#define UART1_AUTO_FLOW 24
#define UART1_STATUS_2 25
#define UART1_STATUS_2_TX_FULL 0x20

#define UART1_BAUD 26

#define UART1_TX_PIN 40
#define UAER1_RX_PIN 41

int uart1Init(uint32_t baud)
{
	if (!auxReg || !gpioReg)
		return -1;

	gpioMode(UART1_TX_PIN, PI_OUTPUT);//gpioMode(14, PI_ALT5);
	gpioMode(UAER1_RX_PIN, PI_ALT5);

	gpioSetHigh(1LL << UART1_TX_PIN);
	auxReg[UART1_AUX_ENAB] |= UART1_AUX_ENAB_MASK;


	auxReg[UART1_BAUD] = (250000000 / baud / 8) -1;
	auxReg[UART1_CNTRL] = 0x1; //8 bit mode
	auxReg[UART1_AUTO_FLOW] = 0x1; //enable rx (tx now bitbanged)
	return 1;
}

int debugTiming(int delay)
{
	if (!auxReg)
		return -1;
	if (!systReg)
		return -2;

	printf("baud Reg: %d\n", auxReg[UART1_BAUD]);
	float bitw = 1000000.0 / (250000000.0 / 8.0 / (auxReg[UART1_BAUD] + 1));
	printf("buad us: %f\n", bitw);
	uint32_t start, stop, cur;
	start = systReg[SYST_CLO];
	float dwell = bitw * delay;
	stop = start + (uint32_t)dwell;
	cur = systReg[SYST_CLO];
	printf("dwell us: %f\n", dwell);
	printf("start clo: %u\n", start);
	printf("stop  clo: %u\n", stop);
	printf("cur   clo: %u\n", cur);
	printf("aft P clo: %u\n", systReg[SYST_CLO]);
	return 1;
}

/////////////////////////////////////////////////////
int debugClock()
{
	initClock(0);
	return 1;
}

int debugDMA(int delay, int count)
{
	if (!dmaReg)
		return -1;
	

	uint32_t dstSet, dstClr;
	dstSet = ((GPIO_BASE + (GPSET1 * 4)) & 0x00ffffff) | PI_PERI_BUS;
	dstClr = ((GPIO_BASE + (GPCLR1 * 4)) & 0x00ffffff) | PI_PERI_BUS;

	uint32_t dwlInf, dwlDst;
	if (gpioCfg.clockPeriph != PI_CLOCK_PCM)
	{
		dwlInf = NORMAL_DMA | TIMED_DMA(2);
		dwlDst = PCM_TIMER;
		pcmReg[PCM_CS] |= 0x8; // clear FIFO
		dmaOOLVirt[20]->slots[0] = 0x0F;
	}
	else
	{
		dwlInf = NORMAL_DMA | TIMED_DMA(5);
		dwlDst = PWM_TIMER;
		pwmReg[PWM_CTL] |= 0x40; //clear fifo
		dmaOOLVirt[20]->slots[0] = 0x01;
	}

	dmaOOLVirt[20]->slots[1] = 1 << (40 - 32);

	int dmaPg = CB_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS - 1;
	int dmaSl = 0;

	//set low
	dmaCBVirt[dmaPg]->slot[dmaSl].info = NORMAL_DMA;
	dmaCBVirt[dmaPg]->slot[dmaSl].src = (uint32_t)&dmaOOLBus[20]->slots[1];
	dmaCBVirt[dmaPg]->slot[dmaSl].dst = dstClr;
	dmaCBVirt[dmaPg]->slot[dmaSl].length = 4;
	dmaCBVirt[dmaPg]->slot[dmaSl].stride = 0;
	dmaCBVirt[dmaPg]->slot[dmaSl].next = (uint32_t)&dmaCBBus[dmaPg]->slot[dmaSl + 1];
	dmaSl++;
	int i;
	for (i = 0; i < count; i++)
	{
		//set high
		dmaCBVirt[dmaPg]->slot[dmaSl].info = NORMAL_DMA;
		dmaCBVirt[dmaPg]->slot[dmaSl].src = (uint32_t)&dmaOOLBus[20]->slots[1];
		dmaCBVirt[dmaPg]->slot[dmaSl].dst = dstSet;
		dmaCBVirt[dmaPg]->slot[dmaSl].length = 4;
		dmaCBVirt[dmaPg]->slot[dmaSl].stride = 0;
		dmaCBVirt[dmaPg]->slot[dmaSl].next = (uint32_t)&dmaCBBus[dmaPg]->slot[dmaSl + 1];
		dmaSl++;


		//delay
		dmaCBVirt[dmaPg]->slot[dmaSl].info = dwlInf;
		dmaCBVirt[dmaPg]->slot[dmaSl].dst = dwlDst;
		dmaCBVirt[dmaPg]->slot[dmaSl].length = BYTS_P_REG * i+1;
		dmaCBVirt[dmaPg]->slot[dmaSl].src = (uint32_t)&dmaOOLBus[20]->slots[0];
		dmaCBVirt[dmaPg]->slot[dmaSl].stride = 0;
		dmaCBVirt[dmaPg]->slot[dmaSl].next = (uint32_t)&dmaCBBus[dmaPg]->slot[dmaSl + 1];
		dmaSl++;

		//set low
		dmaCBVirt[dmaPg]->slot[dmaSl].info = NORMAL_DMA;
		dmaCBVirt[dmaPg]->slot[dmaSl].src = (uint32_t)&dmaOOLBus[20]->slots[1];
		dmaCBVirt[dmaPg]->slot[dmaSl].dst = dstClr;
		dmaCBVirt[dmaPg]->slot[dmaSl].length = 4;
		dmaCBVirt[dmaPg]->slot[dmaSl].stride = 0;
		dmaCBVirt[dmaPg]->slot[dmaSl].next = (uint32_t)&dmaCBBus[dmaPg]->slot[dmaSl + 1];
		dmaSl++;
	}

	//set high
	dmaCBVirt[dmaPg]->slot[dmaSl].info = NORMAL_DMA;
	dmaCBVirt[dmaPg]->slot[dmaSl].src = (uint32_t)&dmaOOLBus[20]->slots[1];
	dmaCBVirt[dmaPg]->slot[dmaSl].dst = dstSet;
	dmaCBVirt[dmaPg]->slot[dmaSl].length = 4;
	dmaCBVirt[dmaPg]->slot[dmaSl].stride = 0;
	dmaCBVirt[dmaPg]->slot[dmaSl].next = 0;
	dmaSl++;



	//dmaOut[DMA_CS] = DMA_CHANNEL_RESET;
	//dmaOut[DMA_CS] = DMA_INTERRUPT_STATUS | DMA_END_FLAG;
	dmaOut[DMA_CONBLK_AD] = (uint32_t)(&dmaCBBus[dmaPg]->slot[0]);
	/* clear READ/FIFO/READ_LAST_NOT_SET error bits */
	dmaOut[DMA_DEBUG] = DMA_DEBUG_READ_ERR | DMA_DEBUG_FIFO_ERR | DMA_DEBUG_RD_LST_NOT_SET_ERR;
	//start dma
	dmaOut[DMA_CS] = DMA_WAIT_ON_WRITES | DMA_PANIC_PRIORITY(8) | DMA_PRIORITY(8) | DMA_ACTIVATE;
	return 1;
	
}
/////////////////////////////////////////////////////

int uart1TxSoft(char * Buff, int SendLen)
{
	if (!Buff)
		return -3;
	if (!auxReg)
		return -1;
	if (!(auxReg[UART1_AUX_ENAB] & UART1_AUX_ENAB_MASK))
		return -2;
	////==== use hardware (HW TX outputs incorrectly) ====
	//int i;
	//for (i = 0; i < SendLen; i++)
	//{
	//	while (auxReg[UART1_STATUS_2] & UART1_STATUS_2_TX_FULL); //WAIT FOR ROOM IN FIFO

	//	auxReg[UART1_IO] = *(Buff + i);
	//}
	//return i;

	//==== use bit bang method in place of hardware ====
	if (!gpioReg)
		return -1;

	float bitw = 1000000.0 / (250000000.0 / 8.0 / (auxReg[UART1_BAUD] + 1)); //calc uS per bit off of baud stored
	float dwell = 0;
	uint8_t state = 0;

	uint32_t cloStart, cloStop;
	int i, b;
	
	for (i = 0; i < SendLen; i++)
	{
		//start bit
		state = 0;
		dwell = 0 + bitw;
		cloStart = systReg[SYST_CLO];
		gpioSetLow(1LL << UART1_TX_PIN);

		for (b = 0; b < 8; b++)
		{
			if (((*(Buff + i) & (1 << b)) != 0) != state)
			{
				cloStop = cloStart + (uint32_t)dwell; //find proper time
				if (cloStop < cloStart)
					while (systReg[SYST_CLO] > 0);
				while (systReg[SYST_CLO] < cloStop); //wait for proper time
				if (state)
					gpioSetLow(1LL << UART1_TX_PIN);
				else
					gpioSetHigh(1LL << UART1_TX_PIN);
				cloStart = systReg[SYST_CLO];
				state = !state;
				dwell = 0 + bitw;
			}
			else
				dwell += bitw;
		}
		if (!state) //ended byte in low state
		{
			cloStop = cloStart + (uint32_t)dwell; //find proper time
			if (cloStop < cloStart)
				while (systReg[SYST_CLO] > 0);
			while (systReg[SYST_CLO] < cloStop); //wait for proper time
			gpioSetHigh(1LL << UART1_TX_PIN);
			cloStart = systReg[SYST_CLO];
			state = !state;
			dwell = 0;
		}

		dwell += bitw;
		cloStop = cloStart + (uint32_t)dwell; //find proper time
		if (cloStop < cloStart)
			while (systReg[SYST_CLO] > 0);
		while (systReg[SYST_CLO] < cloStop); //wait for proper time
	}
	return i;
}

int uart1TxDma(char * Buff, int SendLen)
{
	if (!Buff)
		return -3;
	if (!auxReg)
		return -1;
	if (!(auxReg[UART1_AUX_ENAB] & UART1_AUX_ENAB_MASK))
		return -2;

	if (!gpioReg)
		return -1;

	initClock(0);



	//clear any running dma
	dmaOut[DMA_CS] = DMA_CHANNEL_RESET;
	dmaOut[DMA_CONBLK_AD] = 0;

	float bitw = 1000000.0 / (250000000.0 / 8.0 / (auxReg[UART1_BAUD] + 1)); //calc uS per bit off of baud stored
	float dwell = 0;
	uint8_t state = 0;

	uint8_t dbl = 0;
	int dmaPg = CB_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS - 2;
	int dmaSl = 0;
	int hldSl = 0;
	rawCbs_t hld[25];
	int i, b;
	
	

	uint32_t dwlInf, dwlDst;
	if (gpioCfg.clockPeriph != PI_CLOCK_PCM)
	{
		dwlInf = NORMAL_DMA | TIMED_DMA(2);
		dwlDst = PCM_TIMER;
		pcmReg[PCM_CS] |= 0x8; // clear FIFO
		dmaOOLVirt[0]->slots[0] = 0x0F;
	}
	else
	{
		dwlInf = NORMAL_DMA | TIMED_DMA(5);
		dwlDst = PWM_TIMER;
		pwmReg[PWM_CTL] |= 0x40; //clear fifo
		dmaOOLVirt[0]->slots[0] = 0x01;
	}

	int oolPg = OOL_PG_PER_DMA_BLOCK * PI_WAVE_BLOCKS - 1;
	int oolSl = OOL_PER_DPAGE - 1;
	dmaOOLVirt[oolPg]->slots[oolSl] = 1 << (UART1_TX_PIN < 32 ? UART1_TX_PIN : (UART1_TX_PIN - 32));

	uint32_t dstSet, dstClr;
	if (UART1_TX_PIN < 32)
	{
		dstSet = ((GPIO_BASE + (GPSET0 * 4)) & 0x00ffffff) | PI_PERI_BUS;
		dstClr = ((GPIO_BASE + (GPCLR0 * 4)) & 0x00ffffff) | PI_PERI_BUS;
	}
	else
	{
		dstSet = ((GPIO_BASE + (GPSET1 * 4)) & 0x00ffffff) | PI_PERI_BUS;
		dstClr = ((GPIO_BASE + (GPCLR1 * 4)) & 0x00ffffff) | PI_PERI_BUS;
	}

	//add 20us dwell to fix hardware
	dmaCBVirt[dmaPg + 0]->slot[0].info = dwlInf;
	dmaCBVirt[dmaPg + 0]->slot[0].dst = dwlDst;
	dmaCBVirt[dmaPg + 0]->slot[0].length = BYTS_P_REG * 20;
	dmaCBVirt[dmaPg + 0]->slot[0].src = (uint32_t)&dmaOOLBus[0]->slots[0];
	dmaCBVirt[dmaPg + 0]->slot[0].next =(uint32_t) &dmaCBBus[dmaPg + 0]->slot[1];

	dmaCBVirt[dmaPg + 1]->slot[0].info = dwlInf;
	dmaCBVirt[dmaPg + 1]->slot[0].dst = dwlDst;
	dmaCBVirt[dmaPg + 1]->slot[0].length = BYTS_P_REG * 20;
	dmaCBVirt[dmaPg + 1]->slot[0].src = (uint32_t)&dmaOOLBus[0]->slots[0];
	dmaCBVirt[dmaPg + 1]->slot[0].next = (uint32_t)&dmaCBBus[dmaPg + 1]->slot[1];
	dmaSl = 1;

	for (i = 0; i < SendLen; i++)
	{
		//start bit
		state = 0;
		dwell = 0 + bitw;
		hldSl = 0;
		hld[hldSl].src = (uint32_t)&dmaOOLBus[oolPg]->slots[oolSl];
		hld[hldSl].dst = dstClr;
		hld[hldSl].length = 4;
		hld[hldSl].info = NORMAL_DMA;
		hld[hldSl].next = hldSl + 1;
		hldSl++;

		for (b = 0; b < 8; b++)
		{
			if (((*(Buff + i) & (1 << b)) != 0) != state)
			{
				//add dma dwell
				hld[hldSl].info = dwlInf;
				hld[hldSl].dst = dwlDst;
				hld[hldSl].length = BYTS_P_REG * (uint32_t)dwell;
				hld[hldSl].src = (uint32_t)(&dmaOOLBus[0]->slots[0]);
				hld[hldSl].next = hldSl + 1;
				hldSl++;

				//add dma state change
				hld[hldSl].src = (uint32_t)&dmaOOLBus[oolPg]->slots[oolSl];
				if (state)
					hld[hldSl].dst = dstClr;
				else
					hld[hldSl].dst = dstSet;
				hld[hldSl].length = 4;
				hld[hldSl].info = NORMAL_DMA;
				hld[hldSl].next = hldSl + 1;
				hldSl++;
				
				state = !state;
				dwell = 0 + bitw;
			}
			else
				dwell += bitw;
		}

		//end of byte
		if (!state) //in low state at end of byte
		{
			//add dwell dma
			hld[hldSl].info = dwlInf;
			hld[hldSl].dst = dwlDst;
			hld[hldSl].length = BYTS_P_REG * (uint32_t)dwell;
			hld[hldSl].src = (uint32_t)(&dmaOOLBus[0]->slots[0]);
			hld[hldSl].next = hldSl + 1;
			hldSl++;

			//set high state
			hld[hldSl].src = (uint32_t)&dmaOOLBus[oolPg]->slots[oolSl];
			hld[hldSl].dst = dstSet;
			hld[hldSl].length = 4;
			hld[hldSl].info = NORMAL_DMA;
			hld[hldSl].next = hldSl + 1;
			hldSl++;

			state = !state;
			dwell = 0;
		}

		//add stop bit time
		dwell += bitw;

		//add dwell dma
		hld[hldSl].info = dwlInf;
		hld[hldSl].dst = dwlDst;
		hld[hldSl].length = BYTS_P_REG * (uint32_t)dwell;
		hld[hldSl].src = (uint32_t)(&dmaOOLBus[0]->slots[0]);
		hld[hldSl].next = hldSl + 1;
		hldSl++;

		

		if (hldSl + dmaSl >= CBS_PER_DPAGE) //no room in current page for byte
		{
			dmaCBVirt[dmaPg + dbl]->slot[dmaSl - 1].next = 0;

			while (dmaOut[DMA_CONBLK_AD] != 0); //wait for DMA to be done

			//start next dma
			dmaOut[DMA_CONBLK_AD] = (uint32_t)(&dmaCBBus[dmaPg + dbl]->slot[0]);
			/* clear READ/FIFO/READ_LAST_NOT_SET error bits */
			dmaOut[DMA_DEBUG] = DMA_DEBUG_READ_ERR | DMA_DEBUG_FIFO_ERR | DMA_DEBUG_RD_LST_NOT_SET_ERR;
			//start dma
			dmaOut[DMA_CS] = DMA_WAIT_ON_WRITES | DMA_PANIC_PRIORITY(8) | DMA_PRIORITY(8) | DMA_ACTIVATE;

			dbl = !dbl;
			dmaSl = 1; //sl0 = '20us' hardware fix
		}

		for (b = 0; b < hldSl; b++)
		{
			dmaCBVirt[dmaPg + dbl]->slot[dmaSl + b].src = hld[b].src;
			dmaCBVirt[dmaPg + dbl]->slot[dmaSl + b].dst = hld[b].dst;
			dmaCBVirt[dmaPg + dbl]->slot[dmaSl + b].info = hld[b].info;
			dmaCBVirt[dmaPg + dbl]->slot[dmaSl + b].length = hld[b].length;
			dmaCBVirt[dmaPg + dbl]->slot[dmaSl + b].stride = 0;
			dmaCBVirt[dmaPg + dbl]->slot[dmaSl + b].next = (uint32_t)&dmaCBBus[dmaPg + dbl]->slot[dmaSl + b + 1];
		}
		dmaSl += b;
	}

	if (dmaSl > 1) //finish data send
	{
		dmaCBVirt[dmaPg + dbl]->slot[dmaSl - 1].next = 0;

		while (dmaOut[DMA_CONBLK_AD] != 0); //wait for DMA to be done

		//start next dma
		dmaOut[DMA_CONBLK_AD] = (uint32_t)(&dmaCBBus[dmaPg + dbl]->slot[0]);
		/* clear READ/FIFO/READ_LAST_NOT_SET error bits */
		dmaOut[DMA_DEBUG] = DMA_DEBUG_READ_ERR | DMA_DEBUG_FIFO_ERR | DMA_DEBUG_RD_LST_NOT_SET_ERR;
		//start dma
		dmaOut[DMA_CS] = DMA_WAIT_ON_WRITES | DMA_PANIC_PRIORITY(8) | DMA_PRIORITY(8) | DMA_ACTIVATE;
	}

	b = 0;
	while (dmaOut[DMA_CONBLK_AD] != 0) b++; //wait for DMA to be done

	return i;
}

int uart1RxLen(char * Buff, int buffLen)
{
	if (!Buff)
		return -3;
	if (!auxReg)
		return -1;
	if (!(auxReg[UART1_AUX_ENAB] & UART1_AUX_ENAB_MASK))
		return -2;

	int i;
	
	for (i = 0; i < buffLen; i++)
	{
		while (!(auxReg[UART1_STATUS] & UART1_STATUS_RX_DATA)); //WAIT FOR data in fifo

		*(Buff + i) = auxReg[UART1_IO] & 0xFF;
	}
	return buffLen;
}

int uart1RxTerm(char * Buff, int buffLen,char termChar)
{
	if (!Buff)
		return -3;
	if (!auxReg)
		return -1;
	if (!(auxReg[UART1_AUX_ENAB] & UART1_AUX_ENAB_MASK))
		return -2;
	int i;

	clock_t stop = clock() + 7000000;

	for (i = 0; i < buffLen; i++)
	{
		while (!(auxReg[UART1_STATUS] & UART1_STATUS_RX_DATA)) //WAIT FOR data in fifo
		{
			if (clock() > stop)
				return -4;
		}

		*(Buff + i) = (char)auxReg[UART1_IO];
		stop = clock() + 7000000;

		if (*(Buff + i) == termChar) //if term char, exit
		{
			i++;
			break;
		}
	}
	return i;
}

int uart1RxCurr(char * Buff, int buffLen)
{
	if (!Buff)
		return -3;
	if (!auxReg)
		return -1;
	if (!(auxReg[UART1_AUX_ENAB] & UART1_AUX_ENAB_MASK))
		return -2;
	int i = -1;

	while ((i < buffLen) && (auxReg[UART1_STATUS] & UART1_STATUS_RX_DATA)) //WAIT FOR data in fifo
	{
		i++;
		*(Buff + i) = auxReg[UART1_IO] & 0xFF;
	}
	return i;
}

int uart1RxAvail()
{
	if (!auxReg)
		return -1;
	if (!(auxReg[UART1_AUX_ENAB] & UART1_AUX_ENAB_MASK))
		return -2;
	return (auxReg[UART1_STATUS] & UART1_STATUS_RX_DATA) > 0;
}

int uart1RxOvrRun()
{
	if (!auxReg)
		return -1;
	if (!(auxReg[UART1_AUX_ENAB] & UART1_AUX_ENAB_MASK))
		return -2;
	return (auxReg[UART1_STATUS] & UART1_STATUS_RX_OVR) > 0;
}


//========================================================================
//                            end uart 1
//========================================================================


//========================================================================
//                            end uart 0
//========================================================================
#define UART0_DR 0
#define UART0_DR_RX_OVER 0x800
#define UART0_FR 16
#define UART0_FR_TX_FULL 0x20
#define UART0_FR_RX_EMPTY 0x10 
#define UART0_IBAUD 9
#define UART0_FBAUD 10
#define UART0_LCNTRL 11		
#define UART0_CNTRL 12

int uart0Init(int baud)
{
	if (!uartReg)
		return -1;

	uartReg[UART0_CNTRL] &= 0xfffe; //disable uart

	while (uartReg[UART0_FR] & 0x8); //wait for done with curr byte

	uartReg[UART0_LCNTRL] &= 0xffef; //clear fifo

	gpioMode(14, PI_ALT0);
	gpioMode(15, PI_ALT0);

	float f = 40000000.0 / (16 * baud);
	float dif = 1.0;

	int ipart = (int)f;
	f -= ipart;
	int fpart = 0;
	int i;

	//calc fractional part
	for (i = 0; i < 6; i++)
	{
		dif = dif / 2;
		if (f >= dif)
		{
			fpart |= 1 << i;
			f -= dif;
		}
	}

	uartReg[UART0_IBAUD] = ipart;
	uartReg[UART0_FBAUD] = fpart;
	uartReg[UART0_LCNTRL] = 0x70;
	uartReg[UART0_CNTRL] = 0x301; //rx enab + tx enab + uart enab

	return 1;
}

int uart0Tx(char * Buff, int SendLen)
{
	if (!Buff)
		return -3;
	if (!uartReg)
		return -1;

	int i, j = 0;
	for (i = 0; i < SendLen; i++)
	{
		while (uartReg[UART0_FR] & UART0_FR_TX_FULL); //WAIT FOR ROOM IN FIFO

		uartReg[UART0_DR] = *(Buff + i);
	}
	return i;
}

int uart0RxLen(char * Buff, int buffLen)
{
	if (!Buff)
		return -3;
	if (!uartReg)
		return -1;
	int i;
	for (i = 0; i < buffLen; i++)
	{
		while (uartReg[UART0_FR] & UART0_FR_RX_EMPTY); //WAIT FOR data in fifo

		*(Buff + i) = uartReg[UART0_DR] & 0xFF;
	}
	return buffLen;
}

int uart0RxTerm(char * Buff, int buffLen, char termChar)
{
	if (!Buff)
		return -3;
	if (!uartReg)
		return -1;

	int i;
	for (i = 0; i < buffLen; i++)
	{
		while (uartReg[UART0_FR] & UART0_FR_RX_EMPTY); //WAIT FOR data in fifo

		*(Buff + i) = uartReg[UART0_DR] & 0xFF;

		if (*(Buff + i) == termChar) //if term char, exit
		{
			i++;
			break;
		}
	}
	return i;
}

int uart0RxCurr(char * Buff, int buffLen)
{
	if (!Buff)
		return -3;
	if (!uartReg)
		return -1;

	int i = -1;

	while ((i < buffLen) && !(uartReg[UART0_FR] & UART0_FR_RX_EMPTY)) //WAIT FOR data in fifo
	{
		i++;
		*(Buff + i) = uartReg[UART0_DR] & 0xFF;
	}
	return i;
}

int uart0RxAvail()
{
	if (!uartReg)
		return -1;
	return !(uartReg[UART0_FR] & UART0_FR_RX_EMPTY);
}

int uart0RxOvrRun()
{
	if (!uartReg)
		return -1;
	return (uartReg[UART0_DR] & UART0_DR_RX_OVER) > 0;
}

//========================================================================
//                            end uart 0
//========================================================================
