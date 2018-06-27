#ifndef DNBGPIO_H
#define DNBGPIO_H

#include <stdint.h>
#include <pthread.h>

//==========trash define to make it compile on non PI
#ifndef MAP_FAILED
#define MAP_FAILED -1
#endif // !MAP_FAILED
//=================================================

#define DNBGPIO_VERSION 50

#define PI_INPFIFO "/dev/dnbgpio"
#define PI_OUTFIFO "/dev/dnbgout"
#define PI_ERRFIFO "/dev/dnbgerr"

#define PI_ENVPORT "PIGPIO_PORT"
#define PI_ENVADDR "PIGPIO_ADDR"

#define PI_LOCKFILE "/var/run/dnbgpio.pid"

//memory allock defines
#define PI_MEM_ALLOC_AUTO    0
#define PI_MEM_ALLOC_PAGEMAP 1
#define PI_MEM_ALLOC_MAILBOX 2

#define PI_DEFAULT_BUFFER_MILLIS           120
#define PI_DEFAULT_CLK_MICROS              5
#define PI_DEFAULT_CLK_PERIPHERAL          PI_CLOCK_PCM
#define PI_DEFAULT_IF_FLAGS                0
#define PI_DEFAULT_DMA_CHANNEL             7
#define PI_DEFAULT_DMA_PRIMARY_CHANNEL     7
#define PI_DEFAULT_DMA_SECONDARY_CHANNEL   6
#define PI_DEFAULT_SOCKET_PORT             8888
#define PI_DEFAULT_SOCKET_PORT_STR         "8888"
#define PI_DEFAULT_SOCKET_ADDR_STR         "127.0.0.1"
#define PI_DEFAULT_UPDATE_MASK_UNKNOWN     0xFFFFFFFF
#define PI_DEFAULT_UPDATE_MASK_B1          0x03E7CF93
#define PI_DEFAULT_UPDATE_MASK_A_B2        0xFBC7CF9C
#define PI_DEFAULT_UPDATE_MASK_APLUS_BPLUS 0x0080480FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_ZERO        0x0080000FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_PI2B        0x0080480FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_PI3B        0x0000000FFFFFFCLL
#define PI_DEFAULT_UPDATE_MASK_COMPUTE     0x00FFFFFFFFFFFFLL
#define PI_DEFAULT_MEM_ALLOC_MODE          PI_MEM_ALLOC_AUTO

#define PI_DEFAULT_CFG_INTERNALS         0

#define PI_MAX_BUSY_DELAY 100

#define PI_WAVE_BLOCKS     4

#ifdef __cplusplus
extern "C" {
#endif


//DMA Command Bundle
typedef struct { /* linux/arch/arm/mach-bcm2708/include/mach/dma.h */
	uint32_t info;
	uint32_t src;
	uint32_t dst;
	uint32_t length;
	uint32_t stride;
	uint32_t next;
	uint32_t pad[2];
} rawCbs_t;

typedef union 
{
	uint8_t U8[8];
	uint16_t U16[4];
	uint32_t U32[2];
	uint64_t U64;
}uint64_u;

//pin mode values
#define PI_INPUT  0
#define PI_OUTPUT 1
#define PI_ALT0   4
#define PI_ALT1   5
#define PI_ALT2   6
#define PI_ALT3   7
#define PI_ALT4   3
#define PI_ALT5   2

//pin seting values
#define PI_OFF   0
#define PI_ON    1

#define PI_MAX_GPIO      53

/* gpioCfgInternals */

#define PI_CFG_DBG_LEVEL         0 /* bits 0-3 */
#define PI_CFG_ALERT_FREQ        4 /* bits 4-7 */
#define PI_CFG_RT_PRIORITY       (1<<8)
#define PI_CFG_STATS             (1<<9)

/*DEF_S Error Codes*/

#define PI_INIT_FAILED       -1 // gpioInitialise failed
#define PI_BAD_USER_GPIO     -2 // GPIO not 0-31
#define PI_BAD_GPIO          -3 // GPIO not 0-53
#define PI_BAD_MODE          -4 // mode not 0-7
#define PI_BAD_LEVEL         -5 // level not 0-1
#define PI_BAD_PUD           -6 // pud not 0-2
#define PI_BAD_PULSEWIDTH    -7 // pulsewidth not 0 or 500-2500
#define PI_BAD_DUTYCYCLE     -8 // dutycycle outside set range
#define PI_BAD_TIMER         -9 // timer not 0-9
#define PI_BAD_MS           -10 // ms not 10-60000
#define PI_BAD_TIMETYPE     -11 // timetype not 0-1
#define PI_BAD_SECONDS      -12 // seconds < 0
#define PI_BAD_MICROS       -13 // micros not 0-999999
#define PI_TIMER_FAILED     -14 // gpioSetTimerFunc failed
#define PI_BAD_WDOG_TIMEOUT -15 // timeout not 0-60000
#define PI_NO_ALERT_FUNC    -16 // DEPRECATED
#define PI_BAD_CLK_PERIPH   -17 // clock peripheral not 0-1
#define PI_BAD_CLK_SOURCE   -18 // DEPRECATED
#define PI_BAD_CLK_MICROS   -19 // clock micros not 1, 2, 4, 5, 8, or 10
#define PI_BAD_BUF_MILLIS   -20 // buf millis not 100-10000
#define PI_BAD_DUTYRANGE    -21 // dutycycle range not 25-40000
#define PI_BAD_DUTY_RANGE   -21 // DEPRECATED (use PI_BAD_DUTYRANGE)
#define PI_BAD_SIGNUM       -22 // signum not 0-63
#define PI_BAD_PATHNAME     -23 // can't open pathname
#define PI_NO_HANDLE        -24 // no handle available
#define PI_BAD_HANDLE       -25 // unknown handle
#define PI_BAD_IF_FLAGS     -26 // ifFlags > 3
#define PI_BAD_CHANNEL      -27 // DMA channel not 0-14
#define PI_BAD_PRIM_CHANNEL -27 // DMA primary channel not 0-14
#define PI_BAD_SOCKET_PORT  -28 // socket port not 1024-32000
#define PI_BAD_FIFO_COMMAND -29 // unrecognized fifo command
#define PI_BAD_SECO_CHANNEL -30 // DMA secondary channel not 0-6
#define PI_NOT_INITIALISED  -31 // function called before gpioInitialise
#define PI_INITIALISED      -32 // function called after gpioInitialise
#define PI_BAD_WAVE_MODE    -33 // waveform mode not 0-3
#define PI_BAD_CFG_INTERNAL -34 // bad parameter in gpioCfgInternals call
#define PI_BAD_WAVE_BAUD    -35 // baud rate not 50-250K(RX)/50-1M(TX)
#define PI_TOO_MANY_PULSES  -36 // waveform has too many pulses
#define PI_TOO_MANY_CHARS   -37 // waveform has too many chars
#define PI_NOT_SERIAL_GPIO  -38 // no bit bang serial read in progress on GPIO
#define PI_BAD_SERIAL_STRUC -39 // bad (null) serial structure parameter
#define PI_BAD_SERIAL_BUF   -40 // bad (null) serial buf parameter
#define PI_NOT_PERMITTED    -41 // GPIO operation not permitted
#define PI_SOME_PERMITTED   -42 // one or more GPIO not permitted
#define PI_BAD_WVSC_COMMND  -43 // bad WVSC subcommand
#define PI_BAD_WVSM_COMMND  -44 // bad WVSM subcommand
#define PI_BAD_WVSP_COMMND  -45 // bad WVSP subcommand
#define PI_BAD_PULSELEN     -46 // trigger pulse length not 1-100
#define PI_BAD_SCRIPT       -47 // invalid script
#define PI_BAD_SCRIPT_ID    -48 // unknown script id
#define PI_BAD_SER_OFFSET   -49 // add serial data offset > 30 minutes
#define PI_GPIO_IN_USE      -50 // GPIO already in use
#define PI_BAD_SERIAL_COUNT -51 // must read at least a byte at a time
#define PI_BAD_PARAM_NUM    -52 // script parameter id not 0-9
#define PI_DUP_TAG          -53 // script has duplicate tag
#define PI_TOO_MANY_TAGS    -54 // script has too many tags
#define PI_BAD_SCRIPT_CMD   -55 // illegal script command
#define PI_BAD_VAR_NUM      -56 // script variable id not 0-149
#define PI_NO_SCRIPT_ROOM   -57 // no more room for scripts
#define PI_NO_MEMORY        -58 // can't allocate temporary memory
#define PI_SOCK_READ_FAILED -59 // socket read failed
#define PI_SOCK_WRIT_FAILED -60 // socket write failed
#define PI_TOO_MANY_PARAM   -61 // too many script parameters (> 10)
#define PI_NOT_HALTED       -62 // script already running or failed
#define PI_BAD_TAG          -63 // script has unresolved tag
#define PI_BAD_MICS_DELAY   -64 // bad MICS delay (too large)
#define PI_BAD_MILS_DELAY   -65 // bad MILS delay (too large)
#define PI_BAD_WAVE_ID      -66 // non existent wave id
#define PI_TOO_MANY_CBS     -67 // No more CBs for waveform
#define PI_TOO_MANY_OOL     -68 // No more OOL for waveform
#define PI_EMPTY_WAVEFORM   -69 // attempt to create an empty waveform
#define PI_NO_WAVEFORM_ID   -70 // no more waveforms
#define PI_I2C_OPEN_FAILED  -71 // can't open I2C device
#define PI_SER_OPEN_FAILED  -72 // can't open serial device
#define PI_SPI_OPEN_FAILED  -73 // can't open SPI device
#define PI_BAD_I2C_BUS      -74 // bad I2C bus
#define PI_BAD_I2C_ADDR     -75 // bad I2C address
#define PI_BAD_SPI_CHANNEL  -76 // bad SPI channel
#define PI_BAD_FLAGS        -77 // bad i2c/spi/ser open flags
#define PI_BAD_SPI_SPEED    -78 // bad SPI speed
#define PI_BAD_SER_DEVICE   -79 // bad serial device name
#define PI_BAD_SER_SPEED    -80 // bad serial baud rate
#define PI_BAD_PARAM        -81 // bad i2c/spi/ser parameter
#define PI_I2C_WRITE_FAILED -82 // i2c write failed
#define PI_I2C_READ_FAILED  -83 // i2c read failed
#define PI_BAD_SPI_COUNT    -84 // bad SPI count
#define PI_SER_WRITE_FAILED -85 // ser write failed
#define PI_SER_READ_FAILED  -86 // ser read failed
#define PI_SER_READ_NO_DATA -87 // ser read no data available
#define PI_UNKNOWN_COMMAND  -88 // unknown command
#define PI_SPI_XFER_FAILED  -89 // spi xfer/read/write failed
#define PI_BAD_POINTER      -90 // bad (NULL) pointer
#define PI_NO_AUX_SPI       -91 // no auxiliary SPI on Pi A or B
#define PI_NOT_PWM_GPIO     -92 // GPIO is not in use for PWM
#define PI_NOT_SERVO_GPIO   -93 // GPIO is not in use for servo pulses
#define PI_NOT_HCLK_GPIO    -94 // GPIO has no hardware clock
#define PI_NOT_HPWM_GPIO    -95 // GPIO has no hardware PWM
#define PI_BAD_HPWM_FREQ    -96 // hardware PWM frequency not 1-125M
#define PI_BAD_HPWM_DUTY    -97 // hardware PWM dutycycle not 0-1M
#define PI_BAD_HCLK_FREQ    -98 // hardware clock frequency not 4689-250M
#define PI_BAD_HCLK_PASS    -99 // need password to use hardware clock 1
#define PI_HPWM_ILLEGAL    -100 // illegal, PWM in use for main clock
#define PI_BAD_DATABITS    -101 // serial data bits not 1-32
#define PI_BAD_STOPBITS    -102 // serial (half) stop bits not 2-8
#define PI_MSG_TOOBIG      -103 // socket/pipe message too big
#define PI_BAD_MALLOC_MODE -104 // bad memory allocation mode
#define PI_TOO_MANY_SEGS   -105 // too many I2C transaction segments
#define PI_BAD_I2C_SEG     -106 // an I2C transaction segment failed
#define PI_BAD_SMBUS_CMD   -107 // SMBus command not supported by driver
#define PI_NOT_I2C_GPIO    -108 // no bit bang I2C in progress on GPIO
#define PI_BAD_I2C_WLEN    -109 // bad I2C write length
#define PI_BAD_I2C_RLEN    -110 // bad I2C read length
#define PI_BAD_I2C_CMD     -111 // bad I2C command
#define PI_BAD_I2C_BAUD    -112 // bad I2C baud rate, not 50-500k
#define PI_CHAIN_LOOP_CNT  -113 // bad chain loop count
#define PI_BAD_CHAIN_LOOP  -114 // empty chain loop
#define PI_CHAIN_COUNTER   -115 // too many chain counters
#define PI_BAD_CHAIN_CMD   -116 // bad chain command
#define PI_BAD_CHAIN_DELAY -117 // bad chain delay micros
#define PI_CHAIN_NESTING   -118 // chain counters nested too deeply
#define PI_CHAIN_TOO_BIG   -119 // chain is too long
#define PI_DEPRECATED      -120 // deprecated function removed
#define PI_BAD_SER_INVERT  -121 // bit bang serial invert not 0 or 1
#define PI_BAD_EDGE        -122 // bad ISR edge value, not 0-2
#define PI_BAD_ISR_INIT    -123 // bad ISR initialisation
#define PI_BAD_FOREVER     -124 // loop forever must be last chain command
#define PI_BAD_FILTER      -125 // bad filter parameter
#define PI_WAVE_PAGE_OVER  -127 // not enough WaveForm Pages
#define PI_WAVE_LOC_OVER   -126 // Location is past last wave event

#define PI_PIGIF_ERR_0    -2000
#define PI_PIGIF_ERR_99   -2099

#define PI_CUSTOM_ERR_0   -3000
#define PI_CUSTOM_ERR_999 -3999

typedef struct              /*holds one event (pin value set + dely) for wf generation*/
{							/* NOTE: x << 0 = port 0; x << 32 = port1 */
	uint64_t setMask;		/* masked pins go to logical HIGH */
	uint64_t clrMask;		/* masked ping go to logical LOW */
	uint32_t delayUs;		/* delay in uS till next event*/
} waveMake_t;


int gpioInitialise(void);
void gpioTerminate(void);

int WaveAddEvent(waveMake_t EventData, int WFId);
int WaveAddEventRaw(uint64_t setMask, uint64_t clrMask, uint32_t usDwell, int WFID);
int WaveAddEvents(waveMake_t * EventDatas, int count, int WFId);
int WaveDelete(int WFId);
int WaveInsert(waveMake_t * EventData, int Len, int WFId, int Location);

int WavePrepRun(int WFId, int Repeat);
void WaveStartRun();
void WaveStopRun();
int WaveRunning();
void WaveVerifyDMA();
int WaveVerifyWave(int WFID);

unsigned gpioMode(unsigned gpio, int mode);
uint64_t gpioLevels();
void gpioSetHigh(uint64_t pinMap);
void gpioSetLow(uint64_t pinMap);

#define SPI_SEND_USE_CE1(X) ((X)&1)
#define SPI_SEND_CE_POLARITY(X) (((X)&1)<<1)
#define SPI_SEND_LOSSI_MODE(X) (((X)&1)<<2)
#define SPI_SEND_BIDIR_MODE(X) (((X)&1)<<3)
void spiTransfer(unsigned speed, uint8_t sendFlags, uint8_t * txBuff, uint8_t * rxBuff, unsigned txLen, unsigned rxLen);

#define SPI_INIT_SCLK_PHASE(X) ((X)&1)
#define SPI_INIT_SCLK_REST(X) (((X)&1)<<1)
#define SPI_INIT_DNE_INTRPT(X) (((X)&1)<<2)
#define SPI_INIT_RX_FULL_INTRPT(X) (((X)&1)<<3)
//#define SPI_INIT_CE0_LVL(X) (((X)&1)<<4)
//#define	SPI_INIT_CE1_LVL(X) (((X)&1)<<5)

#define SPI_PIN_USE_CE0(X) ((X)&1)
#define SPI_PIN_CE0_P36(X) (((X)&1)<<2)
#define SPI_PIN_USE_CE1(X) (((X)&1)<<3)
#define SPI_PIN_CE1_P35(X) (((X)&1)<<4)
#define SPI_PIN_MOSI_P37(X) (((X)&1)<<5)
#define SPI_PIN_MISO_P38(X) (((X)&1)<<6)
#define SPI_PIN_SCLK_P39(X) (((X)&1)<<7)
void spiInit(uint8_t initFlags, uint8_t pinFlags);

void spiSoft(int cs, unsigned clkHz, uint8_t * txBuff, uint8_t * rxBuff, int txLen, int rxLen);

//UART1: tx pin 40, rx pin 41, no flow controll
int uart1Init(uint32_t baud);
int uart1Tx(char * Buff, int SendLen);
int uart1RxLen(char * Buff, int buffLen);
int uart1RxTerm(char * Buff, int buffLen, char termChar);
int uart1RxCurr(char * Buff, int buffLen);
int uart1RxAvail();
int uart1RxOvrRun();

int uart0Init(int baud);
int uart0TxSoft(char * Buff, int SendLen);
int uart1TxDma(char * Buff, int SendLen);
int uart0RxLen(char * Buff, int buffLen);
int uart0RxTerm(char * Buff, int buffLen, char termChar);
int uart0RxCurr(char * Buff, int buffLen);
int uart0RxAvail();
int uart0RxOvrRun();

int debugTiming(int delay);
int debugDMA(int delay, int count);
int debugClock();

#endif