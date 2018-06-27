
//========================================
//   uint variable modification
//========================================
#define UINT32_NULL 0xFFFFFFFF
uint32_t setPulseRndMax(uint32_t Value);
uint32_t setPulseRndMin(uint32_t Value);
uint32_t setPulseCount(uint32_t Value);
uint32_t setPulseWidth(uint32_t Value);
uint32_t setBurstCount(uint32_t Value);
uint32_t setInhibitTime(uint32_t Value);
uint32_t setInhibitDelay(uint32_t Value);
uint32_t setChargeTime(uint32_t Value);
uint32_t setChargeDelay(uint32_t Value);
uint32_t setChargeCount(uint32_t Value);
uint32_t setChargeInterval(uint32_t Value);

//========================================
//   double variable modification
//========================================
#define DOUBLE_NULL -1
double setBurstRndMax(double Value);
double setBurstRndMin(double Value);
double setCycleWidth(double Value);

//========================================
//   generator mode modification
//========================================
#define GEN_MODE_NULL -1
#define GEN_MODE_DISABLED 0
#define GEN_MODE_WAVE_ONLY 1
#define GEN_MODE_INHIBIT 2
#define GEN_MODE_CHARGE	3
#define GEN_MODE_DUALINHIBIT 4
#define GEN_MODE_MULTI_CHARGE 5
int setGenMode(int Value);

//========================================
//   waveform logging modification
//========================================
#define WF_LOG_DISCARD 0
#define WF_LOG_SAVE 1
#define WF_LOG_READ -1
int setWFLogMode(int Value);

//========================================
//   pin mask setting
//========================================
void setWavePinMask(uint64_t mask);
void setInhibitMask(uint64_t mask);
void setWaveInhibitMask(uint64_t mask);
void setChargeMask(uint64_t mask);

//========================================
//   dnbGPIO start / stop wrappers
//========================================
int WaveInit(); //any error codes returned are defined in dngGPIO.h
void WaveTerminate();

//========================================
//   dnbGPIO pin access wrappers
//========================================
//pin mode values
#define PIN_MODE_INPUT  0
#define PIN_MODE_OUTPUT 1
#define PIN_MODE_ALT0   4
#define PIN_MODE_ALT1   5
#define PIN_MODE_ALT2   6
#define PIN_MODE_ALT3   7
#define PIN_MODE_ALT4   3
#define PIN_MODE_ALT5   2
unsigned dnbGPIOMode(unsigned gpio, int mode);
uint64_t dnbGPIOLevels();
void dnbGPIOSetHigh(uint64_t pinMap);
void dnbGPIOSetLow(uint64_t pinMap);

//========================================
//   waveform Generation
//========================================
int GenerateWaveForm(int genMode);
int GenerateWaveFormOnly();
int GenerateWaveFormInhibit();
int GenerateWaveFormCharge();
int GenerateWaveFormMultiCharge();

//========================================
//   waveform DMA preperation & running
//========================================
int ReadyWaveForm(int repeat);
void StartWaveform();
void StopWaveform();


void debugPrintWF();
