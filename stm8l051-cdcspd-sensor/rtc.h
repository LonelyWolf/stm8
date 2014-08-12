#ifndef _RTC_H
#define _RTC_H


#define RTC_LSE_INIT_TIMEOUT     0xFFFFFF     // LSE init timeout


typedef enum {
    RTC_OK          = 0x00, // Operation was successful
    RTC_LSE_TIMEOUT = 0x01  // timeout occurred while initializing LSE
} RTC_Result;

// External clock control register (not defined in standard IAR library)
#ifdef __IAR_SYSTEMS_ICC__
typedef struct
{
  unsigned char HSEON       : 1;
  unsigned char HSERDY      : 1;
  unsigned char LSEON       : 1;
  unsigned char LSERDY      : 1;
  unsigned char HSEBYP      : 1;
  unsigned char LSEBYP      : 1;
} __BITS_CLK_ECKCR;
#endif
__IO_REG8_BIT(CLK_ECKCR,   0x50C6, __READ_WRITE, __BITS_CLK_ECKCR);

typedef enum {
    CLK_LSE_OFF    = 0x00, // LSE Diasble
    CLK_LSE_ON     = 0x04, // LSE Enable
    CLK_LSE_Bypass = 0x24  // LSE Bypass and enable
} CLK_LSE_TypeDef;

typedef enum {
    RTC_WUC_RTCCLK_Div16    = 0x00, // (RTC clock) div 16  WUCKSEL[2:0] = 000
    RTC_WUC_RTCCLK_Div8     = 0x01, // (RTC clock) div 8   WUCKSEL[2:0] = 001
    RTC_WUC_RTCCLK_Div4     = 0x02, // (RTC clock) div 4   WUCKSEL[2:0] = 010
    RTC_WUC_RTCCLK_Div2     = 0x03, // (RTC clock) div 2   WUCKSEL[2:0] = 011
    RTC_WUC_CK_SPRE_16bits  = 0x04, // CK SPRE with a counter from 0x0000 to 0xFFFF
    RTC_WUC_CK_SPRE_17bits  = 0x06  // CK SPRE with a counter from 0x10000 to 0x1FFFF
} RTC_WakeupClock_TypeDef;

// RTC time in human readable format
typedef struct {
    uint8_t Hours;   // RTC hours
    uint8_t Minutes; // RTC minutes
    uint8_t Seconds; // RTC seconds
} RTC_TimeTypeDef;

// RTC date in human readable format
typedef struct {
    uint8_t Year;  // RTC year (0..99)
    uint8_t Month; // RTC month (1..12)
    uint8_t Day;   // RTC date (1..31)
    uint8_t DOW;   // RTC day of week (1..7)
} RTC_DateTypeDef;

// RAW RTC time registers
typedef struct {
    uint8_t RTC_TR1;
    uint8_t RTC_TR2;
    uint8_t RTC_TR3;
} RTC_TimeBCDTypeDef;

// RAW RTC date registers
typedef struct {
    uint8_t RTC_DR1;
    uint8_t RTC_DR2;
    uint8_t RTC_DR3;
} RTC_DateBCDTypeDef;


uint32_t CLK_GetClockFreq(void);
uint32_t CLK_GetLSIFreq(uint32_t master_freq);

void RTC_TuneClock(const uint32_t LSI_freq);
RTC_Result RTC_Init(void);
void RTC_WakeupConfig(RTC_WakeupClock_TypeDef WakeupClock);
void RTC_WakeupSet(FunctionalState state);
void RTC_WakeupIT(FunctionalState state);
void RTC_WakeupTimerSet(uint16_t timer_count);

RTC_TimeTypeDef RTC_GetTime(void);
RTC_DateTypeDef RTC_GetDate(void);
RTC_TimeBCDTypeDef RTC_GetTimeBCD(void);
RTC_DateBCDTypeDef RTC_GetDateBCD(void);

#endif // _RTC_H