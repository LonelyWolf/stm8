#include "stdint.h"
#include "iostm8l051f3.h"
#include "stm8l051.h"
#include "rtc.h"


// Get current MCU clock frequency in Hz
uint32_t CLK_GetClockFreq(void) {
    uint32_t src_freq;
    uint8_t clocksource = CLK_SCSR;
    const uint8_t SYSDivFactor[8] = { 1, 2, 4, 8, 16, 32, 64, 128 };

    if ( clocksource == 0x01) { // CLK_SYSCLKSource_HSI
        src_freq = 16000000; // HSI_VALUE = 16MHz
    } else if ( clocksource == 0x02) { // CLK_SYSCLKSource_LSI
        src_freq = 38000; // LSI_VALUE = 38kHz
    } else if ( clocksource == 0x04) { // CLK_SYSCLKSource_HSE
        src_freq = 16000000; // HSE_VALUE = 16MHz
    } else {
        src_freq = 32768; // LSE_VALUE = 32.768kHz
    }

    return (uint32_t)(src_freq / SYSDivFactor[CLK_CKDIVR & 0x07]);
}

// Get real LSI frequency in Hz
// Input: master_freq - master frequency in Hz
uint32_t CLK_GetLSIFreq(uint32_t master_freq) {
    uint16_t IC1_val1, IC1_val2;

    master_freq = CLK_GetClockFreq(); // Master frequency in Hz

    CLK_PCKENR1 |= (1<<0)|(1<<6); // Enable TIM2 and BEEP peripherial
    while (CLK_CBEEPR_bit.BEEPSWBSY); // Wait for BEEPSWBSY flag to clear
    CLK_CBEEPR = 0x02; // Set LSI as BEEP clock source
    BEEP_CSR1_bit.MSR = 1; // Enable BEEPCLK measurement (BEEPCLK -> TIM2 ch1)
    BEEP_CSR2_bit.BEEPEN = 1;

    // Configure TIM2 channel 1
    TIM2_CCER1_bit.CC1E = 0; // Disable the Channel 1
    TIM2_CCMR1 = 0x01; // Input Capture mapped on the direct input
    TIM2_CCER1_bit.CC1E = 1; // Enable the Channel 1
    TIM2_CCMR1 |= 0x0C; // Capture every 8 events
    TIM2_CCER1_bit.CC1P = 0; // Polarity rising
    TIM2_SR1 = 0; // Clear TIM2 flags
    TIM2_SR2 = 0;
    TIM2_CR1_bit.CEN = 1; // Enable TIM2

    while (!TIM2_SR1_bit.CC1IF); // Wait a capture on TIM2 CC1
    //while ((TIM2_SR1 & (uint8_t)0x02) != 0x02);
    IC1_val1  = (TIM2_CCR1H << 8); // Get 1st CCR1 value
    IC1_val1 |=  TIM2_CCR1L;
    TIM2_SR1_bit.CC1IF = 0; // Clear CC1 flag

    while (!TIM2_SR1_bit.CC1IF); // Wait a capture on TIM2 CC1
    //while ((TIM2_SR1 & (uint8_t)0x02) != 0x02);
    IC1_val2  = (TIM2_CCR1H << 8); // Get 2nd CCR1 value
    IC1_val2 |=  TIM2_CCR1L;
    TIM2_SR1_bit.CC1IF = 0; // Clear CC1 flag

    TIM2_CCER1_bit.CC1E = 0; // Disable the Channel 1
    TIM2_CCMR1 = 0x00; // Reset CCMR1 register
    TIM2_CR1_bit.CEN = 0; // Disable TIM2

    BEEP_CSR1_bit.MSR = 0; // Disable BEEPCLK measurement
    CLK_PCKENR1 &= ~((1<<0)|(1<<6)); // Disable TIM2 and BEEP peripherial

    return (uint32_t)((master_freq * 8) / (IC1_val2 - IC1_val1));
}

// Unlock write protection of the RTC registers
void RTC_Unlock(void) {
    RTC_WPR = 0xCA;
    RTC_WPR = 0x53;
}

// Lock write protection of the RTC registers
void RTC_Lock(void) {
    RTC_WPR = 0xFF;
}

// Configure RTC clock according to measured real LSI frequency
// Input: LSI_freq - LSI frequency in Hz
void RTC_TuneClock(const uint32_t LSI_freq) {
    uint16_t prediv_s;
    uint16_t cntr = 0;

    while (CLK_CRTCR_bit.RTCSWBSY); // Wait for RTCSWBSY flag to clear
    CLK_CRTCR = 0xA4; // RTC clock: source LSI, RTCDIV = 32
    RTC_Unlock(); // Disable write protection of RTC registers
    RTC_ISR1_bit.INIT = 1; // Enter initialization mode
    // Poll INITF flag until it is set in RTC_ISR1 (with timeout for any case).
    // It takes around 2 RTCCLK clock cycles according to datasheet
    while ((RTC_ISR1_bit.INITF == 0) && (cntr != 0xFFFF)) cntr++;
    RTC_APRER  = 0x04;
    prediv_s   = (LSI_freq - 160) / 160;
    RTC_SPRERH = (uint8_t)(prediv_s >> 8);
    RTC_SPRERL = (uint8_t)(prediv_s & 0xFF);
    RTC_WUTRH  = 0xFF; // Wakeup timer = 0xFFFF
    RTC_WUTRL  = 0xFF;
    RTC_ISR1_bit.INIT = 0; // Exit initialization mode
    RTC_Lock(); // Enable write protection of RTC registers
}

// RTC initialization
void RTC_Init(void) {
    uint16_t cntr = 0;

    while (CLK_CRTCR_bit.RTCSWBSY); // Wait for RTCSWBSY flag to clear
    CLK_CRTCR = 0xB0; // RTC clock: source LSE, RTCDIV = 32
    while (!CLK_ECKCR_bit.LSERDY); // Wait for LSE stabilization
    CLK_PCKENR2_bit.PCKEN22 = 1; // Enable RTC peripherial (PCKEN22)
    RTC_Unlock(); // Disable write protection of RTC registers
    RTC_ISR1_bit.INIT = 1; // Enter initialization mode
    // Poll INITF flag until it is set in RTC_ISR1 (with timeout for any case).
    // It takes around 2 RTCCLK clock cycles according to datasheet
    while ((RTC_ISR1_bit.INITF == 0) && (cntr != 0xFFFF)) cntr++;
    RTC_APRER  = 0x08;
    RTC_SPRERH = 0x00;
    RTC_SPRERL = 0x80;
    RTC_WUTRH  = 0xFF; // Wakeup timer = 0xFFFF
    RTC_WUTRL  = 0xFF;
    RTC_ISR1_bit.INIT = 0; // Exit initialization mode
    RTC_Lock(); // Enable write protection of RTC registers
}

// Configure RTC Wakeup clock source
// Input: WakeupClock - specifies wakeup clock source
void RTC_WakeupConfig(RTC_WakeupClock_TypeDef WakeupClock) {
    uint16_t cntr = 0;

    RTC_Unlock(); // Disable write protection of RTC registers
    RTC_CR2_bit.WUTE  = 0; // Disable wakeup timer
    RTC_CR2_bit.WUTIE = 0; // Disable wakeup interrupt
    // Poll WUTWF flag until it is set in RTC_ISR1 (with timeout for any case).
    // It takes around 2 RTCCLK clock cycles according to datasheet
    while ((RTC_ISR1_bit.WUTWF == 0) && (cntr != 0xFFFF)) cntr++;
    RTC_CR1 = WakeupClock; // Configure wakeup clock source
    RTC_Lock(); // Enable write protection of RTC registers
}

// Enable or disable wakeup timer
// Input: state = ENABLE or DISABLE
void RTC_WakeupSet(FunctionalState state) {
    RTC_Unlock(); // Disable write protection of RTC registers
    RTC_CR2_bit.WUTE = (state == ENABLE) ? 1 : 0;
    RTC_Lock(); // Enable write protection of RTC registers
}

// Enable or disable wakeup timer interrupt
// Input: state = ENABLE or DISABLE
void RTC_WakeupIT(FunctionalState state) {
    RTC_Unlock(); // Disable write protection of RTC registers
    RTC_CR2_bit.WUTIE = (state == ENABLE) ? 1 : 0;
    RTC_Lock(); // Enable write protection of RTC registers
}

// Set Wakeup timer counter
// Input: cntr = wake up timer count
void RTC_WakeupTimerSet(uint16_t timer_count) {
    uint8_t old_CR2;
    uint16_t cntr = 0;

    RTC_Unlock(); // Disable write protection of RTC registers
    old_CR2 = RTC_CR2;
    RTC_CR2_bit.WUTE = 0; // WUTR* registers can be changed only when WUTE bit is reset
    // Poll WUTWF flag until it is set in RTC_ISR1 (with timeout for any case).
    // It takes around 2 RTCCLK clock cycles according to datasheet
    while ((RTC_ISR1_bit.WUTWF == 0) && (cntr != 0xFFFF)) cntr++;
    RTC_WUTRH = (uint8_t)(timer_count >> 8);
    RTC_WUTRL = (uint8_t)timer_count;
    RTC_CR2 = old_CR2;
    RTC_Lock(); // Enable write protection of RTC registers
}

// Get the RTC current time
// Output: RTC_TimeTypeDef structure with current time
RTC_TimeTypeDef RTC_GetTime(void) {
    RTC_TimeTypeDef time;
    uint8_t t1,t2,t3;

    t1 = RTC_TR1;
    t2 = RTC_TR2;
    t3 = RTC_TR3;

    time.Seconds = ((t1 >> 4) * 10) + (t1 & 0x0F);
    time.Minutes = ((t2 >> 4) * 10) + (t2 & 0x0F);
    time.Hours   = (((t3 & 0x30) >> 4) * 10) + (t3 & 0x0F);

    return time;
}

// Get the RTC current date
// Output: RTC_DateTypeDef structure with current date
RTC_DateTypeDef RTC_GetDate(void) {
    RTC_DateTypeDef date;
    uint8_t d1,d2,d3;

    d1 = RTC_DR1;
    d2 = RTC_DR2;
    d3 = RTC_DR3;

    date.Day   = ((d1 >> 4) * 10) + (d1 & 0x0F);
    date.DOW   = d2 >> 5;
    date.Month = (((d2 & 0x1F) >> 4) * 10) + (d2 & 0x0F);
    date.Year  = ((d3 >> 4) * 10) + (d3 & 0x0F);

    return date;
}

// Get the RTC current time in BCD format
// Output: RTC_TimeBCDTypeDef structure with raw RTC_TR* registers values
RTC_TimeBCDTypeDef RTC_GetTimeBCD(void) {
    RTC_TimeBCDTypeDef time;

    time.RTC_TR1 = RTC_TR1;
    time.RTC_TR2 = RTC_TR2;
    time.RTC_TR3 = RTC_TR3;

    return time;
}

// Get the RTC current date in BCD format
// Output: RTC_DateBCDTypeDef structure with raw RTC_DR* registers values
RTC_DateBCDTypeDef RTC_GetDateBCD(void) {
    RTC_DateBCDTypeDef date;

    date.RTC_DR1 = RTC_DR1;
    date.RTC_DR2 = RTC_DR2;
    date.RTC_DR3 = RTC_DR3;

    return date;
}
