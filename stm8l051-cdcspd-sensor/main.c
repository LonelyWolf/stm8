//////////////////////////
////// STM8L051F3P6 //////
//////////////////////////


#define SOFT_DEBOUNCE     // If defined - use software debounce, no debounce otherwise


///////////////////////////////////////////////////////////////////////////////


#include "stdint.h"
#include "iostm8l051f3.h"

#include "stm8l051.h"
#include "nRF24.h"
#include "rtc.h"


///////////////////////////////////////////////////////////////////////////////


#define TIM2LSE                  // If defined - TIM2 will be clocked from LSE
#define TIM3LSE                  // If defined - TIM3 will be clocked from LSE

#define TX_PAYLOAD           17  // nRF24L01 Payload length
#define RF_CHANNEL           90  // nRF24L01 channel (90ch = 2490MHz)
#define RF_RETR              10  // nRF24L01 TX retransmit count (max 15)

#define TIM_DEBOUNCE         65  // Debounce delay, higher value means lowest high speed can be measured.
#define TIM_TIMEOUT        6000  // Magnetic reed impulse timeout (roughly measured in milliseconds)

#define WU_TIMER             63  // Wakeup timer Period = (WU_TIMER + 1) * 0,015625 seconds (63 = 1 second)

#define VREF_OFF_DUTY       600  // Internal voltage measure off-duty ratio

#define START_BLINKS          3  // How many times the green LED will blink after reset
#define REED_PASSES          60  // How many reed passes the LED will blink after reset
#define LED_BLINK_ON_TIME   252  // How many TIM4 periods the LED will be lit

#define nRF24_TX_Addr     "WBC"  // TX address for nRF24L01
#define nRF24_TX_Addr_Size    3  // TX address size


///////////////////////////////////////////////////////////////////////////////


// 2-pin Bi color LED connected to PD0 and PB2
// Macros to switch colors and turn on/off
#define LED_RED()   PB_ODR_bit.ODR1 = 0; PB_ODR_bit.ODR2 = 1;
#define LED_GREEN() PB_ODR_bit.ODR1 = 1; PB_ODR_bit.ODR2 = 0;
#define LED_OFF()   PB_ODR_bit.ODR1 = 0; PB_ODR_bit.ODR2 = 0;


///////////////////////////////////////////////////////////////////////////////


uint8_t i; // Really need to comment this?

uint16_t vrefint; // Last measured internal voltage
uint16_t factory_vref; // Factory measured internal voltage with 3V external

// EXTI# impulse counters
volatile uint16_t cntr_EXTI0 = 0;
volatile uint16_t cntr_EXTI4 = 0;

// TIM# difference between two pulses of EXTI#
volatile uint16_t tim2_diff = 0;
volatile uint16_t tim3_diff = 0;

// TIM# overwlows counter, a.k.a software timer
volatile uint16_t tim2 = 0;
volatile uint16_t tim3 = 0;
volatile uint16_t tim4 = 0;

// TIM counter from last successfull packet sent
volatile uint16_t ride_time = 0;

// How many times after reset magnet passed the sensors
uint8_t cntr_rst_passes = REED_PASSES;

uint16_t packets_lost = 0; // Lost packets counter

uint8_t buf[TX_PAYLOAD]; // nRF24L01 payload buffer
uint8_t prev_observe_TX = 0; // Last value of nRF24L01 OBSERVE_TX register
uint16_t cntr_wake = 0; // Wakeup counter


///////////////////////////////////////////////////////////////////////////////


// RTC_WAKEUP IRQ handle
#pragma vector=RTC_WAKEUP_vector
__interrupt void RTC_IRQHandler(void) {
    CPU_CFG_GCR_bit.AL = 0; // Set main activation level
    RTC_ISR2_bit.WUTF = 0; // Clear wakeup timer interrupt flag
}

// EXTI0 IRQ handle
#pragma vector=EXTI0_vector
__interrupt void EXTI0_IRQHandler(void) {
    TIM2_CR1_bit.CEN = 1; // Enable TIM2 counter

#ifdef SOFT_DEBOUNCE
    if (tim2 > TIM_DEBOUNCE) {
#endif
        cntr_EXTI0++;
        if (cntr_EXTI0 > 2) tim2_diff = tim2;

        tim2 = 0;
        TIM2_EGR_bit.UG = 1; // Reinitialize TIM2

        if (cntr_rst_passes) {
            TIM4_CR1_bit.CEN = 1; // Enable TIM4 counter
            TIM4_EGR_bit.UG = 1; // Reinitialize TIM4
            LED_RED();
        }
#ifdef SOFT_DEBOUNCE
    }
#endif

    if (!RTC_CR2_bit.WUTE) RTC_WakeupSet(ENABLE); // Enable wakeup timer
    EXTI_SR1_bit.P0F = 1; // Clear EXTI0 IRQ flag
}

// EXTI4 IRQ handle
#pragma vector=EXTI4_vector
__interrupt void EXTI4_IRQHandler(void) {
    uint16_t tmp;

    TIM3_CR1_bit.CEN = 1; // Enable TIM3 counter

#ifdef SOFT_DEBOUNCE
    if (tim3 > TIM_DEBOUNCE) {
#endif
        cntr_EXTI4++;
        if (cntr_EXTI4 > 2) {
            tim3_diff = tim3;
            tmp = tim3;
            ride_time += tmp;
        }

        tim3 = 0;
        TIM3_EGR_bit.UG = 1; // Reinitialize TIM3

        if (cntr_rst_passes) {
            TIM4_CR1_bit.CEN = 1; // Enable TIM4 counter
            TIM4_EGR_bit.UG = 1; // Reinitialize TIM4
            LED_GREEN();
        }
#ifdef SOFT_DEBOUNCE
    }
#endif

    if (!RTC_CR2_bit.WUTE) RTC_WakeupSet(ENABLE); // Enable wakeup timer
    EXTI_SR1_bit.P4F = 1; // Clear EXTI4 IRQ flag
}

// TIM2 Update/Overflow/Trigger/Break interrupt IRQ handle
#pragma vector=TIM2_OVR_UIF_vector
__interrupt void TIM2_UIF_IRQHandler(void) {
    tim2++;
    if (tim2 > TIM_TIMEOUT) {
        TIM2_CR1_bit.CEN = 0; // Disable TIM2 counter
        cntr_EXTI0 = 0;
        tim2_diff  = 0;
    }
    TIM2_SR1_bit.UIF = 0; // Clear TIM2 update interrupt flag
}

// TIM3 Update/Overflow/Trigger/Break interrupt IRQ handle
#pragma vector=TIM3_OVR_UIF_vector
__interrupt void TIM3_UIF_IRQHandler(void) {
    tim3++;
    if (tim3 > TIM_TIMEOUT) {
        TIM3_CR1_bit.CEN = 0; // Disable TIM3 counter
        cntr_EXTI4 = 0;
        tim3_diff  = 0;
    }
    TIM3_SR1_bit.UIF = 0; // Clear TIM3 update interrupt flag
}

// TIM4 Update interrupt IRQ handle
#pragma vector=TIM4_UIF_vector
__interrupt void TIM4_UIF_IRQHandler(void) {
    tim4++;
    if (tim4 > LED_BLINK_ON_TIME) {
        LED_OFF();
        tim4 = 0;
        cntr_rst_passes--;
        TIM4_CR1_bit.CEN = 0; // Disable TIM4 counter
    }
    TIM4_SR1_bit.UIF = 0; // Clear TIM4 update interrupt flag
    if (!cntr_rst_passes) {
        TIM4_CR1_bit.CEN = 0; // Disable TIM4 counter
        CLK_PCKENR1_bit.PCKEN12 = 0; // Disable TIM4 peripherial
    }
}


///////////////////////////////////////////////////////////////////////////////


// Enable ADC and configure Vrefint channel
// Must be some delay after this to stabilize ADC (Twkup = 3us)
void ADC_Vrefint_Init(void) {
    CLK_PCKENR2_bit.PCKEN20 = 1; // Enable ADC peripherial (PCKEN20)
    ADC1_CR1_bit.ADON = 1; // Enable ADC
    ADC1_CR1_bit.EOCIE = 1; // Enable interrupt for end of conversion
    ADC1_TRIGR1_bit.VREFINTON = 1; // Enable internal reference voltage
    ADC1_SQR1_bit.CHSEL_S28 = 1; // Enable CHSEL_SVREFINT fast ADC channel
    ADC1_CR3 = 0x80; // Sampling time = 48 ADC clock cycles, disable analog watchdog channels
    ADC1_SQR1_bit.DMAOFF = 1; // DMA off
    WFE_CR2_bit.ADC1_COMP_EV = 1; // Enable ADC as source of event
}

// Measure Vrefint
uint16_t ADC_Vrefint_Measure(uint8_t count) {
    uint16_t adc_res;
    uint16_t value = 0;
    uint8_t cntr;

    // Measure voltage "count" times and calculate rough average value
    for (cntr = 1; cntr < count; cntr++) {
        ADC1_CR1_bit.START = 1; // Start ADC conversion, by software trigger
        asm("WFE"); // Wait for the conversion ends
        adc_res  = (ADC1_DRH << 8); // Get ADC converted data
        adc_res |= ADC1_DRL;
        value += adc_res;
        if (cntr) value >>= 1;
    }

    return value;
}

// Disable Vrefint ADC channel and ADC module
void ADC_Vrefint_Disable() {
    WFE_CR2_bit.ADC1_COMP_EV = 0; // Disable ADC as source of event
    ADC1_TRIGR1_bit.VREFINTON = 0; // Disable internal reference voltage
    ADC1_SQR1_bit.CHSEL_S28 = 0; // Disable CHSEL_SVREFINT fast ADC channel
    ADC1_CR1_bit.ADON = 0; // Disable ADC
    CLK_PCKENR2_bit.PCKEN20 = 0; // Disable ADC peripherial
}


///////////////////////////////////////////////////////////////////////////////


int main(void)
{
    CLK_PCKENR2_bit.PCKEN27 = 0; // Disable Boot ROM

    // LED pins (PB1, PB2)
    PB_DDR_bit.DDR1 = 1; // Set PB1 as output
    PB_ODR_bit.ODR1 = 0; // Latch "1" in input register
    PB_CR1_bit.C11  = 1; // Slow output
    PB_CR2_bit.C21  = 0; // Disable external interrupt
    PB_DDR_bit.DDR2 = 1; // Set PB2 as output
    PB_ODR_bit.ODR2 = 0; // Latch "1" in input register
    PB_CR1_bit.C12  = 1; // Slow output
    PB_CR2_bit.C22  = 0; // Disable external interrupt

    LED_RED(); // Lit red LED to indicate startup sequence

    // Magnetic reed inputs
    // PB0 = Reed#0 (CDC sensor)
    PB_ODR_bit.ODR0 = 1; // Latch "1" in input register
    PB_DDR_bit.DDR0 = 0; // Input
    PB_CR1_bit.C10  = 0; // Floating input (external pull-up resistor)
    PB_CR2_bit.C20  = 1; // Enable external interrupt
    // PC4 = Reed#1 (SPD sensor)
    PC_ODR_bit.ODR4 = 1; // Latch "1" in input register
    PC_DDR_bit.DDR4 = 0; // Input
    PC_CR1_bit.C14  = 0; // Floating input (external pull-up resistor)
    PC_CR2_bit.C24  = 1; // Enable external interrupt

    // Configure unused GPIO ports as input with pull-up for powersaving
    // PD0
    PD_ODR_bit.ODR0 = 0; // Latch "0" in input register
    PD_DDR_bit.DDR0 = 0; // Input
    PD_CR1_bit.C10  = 1; // With Pull-up
    PD_CR2_bit.C20  = 0; // External interrupt disabled
    // PA2
    PA_ODR_bit.ODR2 = 0; // Latch "0" in input register
    PA_DDR_bit.DDR2 = 0; // Input
    PA_CR1_bit.C12  = 1; // With Pull-up
    PA_CR2_bit.C22  = 0; // External interrupt disabled
    // PA3
    PA_ODR_bit.ODR3 = 0; // Latch "0" in input register
    PA_DDR_bit.DDR3 = 0; // Input
    PA_CR1_bit.C13  = 1; // With Pull-up
    PA_CR2_bit.C23  = 0; // External interrupt disabled
    // PC1
    PC_ODR_bit.ODR1 = 0; // Latch "0" in input register
    PC_DDR_bit.DDR1 = 0; // Input
    PC_CR1_bit.C11  = 1; // With Pull-up
    PC_CR2_bit.C21  = 0; // External interrupt disabled

    // Configure RTC
    RTC_Init();
    RTC_WakeupConfig(RTC_WUC_RTCCLK_Div16); // RTC wakeup clock = LSE/RTCDIV/16
    RTC_WakeupIT(ENABLE); // Enable wakeup interrupt
    RTC_WakeupSet(DISABLE); // Disable wakeup counter

    // If VREFINT is not set on factory, assign average standard value
    if (Factory_VREFINT) {
        factory_vref = 0x0600 | (uint16_t)Factory_VREFINT;
    } else {
        factory_vref = 0x0687;
    }
    ADC_Vrefint_Init();
    vrefint = ADC_Vrefint_Measure(10);
    ADC_Vrefint_Disable();
    vrefint = (((uint32_t)factory_vref * 300) / vrefint);
    if (vrefint > 1023) vrefint = 1023;

    // nRF24L01 pinout
    // PB3 - CE
    // PB4 - NSS
    // PB5 - SCK
    // PB6 - MOSI
    // PB7 - MISO
    // PC0 - IRQ
    nRF24_init(); // Init SPI interface for nRF24L01 communications
    if (!nRF24_Check()) {
        // Some banana happens - no answer from nRF24L01+
        // Disable all the peripherials to conserve battery power
        CLK_PCKENR1 = 0;
        CLK_PCKENR2 = 0;
        CLK_PCKENR3 = 0;
        // Disable the magnetic reed pins external interrupts
        PB_CR2_bit.C20 = 0;
        PB_CR2_bit.C21 = 0;
        // Blink red LED
        CLK_PCKENR2_bit.PCKEN22 = 1; // Enable RTC peripherial (PCKEN22)
        RTC_WakeupSet(ENABLE);
        while(1) {
            // Repeatedly blink red LED to indicate what some banana happens
            RTC_WakeupTimerSet(WU_TIMER / 8); // 1/8 second
            for(i = 0; i < 4; i++) {
                LED_RED();
                asm("HALT");
                LED_OFF();
                asm("HALT");
            }
            RTC_WakeupTimerSet(WU_TIMER * 10); // 10 seconds
            asm("HALT");
        }
    }

    // Configure nRF24L01 for TX mode:
    // Automatic retransmit RF_RETR times
    // Automatic retransmit delay is 1250us (payload <= 24 bytes)
    // RF channel 90 (2490MHz), 0dBm TX power, 1-byte CRC
    // Power down mode initially
    // 250kbps data rate
    nRF24_TXMode(RF_RETR,4,RF_CHANNEL,nRF24_DataRate_250kbps,nRF24_TXPower_0dBm,
                 nRF24_CRC_on,nRF24_CRC_1byte,nRF24_PWR_Down,
                 nRF24_TX_Addr,nRF24_TX_Addr_Size);
    for (i = 0; i < TX_PAYLOAD; i++) buf[i] = 0x00; // it's obvious :)

    // Timers initialization
    CLK_PCKENR1_bit.PCKEN10 = 1; // Enable TIM2 peripherial
#ifdef TIM2LSE
    SYSCFG_RMPCR2_bit.TIM2TRIGLSE_REMAP = 1; // TIM2 trigger controlled by LSE (OSC32_IN)
    TIM2_PSCR = 0; // Prescaler is off
    TIM2_ARRH = 0x00; // Auto-reload value (32768 / 33 -> 0.9929696969)
    TIM2_ARRL = 0x20;
    TIM2_ETR  = 0x40; // External trigger: enabled, non-inverted, prescaler off, filter off
#else
    TIM2_PSCR = 4; // Prescaler is 16
    TIM2_ARRH = 0; // Auto-reload value (TIM2 overflow in 1.0008ms at 2MHz)
    TIM2_ARRL = 0x7b;
#endif
    TIM2_IER_bit.UIE = 1; // Update interrupt enable
    TIM2_EGR_bit.UG = 1; // Generate UEV (update event) to reload TIM2 and set the prescaler

    CLK_PCKENR1_bit.PCKEN11 = 1; // Enable TIM3 peripherial
#ifdef TIM3LSE
    SYSCFG_RMPCR2_bit.TIM3TRIGLSE_REMAP = 1; // TIM3 trigger controlled by LSE (OSC32_IN)
    TIM3_PSCR = 0; // Prescaler is off
    TIM3_ARRH = 0x00; // Auto-reload value (32768 / 33 -> 0.9929696969)
    TIM3_ARRL = 0x20;
    TIM3_ETR  = 0x40; // External trigger: enabled, non-inverted, prescaler off, filter off
#else
    TIM3_PSCR = 4; // Prescaler is 16
    TIM3_ARRH = 0; // Auto-reload value (TIM3 overflow in 1.0008ms at 2MHz)
    TIM3_ARRL = 0x7b;
#endif
    TIM3_IER_bit.UIE = 1; // Update interrupt enable
    TIM3_EGR_bit.UG = 1; // Generate UEV (update event) to reload TIM3 and set the prescaler

    CLK_PCKENR1_bit.PCKEN12 = 1; // Enable TIM4 peripherial
    TIM4_PSCR = 0; // Prescaler is off
    TIM4_ARR = 0x7b; // TIM4 overflow in 1.0008ms
    TIM4_IER_bit.UIE = 1; // Update interrupt enable
    TIM4_EGR_bit.UG = 1; // Generate UEV (update event) to reload TIM4 and set the prescaler

    // Configure external interrupts
    CPU_CFG_GCR_bit.AL = 1; // Interrupt-only activation level (IRET causes the CPU to go back to Halt mode)
    EXTI_CR1_bit.P0IS = 0x02; // EXTI0 on falling edge
    EXTI_CR2_bit.P4IS = 0x02; // EXTI4 on falling edge
    ITC_SPR3_bit.VECT8SPR = 0x03; // EXTI0 IRQ level 3 (high priority) (EXTI0 IRQ = Vector 8)
    ITC_SPR4_bit.VECT12SPR = 0x03; // EXTI4 IRQ level 3 (high priority) (EXTI4 IRQ = Vector 12)
    asm("RIM"); // Enable global interrupts (enable priorities)

    // Configure system clock
    CLK_CKDIVR_bit.CKM = 0x07; // System clock source /128 (125kHz from HSI)

    // LED blinks shortly several times to show that the system is started normally
    RTC_WakeupSet(ENABLE); // For Active-Halt mode
    RTC_WakeupTimerSet(WU_TIMER / 4);
    for(i = 0; i < START_BLINKS; i++) {
        LED_GREEN();
        asm("HALT");
        LED_OFF();
        asm("HALT");
    }

    // Configure 1 second wakeup and disable wakeup counter for Halt mode
    RTC_WakeupTimerSet(WU_TIMER);
    RTC_WakeupSet(DISABLE);

    // Go deep sleep mode and wait for sensors alarm
    asm("HALT");

    // Main loop ^_^
    while(1) {
        // Wake nRF24L01 (Power Down -> Standby-I mode)
        // This will take about 1.5ms with internal nRF24L01 oscillator or 150us with external.
        // Delay here is unnecessary since nRF24L01 uses external oscillator.
        nRF24_Wake();

        cntr_wake++; // Count wakeups for debug purposes

        // Measure supply voltage
        if (!(cntr_wake % VREF_OFF_DUTY)) {
            ADC_Vrefint_Init();
            vrefint = ADC_Vrefint_Measure(10);
            ADC_Vrefint_Disable();
            vrefint = (((uint32_t)factory_vref * 300) / vrefint);
            if (vrefint > 1023) vrefint = 1023;
        }

        // Prepare data packet for nRF24L01
        buf[0]  = cntr_EXTI4 >> 8;
        buf[1]  = cntr_EXTI4 & 0xff;
        buf[2]  = tim3_diff >> 8;
        buf[3]  = tim3_diff & 0xff;
        buf[4]  = tim2_diff >> 8;
        buf[5]  = tim2_diff & 0xff;
        buf[6] |= (vrefint >> 8) & 0x03;
        buf[7]  = vrefint & 0xff;
        buf[8]  = prev_observe_TX;
        buf[9]  = cntr_wake >> 8;
        buf[10] = cntr_wake & 0xff;
        buf[11] = packets_lost >> 8;
        buf[12] = packets_lost & 0xff;
        buf[13] = ride_time >> 8;
        buf[14] = ride_time & 0xff;

        nRF24_SetRFChannel(RF_CHANNEL); // Set RF channel to clean "PLOS_CNT" part of the OBERVER_TX register
        i = nRF24_TXPacket(buf,TX_PAYLOAD);
        prev_observe_TX = nRF24_ReadReg(nRF24_REG_OBSERVE_TX);
        nRF24_PowerDown(); // Standby-I -> Power down

        if (i & nRF24_MASK_TX_DS) {
            // Packet sent successfully
            ride_time = 0;
        }
        if (i & nRF24_MASK_MAX_RT) {
            // Max retransmit count reached -> packet was not delivered
            packets_lost += prev_observe_TX >> 4;
        }

        if (TIM2_CR1_bit.CEN || TIM3_CR1_bit.CEN) {
            CPU_CFG_GCR_bit.AL = 1; // Set interrupt-only activation level
            asm("WFI"); // Wait mode
        } else {
            RTC_WakeupSet(DISABLE); // Disable wakeup timer
            CPU_CFG_GCR_bit.AL = 0; // Set main activation level
            asm("HALT"); // Halt mode
        }
    }
}
