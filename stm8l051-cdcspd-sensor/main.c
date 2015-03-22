//////////////////////////
////// STM8L051F3P6 //////
//////////////////////////


#include <stdint.h>
#include <iostm8l051f3.h>

#include "main.h"
#include "nRF24.h"
#include "rtc.h"


///////////////////////////////////////////////////////////////////////////////


#define SOFT_DEBOUNCE    // If defined - software debounce is used, no debounce otherwise
#define TIM2LSE          // If defined - TIM2 will be clocked from LSE
#define TIM3LSE          // If defined - TIM3 will be clocked from LSE


///////////////////////////////////////////////////////////////////////////////


#define RF_CHANNEL           90  // nRF24L01 channel (90ch = 2490MHz)

#define TIM_DEBOUNCE         65  // Debounce delay, higher value means lowest high speed can be measured.
#define TIM_TIMEOUT        6000  // Magnetic reed impulse timeout (roughly measured in milliseconds)

#define WU_TIMER             63  // Wakeup timer Period = (WU_TIMER + 1) * 0,015625 seconds (63 = 1 second)

#define VREF_OFF_DUTY       600  // Internal voltage measure off-duty ratio
#define VREF_COUNT           10  // How many measurements of Vrefint will be done

#define START_BLINKS          3  // How many times the green LED will blink after reset
#define REED_PASSES          40  // How many reed passes the LED will blink after reset
#define LED_BLINK_ON_TIME   252  // How many TIM4 periods the LED will be lit

#define nRF24_TX_Addr     "WBC"  // TX address for nRF24L01
#define nRF24_TX_Addr_Size    3  // TX address size


///////////////////////////////////////////////////////////////////////////////


// 2-pin Bi color LED connected to PB1 and PB2
// Macros to switch colors and turn off
#define LED_RED()   PB_ODR_bit.ODR1 = 0; PB_ODR_bit.ODR2 = 1;
#define LED_GREEN() PB_ODR_bit.ODR1 = 1; PB_ODR_bit.ODR2 = 0;
#define LED_OFF()   PB_ODR_bit.ODR1 = 0; PB_ODR_bit.ODR2 = 0;


///////////////////////////////////////////////////////////////////////////////


uint8_t i; // Really need to comment this?

// Last measured internal voltage
uint16_t vrefint;
// Factory measured internal voltage with 3V external
uint16_t factory_vref;

// EXTI# impulse counters
volatile uint16_t cntr_EXTI0 = 0;
volatile uint16_t cntr_EXTI4 = 0;

// TIM# difference between two pulses of EXTI#
volatile uint16_t tim2_diff = 0;
volatile uint16_t tim3_diff = 0;

// TIM# overflows counter
volatile uint16_t tim2 = 0;
volatile uint16_t tim3 = 0;
volatile uint16_t tim4 = 0;

// How many times after reset magnet passed the sensors
uint8_t cntr_rst_passes = REED_PASSES;

// nRF24L01 payload buffer
struct __packed {
    uint16_t cntr_SPD;  // SPD impulses counter
    uint16_t tim_SPD;   // SPD interval
    uint16_t tim_CDC;   // CDC interval
    uint16_t vrefint;   // Measured voltage
    uint16_t cntr_wake; // Wake counter
} payload;

// Wakeup counter
uint16_t cntr_wake = 0;

// ADC measurements buffer
uint16_t ADC_buf[VREF_COUNT];


///////////////////////////////////////////////////////////////////////////////


// RTC_WAKEUP IRQ handler
#pragma vector=RTC_WAKEUP_vector
__interrupt void RTC_IRQHandler(void) {
    CPU_CFG_GCR_bit.AL = 0; // Set main activation level
    // ST errata sheet says what AL bit must be reset in ISR at least two
    // instructions before the IRET instruction
    asm("NOP");
    RTC_ISR2_bit.WUTF = 0; // Clear wakeup timer interrupt flag
}

// EXTI0 IRQ handler
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

// EXTI4 IRQ handler
#pragma vector=EXTI4_vector
__interrupt void EXTI4_IRQHandler(void) {
    TIM3_CR1_bit.CEN = 1; // Enable TIM3 counter

#ifdef SOFT_DEBOUNCE
    if (tim3 > TIM_DEBOUNCE) {
#endif
        cntr_EXTI4++;
        if (cntr_EXTI4 > 2) tim3_diff = tim3;

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

// TIM2 Update/Overflow/Trigger/Break interrupt IRQ handler
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

// TIM3 Update/Overflow/Trigger/Break interrupt IRQ handler
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

// TIM4 Update interrupt IRQ handler
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

// DMA1 channel 0 transfer complete IRQ handler
#pragma vector=DMA1_CH0_TC_vector
__interrupt void DMA1_CHANNEL0_1_IRQHandler(void) {
    // Channel 0 interrupt
    if (DMA1_GIR1_bit.IFC0) {
        // Set main activation level
        CPU_CFG_GCR_bit.AL = 0;
        // Clear the TC interrupt flag
        DMA1_C0SPR_bit.TCIF = 0;
        // Disable the DMA channel 0
        DMA1_C0CR_bit.EN = 0;
        // Disable the DMA1 peripheral clock (to save power)
        CLK_PCKENR2_bit.PCKEN24 = 0;
    }
}


///////////////////////////////////////////////////////////////////////////////


// Enable then ADC and Vrefint channel
// note: there must be some delay after this to stabilize the ADC (Twkup = 3us)
void ADC_Vrefint_Init(void) {
    // Enable the ADC peripherial (PCKEN20)
    CLK_PCKENR2_bit.PCKEN20 = 1;
    // Enable internal reference voltage
    ADC1_TRIGR1_bit.VREFINTON = 1;
    /*
    // Configure the ADC
    ADC1_CR1_bit.ADON  = 1; // Enable the ADC
    ADC1_CR1_bit.CONT  = 1; // ADC continuous conversion
    ADC1_CR3_bit.SMTP2 = 4; // Sampling time = 48 ADC clock cycles
    ADC1_CR3_bit.CHSEL = 0; // Analog watchdog channels disabled
    */
    // Configure the ADC
    //   - 12-bit resolution
    //   - continuous conversion
    //   - wake-up ADC
    ADC1_CR1 = 0x05;
    //   - ADC clock prescaler = CK
    //   - External triggers disabled
    //ADC1_CR2 = 0x00; // This is reset value, don't tuch this...
    //   - Sampling time = 48 ADC clock cycles
    //   - Analog watchdog channels disabled
    ADC1_CR3 = 0x80;
    /*
    // Scan sequence:
    ADC1_SQR1_bit.CHSEL_S28 = 1; // Vrefint channel
    ADC1_SQR1_bit.DMAOFF    = 0; // DMA enabled
    */
    // Scan sequence:
    //   - DMA enabled
    //   - VREFINT channel
    ADC1_SQR1 = 0x10;

    // Enable the DMA1 peripheral clock (PCKEN24)
    CLK_PCKENR2_bit.PCKEN24 = 1;

    // No DMA timeout
    //DMA1_GCSR_bit.TO = 0;

    // Remap ADC1 DMA request/acknowledge to DMA1 channel 0
    SYSCFG_RMPCR1_bit.ADC1DMA_REMAP = 0;

    /*
    // Configure the DMA channel 0 (DMA1_EOC)
    DMA1_C0CR_bit.MINCDEC = 1; // Memory address increment
    DMA1_C0CR_bit.CIRC    = 0; // Circular mode disabled
    DMA1_C0CR_bit.DIR     = 0; // Peripheral to memory transfers
    DMA1_C0CR_bit.HTIE    = 0; // Half-transaction interrupt disabled
    DMA1_C0CR_bit.TCIE    = 1; // Transaction complete interrupt enabled
    DMA1_C0CR_bit.EN      = 0; // Channel disabled
    DMA1_C0SPR_bit.PL1    = 0; // Channel priority: medium
    DMA1_C0SPR_bit.PL0    = 1;
    DMA1_C0SPR_bit.TSIZE  = 1; // 16-bit transactions
    DMA1_C0SPR_bit.HTIF   = 0; // Clear the HT flag
    DMA1_C0SPR_bit.TCIF   = 0; // Clear the TC flag
    */
    // Configure the DMA channel 0 (DMA1_EOC)
    //   - memory address increment
    //   - normal buffer mode
    //   - peripheral to memory transfers
    //   - transaction complete interrupt enabled
    //   - channel disabled
    DMA1_C0CR   = 0x22;
    //   - medium channel priority
    //   - 16-bit transactions
    //   - clear the TC and HT flags
    DMA1_C0SPR  = 0x18;
    // ADC1 data register high byte address
    DMA1_C0PARH = (uint8_t)((uint16_t)(&ADC1_DRH) >> 8);
    DMA1_C0PARL = (uint8_t)((uint16_t)(&ADC1_DRH) & 0xFF);

    // Global enable of the DMA channels
    DMA1_GCSR_bit.GEN = 1;
}

// Measure Vrefint
uint16_t ADC_Vrefint_Measure(void) {
    uint8_t i;
    uint16_t result;

    // Memory buffer address
    DMA1_C0M0ARH = (uint8_t)((uint16_t)ADC_buf >> 8);
    DMA1_C0M0ARL = (uint8_t)((uint16_t)ADC_buf & 0xFF);
    // Number of the DMA transactions
    DMA1_C0NDTR = VREF_COUNT;
    // Enable the DMA1 channel 0
    DMA1_C0CR_bit.EN = 1;

    // Wait until Vrefint becomes stable
    while (!PWR_CSR2_bit.VREFINTF);

    // Start ADC conversion in continuous mode by software trigger
    ADC1_CR1_bit.START = 1;
    // In theory there should be a WFE instruction instead of WFI, but according to the ST errata sheet
    // my be "incorrect code execution when WFE instruction is interrupted by ISR or event"
    // So WFI executed here and IRQ bit is cleared in DMA1_CHANNEL0_1_IRQHandler() procedure
    // Repeatedly put the MCU into a sleep mode while the DMA channel is enabled
    // This loop made to ensure what the MCU will sleep all the time while the DMA is working, beacuse
    // EXTI or RTC interrupt can wake it earlier
    do {
        asm("WFI");
    } while (DMA1_C0CR_bit.EN);

    // Calculate rough average of ADC values
    result = ADC_buf[0];
    for (i = 1; i < VREF_COUNT; i++) {
        result += ADC_buf[i];
        result >>= 1;
    }

    return result;
}

// Disable the Vrefint channel and the ADC peripheral
void ADC_Vrefint_Disable(void) {
    // Disable the internal reference voltage
    ADC1_TRIGR1_bit.VREFINTON = 0;
    // Put the ADC in power-down mode
    ADC1_CR1_bit.ADON = 0;
    // Disable the ADC peripherial clock
    CLK_PCKENR2_bit.PCKEN20 = 0;
}


///////////////////////////////////////////////////////////////////////////////


int main(void) {
    // Disable the Boot ROM
    CLK_PCKENR2_bit.PCKEN27 = 0;

    // Disable global interrupts (level 3 set)
    asm("SIM");

    // LED pins (PB1, PB2)
    PB_DDR_bit.DDR1 = 1; // Set PB1 as output
    PB_ODR_bit.ODR1 = 0; // Latch "1" in output register
    PB_CR1_bit.C11  = 1; // Slow output
    PB_CR2_bit.C21  = 0; // Disable external interrupt
    PB_DDR_bit.DDR2 = 1; // Set PB2 as output
    PB_ODR_bit.ODR2 = 0; // Latch "1" in output register
    PB_CR1_bit.C12  = 1; // Slow output
    PB_CR2_bit.C22  = 0; // Disable external interrupt

    LED_RED(); // Lit red LED to indicate startup sequence

    // Magnetic reed inputs
    // PB0 = Reed#0 (CDC sensor)
    PB_ODR_bit.ODR0 = 1; // Latch "1" in output register
    PB_DDR_bit.DDR0 = 0; // Input
    PB_CR1_bit.C10  = 0; // Floating input (external pull-up resistor)
    PB_CR2_bit.C20  = 1; // Enable external interrupt
    // PC4 = Reed#1 (SPD sensor)
    PC_ODR_bit.ODR4 = 1; // Latch "1" in output register
    PC_DDR_bit.DDR4 = 0; // Input
    PC_CR1_bit.C14  = 0; // Floating input (external pull-up resistor)
    PC_CR2_bit.C24  = 1; // Enable external interrupt

    // nRF24L01 power enable (PA2)
    PA_ODR_bit.ODR2 = 0; // Latch "0" in output register
    PA_DDR_bit.DDR2 = 1; // Output
    PA_CR1_bit.C12  = 1; // Push-pull
    PA_CR2_bit.C22  = 0; // 2MHz output speed

    // Enable power for the nRF24L01 (power-on reset 100ms)
    PA_ODR_bit.ODR2 = 1;

    // Configure unused GPIO ports as input with pull-up for powersaving
    // PD0
    PD_ODR_bit.ODR0 = 0; // Latch "0" in output register
    PD_DDR_bit.DDR0 = 0; // Input
    PD_CR1_bit.C10  = 1; // With Pull-up
    PD_CR2_bit.C20  = 0; // External interrupt disabled
    // PA3
    PA_ODR_bit.ODR3 = 0; // Latch "0" in output register
    PA_DDR_bit.DDR3 = 0; // Input
    PA_CR1_bit.C13  = 1; // With Pull-up
    PA_CR2_bit.C23  = 0; // External interrupt disabled
    // PC0
    PC_ODR_bit.ODR0 = 0; // Latch "0" in output register
    PC_DDR_bit.DDR0 = 0; // Input
    PC_CR1_bit.C10  = 1; // With Pull-up
    PC_CR2_bit.C20  = 0; // External interrupt disabled

    // Configure the RTC
    LED_RED();
    if (RTC_Init() != RTC_OK) {
        // LSE not initialized --> there is no sense to continue
        // Put the MCU in HALT mode to prevent battery drain

        // Power-off the LED
        LED_OFF();

        // Disable all the peripherials
        CLK_PCKENR1 = 0;
        CLK_PCKENR2 = 0;
        CLK_PCKENR3 = 0;

        // Disable the magnetic reed pins external interrupts
        PB_CR2_bit.C20 = 0;
        PB_CR2_bit.C21 = 0;

        // Disable the nRF24L01 IRQ pin external interrupt
        PC_CR2_bit.C21 = 0;

        // Swith-off power for the nRF24L01
        PA_ODR_bit.ODR2 = 0;

        // Halt until reset
        while(1) asm("HALT");
    }
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
    vrefint = ADC_Vrefint_Measure();
    ADC_Vrefint_Disable();
    vrefint = (((uint32_t)factory_vref * 300) / vrefint);
    if (vrefint > 1023) vrefint = 1023;

    // nRF24L01 pinout
    // PB3 - CE
    // PB4 - NSS
    // PB5 - SCK
    // PB6 - MOSI
    // PB7 - MISO
    // PC1 - IRQ
    nRF24_Init(); // Init SPI interface
    if (!nRF24_Check()) {
        // No answer from the nRF24L01 --> some banana happens
        // There is no sense to continue, blink LED to indicate troubles

        // Disable the magnetic reed pins external interrupts
        PB_CR2_bit.C20 = 0;
        PB_CR2_bit.C21 = 0;

        // Disable nRF24L01 IRQ pin external interrupt
        PC_CR2_bit.C21 = 0;

        // Swith-off power for the nRF24L01
        PA_ODR_bit.ODR2 = 0;

        // Blink red LED four times to indicate what some banana happens
        RTC_WakeupSet(ENABLE);
        for(i = 0; i < 4; i++) {
            LED_RED();
            RTC_WakeupTimerSet(WU_TIMER / 8); // light for 1/8 second
            asm("HALT");
            LED_OFF();
            RTC_WakeupTimerSet(WU_TIMER / 2); // half second off time
            asm("HALT");
        }

        // Disable the RTC wake-up
        RTC_WakeupSet(DISABLE);

        // Disable all the peripherials
        CLK_PCKENR1 = 0;
        CLK_PCKENR2 = 0;
        CLK_PCKENR3 = 0;

        // Halt until reset
        while(1) asm("HALT");
    }

    // Configure the nRF24L01+ in TX mode
    nRF24_TXMode(0,                      // auto retransmit disabled
                 7,                      // retransmit delay 2000us ((7+1)*250us) (ignored)
                 RF_CHANNEL,             // RF channel
                 nRF24_DataRate_250kbps, // data rate
                 nRF24_TXPower_6dBm,     // TX power
                 nRF24_CRC_2byte,        // CRC scheme
                 nRF24_PWR_Down,         // Initial power state
                 nRF24_TX_Addr,          // TX address
                 nRF24_TX_Addr_Size      // TX address size
    );

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
    EXTI_CR1_bit.P0IS = 0x02; // EXTI0 on falling edge only
    EXTI_CR2_bit.P4IS = 0x02; // EXTI4 on falling edge only
    EXTI_CR1_bit.P1IS = 0x02; // EXTI1 on falling edge only
    ITC_SPR3_bit.VECT8SPR  = 0x03; // EXTI0 IRQ level 3 (high priority) (EXTI0 IRQ = Vector 8)
    ITC_SPR4_bit.VECT12SPR = 0x03; // EXTI4 IRQ level 3 (high priority) (EXTI4 IRQ = Vector 12)
    ITC_SPR3_bit.VECT9SPR  = 0x00; // EXTI1 IRQ level 2 (below high priority) (EXTI1 IRQ = Vector 9)
    asm("RIM"); // Enable global interrupts (enable priorities)

    // Configure the system clock
//    CLK_CKDIVR_bit.CKM = 7; // System clock source /128 (125kHz from HSI)
    CLK_CKDIVR_bit.CKM = 1; // System clock source /2 (8MHz from HSI)
//    CLK_CKDIVR_bit.CKM = 0; // System clock source /1 (16MHz from HSI)

    // Configure FLASH power savings
    FLASH_CR1_bit.WAITM = 1; // Flash program and data EEPROM in IDDQ during wait modes

    // Enable automatic switch off the internal voltage reference
    PWR_CSR2_bit.ULP = 1;

    // Check for low voltage and blink the LED if it below 2.3V
    if (vrefint < 230) {
        RTC_WakeupSet(ENABLE); // For Active-Halt mode
        RTC_WakeupTimerSet(WU_TIMER / 8);
        for(i = 0; i < START_BLINKS; i++) {
            LED_RED();
            asm("HALT");
            LED_GREEN();
            asm("HALT");
            LED_OFF();
            asm("HALT");
        }
    } else {
        // LED blinks shortly several times to show that the system is started normally
        RTC_WakeupSet(ENABLE); // For Active-Halt mode
        RTC_WakeupTimerSet(WU_TIMER / 8);
        for(i = 0; i < START_BLINKS; i++) {
            LED_GREEN();
            asm("HALT");
            LED_OFF();
            asm("HALT");
        }
    }

    // Configure 1 second wakeup and disable wakeup counter for Halt mode
    RTC_WakeupTimerSet(WU_TIMER);
    RTC_WakeupSet(DISABLE);

    // Go deep sleep mode and wait for sensors alarm
    asm("HALT");

    // Main loop ^_^
    while(1) {
        cntr_wake++; // Count wakeups

        // Measure supply voltage
        if (!(cntr_wake % VREF_OFF_DUTY)) {
            ADC_Vrefint_Init();
            vrefint = ADC_Vrefint_Measure();
            ADC_Vrefint_Disable();
            vrefint = (((uint32_t)factory_vref * 300) / vrefint);
            if (vrefint > 1023) vrefint = 1023;
        }

        // Enable the SPI1 peripheral clock (to communicate with nRF24L01)
        CLK_PCKENR1_bit.PCKEN14 = 1;

        // Wake nRF24L01 (Power Down -> Standby-I mode)
        // This will take about 1.5ms with internal nRF24L01 oscillator or 150us with external.
        // Delay here is unnecessary since nRF24L01 uses external oscillator.
        nRF24_Wake();

        // Prepare data packet for nRF24L01
        payload.cntr_SPD  = cntr_EXTI4;
        payload.tim_SPD   = tim3_diff;
        payload.tim_CDC   = tim2_diff;
        payload.vrefint   = vrefint & 0x03FF; // Save bits [15..10] for future use
        payload.cntr_wake = cntr_wake;

        // Send data packet
        nRF24_TXPacket((uint8_t *)&payload,sizeof(payload));

        // Standby-I -> Power down
        nRF24_PowerDown();

        // Disable the SPI1 peripheral clock (to save power)
        CLK_PCKENR1_bit.PCKEN14 = 0;

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
